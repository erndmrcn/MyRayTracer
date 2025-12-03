//
//  Renderer.swift
//  RayTracer
//

import ParsingKit
import simd
import Foundation
#if canImport(UIKit)
import UIKit
#endif

final class Renderer: @unchecked Sendable {
    public let ctx: RTContext

    public init(ctx: RTContext) {
        self.ctx = ctx
    }
}

// MARK: - Rendering (Whitted-style Ray Tracer)
public actor ProgressRelay {
    private var done = 0
    private let total: Int
    private let cb: (@Sendable (Int) -> Void)?

    init(total: Int, cb: (@Sendable (Int) -> Void)?) {
        self.total = total
        self.cb = cb
    }

    func rowFinished() async {
        done &+= 1
        guard let cb else { return }
        if (done & 7) == 0 || done == total {
            DispatchQueue.main.async { [done] in
                cb(done)
            }
        }
    }
}

extension Renderer {
    public struct ChunkResult: Sendable {
        let startRow: Int
        let endRow: Int
        let pixels: [Vec3]
//        let rays: Int64
        let nodeVisits: Int64
    }

    @Sendable public func render(
        scene: Scene,
        cameraIndex: Int = 0,
        progressRow: (@Sendable (Int) -> Void)? = nil
    ) async throws -> [Vec3] {

        let cam = scene.cameras[cameraIndex]
        let numSamples = cam.numSamples
        let apertureSize = cam.apertureSize
        let focusDistance = cam.focusDistance
        let width  = max(1, cam.imageResolution.0)
        let height = max(1, cam.imageResolution.1)

        let (eye, u, v, w, l, r, b, t, nd, isLookAt) = makeCameraBasis(from: cam, aspect: Double(width)/Double(height))

        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)
        let m: Vec3 = eye - w * nd
        let q00: Vec3 = m + u * l + v * t

        var out = [Vec3](repeating: .zero, count: width * height)
        let relay: ProgressRelay? = (progressRow == nil) ? nil : ProgressRelay(total: height, cb: progressRow)

        let CH = 8
        var chunks: [(Int, Int)] = []
        var row = 0
        while row < height {
            let end = min(row + CH, height)
            chunks.append((row, end))
            row = end
        }

        var totalNodeVisits: Int64 = 0
        let bvh = ctx.bvh
        let materials = ctx.materials
        let eps = ctx.intersectionTestEpsilon
        let maxDepth = scene.maxRecursionDepth
        let shadowEps = ctx.shadowRayEpsilon

        // --- Build stratified jitter table (once per pixel)
        var rng = PCG32(seed: 0x123456789ABCDEF)
        let (jitterX, jitterY) = buildStratifiedJitter(&rng)

        @inline(__always)
        @Sendable func trace(_ inRay: inout Ray, _ depth: Int, _ localNodeVisits: inout Int64, _ rng: inout PCG32, jitterIndex: inout Int) -> Vec3 {
            if depth > maxDepth { return scene.backgroundColor }
            guard let tlas = ctx.tlas else { return .zero }
            let visits = ctx.intersectTLAS(ray: &inRay, tlas: tlas , eps: eps)
            localNodeVisits += Int64(visits)
            if inRay.hit.kind == .none {
                return scene.backgroundColor
            }

            let matIndex = max(0, min(self.ctx.materials.count - 1, inRay.hit.mat - 1))
            let mat = self.ctx.materials[matIndex]
            let p = inRay.hit.p
            let Ngeo = inRay.hit.n
            let frontFacing = dot(inRay.dir, Ngeo) < 0
            let N = frontFacing ? Ngeo : -Ngeo

            let ior = mat.ior
            let hasRefraction = (ior > 0)

            let computeDirectLight = !hasRefraction || frontFacing
            var Lo = computeDirectLight ? scene.lights.ambient * mat.ambient : .zero
            if computeDirectLight {
                for pointLight in ctx.pointLights {
                    var wi = pointLight.position - p
                    let dist = length(wi)
                    wi = normalize(wi)

                    var sRay = Ray(origin: p + wi * shadowEps, dir: wi, time: inRay.time)
                    sRay.tMax = dist
                    let blocked = self.occluded(shadowRay: sRay)
                    if !blocked {
                        let NdotL = max(0.0, dot(N, wi))
                        if NdotL > 0 {
                            let kd = mat.diffuse
                            let ks = mat.specular
                            let shininess = max(1.0, mat.phong)

                            let Ld = kd * NdotL
                            let view = normalize(-inRay.dir)
                            let h = normalize(wi + view)
                            let NdotH = max(0.0, dot(N, h))
                            let Ls = ks * pow(NdotH, shininess)

                            let atten = pointLight.intensity / max(dist*dist, 1e-12)
                            Lo += (Ld + Ls) * atten
                        }
                    }
                }

                for areaLight in ctx.areaLights {
                    let nL = normalize(areaLight.normal)
                    let (t, b) = orthonormalBasis(nL)
                    let size = areaLight.size
                    let half = size * 0.5
                    let area = size * size
                    let pdf = 1.0 / area
                    let r1 = jitterX[jitterIndex % 100] / 10.0 - 0.5
                    let r2 = jitterY[jitterIndex % 100] / 10.0 - 0.5
                    jitterIndex &+= 1

                    let samplePos = areaLight.position + t * (r1 * size) + b * (r2 * size)

                    var wi = samplePos - p
                    let dist2 = simd_length_squared(wi)
                    let dist = sqrt(dist2)
                    wi /= dist

                    // ---- Cosine at shading point ----
                    let NdotL = dot(N, wi)
                    if NdotL <= 0 { continue }

                    // ---- Cosine at light surface: use ABS ----
                    let Ln = Swift.abs(dot(nL, -wi))
                    if Ln <= 0 { continue }

                    // ---- Shadow Ray ----
                    var sRay = Ray(origin: p + wi * shadowEps, dir: wi, time: inRay.time)
                    sRay.tMax = dist - shadowEps
                    if occluded(shadowRay: sRay) { continue }
                    // ---- BRDF ----
                    let view = normalize(-inRay.dir)
                    let h = normalize(wi + view)
                    let Ld = mat.diffuse * NdotL
                    let Ls = mat.specular * pow(max(0, dot(N, h)), mat.phong)
                    let brdf = Ld + Ls

                    // ---- Final contribution ----
                    let contrib = brdf * areaLight.radiance * (Ln / dist2) * area

                    Lo += contrib
                }
            }

            if mat.type == "mirror" && depth < maxDepth {
                var rd = normalize(reflect(inRay.dir, N))
                if !mat.roughness.isZero {
                    let (t, b) = orthonormalBasis(rd)
                    let rand1 = rng.nextFloat() - 0.5
                    let rand2 = rng.nextFloat() - 0.5
                    rd = rd + (mat.roughness * rand1) * b + (mat.roughness * rand2) * t
                    rd = normalize(rd)
                }
                var rRay = inRay
                rRay.tMin = 0
                rRay.origin = p + N * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()
                let Li = trace(&rRay, depth + 1, &localNodeVisits, &rng, jitterIndex: &jitterIndex)

                Lo += mat.mirror * Li
            } else if mat.type == "dielectric" && depth < maxDepth {

                var microfacetN = N
                if !mat.roughness.isZero {
                    let (t, b) = orthonormalBasis(N)
                    let r1 = (rng.nextFloat() * 2.0) - 1.0
                    let r2 = (rng.nextFloat() * 2.0) - 1.0

                    microfacetN = normalize(N + (t * r1 * mat.roughness) + (b * r2 * mat.roughness))
                }
                let entering = frontFacing
                let n1 = entering ? 1.0 : mat.ior
                let n2 = entering ? mat.ior : 1.0

                let cosI = -dot(inRay.dir, microfacetN)
                let (R, cosTopt, sin2T) = fresnelDielectric(n1: n1, n2: n2, cosTheta: cosI)

                let rd = normalize(reflect(inRay.dir, microfacetN))

                var rRay = inRay
                rRay.tMin = 0
                rRay.origin = p + rd * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()

                let LiR = trace(&rRay, depth + 1, &localNodeVisits, &rng, jitterIndex: &jitterIndex)

                if cosTopt == nil || sin2T > 1 {
                    Lo += LiR
                } else {
                    let cosT = cosTopt!
                    let eta = n1 / n2
                    let td = normalize((inRay.dir * eta) + (microfacetN * (eta * cosI - cosT)))

                    var tRay = Ray(origin: p + td * shadowEps, dir: td, time: inRay.time)
                    var LiT = trace(&tRay, depth + 1, &localNodeVisits, &rng, jitterIndex: &jitterIndex)

                    if entering, mat.absorption != .zero, tRay.hit.kind != .none, tRay.hit.mat == inRay.hit.mat {
                        let d = max(tRay.hit.t, 0.0)
                        LiT *= beerAttenuate(LiT, c: mat.absorption, distance: d)
                    }

                    Lo += LiR * R + LiT * (1.0 - R)
                }
            } else if mat.type == "conductor" && depth < maxDepth {
                let cosI = max(0.0, -dot(inRay.dir, N))
                let Rf = fresnelConductorRGB(eta: mat.ior, k: mat.absorptionIndex, cosI_: cosI)

                // 2. Calculate Direction
                var rd = normalize(reflect(inRay.dir, N))

                if !mat.roughness.isZero {
                    let (t, b) = orthonormalBasis(rd)
                    let rand1 = rng.nextFloat() - 0.5
                    let rand2 = rng.nextFloat() - 0.5
                    rd = rd + (mat.roughness * rand1) * b + (mat.roughness * rand2) * t
                    rd = normalize(rd)
                }

                var rRay = inRay
                rRay.tMin = 0
                rRay.origin = p + N * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()
                let Li = trace(&rRay, depth + 1, &localNodeVisits, &rng, jitterIndex: &jitterIndex)
                Lo += (Rf * mat.mirror) * Li
            }

            if !(Lo.x.isFinite && Lo.y.isFinite && Lo.z.isFinite) || (Lo.x.isNaN || Lo.y.isNaN || Lo.z.isNaN) {
                print("found NaN and infinite value of \(Lo)")
                return Vec3(0, 0, 0)
            }

            return Lo
        }

        try await withThrowingTaskGroup(of: ChunkResult.self) { group in
            for (startRow, endRow) in chunks {
                group.addTask {
                    var jitterIndex = 0
                    var local = [Vec3](repeating: .zero, count: (endRow - startRow) * width)
                    var localNodeVisits: Int64 = 0

                    for j in startRow..<endRow {
                        for i in 0..<width {
                            var rng = PCG32(seed: (UInt64(j) << 32) ^ UInt64(i) &+ 0x9E3779B97F4A7C15)

                            var pixelColor = Vec3.zero
                            var base = (j - startRow) * width + i
                            let samples = max(1, numSamples)

                            let n = Int(Double(samples).squareRoot())
                            let nRows = n
                            let nCols = n

                            var sampleIndex = 0
                            for sy in 0..<nRows {
                                for sx in 0..<nCols {
                                    let ξ1 = rng.nextFloat()
                                    let ξ2 = rng.nextFloat()

                                    let iOffset = (Double(sx) + ξ1) / Double(nCols)
                                    let jOffset = (Double(sy) + ξ2) / Double(nRows)

                                    let currentI = Double(i) + iOffset
                                    let currentJ = Double(j) + jOffset

                                    let vOff = v * (currentJ * dv)
                                    let rowTopLeft = q00 - vOff
                                    let uOff = u * (currentI * du)
                                    let s = rowTopLeft + uOff

                                    var dir0 = normalize(s - eye)
                                    var dir = dir0
                                    var camEye = eye

                                    if apertureSize > 0 && focusDistance > 0 {
                                        let forward = -w
                                        let denom = dot(dir0, forward)
                                        let tFocus = Swift.abs(denom) < 1e-6 ? focusDistance : (focusDistance / denom)
                                        let pFocus = eye + dir0 * tFocus

                                        let uRand = rng.nextFloat() - 0.5
                                        let vRand = rng.nextFloat() - 0.5
                                        let lensOffset = (uRand * u + vRand * v) * apertureSize
                                        let a = eye + lensOffset

                                        dir = normalize(pFocus - a)
                                        camEye = a
                                    }

                                    var ray = Ray(origin: camEye, dir: dir, time: rng.nextFloat())
                                    let denom = dot(dir, w)
                                    let tImg = dot((eye - w * nd) - camEye, w) / (denom.isZero ? Scalar.leastNonzeroMagnitude : denom)

                                    ray.tMin = max(tImg, 0.0)
                                    let sampleColor = trace(&ray, 0, &localNodeVisits, &rng, jitterIndex: &jitterIndex)
                                    pixelColor += sampleColor/* * w*/
                                    //                                            weightSum += w
                                    sampleIndex += 1
                                    if sampleIndex >= samples { break }
                                }
                                if sampleIndex >= samples { break }
                            }

                            local[base] = pixelColor / Double(samples)
                        }
                    }

                    if let relay { await relay.rowFinished() }
                    return ChunkResult(startRow: startRow, endRow: endRow, pixels: local, nodeVisits: localNodeVisits)
                }
            }

            for try await chunk in group {
                let start = chunk.startRow, end = chunk.endRow
                let rows = end - start
                chunk.pixels.withUnsafeBufferPointer { src in
                    out.withUnsafeMutableBufferPointer { dst in
                        dst.baseAddress!.advanced(by: start * width)
                            .assign(from: src.baseAddress!, count: rows * width)
                    }
                }
                totalNodeVisits &+= chunk.nodeVisits
            }

            try await group.waitForAll()
        }

        return out
    }

    // --- Camera assembly
    public func makeCameraBasis(from cam: ParsingKit.Camera, aspect: Double)
    -> (eye: Vec3, u: Vec3, v: Vec3, w: Vec3, l: Double, r: Double, b: Double, t: Double, nd: Double, isLookAt: Bool) {

        let e  = cam.position
        let nd = Double(cam.nearDistance)
        var l: Double = 0, r: Double = 0, b: Double = 0, t: Double = 0
        var gazeNorm: Vec3
        var wv: Vec3
        var upNorm: Vec3
        var u: Vec3
        var v: Vec3

        if cam.type?.lowercased() == "lookat" {
            let gaze = cam.gazePoint - cam.position
            gazeNorm = normalize(gaze)
            wv = -gazeNorm
            upNorm = normalize(cam.up)
            u = normalize(cross(upNorm, wv))
            v = normalize(cross(wv, u))

            if let fovy = cam.fovy {
                let fovYRad = Double(fovy * .pi) / (2.0 * 180.0)
                t = nd * tan(fovYRad)
                b = -t
                r = t * aspect
                l = -r
            } else {
                t = nd * 0.5
                b = -t
                r = t * aspect
                l = -r
            }
            return (e, u, v, wv, l, r, b, t, nd, true)
        } else {
            l = Double(cam.nearPlane[0])
            r = Double(cam.nearPlane[1])
            b = Double(cam.nearPlane[2])
            t = Double(cam.nearPlane[3])
            gazeNorm = normalize(cam.gaze)
            wv = -gazeNorm
            upNorm = normalize(cam.up)
            u = normalize(cross(upNorm, wv))
            v = normalize(cross(wv, u))
            return (e, u, v, wv, l, r, b, t, nd, false)
        }
    }

    public func occluded(shadowRay rIn: Ray) -> Bool {
        guard let tlas = ctx.tlas else { return false }
        let eps = ctx.intersectionTestEpsilon
        return ctx.occludedTLAS(ray: rIn, tlas: tlas, eps: eps)
    }
}

// MARK: - Material helpers
extension Renderer {

    @inline(__always)
    func sampleGlossyLobe(perfectDirection: Vec3, roughness: Scalar, rng: inout PCG32) -> Vec3 {
        if roughness < 1e-5 { return perfectDirection }

        let r = perfectDirection
        var rPrime = r
        let absX = Swift.abs(r.x)
        let absY = Swift.abs(r.y)
        let absZ = Swift.abs(r.z)

        if absX <= absY && absX <= absZ {
            rPrime.x = 1.0
        } else if absY <= absX && absY <= absZ {
            rPrime.y = 1.0
        } else {
            rPrime.z = 1.0
        }
        let u = normalize(cross(r, rPrime))
        let v = cross(r, u)

        let xi1 = Double(rng.nextFloat()) - 0.5
        let xi2 = Double(rng.nextFloat()) - 0.5

        let rReal = normalize(r + xi1 * u + xi2 * v)
        return rReal
    }

    @inline(__always)
    public func reflect(_ d: Vec3, _ n: Vec3) -> Vec3 { d - 2.0 * dot(d, n) * n }

    @inline(__always)
    public func beerAttenuate(_ Li: Vec3, c: Vec3, distance x: Double) -> Vec3 {
        guard x.isFinite, x > 0 else { return Li }
        let att = exp(-c * x)
        return att
    }

    @inline(__always)
    public func fresnelDielectric(n1: Double, n2: Double, cosTheta cosI: Double)
    -> (R: Double, cosT: Double?, sin2T: Double) {
        let cosTheta = max(0.0, min(1.0, Swift.abs(cosI)))
        let eta  = n1 / n2
        let sin2T = eta * eta * max(0.0, 1.0 - cosTheta * cosTheta)
        if sin2T > 1.0 {
            return (R: 1.0, cosT: nil, sin2T: sin2T)
        }
        let cosPhi = sqrt(max(0.0, 1.0 - sin2T))
        let Rs = (n2 * cosTheta - n1 * cosPhi) / (n2 * cosTheta + n1 * cosPhi)
        let Rp = (n1 * cosTheta - n2 * cosPhi) / (n1 * cosTheta + n2 * cosPhi)
        let R  = 0.5 * (Rs * Rs + Rp * Rp)
        return (R: R, cosT: cosPhi, sin2T: sin2T)
    }

    @inline(__always)
    public func fresnelConductorRGB(eta: Scalar, k: Scalar, cosI_: Double) -> Vec3 {
        let cosI = max(0.0, min(1.0, Swift.abs(cosI_)))
        let cos2 = cosI * cosI
        let one  = Vec3(1, 1, 1)
        let eta2 = eta * eta
        let k2   = k * k
        let eta2k2 = eta2 + k2
        let twoEtaCos = 2.0 * eta * cosI
        let cos2v = Vec3(cos2, cos2, cos2)
        let Rs = (eta2k2 - twoEtaCos + cos2v) / (eta2k2 + twoEtaCos + cos2v)
        let Rp: Vec3 = ((eta2k2 * cos2v) - twoEtaCos + one) / ((eta2k2 * cos2v) + twoEtaCos + one)
        return 0.5 * (Rs + Rp)
    }

    @inline(__always)
    func makeONB(n: Vec3, roughness: Scalar, forAreaLight: Bool = false) -> Vec3 {
        var x = Swift.abs(n.x)
        var y = Swift.abs(n.y)
        var z = Swift.abs(n.z)

        var reflect: Vec3 = .zero
        if x <= y && x <= z {
            reflect = Vec3(1, n.y, n.z)
        } else if y <= x && y <= z {
            reflect = Vec3(n.x, 1, n.z)
        } else {
            reflect = Vec3(n.x, n.y, 1)
        }

        let u = normalize(cross(n, reflect))
        let v = normalize(cross(n, u))

        let e1 = Scalar.random(in: 0...1) - (forAreaLight ? 0.5 : 0.0)
        let e2 = Scalar.random(in: 0...1) - (forAreaLight ? 0.5 : 0.0)
        return normalize(n + roughness * (e1 * u + e2 * v))
    }

    @inlinable
    func orthonormalBasis(_ n: Vec3) -> (Vec3, Vec3) {
        // n is assumed normalized externally (or normalize it if needed)
        let sign = n.z >= 0 ? 1.0 : -1.0
        let a = -1.0 / (sign + n.z)
        let b = n.x * n.y * a

        // Tangent (x axis)
        let tangent = normalize(Vec3(
            1.0 + sign * n.x * n.x * a,
            sign * b,
            -sign * n.x
        ))

        // Bitangent (y axis)
        let bitangent = normalize(Vec3(
            b,
            sign + n.y * n.y * a,
            -n.y
        ))

        return (tangent, bitangent)
    }
}


public struct PCG32: Sendable {
    private var state: UInt64
    private var inc: UInt64

    @inline(__always)
    public init(seed: UInt64) {
        self.state = 0
        self.inc = (seed << 1) | 1
        _ = next()
        state &+= 0x9E3779B97F4A7C15
        _ = next()
    }

    @inline(__always)
    public mutating func next() -> UInt32 {
        let old = state
        // 1. Update state (overflow operators are correct here)
        state = old &* 6364136223846793005 &+ inc

        // 2. Calculate output function (XSH-RR)
        // FIX: Use 'truncatingIfNeeded' to discard high bits safely
        let xorshifted = UInt32(truncatingIfNeeded: ((old >> 18) ^ old) >> 27)
        let rot = UInt32(truncatingIfNeeded: old >> 59)

        // 3. Rotate
        return (xorshifted >> rot) | (xorshifted << ((~rot &+ 1) & 31))
    }

    @inline(__always)
    public mutating func nextFloat() -> Double {
        // Multiply by 2^-32 to get [0, 1) range
        return Double(next()) * 2.3283064365386963e-10
    }
}

// MARK: - Post-Processing
extension Renderer {
    /// Applies a 3x3 Box Filter (Average) to the image buffer.
    /// This helps reduce noise (fireflies) but effectively blurs the image slightly.
    private func applyBoxFilter(_ buffer: [Vec3], width: Int, height: Int) -> [Vec3] {
        print("applyig box filter")
        var output = [Vec3](repeating: .zero, count: buffer.count)

        // Use UnsafeBuffers to avoid Swift array bounds-checking overhead in hot loops
        buffer.withUnsafeBufferPointer { src in
            output.withUnsafeMutableBufferPointer { dst in
                for y in 0..<height {
                    for x in 0..<width {
                        var sum = Vec3.zero
                        var count: Double = 0

                        // 3x3 Kernel Loop
                        for ky in -1...1 {
                            for kx in -1...1 {
                                let nx = x + kx
                                let ny = y + ky

                                // Boundary check (Clamp to edges)
                                if nx >= 0 && nx < width && ny >= 0 && ny < height {
                                    let index = ny * width + nx
                                    sum += src[index]
                                    count += 1
                                }
                            }
                        }

                        // Average the kernel
                        dst[y * width + x] = sum / count
                    }
                }
            }
        }
        return output
    }

    @inline(__always)
    func reconstructionWeight(_ dx: Double, _ dy: Double, sigma: Double? = nil) -> Double {
        if let sigma {
            let x = dx - 0.5
            let y = dy - 0.5
            return exp(-(x*x + y*y) / (2*sigma*sigma))
        } else {
            let x = 1.0 - Swift.abs(dx - 0.5) * 2.0
            let y = 1.0 - Swift.abs(dy - 0.5) * 2.0
            return max(0, x) * max(0, y)
        }
    }

    // Precompute 10x10 stratified offsets just like C++
    @inline(__always)
    func buildStratifiedJitter(_ rng: inout PCG32) -> ([Double], [Double]) {
        var xs = [Double](repeating: 0, count: 100)
        var ys = [Double](repeating: 0, count: 100)

        var k = 0
        for gy in 0..<10 {
            for gx in 0..<10 {
                xs[k] = Double(gx) + Double(rng.nextFloat())
                ys[k] = Double(gy) + Double(rng.nextFloat())
                k += 1
            }
        }
        return (xs, ys)
    }

}
