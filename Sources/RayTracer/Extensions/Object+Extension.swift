//
//  Object+Extension.swift
//  RayTracer
//
//  Created by Eren Demircan on 7.10.2025.
//

import ParsingKit
import UIKit
import simd

private actor ProgressRelay {
    private var done: Int = 0
    private let total: Int
    private let cb: (@Sendable (Int) -> Void)?

    init(total: Int, cb: (@Sendable (Int) -> Void)?) {
        self.total = total
        self.cb = cb
    }

    /// Call after finishing a scanline. Throttled: every 8 rows or on completion.
    func rowFinished() async {
        done &+= 1
        guard let cb else { return }
        if (done & 7) == 0 || done == total {          // every 8 rows or final row
            DispatchQueue.main.async { [weak self] in
                guard let self else { return }
                cb(done)
            }
        }
    }
}


// MARK: - Renderer
final class Renderer: @unchecked Sendable {
    let ctx: RTContext

    // Cached pointers for hot triangle loops
    private let triIndexPtr: UnsafePointer<Int32>
    private let v0Ptr: UnsafePointer<Vec3>
    private let v1Ptr: UnsafePointer<Vec3>
    private let v2Ptr: UnsafePointer<Vec3>
    private let e1Ptr: UnsafePointer<Vec3>
    private let e2Ptr: UnsafePointer<Vec3>

    init(ctx: RTContext) {
        self.ctx = ctx

        // These pointers remain valid as long as the backing arrays are never mutated.
        triIndexPtr = ctx.triIndex.withUnsafeBufferPointer { $0.baseAddress! }
        v0Ptr       = ctx.v0.withUnsafeBufferPointer        { $0.baseAddress! }
        v1Ptr       = ctx.v1.withUnsafeBufferPointer        { $0.baseAddress! }
        v2Ptr       = ctx.v2.withUnsafeBufferPointer        { $0.baseAddress! }
        e1Ptr       = ctx.e1.withUnsafeBufferPointer        { $0.baseAddress! }
        e2Ptr       = ctx.e2.withUnsafeBufferPointer        { $0.baseAddress! }
    }
}

// MARK: - Intersections

extension Renderer {

    @inline(__always)
    func intersectPlanes(ray: inout Ray) -> Bool {
        var hit = false
        let eps = ctx.intersectionTestEpsilon

        let nPlanes = ctx.planeCenter.count
        let C = ctx.planeCenter
        let N = ctx.planeNormal
        let O = ctx.planeObj
        let M = ctx.planeMat

        for i in 0..<nPlanes {
            let n = N[i]
            let denom = dot(ray.dir, n)
            if abs(denom) < eps { continue }            // parallel

            let t = dot(C[i] - ray.origin, n) / denom
            if t < eps || t >= ray.tMax { continue }

            hit = true
            ray.kind   = .plane
            ray.prim   = Int32(i)
            ray.obj    = O[i]
            ray.mat    = M[i]
            ray.tMax   = t
            // Face the normal against the ray
            ray.normal = (denom < 0) ? n : -n
        }

        return hit
    }

    @inline(__always)
    func intersectTriangles(ray: inout Ray) -> Bool {
        var hit = false
        let eps = ctx.intersectionTestEpsilon

        let nTris = ctx.triIndex.count / 3
        // Using cached pointers for hot loop
        let v0p = v0Ptr, e1p = e1Ptr, e2p = e2Ptr
        let triObj = ctx.triObj
        let triMat = ctx.triMat

        for t in 0..<nTris {
            // Möller–Trumbore with precomputed edges
            let e1 = e1p[t]
            let e2 = e2p[t]
            let p  = cross(ray.dir, e2)
            let det = dot(e1, p)
            if det < eps { continue }                   // backface culled; change to |det| if you want two-sided

            let invDet = 1.0 / det
            let tvec = ray.origin - v0p[t]
            let u = dot(tvec, p) * invDet
            if u < 0.0 || u > 1.0 { continue }

            let q = cross(tvec, e1)
            let v = dot(ray.dir, q) * invDet
            if v < 0.0 || u + v > 1.0 { continue }

            let tHit = dot(e2, q) * invDet
            if tHit <= eps || tHit >= ray.tMax { continue }

            ray.tMax  = tHit
            ray.kind  = .triangle
            ray.prim  = Int32(t)
            ray.obj   = triObj[t]
            ray.mat   = triMat[t]
            ray.bary  = SIMD3<Scalar>(1 - u - v, u, v)

            #if SMOOTH_NORMAL
            // If you have per-vertex normals in context:
            let i0 = Int(triIndexPtr[3*t+0])
            let i1 = Int(triIndexPtr[3*t+1])
            let i2 = Int(triIndexPtr[3*t+2])
            let n0 = ctx.vertexNormals[i0]
            let n1 = ctx.vertexNormals[i1]
            let n2 = ctx.vertexNormals[i2]
            let w  = 1 - u - v
            ray.normal = normalize(n0 * w + n1 * u + n2 * v)
            #else
            ray.normal = normalize(cross(e1, e2))       // geometric normal
            #endif

            hit = true
        }

        return hit
    }

    @inline(__always)
    func intersectSpheres(ray: inout Ray) -> Bool {
        var hit = false
        let eps = ctx.intersectionTestEpsilon

        let C = ctx.sphCenter
        let R = ctx.sphRadius
        let O = ctx.sphObj
        let M = ctx.sphMat

        for i in 0..<C.count {
            // Quadratic with "halved b" for stability:
            // a = d·d, B = d·oc, c = oc·oc - r^2
            let oc = ray.origin - C[i]
            let a  = length_squared(ray.dir)
            let B  = dot(ray.dir, oc)
            let c  = length_squared(oc) - R[i] * R[i]
            let disc = B*B - a*c
            if disc < 0 { continue }

            let tHit = (-B - sqrt(disc)) / a
            if tHit <= eps || tHit >= ray.tMax { continue }

            ray.tMax   = tHit
            ray.kind   = .sphere
            ray.prim   = Int32(i)
            ray.obj    = O[i]
            ray.mat    = M[i]
            ray.normal = (ray.origin + ray.dir * tHit - C[i]) / R[i]
            hit = true
        }
        return hit
    }
}

// MARK: - Shadows

extension Renderer {

    @inline(__always)
    public func occluded(shadowRay rIn: Ray,
                         excludeObj: Int32 = -1,
                         backfaceCulling: Bool = false) -> Bool
    {
        var r = rIn
        return trianglesShadow(&r, excludeObj: excludeObj, backfaceCulling: backfaceCulling)
            || spheresShadow(&r, excludeObj: excludeObj)
            || planeShadow(ray: &r, excludeObj: excludeObj)
    }

    @inline(__always)
    private func trianglesShadow(_ ray: inout Ray,
                                 excludeObj: Int32,
                                 backfaceCulling: Bool) -> Bool
    {
        let eps = ctx.intersectionTestEpsilon
        let nTris = ctx.triIndex.count / 3

        let v0p = v0Ptr, e1p = e1Ptr, e2p = e2Ptr
        let triObj = ctx.triObj

        for t in 0..<nTris {
            if excludeObj != -1 && triObj[t] == excludeObj { continue }

            let e1 = e1p[t], e2 = e2p[t]
            let p  = cross(ray.dir, e2)
            let det = dot(e1, p)

            if backfaceCulling {
                if det <= eps { continue }
            } else if det > -eps && det < eps {
                continue
            }

            let invDet = 1.0 / det
            let tVec = ray.origin - v0p[t]
            let u = dot(tVec, p) * invDet
            if u < 0.0 || u > 1.0 { continue }

            let q = cross(tVec, e1)
            let v = dot(ray.dir, q) * invDet
            if v < 0.0 || u + v > 1.0 { continue }

            let tHit = dot(e2, q) * invDet
            if tHit > eps && tHit < ray.tMax { return true } // any hit blocks
        }
        return false
    }

    @inline(__always)
    private func spheresShadow(_ ray: inout Ray,
                               excludeObj: Int32) -> Bool
    {
        let eps = ctx.intersectionTestEpsilon

        let C = ctx.sphCenter
        let R = ctx.sphRadius
        let O = ctx.sphObj

        for i in 0..<C.count {
            if excludeObj != -1 && O[i] == excludeObj { continue }

            let oc = ray.origin - C[i]
            let a  = length_squared(ray.dir)
            let B  = dot(ray.dir, oc)
            let c  = length_squared(oc) - R[i] * R[i]
            let disc = B*B - a*c
            if disc < 0 { continue }

            let tHit = (-B - sqrt(disc)) / a
            if tHit > eps && tHit < ray.tMax { return true } // blocked
        }
        return false
    }

    @inline(__always)
    func planeShadow(ray: inout Ray, excludeObj: Int32) -> Bool {
        let eps = ctx.intersectionTestEpsilon

        let C = ctx.planeCenter
        let N = ctx.planeNormal
        let O = ctx.planeObj

        for i in 0..<C.count {
            if excludeObj != -1 && O[i] == excludeObj { continue }

            let n = N[i]
            let denom = dot(ray.dir, n)
            if denom > 0 { continue }               // backface cull to speed up
            if abs(denom) < eps { continue }        // parallel

            let t = dot(C[i] - ray.origin, n) / denom
            if t >= eps && t < ray.tMax { return true }
        }
        return false
    }
}

// MARK: - Rendering
extension Renderer {
    private struct ChunkResult: Sendable {
        let startRow: Int
        let endRow: Int
        let pixels: [Vec3]
        let rays: Int64
    }

    @inline(__always)
    func render(
        scene: Scene,
        cameraIndex: Int = 0,
        progressRow: (@Sendable (Int) -> Void)? = nil,
        raysTraced: inout Int64
    ) async throws -> [Vec3] {

        let cam = scene.cameras[cameraIndex]
        let width  = cam.imageResolution.0
        let height = cam.imageResolution.1

        // Camera basis (immutable, captured by tasks safely)
        let e = cam.position
        let nd = Double(cam.nearDistance)
        let l = Double(cam.nearPlane[0])
        let r = Double(cam.nearPlane[1])
        let b = Double(cam.nearPlane[2])
        let t = Double(cam.nearPlane[3])
        let gazeNorm = normalize(cam.gaze)
        let wv = -gazeNorm
        let upNorm = normalize(cam.up)
        let u = normalize(cross(upNorm, wv))
        let v = normalize(cross(wv, u))

        // Raster deltas + near plane geometry
        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)
        let m: Vec3 = e - wv * nd

        // near plane center
        let q0Base: Vec3 = m + u * l + v * t // near plane top-left

        // Output image
        var out = [Vec3](repeating: .zero, count: width * height)

        let relay: ProgressRelay? = (progressRow == nil) ? nil : ProgressRelay(total: height, cb: progressRow)

        // --- chunking (unchanged) ---
        // Chunking (tune CH for your CPU/cache)
        let CH = 16
        var chunks: [(Int, Int)] = []
        chunks.reserveCapacity((height + CH - 1) / CH)
        var row = 0
        while row < height {
            let end = min(row + CH, height)
            chunks.append((row, end))
            row = end
        }

        var raysSum: Int64 = 0

        // 👇 Add this helper just BEFORE the task group to capture the scene/camera constants
        @inline(__always)
        func reflect(_ d: Vec3, _ n: Vec3) -> Vec3 {
            d - 2.0 * dot(d, n) * n
        }

        // NOTE: if your Scene stores recursion as something else, adjust this:
        let maxDepth: Int = scene.maxRecursionDepth   // or Int(scene.maxRecursionDepth)

        // Recursive shading: direct light + perfect mirror
        @inline(__always)
        func trace(_ inRay: inout Ray, _ depth: Int) -> Vec3 {
            // Intersect current ray
            _ = self.intersectTriangles(ray: &inRay)
            _ = self.intersectSpheres(ray: &inRay)
            _ = self.intersectPlanes(ray: &inRay)

            if inRay.kind == .none {
                // background
                let bg = scene.backgroundColor
                return Vec3(x: bg.x, y: bg.y, z: bg.z)
            }

            // Material at hit
            let mat = scene.materials[Int(inRay.mat)]

            // Face-forward normal and hit point
            let p = inRay.origin + inRay.dir * inRay.tMax
            var N = inRay.normal
            if dot(N, -inRay.dir) < 0 { N = -N }

            // Direct lighting (ambient + un-occluded points)
            var Lo = scene.lights.ambient * mat.ambient

            for light in scene.lights.points {
                var wi = light.position - p
                let dist = length(wi)
                wi = normalize(wi)

                // Shadow ray
                var sRay = makeRay(origin: p + N * self.ctx.shadowRayEpsilon, dir: wi)
                sRay.tMax = dist - self.ctx.shadowRayEpsilon
                let blocked = self.occluded(shadowRay: sRay, excludeObj: inRay.obj, backfaceCulling: false)
                if !blocked {
                    Lo += light.shade(material: mat, ray: inRay, cameraPos: e, normal: N, at: p)
                }
            }

//            let mirror = mat.mirror
//            let hasMirror = mirror.x > 0 || mirror.y > 0 || mirror.z > 0
//
//            let refractionIdx = mat.ior
//            let hasRefraction = refractionIdx.isZero == false
//            let absorption = mat.absorption
//            let hasAbsorption = absorption.x > 0 || absorption.y > 0 || absorption.z > 0
//
//
//            if hasMirror && depth < maxDepth {
//                let rd = normalize(reflect(inRay.dir, N))
//                var rRay = makeRay(origin: p + N * self.ctx.shadowRayEpsilon, dir: rd)
//                // Recurse
//                let Li = trace(&rRay, depth + 1)
//                Lo += mirror * Li
//            }
//
//            return Lo

            let mirror = mat.mirror
            let hasMirror = mirror.x > 0 || mirror.y > 0 || mirror.z > 0

            let ior = Double(mat.ior)
            let hasRefraction = (ior > 0)   // your sentinel: 0 = no refraction
            let absorption = mat.absorption
            let hasAbsorption = absorption.x > 0 || absorption.y > 0 || absorption.z > 0

            // Use the *geometric* normal to determine entering/exiting,
            // not the face-forwarded one, because we flipped N above.
            let Ngeo = inRay.normal
            let entering = dot(inRay.dir, Ngeo) < 0.0

            if hasRefraction && depth < maxDepth {
                // Dielectric (glass-like): Fresnel mix of reflection + refraction
                // N is face-forwarded already (points against incoming dir)
                let n = N

                let n1: Double = entering ? 1.0 : ior
                let n2: Double = entering ? ior : 1.0
                let eta = n1 / n2

                // Reflection ray (always valid)
                let rd = normalize(reflect(inRay.dir, n))
                var rRay = makeRay(origin: p + n * self.ctx.shadowRayEpsilon, dir: rd)

                // Refraction test (Snell)
                let cosi = max(0.0, min(1.0, -dot(inRay.dir, n)))  // cosθ_i with face-forward normal
                let sin2t = eta * eta * max(0.0, 1.0 - cosi * cosi)

                if sin2t > 1.0 {
                    // Total Internal Reflection → 100% reflection
                    let LiR = trace(&rRay, depth + 1)
                    Lo += LiR
                } else {
                    let cost = sqrt(1.0 - sin2t)
                    // Refracted direction
                    let td = normalize(eta * inRay.dir + (eta * cosi - cost) * n)
                    // Offset *into* the transmitted side:
                    // - entering: go slightly into the object (−n)
                    // - exiting : go slightly into the air (+n)
                    let tOrigin = entering ? (p - n * self.ctx.shadowRayEpsilon)
                                           : (p + n * self.ctx.shadowRayEpsilon)
                    var tRay = makeRay(origin: tOrigin, dir: td)

                    // Fresnel reflectance (Schlick)
                    let R = schlickFresnel(cosTheta: entering ? cosi : max(0.0, -dot(td, n)), n1: n1, n2: n2)

                    var LiR = trace(&rRay, depth + 1)
                    var LiT = trace(&tRay, depth + 1)

                    // Beer–Lambert absorption only while traveling *inside* the medium
                    if hasAbsorption && entering {
                        let d = tRay.tMax  // distance inside dielectric until the next hit
                        let att = Vec3(x: exp(-absorption.x * d),
                                       y: exp(-absorption.y * d),
                                       z: exp(-absorption.z * d))
                        LiT = LiT * att
                    }

                    // Mix reflection & refraction by Fresnel
                    Lo += LiR * R + LiT * (1.0 - R)
                }

            } else if hasMirror && depth < maxDepth {
                // Perfect mirror (metals/specular) only if no refraction
                let rd = normalize(reflect(inRay.dir, N))
                var rRay = makeRay(origin: p + N * self.ctx.shadowRayEpsilon, dir: rd)
                let Li = trace(&rRay, depth + 1)
                Lo += mirror * Li
            }

            return Lo
        }

        try await withThrowingTaskGroup(of: ChunkResult.self) { group in
            for (startRow, endRow) in chunks {
                group.addTask { [self] in
                    var local = [Vec3](repeating: .zero, count: (endRow - startRow) * width)
                    var localRays: Int64 = 0

                    for j in startRow..<endRow {
                        if Task.isCancelled { throw CancellationError() }

                        let rowTopLeft = q0Base - v * (Double(j) + 0.5) * dv
                        let base = (j - startRow) * width

                        for i in 0..<width {
                            if Task.isCancelled { throw CancellationError() }

                            let s = rowTopLeft + (Double(i) + 0.5) * du * u
                            var ray = makeRay(origin: e, dir: s - e)

                            // 🔁 Use recursive tracer (does intersections internally)
                            let pixel = trace(&ray, 0)
                            local[base + i] = pixel

                            // localRays &+= 1  // (optional)
                        }
                        if let relay { await relay.rowFinished() }
                    }

                    return ChunkResult(startRow: startRow, endRow: endRow, pixels: local, rays: localRays)
                }
            }

            // --- collector loop (unchanged except your existing progressRow call) ---
            for try await chunk in group {
                let start = chunk.startRow
                let end   = chunk.endRow
                let rows  = end - start

                chunk.pixels.withUnsafeBufferPointer { src in
                    out.withUnsafeMutableBufferPointer { dst in
                        let dstStart = start * width
                        dst.baseAddress!.advanced(by: dstStart)
                            .assign(from: src.baseAddress!, count: rows * width)
                    }
                }

                if let progressRow {
                    for r in start..<end { progressRow(r) }
                }
                raysSum += chunk.rays
            }
        }

        raysTraced += raysSum
        if Task.isCancelled { throw CancellationError() }
        return out
    }

    @inline(__always)
    func schlickFresnel(cosTheta: Double, n1: Double, n2: Double) -> Double {
        // R(θ) ≈ R0 + (1 − R0) (1 − cosθ)^5
        let r0 = (n1 - n2) / (n1 + n2)
        let r0sq = r0 * r0
        return r0sq + (1.0 - r0sq) * pow(1.0 - cosTheta, 5.0)
    }

}
