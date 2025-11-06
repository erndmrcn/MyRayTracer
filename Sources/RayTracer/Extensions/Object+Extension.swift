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
    let ctx: RTContext

    init(ctx: RTContext) {
        self.ctx = ctx
    }
}

// MARK: - Intersections
extension Renderer {

    @inline(__always)
    func intersectScene(_ ray: inout Ray, backfaceCulling: Bool) -> Bool {
        guard let bvh = ctx.bvh else { return false }
        let eps = ctx.intersectionTestEpsilon
        bvh.intersectBVH(ray: &ray, nodeIdx: 0, ctx: ctx, eps: eps)
        return ray.hit.kind != .none
    }

    // --- Per-mesh BLAS traversal ---
//    @inline(__always)
//    func intersectBLAS(ray: inout Ray, mesh: MeshBase) {
//        guard let root = mesh.blasRoot else { return }
//
//        var stack: [BVHNode] = [root]
//        let eps = ctx.intersectionTestEpsilon
//
//        while let node = stack.popLast() {
//            if !node.bounds.hit(ray.origin, ray.dir, 1.0 / ray.dir, tMax: ray.tMax, eps: eps) { continue }
//
//            if node.isLeaf {
//                let end = node.start + node.count
//                for i in node.start..<end {
//                    let prim = mesh.blasPrims[i]
//                    let i0 = mesh.indices[3*prim.index + 0]
//                    let i1 = mesh.indices[3*prim.index + 1]
//                    let i2 = mesh.indices[3*prim.index + 2]
//
//                    let v0 = mesh.positions[i0]
//                    let v1 = mesh.positions[i1]
//                    let v2 = mesh.positions[i2]
//
//                    intersectTriangle(ray: &ray,
//                                      v0: v0,
//                                      v1: v1,
//                                      v2: v2,
//                                      eps: eps,
//                                      backfaceCulling: false,
//                                      material: mesh.materialIndex)
//                }
//            } else {
//                if let l = node.left { stack.append(l) }
//                if let r = node.right { stack.append(r) }
//            }
//        }
//    }

    // --- Triangle intersection ---
    @inline(__always)
    private func intersectTriangle(
        ray: inout Ray,
        v0: Vec3, v1: Vec3, v2: Vec3,
        eps: Scalar,
        backfaceCulling: Bool,
        material: Int
    ) {
        let e1 = v1 - v0
        let e2 = v2 - v0
        let pvec = cross(ray.dir, e2)
        let det = dot(e1, pvec)
        if Swift.abs(det) < eps { return }
        if backfaceCulling && det < 0 { return }

        let invDet = 1.0 / det
        let tvec = ray.origin - v0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return }

        let qvec = cross(tvec, e1)
        let v = dot(ray.dir, qvec) * invDet
        if v < 0.0 || u + v > 1.0 { return }

        let t = dot(e2, qvec) * invDet
        if t <= eps || t >= ray.hit.t { return }

        let n = normalize(cross(e1, e2))
        let p = ray.origin + ray.dir * t

        if t < ray.hit.t {
            ray.hit.t = t
            ray.hit.kind = .triangle
            ray.hit.p = p
            ray.hit.n = n
            ray.hit.mat = material
        }
    }

//    func traverseTLAS(ray: inout Ray, node: BVHNode?, ctx: RTContext) {
//        guard let node else { return }
//        let invDir = 1.0 / ray.dir
//        if !node.bounds.hit(ray.origin, ray.dir, ray.dir, tMax: ray.tMax, eps: ctx.shadowRayEpsilon) {
//            return
//        }
//
//        if node.isLeaf {
//            for j in node.start ..< node.start + node.count {
//                let instIdx = ctx.tlasPrims[j].index
//                let inst = ctx.instances[instIdx]
//
//                var rLocal = ray.transformed(by: inst.invTransform, inverse: true)
//                intersectBLAS(ray: &rLocal, mesh: ctx.meshes[inst.baseIndex])
//
//                if rLocal.hit.kind != .none {
//                    let pW = (inst.transform * SIMD4(rLocal.hit.p, 1)).xyz
//                    let nW = normalize((inst.invTransform.transpose * SIMD4(rLocal.hit.n, 0)).xyz)
//
//                    if rLocal.hit.t < ray.hit.t {
//                        ray.hit.t = rLocal.hit.t
//                        ray.hit.p = pW
//                        ray.hit.n = nW
//                        ray.hit.kind = .triangle
//                        ray.hit.mat = inst.materialIndex
//                        ray.tMax = rLocal.hit.t
//                    }
//                }
//            }
//        } else {
//            traverseTLAS(ray: &ray, node: node.left,  ctx: ctx)
//            traverseTLAS(ray: &ray, node: node.right, ctx: ctx)
//        }
//    }
}

// MARK: - Rendering (Whitted-style Ray Tracer)
private actor ProgressRelay {
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
    private struct ChunkResult: Sendable {
        let startRow: Int
        let endRow: Int
        let pixels: [Vec3]
        let rays: Int64
        let nodeVisits: Int64
    }

    @Sendable func render(
        scene: Scene,
        cameraIndex: Int = 0,
        progressRow: (@Sendable (Int) -> Void)? = nil,
        raysTraced: inout Int64
    ) async throws -> [Vec3] {

        let cam = scene.cameras[cameraIndex]
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

        var raysSum: Int64 = 0
        var totalNodeVisits: Int64 = 0
        let bvh = ctx.bvh
        let materials = ctx.materials
        let eps = ctx.intersectionTestEpsilon
        let maxDepth = scene.maxRecursionDepth
        let shadowEps = ctx.shadowRayEpsilon

        @inline(__always)
        @Sendable func trace(_ inRay: inout Ray, _ depth: Int, _ localNodeVisits: inout Int64) -> Vec3 {
            if depth > maxDepth { return scene.backgroundColor }
            guard let bvh = ctx.bvh else { return .zero }
            let visits = bvh.intersectBVH(ray: &inRay, nodeIdx: 0, ctx: ctx, eps: eps)
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
                for light in scene.lights.points {
                    var wi = light.position - p
                    let dist = length(wi)
                    wi = normalize(wi)

                    var sRay = Ray(origin: p + N * shadowEps, dir: wi)
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

                            let atten = light.intensity / max(dist*dist, 1e-12)
                            Lo += (Ld + Ls) * atten
                        }
                    }
                }
            }

            if mat.type == "mirror" && depth < maxDepth {
                let rd = reflect(normalize(inRay.dir), N)
                var rRay = inRay
                rRay.origin = p + N * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()
                let Li = trace(&rRay, depth + 1, &localNodeVisits)

                Lo += mat.mirror * Li
            } else if mat.type == "dielectric" && depth < maxDepth {
                let entering = frontFacing
                let n1 = entering ? 1.0 : mat.ior
                let n2 = entering ? mat.ior : 1.0

                let cosI = -dot(inRay.dir, N)
                let (R, cosTopt, sin2T) = fresnelDielectric(n1: n1, n2: n2, cosTheta: cosI)

                let rd = reflect(inRay.dir, N)
                var rRay = inRay
                rRay.origin = p + N * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()
                let LiR = trace(&rRay, depth + 1, &localNodeVisits)


                if cosTopt == nil || sin2T > 1 {
                    Lo += LiR
                } else {
                    let cosT = cosTopt!
                    let td = normalize((inRay.dir + (N * cosI)) * (n1/n2) - N * cosT)
                    var tRay = Ray(origin: p + td * shadowEps, dir: td)
                    var LiT = trace(&tRay, depth + 1, &localNodeVisits)

                    if mat.absorption != .zero {
                        let d = tRay.hit.t
                        LiT = beerAttenuate(LiT, c: mat.absorption, distance: d)
                    }
                    Lo += LiR * R + LiT * (1.0 - R)
                }
            } else if mat.type == "conductor" && depth < maxDepth {
                let cosI = max(0.0, -dot(inRay.dir, N))
                let Rf = fresnelConductorRGB(eta: mat.ior, k: mat.absorptionIndex, cosI_: cosI)
                let rd = normalize(reflect(inRay.dir, N))
                var rRay = inRay
                rRay.origin = p + N * shadowEps
                rRay.dir = rd
                rRay.invDir = 1.0 / rd
                rRay.hit = .init()
                let Li = trace(&rRay, depth + 1, &localNodeVisits)
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
                    var local = [Vec3](repeating: .zero, count: (endRow - startRow) * width)
                    var localRays: Int64 = 0
                    var localNodeVisits: Int64 = 0

                    for j in startRow..<endRow {
                        let jOff = (Double(j) + 0.5)
                        let vOff = v * (jOff * dv)
                        let rowTopLeft = q00 - vOff
                        let base = (j - startRow) * width

                        for i in 0..<width {
                            let iOff = (Double(i) + 0.5)
                            let uOff = u * (iOff * du)
                            let s = rowTopLeft + uOff
                            var dir = s - eye
                            dir = simd_normalize(dir)
                            var ray: Ray
                            if isLookAt {
                                ray = Ray(origin: eye + dir * shadowEps, dir: dir)
                            } else {
                                ray = Ray(origin: eye, dir: dir)
                            }

                            let pixel = trace(&ray, 0, &localNodeVisits)
                            local[base + i] = pixel

                            localRays &+= 1
                        }
                    }

                    if let relay { await relay.rowFinished() }
                    return ChunkResult(startRow: startRow, endRow: endRow, pixels: local, rays: localRays, nodeVisits: localNodeVisits)
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
                raysSum &+= chunk.rays
                totalNodeVisits &+= chunk.nodeVisits
            }

            try await group.waitForAll()
        }

        raysTraced &+= raysSum
        let avgVisits = Double(totalNodeVisits) / Double(raysSum)
        print("📊 Avg BVH node visits per ray: \(avgVisits)")

        return out
    }

    // --- Camera assembly
    private func makeCameraBasis(from cam: ParsingKit.Camera, aspect: Double)
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

    func occluded(shadowRay rIn: Ray) -> Bool {
        guard let bvh = ctx.bvh else { return false }
        let eps = ctx.intersectionTestEpsilon
        return bvh.intersectBVHShadow(ray: rIn, nodeIdx: 0, ctx: ctx, eps: eps)
    }

//    @inline(__always)
//    private func traverseTLASShadow(ray: inout Ray, node: BVHNode?) {
//        guard let node else { return }
//        let invDir = 1.0 / ray.dir
//        if !node.bounds.hit(ray.origin, ray.dir, ray.dir, tMax: ray.tMax, eps: ctx.shadowRayEpsilon) {
//            return
//        }
//        if node.isLeaf {
//            for j in node.start ..< node.start + node.count {
//                let instIdx = ctx.tlasPrims[j].index
//                let inst = ctx.instances[instIdx]
//                var rLocal = ray.transformed(by: inst.invTransform, inverse: true)
//                if occludedBLAS(ray: &rLocal, mesh: ctx.meshes[inst.baseIndex]) {
//                    ray.hit.t = rLocal.hit.t
//                    ray.hit.kind = .triangle
//                    return
//                }
//            }
//        } else {
//            traverseTLASShadow(ray: &ray, node: node.left)
//            if ray.hit.kind != .none { return }
//            traverseTLASShadow(ray: &ray, node: node.right)
//        }
//    }

//    @inline(__always)
//    private func occludedBLAS(ray: inout Ray, mesh: MeshBase) -> Bool {
//        guard let root = mesh.blasRoot else { return false }
//        var stack: [BVHNode] = [root]
//        let eps = ctx.intersectionTestEpsilon
//
//        while let node = stack.popLast() {
//            if !node.bounds.hit(ray.origin, ray.dir, 1.0 / ray.dir, tMax: ray.tMax, eps: eps) { continue }
//            if node.isLeaf {
//                let end = node.start + node.count
//                for i in node.start..<end {
//                    let prim = mesh.blasPrims[i]
//                    let i0 = mesh.indices[3*prim.index + 0]
//                    let i1 = mesh.indices[3*prim.index + 1]
//                    let i2 = mesh.indices[3*prim.index + 2]
//                    if triShadowHit(ray, v0: mesh.positions[i0], v1: mesh.positions[i1], v2: mesh.positions[i2], eps: eps) {
//                        ray.hit.t = min(ray.hit.t, ray.tMax)
//                        ray.hit.kind = .triangle
//                        return true
//                    }
//                }
//            } else {
//                if let l = node.left { stack.append(l) }
//                if let r = node.right { stack.append(r) }
//            }
//        }
//        return false
//    }

    @inline(__always)
    private func triShadowHit(_ ray: Ray, v0: Vec3, v1: Vec3, v2: Vec3, eps: Double) -> Bool {
        let e1 = v1 - v0
        let e2 = v2 - v0
        let pvec = cross(ray.dir, e2)
        let det  = dot(e1, pvec)
        if Swift.abs(det) < eps { return false }
        let invDet = 1.0 / det
        let tvec = ray.origin - v0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return false }
        let q = cross(tvec, e1)
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { return false }
        let t = dot(e2, q) * invDet
        return (t > eps && t < ray.hit.t)
    }

    // --- Utilities ---
    @inline(__always)
    private func reflect(_ d: Vec3, _ n: Vec3) -> Vec3 { d - 2.0 * dot(d, n) * n }

    @inline(__always)
    private func beerAttenuate(_ Li: Vec3, c: Vec3, distance x: Double) -> Vec3 {
        Vec3(x: Li.x * exp(-c.x * x),
             y: Li.y * exp(-c.y * x),
             z: Li.z * exp(-c.z * x))
    }

    @inline(__always)
    private func fresnelDielectric(n1: Double, n2: Double, cosTheta cosI: Double)
    -> (R: Double, cosT: Double?, sin2T: Double) {
        let cosTheta = max(0.0, min(1.0, Swift.abs(cosI)))
        let eta  = n1 / n2
        let sin2T = eta * eta * max(0.0, 1.0 - cosTheta * cosTheta)
        if sin2T > 1.0 {
            return (R: 1.0, cosT: nil, sin2T: sin2T)
        }
        let cosPhi = sqrt(max(0.0, 1.0 - sin2T))
        let Rs = (n1 * cosTheta - n2 * cosPhi) / (n1 * cosTheta + n2 * cosPhi)
        let Rp = (n1 * cosPhi - n2 * cosTheta) / (n1 * cosPhi + n2 * cosTheta)
        let R  = 0.5 * (Rs * Rs + Rp * Rp)
        return (R: R, cosT: cosPhi, sin2T: sin2T)
    }

    @inline(__always)
    private func fresnelConductorRGB(eta: Scalar, k: Scalar, cosI_: Double) -> Vec3 {
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
}
