//
//  Object+Extension.swift
//  RayTracer
//
//  Created by Eren Demircan on 7.10.2025.
//

import ParsingKit
#if canImport(UIKit)
import UIKit
#else
import Foundation
#endif
import simd

// MARK: - Progress Relay
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

// MARK: - Renderer
final class Renderer: @unchecked Sendable {
    let ctx: RTContext

    private let triIndexPtr: UnsafePointer<Int32>
    private let v0Ptr: UnsafePointer<Vec3>
    private let v1Ptr: UnsafePointer<Vec3>
    private let v2Ptr: UnsafePointer<Vec3>
    private let e1Ptr: UnsafePointer<Vec3>
    private let e2Ptr: UnsafePointer<Vec3>

    init(ctx: RTContext) {
        self.ctx = ctx
        triIndexPtr = ctx.triIndex.withUnsafeBufferPointer { $0.baseAddress! }
        v0Ptr = ctx.v0.withUnsafeBufferPointer { $0.baseAddress! }
        v1Ptr = ctx.v1.withUnsafeBufferPointer { $0.baseAddress! }
        v2Ptr = ctx.v2.withUnsafeBufferPointer { $0.baseAddress! }
        e1Ptr = ctx.e1.withUnsafeBufferPointer { $0.baseAddress! }
        e2Ptr = ctx.e2.withUnsafeBufferPointer { $0.baseAddress! }
    }
}

// MARK: - Intersections
extension Renderer {

    // Unified scene intersection
    @inline(__always)
    func intersectScene(_ ray: inout Ray, backfaceCulling: Bool) -> Bool {
        var any = false
        let eps = ctx.intersectionTestEpsilon

        if let root = ctx.bvhRoot {
            any = any || traverseBVH(ray: &ray,
                                     root: root,
                                     prims: ctx.bvhPrims,
                                     eps: eps, backfaceCulling: backfaceCulling)
        } else {
            for i in 0..<ctx.v0.count {
                any = any || intersectTriangleIndexed(ray: &ray, triIndex: i, eps: eps, backfaceCulling: backfaceCulling)
            }
            for i in 0..<ctx.sphCenter.count {
                any = any || intersectSphereIndexed(ray: &ray, sphIndex: i, eps: eps)
            }
            for i in 0..<ctx.planeCenter.count {
                any = any || intersectPlaneIndexed(ray: &ray, planeIndex: i, eps: eps)
            }
        }

        return any
    }

    // --- Triangle
    @inline(__always)
    private func intersectTriangleIndexed(ray: inout Ray, triIndex: Int, eps: Double, backfaceCulling: Bool) -> Bool {
        let p0 = ctx.v0[triIndex]
        let e1 = ctx.e1[triIndex]
        let e2 = ctx.e2[triIndex]

        let pvec = cross(ray.dir, e2)
        let det = dot(e1, pvec)
        if abs(det) < eps { return false }
        if backfaceCulling && det < 0 { return false } // <<< add this
        let invDet = 1.0 / det
        let tvec = ray.origin - p0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return false }
        let qvec = cross(tvec, e1)
        let v = dot(ray.dir, qvec) * invDet
        if v < 0.0 || u + v > 1.0 { return false }
        let t = dot(e2, qvec) * invDet
        if t <= eps || t >= ray.tMax { return false }

        ray.tMax = t
        ray.kind = .triangle
        ray.mat = ctx.triMat[triIndex]
        ray.obj = ctx.triObj[triIndex]
        ray.prim = Int32(triIndex)
        ray.normal = normalize(cross(e1, e2))
        ray.bary = SIMD3<Scalar>(1 - u - v, u, v)
        return true
    }

    // --- Sphere
    @inline(__always)
    private func intersectSphereIndexed(ray: inout Ray, sphIndex: Int, eps: Double) -> Bool {
        let c = ctx.sphCenter[sphIndex]
        let r = ctx.sphRadius[sphIndex]
        let oc = ray.origin - c
        let a = dot(ray.dir, ray.dir)
        let b = 2.0 * dot(oc, ray.dir)
        let c2 = dot(oc, oc) - r * r
        let disc = b * b - 4 * a * c2
        if disc < 0 { return false }
        let s = sqrt(disc)
        let inv2a = 0.5 / a
        var t = (-b - s) * inv2a
        if t <= eps || t >= ray.tMax { t = (-b + s) * inv2a }
        if t <= eps || t >= ray.tMax { return false }

        ray.tMax = t
        ray.kind = .sphere
        ray.prim = Int32(sphIndex)
        ray.mat = ctx.sphMat[sphIndex]
        ray.obj = ctx.sphObj[sphIndex]
        let hitPoint = ray.origin + ray.dir * t
        ray.normal = normalize(hitPoint - c)
        return true
    }

    // --- Plane
    @inline(__always)
    private func intersectPlaneIndexed(ray: inout Ray, planeIndex: Int, eps: Double) -> Bool {
        let n = normalize(ctx.planeNormal[planeIndex])
        let c = ctx.planeCenter[planeIndex]
        let denom = dot(ray.dir, n)
        if abs(denom) < eps { return false }
        let t = dot(c - ray.origin, n) / denom
        if t <= eps || t >= ray.tMax { return false }

        ray.tMax = t
        ray.kind = .plane
        ray.prim = Int32(planeIndex)
        ray.mat = ctx.planeMat[planeIndex]
        ray.obj = ctx.planeObj[planeIndex]
        ray.normal = (denom < 0) ? n : -n
        return true
    }

    private func traverseBVH(ray: inout Ray,
                             root: BVHNode,
                             prims: [BVHPrim],
                             eps: Scalar,
                             backfaceCulling: Bool
    ) -> Bool {
        var hit = false

        struct StackEntry {
            let node: BVHNode
            let nearT: Double
        }

        var stack: [StackEntry] = []
        stack.reserveCapacity(64)

        let ro = ray.origin
        let rd = ray.dir
        let invDir = Vec3(
            abs(rd.x) > 1e-8 ? 1.0 / rd.x : 1e8,
            abs(rd.y) > 1e-8 ? 1.0 / rd.y : 1e8,
            abs(rd.z) > 1e-8 ? 1.0 / rd.z : 1e8
        )

        let (rootHit, rootNearT, _) = boxIntersect(root.bounds, ro, invDir, ray.tMax, eps: eps)
        if rootHit {
            stack.append(StackEntry(node: root, nearT: rootNearT))
        }

        while let entry = stack.popLast() {
            let node = entry.node

            let (nodeHit, _, _) = boxIntersect(node.bounds, ro, invDir, ray.tMax, eps: eps)
            if !nodeHit { continue }

            if node.isLeaf {
                let end = node.start + node.count
                for i in node.start..<end {
                    let p = prims[i]
                    if p.type == 0 {
                        if intersectTriangleIndexed(ray: &ray, triIndex: p.index, eps: eps, backfaceCulling: backfaceCulling) {
                            hit = true
                        }
                    } else if p.type == 1 {
                        if intersectSphereIndexed(ray: &ray, sphIndex: p.index, eps: eps) {
                            hit = true
                        }
                    } else {
                        if intersectPlaneIndexed(ray: &ray, planeIndex: p.index, eps: eps) {
                            hit = true
                        }
                    }
                }
            } else {
                if let l = node.left, let r = node.right {
                    let (lHit, lNear, _) = boxIntersect(l.bounds, ro, invDir, ray.tMax, eps: eps)
                    let (rHit, rNear, _) = boxIntersect(r.bounds, ro, invDir, ray.tMax, eps: eps)

                    if lHit && rHit {
                        if lNear < rNear {
                            stack.append(StackEntry(node: r, nearT: rNear))
                            stack.append(StackEntry(node: l, nearT: lNear))
                        } else {
                            stack.append(StackEntry(node: l, nearT: lNear))
                            stack.append(StackEntry(node: r, nearT: rNear))
                        }
                    } else if lHit {
                        stack.append(StackEntry(node: l, nearT: lNear))
                    } else if rHit {
                        stack.append(StackEntry(node: r, nearT: rNear))
                    }
                } else {
                    if let l = node.left {
                        let (lHit, lNear, _) = boxIntersect(l.bounds, ro, invDir, ray.tMax, eps: eps)
                        if lHit { stack.append(StackEntry(node: l, nearT: lNear)) }
                    }
                    if let r = node.right {
                        let (rHit, rNear, _) = boxIntersect(r.bounds, ro, invDir, ray.tMax, eps: eps)
                        if rHit { stack.append(StackEntry(node: r, nearT: rNear)) }
                    }
                }
            }
        }

        return hit
    }

    @inline(__always)
    private func boxIntersect(_ b: AABB, _ ro: Vec3, _ invDir: Vec3, _ tMax: Scalar, eps: Scalar) -> (Bool, Scalar, Double) {
        let t1 = (b.minP - ro) * invDir
        let t2 = (b.maxP - ro) * invDir
        let tmin = simd.min(t1, t2)
        let tmax = simd.max(t1, t2)

        let tNear = max(max(tmin.x, tmin.y), tmin.z)
        let tFar = min(min(tmax.x, tmax.y), tmax.z)

        if tNear > tFar || tFar <= 0 || tNear >= tMax {
            return (false, 0, 0)
        }

        return (true, max(eps, tNear), tFar)
    }
}

// MARK: - Shadows
extension Renderer {
    @inline(__always)
    private func occludedBVH(_ rayIn: Ray, excludeObj: Int32, backfaceCulling: Bool) -> Bool {
        guard let root = ctx.bvhRoot else { return false }

        struct StackEntry {
            let node: BVHNode
        }

        var stack: [StackEntry] = []
        stack.reserveCapacity(64)

        let ro = rayIn.origin
        let rd = rayIn.dir
        let invDir = Vec3(
            abs(rd.x) > 1e-8 ? 1.0 / rd.x : 1e8,
            abs(rd.y) > 1e-8 ? 1.0 / rd.y : 1e8,
            abs(rd.z) > 1e-8 ? 1.0 / rd.z : 1e8
        )
        let eps = ctx.shadowRayEpsilon

        let (rootHit, _, _) = boxIntersect(root.bounds, ro, invDir, rayIn.tMax, eps: eps)
        if rootHit {
            stack.append(StackEntry(node: root))
        }

        while let entry = stack.popLast() {
            let node = entry.node

            let (nodeHit, _, _) = boxIntersect(node.bounds, ro, invDir, rayIn.tMax, eps: eps)
            if !nodeHit { continue }

            if node.isLeaf {
                let end = node.start + node.count
                for i in node.start..<end {
                    let p = ctx.bvhPrims[i]
                    if p.type == 0 {
                        if ctx.triObj[p.index] == excludeObj { continue }
                        if triShadowHit(rayIn, triIndex: p.index, eps: eps, backfaceCulling: backfaceCulling) { return true }
                    } else if p.type == 1 {
                        if ctx.sphObj[p.index] == excludeObj { continue }
                        if sphShadowHit(rayIn, sphIndex: p.index, eps: eps) { return true }
                    } else {
                        if ctx.planeObj[p.index] == excludeObj { continue }
                        if planeShadowHit(rayIn, planeIndex: p.index, eps: eps) { return true }
                    }
                }
            } else {
                if let l = node.left {
                    let (lHit, _, _) = boxIntersect(l.bounds, ro, invDir, rayIn.tMax, eps: eps)
                    if lHit { stack.append(StackEntry(node: l)) }
                }
                if let r = node.right {
                    let (rHit, _, _) = boxIntersect(r.bounds, ro, invDir, rayIn.tMax, eps: eps)
                    if rHit { stack.append(StackEntry(node: r)) }
                }
            }
        }

        return false
    }

    public func occluded(shadowRay rIn: Ray,
                         excludeObj: Int32 = -1,
                         backfaceCulling: Bool = false) -> Bool {
        if let _ = ctx.bvhRoot {
            return occludedBVH(rIn, excludeObj: excludeObj, backfaceCulling: backfaceCulling)
        } else {
            let eps = ctx.intersectionTestEpsilon
            for i in 0..<ctx.v0.count {
                if ctx.triObj[i] == excludeObj { continue }
                if triShadowHit(rIn, triIndex: i, eps: eps, backfaceCulling: backfaceCulling) { return true }
            }
            for i in 0..<ctx.sphCenter.count {
                if ctx.sphObj[i] == excludeObj { continue }
                if sphShadowHit(rIn, sphIndex: i, eps: eps) { return true }
            }
            for i in 0..<ctx.planeCenter.count {
                if ctx.planeObj[i] == excludeObj { continue }
                if planeShadowHit(rIn, planeIndex: i, eps: eps) { return true }
            }
            return false
        }
    }

    @inline(__always)
    private func triShadowHit(_ ray: Ray, triIndex: Int, eps: Double, backfaceCulling: Bool) -> Bool {
        let p0 = ctx.v0[triIndex]
        let e1 = ctx.e1[triIndex], e2 = ctx.e2[triIndex]
        let pvec = cross(ray.dir, e2)
        let det  = dot(e1, pvec)
        if abs(det) < eps { return false }
        if backfaceCulling && det < 0 { return false } // <<< add this
        let invDet = 1.0 / det
        let tvec = ray.origin - p0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return false }
        let q = cross(tvec, e1)
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { return false }
        let t = dot(e2, q) * invDet
        return (t > eps && t < ray.tMax)
    }

    @inline(__always)
    private func sphShadowHit(_ ray: Ray, sphIndex: Int, eps: Double) -> Bool {
        let c = ctx.sphCenter[sphIndex], r = ctx.sphRadius[sphIndex]
        let oc = ray.origin - c
        let a  = dot(ray.dir, ray.dir)
        let B  = dot(ray.dir, oc)
        let c2 = dot(oc, oc) - r*r
        let disc = B*B - a*c2
        if disc < 0 { return false }
        let t = (-B - sqrt(disc)) / a
        return (t > eps && t < ray.tMax)
    }

    @inline(__always)
    private func planeShadowHit(_ ray: Ray, planeIndex: Int, eps: Double) -> Bool {
        let n = normalize(ctx.planeNormal[planeIndex])
        let c = ctx.planeCenter[planeIndex]
        let denom = dot(ray.dir, n)

        if abs(denom) < eps { return false }

        let t = dot(c - ray.origin, n) / denom
        return (t > eps && t < ray.tMax)
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

        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)
        let m: Vec3 = e - wv * nd
        let q0Base: Vec3 = m + u * l + v * t

        var out = [Vec3](repeating: .zero, count: width * height)

        let relay: ProgressRelay? = (progressRow == nil) ? nil : ProgressRelay(total: height, cb: progressRow)

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

        @inline(__always)
        func reflect(_ d: Vec3, _ n: Vec3) -> Vec3 {
            d - 2.0 * dot(d, n) * n
        }

        let maxDepth: Int = scene.maxRecursionDepth
        let shadowEps = self.ctx.shadowRayEpsilon

        @inline(__always)
        func trace(_ inRay: inout Ray, _ depth: Int) -> Vec3 {
            _ = self.intersectScene(&inRay, backfaceCulling: true)

            if inRay.kind == .none {
                let bg = scene.backgroundColor
                return Vec3(x: bg.x, y: bg.y, z: bg.z)
            }

            let mat = scene.materials[Int(inRay.mat)]
            let p = inRay.origin + inRay.dir * inRay.tMax

            // Determine front/back facing
            let Ngeo = inRay.normal
            let frontFacing = dot(inRay.dir, Ngeo) < 0
            let N = frontFacing ? Ngeo : -Ngeo

            let ior = mat.ior
            let hasRefraction = (ior > 0)

            // Only compute direct lighting if:
            // 1. No refraction (opaque/mirror objects), OR
            // 2. Front-facing hit on refractive object (entering)
            let computeDirectLight = !hasRefraction || frontFacing


            // Start with ambient
            var Lo = computeDirectLight ? scene.lights.ambient * mat.ambient : .zero


            if computeDirectLight {
                for light in scene.lights.points {
                    var wi = light.position - p
                    let dist = length(wi)
                    wi = normalize(wi)

                    var sRay = makeRay(origin: p + N * shadowEps, dir: wi)
                    sRay.tMax = dist
                    let blocked = self.occluded(shadowRay: sRay, excludeObj: inRay.obj, backfaceCulling: true)
                    if !blocked {
                        Lo += light.shade(material: mat, ray: inRay, cameraPos: e, normal: N, at: p)
                    }
                }
            }

            let mirror = mat.mirror
            let hasMirror = mirror.x > 0 || mirror.y > 0 || mirror.z > 0
            let absorption = mat.absorption
            let hasAbsorption = absorption.x > 0 || absorption.y > 0 || absorption.z > 0

            // --- REFRACTION (Dielectrics) ---
            if hasRefraction && depth < maxDepth {
                let entering = frontFacing
                let n1: Double = entering ? 1.0 : ior
                let n2: Double = entering ? ior : 1.0
                let eta = n1 / n2

                // Incident angle with proper normal
                let cosi = max(0.0, min(1.0, -dot(inRay.dir, N)))
                let sin2t = eta * eta * (1.0 - cosi * cosi)

                // Check for Total Internal Reflection
                if sin2t > 1.0 {
                    // TIR: only reflection, no transmission
                    let rd = normalize(reflect(inRay.dir, N))
                    var rRay = makeRay(origin: p + N * shadowEps, dir: rd)
                    let LiR = trace(&rRay, depth + 1)
                    Lo += LiR
                } else {
                    // Both reflection and refraction
                    let cost = sqrt(1.0 - sin2t)

                    let rd = normalize(reflect(inRay.dir, N))
                    let td = normalize(eta * inRay.dir + (eta * cosi - cost) * N)

                    // Offset reflection ray along normal
                    let rOrigin = p + N * shadowEps

                    // Offset transmission ray along transmission direction
                    let tOrigin = p + td * shadowEps

                    var rRay = makeRay(origin: rOrigin, dir: rd)
                    var tRay = makeRay(origin: tOrigin, dir: td)

                    // Schlick Fresnel approximation
                    let Fr = schlickFresnel(cosTheta: cosi, n1: n1, n2: n2)
                    let Ft = 1.0 - Fr

                    let LiR = trace(&rRay, depth + 1)
                    var LiT = trace(&tRay, depth + 1)

                    // Apply Beer-Lambert absorption when EXITING the medium
                    if hasAbsorption && !entering {
                        let d = tRay.tMax
                        let att = Vec3(
                            x: exp(-absorption.x * d),
                            y: exp(-absorption.y * d),
                            z: exp(-absorption.z * d)
                        )
                        LiT *= att
                    }

                    Lo += LiR * Fr + LiT * Ft
                }
            } else if hasMirror && depth < maxDepth {
                // --- PERFECT MIRROR ---
                let rd = normalize(reflect(inRay.dir, N))
                var rRay = makeRay(origin: p + N * shadowEps, dir: rd)
                let Li = trace(&rRay, depth + 1)
                Lo += mirror * Li
            }

            // Safety check for NaN/Inf
            if !(Lo.x.isFinite && Lo.y.isFinite && Lo.z.isFinite) {
                return Vec3(0, 0, 0)
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

                            let pixel = trace(&ray, 0)
                            local[base + i] = pixel
                            localRays &+= 1
                        }
                        if let relay { await relay.rowFinished() }
                    }

                    return ChunkResult(startRow: startRow, endRow: endRow, pixels: local, rays: localRays)
                }
            }

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

                raysSum += chunk.rays
            }
        }

        raysTraced += raysSum
        if Task.isCancelled { throw CancellationError() }
        return out
    }

    @inline(__always)
    func schlickFresnel(cosTheta: Double, n1: Double, n2: Double) -> Double {
        let r0 = (n1 - n2) / (n1 + n2)
        let r0sq = r0 * r0
        return r0sq + (1.0 - r0sq) * pow(1.0 - cosTheta, 5.0)
    }
}
