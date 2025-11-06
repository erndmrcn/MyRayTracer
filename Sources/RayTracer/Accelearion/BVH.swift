//
//  BVH.swift
//  ParsingKit
//
//  Created by Eren Demircan on 20.10.2025.
//

import simd
import Foundation
import ParsingKit

// MARK: - Split mode
public enum BVHSplitMode: Sendable { case midPoint, sah, median }

// MARK: - Node
@frozen public struct BVHNode: Sendable {
    public var aabbMin: Vec3
    public var aabbMax: Vec3
    public var leftFirst: UInt
    public var primitiveCount: UInt
    @inlinable public var isLeaf: Bool { primitiveCount > 0 }

    @inlinable
    public init() {
        self.aabbMin = .zero
        self.aabbMax = .zero
        self.leftFirst = 0
        self.primitiveCount = 0
    }
}


struct Bin {
    var bounds = AABB()
    var primcount: Int = 0 // Renamed from tricount
}
// MARK: - Builder
public struct BVHBuilder: Sendable {

    public var primitives: [PrimitiveInfo]
    public var primitiveIdx: [UInt]
    public var bvhNode: [BVHNode]
    public var split: BVHSplitMode
    public var maxLeaf: Int
    public var rootNodeIdx: UInt = 0
    public var nodesUsed: UInt = 0
    public var binCount: Int

    public init(primitives: [PrimitiveInfo], maxLeaf: Int = 2, split: BVHSplitMode = .sah, binCount: Int = 12) {
        self.primitives = primitives
        self.primitiveIdx = (0..<UInt(primitives.count)).map { $0 }
        self.split = split
        self.maxLeaf = maxLeaf
        self.binCount = binCount

        // pre-allocate node buffer (safe upper bound)
        let nodeCount = max(1, primitives.count * 2 - 1)
        self.bvhNode = (0..<nodeCount).map { _ in BVHNode() }

        // Root node
        var root = bvhNode[0]
        root.leftFirst = 0
        root.primitiveCount = UInt(primitives.count) // Use primitiveCount
        bvhNode[0] = root

        nodesUsed = 0
        rootNodeIdx = 0
        print("Starting BVH Building")
        let t = DispatchTime.now()
        updateNodeBounds(0)
        subdivide(0)
        let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - t.uptimeNanoseconds) / 1_000_000.0)
        print("BVH Build time: \(elapsedMs) ms")
        print("🌲 BVH nodes used:", nodesUsed, " root primitives:", bvhNode[0].primitiveCount)
    }

    // MARK: - Bounds (Updated)
    mutating func updateNodeBounds(_ nodeIdx: UInt) {
        bvhNode.withUnsafeMutableBufferPointer { buf in
            var node = buf[Int(nodeIdx)]
            node.aabbMin = Vec3(repeating: .infinity)
            node.aabbMax = Vec3(repeating: -.infinity)

            let first = Int(node.leftFirst)
            let count = Int(node.primitiveCount)
            for i in 0..<count {
                let primID = Int(primitiveIdx[first + i])
                let prim = primitives[primID]

                node.aabbMin = min(node.aabbMin, prim.bounds.minP)
                node.aabbMax = max(node.aabbMax, prim.bounds.maxP)
            }
            buf[Int(nodeIdx)] = node

            // --- 🖨️ ADD THIS DEBUGGING ---
            if nodeIdx == 0 {
                 print("--- Root Node Bounds Updated ---")
                 print("  Min: \(node.aabbMin)")
                 print("  Max: \(node.aabbMax)")
                 let e = node.aabbMax - node.aabbMin
                 let area = e.x * e.y + e.y * e.z + e.z * e.x
                 print("  Dimensions (e): \(e)")
                 print("  Surface Area (calc): \(area)")
                 print("  Primitive Count: \(count)")
                 print("  Resulting noSplitCost: \(Double(count) * area)")
                 print("---------------------------------")
            }
        }
    }


    // MARK: - Subdivide (Updated)
    mutating func subdivide(_ nodeIdx: UInt) {
        var node = bvhNode[Int(nodeIdx)]
        let primCount = Int(node.primitiveCount)

        // choose split axis/position
        var bestAxis = 0
        var bestPos: Double = 0
        var bestCost: Double = .infinity

        switch split {
        case .midPoint:
            let ext = node.aabbMax - node.aabbMin
            bestAxis = (ext.y > ext.x) ? 1 : 0
            if ext.z > ext[bestAxis] { bestAxis = 2 }
            bestPos = node.aabbMin[bestAxis] + ext[bestAxis] * 0.5

        case .sah:
            bestCost = findBestSplitPlane(&node, &bestAxis, splitPos: &bestPos)

        case .median:
            let ext = node.aabbMax - node.aabbMin
            bestAxis = (ext.y > ext.x) ? 1 : 0
            if ext.z > ext[bestAxis] { bestAxis = 2 }
            bestPos = node.aabbMin[bestAxis] + ext[bestAxis] * 0.5
        }

        // stop condition
        let noSplitCost = calculateNodeCost(&node)
//        print("SAH check: best cost is \(bestCost)")
        if primCount <= maxLeaf && nodeIdx != 0 { return } // allow root to split at least once
//            print("🔴 STOPPING: SAH cost is too high.")
//            return
//        }

        // partition primitives (uses centroid from PrimitiveInfo)
        var i = Int(node.leftFirst)
        var j = i + primCount - 1
        while i <= j {
            let primID = Int(primitiveIdx[i]) // Use primitiveIdx
            // Use the centroid from the PrimitiveInfo for the split
            if primitives[primID].centroid[bestAxis] < bestPos {
                i += 1
            } else {
                primitiveIdx.swapAt(i, j) // Use primitiveIdx
                j -= 1
            }
        }

        let leftCount = i - Int(node.leftFirst)
//        print("Partition results: leftCount = \(leftCount), primCount = \(primCount)")
        if leftCount == 0 || leftCount == primCount {
            print("🔴 STOPPING: Partition failed.")
            return
        }

        // allocate children from global counter
        nodesUsed &+= 1
        let leftChild  = nodesUsed
        nodesUsed &+= 1
        let rightChild = nodesUsed

        // assign children
        bvhNode[Int(leftChild)].leftFirst     = node.leftFirst
        bvhNode[Int(leftChild)].primitiveCount = UInt(leftCount)

        bvhNode[Int(rightChild)].leftFirst     = UInt(i)
        bvhNode[Int(rightChild)].primitiveCount = UInt(primCount - leftCount)

        node.leftFirst = leftChild
        node.primitiveCount = 0
        bvhNode[Int(nodeIdx)] = node

        if leftChild >= bvhNode.count || rightChild >= bvhNode.count {
            print("⚠️ BVH overflow: not enough node capacity (need \(rightChild), have \(bvhNode.count))")
            return
        }

        updateNodeBounds(leftChild)
        updateNodeBounds(rightChild)
        subdivide(leftChild)
        subdivide(rightChild)
    }

    // MARK: - SAH (Updated)
    func findBestSplitPlane(_ node: inout BVHNode, _ axis: inout Int, splitPos: inout Double) -> Double {
        var bestCost: Double = .infinity
        for a in 0..<3 {
            var boundsMin: Double = .infinity
            var boundsMax: Double = -.infinity
            let count = Int(node.primitiveCount)
            let first = Int(node.leftFirst)

            for i in 0..<count {
                let prim = primitives[Int(primitiveIdx[first + i])]
                boundsMin = min(boundsMin, prim.centroid[a])
                boundsMax = max(boundsMax, prim.centroid[a])
            }

            if boundsMax <= boundsMin { continue }

            var bin: [Bin] = .init(repeating: .init(), count: binCount)
            let scale = Double(binCount) / (boundsMax - boundsMin)

            for i in 0..<count {
                let prim = primitives[Int(primitiveIdx[first + i])]
                let binIdx = min(binCount - 1, Int((prim.centroid[a] - boundsMin) * scale))
                bin[binIdx].primcount += 1

                bin[binIdx].bounds.grow(prim.bounds)
            }

            var leftArea: [Double] = .init(repeating: 0, count: binCount - 1)
            var rightArea: [Double] = .init(repeating: 0, count: binCount - 1)
            var leftCount: [Int] = .init(repeating: 0, count: binCount - 1)
            var rightCount: [Int] = .init(repeating: 0, count: binCount - 1)
            var leftBox = AABB()
            var rightBox = AABB()
            var leftSum: Int = 0
            var rightSum: Int = 0

            for i in 0..<binCount-1 {
                leftSum += bin[i].primcount
                leftCount[i] = leftSum
                leftBox.grow(bin[i].bounds)
                leftArea[i] = leftBox.area()

                rightSum += bin[binCount - 1 - i].primcount
                rightCount[binCount - 2 - i] = rightSum
                rightBox.grow(bin[binCount - 1 - i].bounds)
                rightArea[binCount - 2 - i] = rightBox.area()
            }

            let splitScale = (boundsMax - boundsMin) / Double(binCount)
            for i in 0..<binCount-1 {
                let planeCost = Double(leftCount[i]) * leftArea[i] + Double(rightCount[i]) * rightArea[i]
                if planeCost < bestCost {
                    axis = a

                    splitPos = boundsMin + splitScale * Double(i + 1)
                    bestCost = planeCost
                }
            }
        }

        return bestCost
    }

    func calculateNodeCost(_ node: inout BVHNode) -> Double {
        let e: Vec3 = node.aabbMax - node.aabbMin
        let surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x
        return Double(node.primitiveCount) * surfaceArea // Use primitiveCount
    }

    // `evaluateSAH` is now redundant if `findBestSplitPlane` is correct but
    // kept for completeness (needs updates too).
    /*
    func evaluateSAH(_ node: BVHNode, _ axis: Int, _ pos: Double) -> Double {
        var leftBox = AABB(), rightBox = AABB()
        var leftCount = 0, rightCount = 0
        let first = Int(node.leftFirst)
        let count = Int(node.primitiveCount)
        for k in 0..<count {
            let prim = primitives[Int(primitiveIdx[first + k])]
            if prim.centroid[axis] < pos {
                leftCount += 1
                leftBox.grow(prim.bounds)
            } else {
                rightCount += 1
                rightBox.grow(prim.bounds)
            }
        }
        guard leftCount > 0 && rightCount > 0 else { return .infinity }
        let lc = Double(leftCount), rc = Double(rightCount)
        return lc * leftBox.area() + rc * rightBox.area()
    }
    */
}

// MARK: - Traversal (Updated)
extension BVHBuilder {
    @inlinable @inline(__always)
    func intersectBVH(ray: inout Ray, nodeIdx: UInt, ctx: borrowing RTContext, eps: Scalar) -> Int {
        // --- Preload references to avoid ARC and dictionary lookups ---
        let nodes = bvhNode
        let primitives = self.primitives
        let primIdx = self.primitiveIdx
        let tris = ctx.triangles
        let sphs = ctx.spheres
        let plns = ctx.planes
        let invDir = ray.invDir

        var nodeVisits = 0

        // --- Small local traversal stack ---
        var stack = UnsafeMutableBufferPointer<UInt>.allocate(capacity: 128)
        defer { stack.deallocate() }
        var sp = 0
        stack[sp] = nodeIdx

        @inline(__always)
        func hitAABB(_ node: BVHNode) -> Double {
            // Parametric intersection with each axis
            let t1 = (node.aabbMin - ray.origin) * invDir
            let t2 = (node.aabbMax - ray.origin) * invDir

            // Entry / exit per axis
            let tminv = simd.min(t1, t2)
            let tmaxv = simd.max(t1, t2)

            // Combine across axes
            let tmin = max(tminv.x, max(tminv.y, tminv.z))
            let tmax = min(tmaxv.x, min(tmaxv.y, tmaxv.z))

            // Valid hit if intervals overlap within ray range
            return (tmax >= max(tmin, eps) && tmin < ray.tMax) ? tmin : .infinity
        }

        // --- Iterative traversal ---
        while sp >= 0 {
            nodeVisits &+= 1
            let idx = stack[sp]
            sp &-= 1
            let node = nodes[Int(idx)]

            // Cull if no AABB hit
            if hitAABB(node) == .infinity { continue }

            if node.isLeaf {
                let first = Int(node.leftFirst)
                let count = Int(node.primitiveCount)

                for k in 0..<count {
                    let primID = Int(primIdx[first + k])
                    let prim = primitives[primID]

                    switch prim.type {
                    case .triangle:
                        let tri = tris[prim.primitiveIndex]
                        intersectTriangle(ray: &ray, triangle: tri, isSmooth: prim.shadingMode == .smooth, matIdx: prim.materialID, eps: eps)

                    case .sphere:
                        let sph = sphs[prim.primitiveIndex]
                        intersectSphere(ray: &ray, sphere: sph, matIdx: prim.materialID, eps: eps)

                    case .plane:
                        let pln = plns[prim.primitiveIndex]
                        intersectPlane(ray: &ray, plane: pln, matIdx: prim.materialID, eps: eps)

                    default:
                        break
                    }
                }
                continue
            }

            // Internal node
            var leftIdx = node.leftFirst
            var rightIdx = leftIdx + 1
            let leftNode = nodes[Int(leftIdx)]
            let rightNode = nodes[Int(rightIdx)]

            var d1 = hitAABB(leftNode)
            var d2 = hitAABB(rightNode)

            // Process nearer node first
            if d1 > d2 {
                swap(&d1, &d2)
                swap(&leftIdx, &rightIdx)
            }

            if d2 != .infinity {
                sp &+= 1
                stack[sp] = rightIdx
            }
            if d1 != .infinity {
                sp &+= 1
                stack[sp] = leftIdx
            }
        }

//        print("intersectBVH traversed nodes = \(nodeVisits)")
        return nodeVisits
    }

    @inlinable @inline(__always)
    func intersectAABB(ray: Ray, bmin: Vec3, bmax: Vec3, eps: Scalar) -> Double {
        let t1 = (bmin - ray.origin) * ray.invDir
        let t2 = (bmax - ray.origin) * ray.invDir
        let tminv = simd.min(t1, t2)
        let tmaxv = simd.max(t1, t2)
        let tmin = max(max(tminv.x, tminv.y), max(tminv.z, eps))
        let tmax = min(min(tmaxv.x, tmaxv.y), ray.tMax)
        return (tmax >= max(tmin, eps) && tmin < ray.tMax) ? tmin : .infinity
    }

    @inlinable @inline(__always)
    func intersectTriangle(ray: inout Ray, triangle: Triangle, isSmooth: Bool, matIdx: Int, eps: Scalar) {

        let h = cross(ray.dir, triangle.e2)
        let a = dot(triangle.e1, h)
        if Swift.abs(a) < eps { return }
        let f = 1.0 / a
        let s = ray.origin - triangle.v0
        let u = f * dot(s, h)
        if u < 0 || u > 1 { return }
        let q = cross(s, triangle.e1)
        let v = f * dot(ray.dir, q)
        if v < 0 || u + v > 1 { return }
        let t = f * dot(triangle.e2, q)
        if t <= eps || t >= ray.hit.t || t >= ray.tMax { return }

        let w = 1.0 - u - v
        var n: Vec3 = triangle.n0

        if isSmooth {
            let n0 = triangle.n0
            let n1 = triangle.n1
            let n2 = triangle.n2
            let interpolated = w * n0 + u * n1 + v * n2
            n = normalize(interpolated)
        }

//        if dot(n, ray.dir) < 0 {
//            n *= -1
//        }

        let p = triangle.v0 + u * triangle.e1 + v * triangle.e2// (*V0) + u * ((*V1) - (*V0)) + v * ((*V2) - (*V0));
//        let p = ray.origin + ray.dir * t
        ray.hit.t = t
        ray.hit.p = p
        ray.hit.n = n
        ray.hit.kind = .triangle
        ray.hit.mat = matIdx
    }

    @inlinable @inline(__always)
    func intersectSphere(ray: inout Ray, sphere: Sphere, matIdx: Int, eps: Scalar) {
        let oc = ray.origin - sphere.center
        let a = dot(ray.dir, ray.dir)
        let b = 2.0 * dot(oc, ray.dir)
        let c = dot(oc, oc) - sphere.radius * sphere.radius
        let disc = b*b - 4*a*c
        if disc < 0 { return }
        let sqrtD = sqrt(disc)
        var t = (-b - sqrtD) / (2*a)
        if t < eps { t = (-b + sqrtD) / (2*a) }
        if t < eps || t >= ray.hit.t { return }

        let p = ray.origin + ray.dir * t
        let n = normalize(p - sphere.center)
        ray.hit.t = t
        ray.hit.p = p
        ray.hit.n = n
        ray.hit.kind = .sphere
        ray.hit.mat = matIdx
    }

    @inlinable @inline(__always)
    func intersectPlane(ray: inout Ray, plane: Plane, matIdx: Int, eps: Scalar) {
        let denom = dot(plane.normal, ray.dir)
        if Swift.abs(denom) < eps { return }
        let t = dot(plane.center - ray.origin, plane.normal) / denom
        if t <= eps || t >= ray.hit.t { return }

        let p = ray.origin + ray.dir * t
        ray.hit.t = t
        ray.hit.p = p
        ray.hit.n = normalize(plane.normal)
        ray.hit.kind = .plane
        ray.hit.mat = matIdx
    }
}

extension BVHBuilder {

    /// Optimized BVH traversal that returns immediately upon finding any intersection
    /// within the ray's tMax (the distance to the light).
    @inlinable @inline(__always)
    public func intersectBVHShadow(ray: Ray, nodeIdx: UInt, ctx: RTContext, eps: Scalar) -> Bool {
        var stack: [UInt] = [nodeIdx]
        let invDir = ray.invDir

        while let idx = stack.popLast() {
            let node = bvhNode[Int(idx)]

            if intersectAABB(ray: ray, bmin: node.aabbMin, bmax: node.aabbMax, eps: eps) == .infinity {
                continue
            }

            if node.isLeaf {
                let first = Int(node.leftFirst)
                let count = Int(node.primitiveCount)

                for k in 0..<count {
                    let primID = Int(primitiveIdx[first + k])
                    let primInfo = primitives[primID]

                    switch primInfo.type {
                    case .triangle:
                        let tri = ctx.triangles[primInfo.primitiveIndex]
                        if triShadowHit(ray, triangle: tri, eps: eps) { return true }
                    case .sphere:
                        let sph = ctx.spheres[primInfo.primitiveIndex]
                        if sphereShadowHit(ray, sphere: sph, eps: eps) { return true }
                    case .plane:
                        let pln = ctx.planes[primInfo.primitiveIndex]
                        if planeShadowHit(ray, plane: pln, eps: eps) { return true }
                    default: break
                    }
                }
            } else {
                // 3. Internal Node Traversal (Same logic as before, but return if we find a shadow)
                var leftIdx = node.leftFirst
                var rightIdx = leftIdx + 1

                let leftNode = bvhNode[Int(leftIdx)]
                let rightNode = bvhNode[Int(rightIdx)]

                // Use the full AABB intersection for distance checks
                var d1 = intersectAABB(ray: ray, bmin: leftNode.aabbMin, bmax: leftNode.aabbMax, eps: eps)
                var d2 = intersectAABB(ray: ray, bmin: rightNode.aabbMin, bmax: rightNode.aabbMax, eps: eps)

                // Process nearer node first (Coherent Traversal)
                if d1 > d2 {
                    swap(&d1, &d2)
                    swap(&leftIdx, &rightIdx)
                }

                // Push nearer node first (so it's popped last)
                if d2 != .infinity { stack.append(rightIdx) }
                if d1 != .infinity { stack.append(leftIdx) }
            }
        }
        return false // No hit found
    }

    @inlinable @inline(__always)
    func triShadowHit(_ ray: Ray, triangle: Triangle, eps: Double) -> Bool {
        let pvec = cross(ray.dir, triangle.e2)
        let det = dot(triangle.e1, pvec)
        if Swift.abs(det) < eps { return false }
        let invDet = 1.0 / det
        let tvec = ray.origin - triangle.v0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return false }
        let q = cross(tvec, triangle.e1)
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { return false }
        let t = dot(triangle.e2, q) * invDet
        return (t > eps && t < ray.tMax)
    }

    @inlinable @inline(__always)
    func sphereShadowHit(_ ray: Ray, sphere: Sphere, eps: Scalar) -> Bool {
        let oc = ray.origin - sphere.center
        let a = dot(ray.dir, ray.dir)
        let b = 2.0 * dot(oc, ray.dir)
        let c = dot(oc, oc) - sphere.radius * sphere.radius
        let disc = b*b - 4*a*c
        if disc < 0 { return false }
        let sqrtD = sqrt(disc)
        var t = (-b - sqrtD) / (2*a)
        if t < eps { t = (-b + sqrtD) / (2*a) }
        // CRITICAL: t must be within (eps, ray.hit.t)
        return (t > eps && t < ray.tMax)
    }

    @inlinable @inline(__always)
    func planeShadowHit(_ ray: Ray, plane: Plane, eps: Scalar) -> Bool {
        let denom = dot(plane.normal, ray.dir)
        if Swift.abs(denom) < eps { return false }
        let t = dot(plane.center - ray.origin, plane.normal) / denom
        // CRITICAL: t must be within (eps, ray.hit.t)
        return (t > eps && t < ray.tMax)
    }
}
