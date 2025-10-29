//
//  BVH.swift
//  ParsingKit
//
//  Created by Eren Demircan on 20.10.2025.
//

import simd
import ParsingKit

// MARK: - Public split mode
public enum BVHSplitMode: Sendable { case sah, median }

// MARK: - Node
public final class BVHNode: @unchecked Sendable {
    public var bounds: AABB = .init()
    public var left: BVHNode? = nil
    public var right: BVHNode? = nil
    public var start: Int = 0
    public var count: Int = 0
    @inlinable public var isLeaf: Bool { left == nil && right == nil }

    public init() {}
}

// MARK: - Primitive tags / payload
@frozen enum PrimType: UInt8 { case tri = 0, sphere = 1, plane = 2}

/// Internal reference used while building/splitting.
/// Contains centroid & bounds (required for partitioning).
private struct PrimRef: Sendable {
    let type: PrimType
    let idx: Int
    let centroid: Vec3
    let bounds: AABB
}

// MARK: - Builder
public struct BVHBuilder: Sendable {
    public var maxLeaf: Int
    public var split: BVHSplitMode

    public init(maxLeaf: Int = 8, split: BVHSplitMode = .sah) {
        self.maxLeaf = max(1, maxLeaf)  // Ensure at least 1
        self.split = split
    }

    /// Build a BVH over triangles + spheres in `context`.
    /// Returns the root and a compact primitive array laid out in leaf order.
    public func buildFinite(triCount: Int,
                            sphereCount: Int,
                            context: RTContext)
    -> (root: BVHNode, prims: [BVHPrim]) {
        guard triCount >= 0 && sphereCount >= 0 else {
            return (BVHNode(), [])
        }

        var refs: [PrimRef] = []
        refs.reserveCapacity(triCount + sphereCount + context.planeCenter.count)

        for i in 0..<min(triCount, context.v0.count) {
            let p0 = context.v0[i]
            let p1 = context.v1[i]
            let p2 = context.v2[i]
            var b = AABB()
            b.grow(p0); b.grow(p1); b.grow(p2)
            let c = (p0 + p1 + p2) / Scalar(3)
            refs.append(.init(type: .tri, idx: i, centroid: c, bounds: b))
        }

        for i in 0..<min(sphereCount, context.sphCenter.count) {
            let c = context.sphCenter[i]
            let r = context.sphRadius[i]
            let rr = Vec3(repeating: r)
            let b = AABB(min: c - rr, max: c + rr)
            refs.append(.init(type: .sphere, idx: i, centroid: c, bounds: b))
        }

        for i in 0..<context.planeCenter.count {
            let c = context.planeCenter[i]
            let n = context.planeNormal[i]

            let len = length(n)
            guard len > Scalar(1e-8) else { continue }
            let nNorm = n / len

            let size: Scalar = 1e3
            let axisU: Vec3 = Swift.abs(nNorm.x) > 0.5 ? Vec3(0.0, 1.0, 0.0) : Vec3(1.0, 0.0, 0.0)
            let tangent = normalize(cross(nNorm, axisU))
            let bitangent = normalize(cross(nNorm, tangent))
            let extent = tangent * size + bitangent * size + nNorm * 1.0

            var b = AABB()
            b.grow(c - extent)
            b.grow(c + extent)

            refs.append(.init(type: .plane, idx: i, centroid: c, bounds: b))
        }

        guard !refs.isEmpty else {
            return (BVHNode(), [])
        }

        let root = buildRecursive(&refs, 0, refs.count)

        let prims: [BVHPrim] = refs.map { BVHPrim(type: $0.type.rawValue, index: $0.idx) }
        return (root, prims)
    }

    private func buildRecursive(_ refs: inout [PrimRef],
                                _ begin: Int,
                                _ end: Int) -> BVHNode {
        let node = BVHNode()
        let count = end - begin

        guard count > 0 else {
            return node
        }

        var nodeBounds = AABB()
        var centroidBounds = AABB()
        for i in begin..<end {
            nodeBounds.grow(refs[i].bounds)
            centroidBounds.grow(refs[i].centroid)
        }
        node.bounds = nodeBounds

        if count <= maxLeaf || isDegenerate(centroidBounds) {
            node.start = begin
            node.count = count
            return node
        }

        let ext = centroidBounds.extent
        let axis: Int = (ext.x > ext.y && ext.x > ext.z) ? 0 : ((ext.y > ext.z) ? 1 : 2)

        var mid: Int
        switch split {
        case .median:
            refs[begin..<end].sort {
                if $0.centroid[axis] == $1.centroid[axis] {
                    let ax2 = (axis + 1) % 3
                    return $0.centroid[ax2] < $1.centroid[ax2]
                }
                return $0.centroid[axis] < $1.centroid[axis]
            }
            var m = (begin + end) >> 1
            m = max(begin + 1, m)
            m = min(end - 1, m)
            mid = m

        case .sah:
            let bins = 12
            var counts = Array(repeating: 0, count: bins)
            var binBox = Array(repeating: AABB(), count: bins)

            let cmin = centroidBounds.minP[axis]
            let cmax = centroidBounds.maxP[axis]
            let extentVal = cmax - cmin
            let invExtent = Scalar(1) / max(Scalar(1e-12), extentVal)

            for i in begin..<end {
                let c = refs[i].centroid[axis]
                let t = (c - cmin) * invExtent
                var b = Int(Scalar(bins) * clamp(t, Scalar(0), Scalar(1) - Scalar(1e-8)))
                b = max(0, min(bins - 1, b))
                counts[b] &+= 1
                binBox[b].grow(refs[i].bounds)
            }

            var leftCount = Array(repeating: 0, count: bins)
            var rightCount = Array(repeating: 0, count: bins)
            var leftBox = Array(repeating: AABB(), count: bins)
            var rightBox = Array(repeating: AABB(), count: bins)

            var acc = 0
            var box = AABB()
            for i in 0..<bins {
                acc += counts[i]
                leftCount[i] = acc
                if counts[i] > 0 { box.grow(binBox[i]) }
                leftBox[i] = box
            }

            acc = 0; box = AABB()
            for i in (0..<bins).reversed() {
                acc += counts[i]
                rightCount[i] = acc
                if counts[i] > 0 { box.grow(binBox[i]) }
                rightBox[i] = box
            }

            var bestCost = Scalar.greatestFiniteMagnitude
            var bestBin = 0
            let S = max(surfaceArea(nodeBounds), Scalar(1e-18))

            for i in 0..<(bins - 1) where leftCount[i] > 0 && rightCount[i + 1] > 0 {
                let cost = Scalar(1) +
                (surfaceArea(leftBox[i])      / S) * Scalar(leftCount[i]) +
                (surfaceArea(rightBox[i + 1]) / S) * Scalar(rightCount[i + 1])
                if cost < bestCost { bestCost = cost; bestBin = i }
            }

            refs[begin..<end].sort {
                if $0.centroid[axis] == $1.centroid[axis] {
                    let ax2 = (axis + 1) % 3
                    return $0.centroid[ax2] < $1.centroid[ax2]
                }
                return $0.centroid[axis] < $1.centroid[axis]
            }

            let splitIdx = begin + leftCount[bestBin]
            var m = splitIdx
            m = max(begin + 1, m)
            m = min(end - 1, m)
            mid = m

            guard mid > begin && mid < end else {
                node.start = begin
                node.count = count
                return node
            }
        }

        node.left  = buildRecursive(&refs, begin, mid)
        node.right = buildRecursive(&refs, mid, end)
        return node
    }

    // MARK: - Helpers
    @inline(__always)
    private func surfaceArea(_ b: AABB) -> Scalar {
        let e = b.extent
        // TODO: play with this Scalar(2) to get better results
        return Scalar(2) * (e.x*e.y + e.x*e.z + e.y*e.z)
    }

    @inline(__always)
    private func isDegenerate(_ cbox: AABB) -> Bool {
        let e = cbox.extent
        let eps: Scalar = 1e-12
        return e.x <= eps && e.y <= eps && e.z <= eps
    }

    @inline(__always)
    private func clamp(_ v: Scalar, _ min: Scalar, _ max: Scalar) -> Scalar {
        return Swift.max(min, Swift.min(max, v))
    }
}
