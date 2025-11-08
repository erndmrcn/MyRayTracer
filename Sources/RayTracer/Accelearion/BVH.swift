//
//  BVH.swift
//  ParsingKit
//
//  Created by Eren Demircan on 20.10.2025.
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
    public var leftFirst: UInt   // if leaf: first primitive index; if inner: left child index
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

// MARK: - PrimitiveInfo (builder-facing shape)
public enum GeometryType: UInt8, Sendable {
    case mesh     // used only in TLAS (not inside BLAS)
    case triangle
    case sphere
    case plane
}

public enum ShadingMode: UInt8, Sendable { case flat, smooth }

public struct PrimitiveInfo: Sendable {
    public let type: GeometryType
    public let primitiveIndex: Int
    public let bounds: AABB
    public var centroid: Vec3
    public let materialID: Int
    public var shadingMode: ShadingMode = .smooth

    public init(type: GeometryType,
                primitiveIndex: Int,
                bounds: AABB,
                centroid: Vec3,
                materialID: Int,
                shadingMode: ShadingMode = .smooth)
    {
        self.type = type
        self.primitiveIndex = primitiveIndex
        self.bounds = bounds
        self.centroid = centroid
        self.materialID = materialID
        self.shadingMode = shadingMode
    }
}

// MARK: - Builder
@frozen public struct BVHBuilder: Sendable {

    public var primitives: [PrimitiveInfo]
    public var primitiveIdx: [UInt]
    public var bvhNode: [BVHNode]
    public var split: BVHSplitMode
    public var maxLeaf: Int
    public var rootNodeIdx: UInt = 0
    public var nodesUsed: UInt = 0
    public var binCount: Int

    public init(primitives: [PrimitiveInfo],
                maxLeaf: Int = 2,
                split: BVHSplitMode = .sah,
                binCount: Int = 12)
    {
        self.primitives = primitives
        self.primitiveIdx = (0..<UInt(primitives.count)).map { $0 }
        self.split = split
        self.maxLeaf = maxLeaf
        self.binCount = max(2, binCount)

        // pre-allocate node buffer (safe upper bound)
        let nodeCount = max(1, primitives.count * 2 - 1)
        self.bvhNode = (0..<nodeCount).map { _ in BVHNode() }

        // root
        bvhNode[0].leftFirst = 0
        bvhNode[0].primitiveCount = UInt(primitives.count)

        nodesUsed = 0
        rootNodeIdx = 0

        updateNodeBounds(0)
        subdivide(0)
    }

    // MARK: - Bounds
    @inline(__always) mutating
    func updateNodeBounds(_ nodeIdx: UInt) {
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
        }
    }

    // MARK: - Subdivide (SAH/binning)
    @inline(__always) mutating
    func subdivide(_ nodeIdx: UInt) {
        var node = bvhNode[Int(nodeIdx)]
        let primCount = Int(node.primitiveCount)
        if primCount <= maxLeaf && nodeIdx != 0 { return }

        // choose split axis/pos
        var bestAxis = 0
        var bestPos: Double = 0
        var bestCost: Double = .infinity

        switch split {
        case .midPoint, .median:
            let ext = node.aabbMax - node.aabbMin
            bestAxis = (ext.y > ext.x) ? 1 : 0
            if ext.z > ext[bestAxis] { bestAxis = 2 }
            bestPos = node.aabbMin[bestAxis] + ext[bestAxis] * 0.5
        case .sah:
            bestCost = findBestSplitPlane(&node, &bestAxis, splitPos: &bestPos)
            _ = bestCost
        }

        // partition by centroid
        var i = Int(node.leftFirst)
        var j = i + primCount - 1
        while i <= j {
            let primID = Int(primitiveIdx[i])
            if primitives[primID].centroid[bestAxis] < bestPos {
                i += 1
            } else {
                primitiveIdx.swapAt(i, j)
                j -= 1
            }
        }
        let leftCount = i - Int(node.leftFirst)
        if leftCount == 0 || leftCount == primCount { return }

        // allocate children
        nodesUsed &+= 1
        let leftChild = nodesUsed
        nodesUsed &+= 1
        let rightChild = nodesUsed
        precondition(Int(rightChild) < bvhNode.count, "BVH node overflow")

        // set children leaves
        bvhNode[Int(leftChild)].leftFirst       = node.leftFirst
        bvhNode[Int(leftChild)].primitiveCount  = UInt(leftCount)

        bvhNode[Int(rightChild)].leftFirst      = UInt(i)
        bvhNode[Int(rightChild)].primitiveCount = UInt(primCount - leftCount)

        // convert current node to inner
        node.leftFirst = leftChild
        node.primitiveCount = 0
        bvhNode[Int(nodeIdx)] = node

        // recurse
        updateNodeBounds(leftChild)
        updateNodeBounds(rightChild)
        subdivide(leftChild)
        subdivide(rightChild)
    }

    // MARK: - SAH binning
    @inline(__always)
    func findBestSplitPlane(_ node: inout BVHNode, _ axis: inout Int, splitPos: inout Double) -> Double {
        var bestCost: Double = .infinity
        for a in 0..<3 {
            var boundsMin = Double.infinity
            var boundsMax = -Double.infinity
            let count = Int(node.primitiveCount)
            let first = Int(node.leftFirst)

            for i in 0..<count {
                let prim = primitives[Int(primitiveIdx[first + i])]
                boundsMin = Swift.min(boundsMin, prim.centroid[a])
                boundsMax = Swift.max(boundsMax, prim.centroid[a])
            }
            if boundsMax <= boundsMin { continue }

            let N = binCount
            var bin: [AABB] = .init(repeating: .init(), count: N)
            var cnt: [Int]  = .init(repeating: 0, count: N)

            let scale = Double(N) / (boundsMax - boundsMin)
            for i in 0..<count {
                let prim = primitives[Int(primitiveIdx[first + i])]
                let idx = min(N - 1, Int((prim.centroid[a] - boundsMin) * scale))
                cnt[idx] += 1
                bin[idx].grow(prim.bounds)
            }

            // prefix/suffix
            var leftArea  = [Double](repeating: 0, count: N - 1)
            var rightArea = [Double](repeating: 0, count: N - 1)
            var leftCnt   = [Int](repeating: 0, count: N - 1)
            var rightCnt  = [Int](repeating: 0, count: N - 1)
            var L = AABB(), R = AABB()
            var sL = 0, sR = 0
            for i in 0..<(N - 1) {
                sL += cnt[i]
                leftCnt[i] = sL
                L.grow(bin[i])
                leftArea[i] = L.area()

                sR += cnt[N - 1 - i]
                rightCnt[N - 2 - i] = sR
                R.grow(bin[N - 1 - i])
                rightArea[N - 2 - i] = R.area()
            }

            let step = (boundsMax - boundsMin) / Double(N)
            for i in 0..<(N - 1) {
                if leftCnt[i] == 0 || rightCnt[i] == 0 { continue }
                let cost = Double(leftCnt[i]) * leftArea[i] + Double(rightCnt[i]) * rightArea[i]
                if cost < bestCost {
                    bestCost = cost
                    axis = a
                    splitPos = boundsMin + step * Double(i + 1)
                }
            }
        }
        return bestCost
    }
}
