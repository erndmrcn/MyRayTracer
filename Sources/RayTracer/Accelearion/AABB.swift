//
//  AABB.swift
//  ParsingKit
//
//  Created by Eren Demircan on 20.10.2025.
//

import Foundation
import simd
import ParsingKit

@frozen
public struct AABB: Sendable {
    public var minP: Vec3
    public var maxP: Vec3

    @inlinable public init() {
        self.minP = .init(repeating: .infinity)
        self.maxP = .init(repeating: -.infinity)
    }

    @inlinable public init(min: Vec3, max: Vec3) {
        self.minP = min; self.maxP = max
    }

    @inlinable public mutating func grow(_ p: Vec3) {
        minP = min(minP, p)
        maxP = max(maxP, p)
    }

    @inlinable public mutating func grow(_ b: AABB) {
        minP = simd.min(minP, b.minP)
        maxP = simd.max(maxP, b.maxP)
    }

    @inlinable public var extent: SIMD3<Double> { maxP - minP }

    @inline(__always)
    func hit(_ ro: Vec3, _ rd: Vec3, _ invDir: Vec3, tMax: Scalar, eps: Scalar) -> Bool {
        let t0x = (minP.x - ro.x) * invDir.x
        let t1x = (maxP.x - ro.x) * invDir.x
        let tminx = min(t0x, t1x)
        let tmaxx = max(t0x, t1x)

        let t0y = (minP.y - ro.y) * invDir.y
        let t1y = (maxP.y - ro.y) * invDir.y
        let tminy = min(t0y, t1y)
        let tmaxy = max(t0y, t1y)

        let t0z = (minP.z - ro.z) * invDir.z
        let t1z = (maxP.z - ro.z) * invDir.z
        let tminz = min(t0z, t1z)
        let tmaxz = max(t0z, t1z)

        let tNear = max(tminx, max(tminy, tminz))
        let tFar  = min(tmaxx, min(tmaxy, tmaxz))

        return (tNear <= tFar) && (tFar > eps) && (tNear < tMax)
    }

}
