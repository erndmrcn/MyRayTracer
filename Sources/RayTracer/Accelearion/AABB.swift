//
//  AABB.swift
//  ParsingKit
//
//  Created by Eren Demircan on 20.10.2025.
//

import Foundation
import simd
import ParsingKit

public struct AABB: Sendable {
    // Renamed min/max to standard `min` and `max` for brevity and clarity
    public var minP: Vec3 = .init(repeating: .infinity)
    public var maxP: Vec3 = .init(repeating: -.infinity)

    // MARK: - Core AABB Functions

    init() {

    }

    init(minP: Vec3, maxP: Vec3) {
        self.minP = minP
        self.maxP = maxP
    }

    // CRITICAL FIX: The correct surface area calculation
    @inlinable public func area() -> Double {
        let e = maxP - minP
        // Sum of the area of the three unique face pairs (XY, YZ, ZX) times two.
        return 2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
    }

    // Alias for area() as `surfaceArea()` is often more descriptive in SAH
    @inlinable public func surfaceArea() -> Double {
        return area()
    }

    // Centroid of the AABB
    @inlinable public func centroid() -> Vec3 {
        return (minP + maxP) * 0.5
    }

    // Accesses the center coordinate along a specific axis (0=X, 1=Y, 2=Z)
    @inlinable public func center(axis: Int) -> Double {
        return (minP[axis] + maxP[axis]) * 0.5
    }

    // Returns the index of the longest axis (0, 1, or 2)
    @inlinable public func maxExtent() -> Int {
        let e = maxP - minP
        if e.x > e.y && e.x > e.z { return 0 }
        if e.y > e.z { return 1 }
        return 2
    }

    // MARK: - Growing and Transformations

    @inlinable public mutating func grow(_ p: Vec3) {
        minP = min(minP, p)
        maxP = max(maxP, p)
    }

    @inlinable public mutating func grow(_ b: AABB) {
        minP = min(minP, b.minP)
        maxP = max(maxP, b.maxP)
    }

    public var extent: Vec3 { maxP - minP }
    func transformed(by M: Mat4) -> AABB {
        // Eski merkez ve extent
        let cOld = 0.5 * (minP + maxP)
        let eOld = 0.5 * (maxP - minP)

        // Lineer kısım (3x3) ve translasyon
        let r: Mat3 = Mat3(M.columns.0.xyz, M.columns.1.xyz, M.columns.2.xyz) // sütunlar
        let t = Vec3(M.columns.3.x, M.columns.3.y, M.columns.3.z)

        // Yeni merkez
        let cNew = r * cOld + t

        let ar: Mat3 = Mat3(simd_abs(r.columns.0), simd_abs(r.columns.1), simd_abs(r.columns.2))

        let eNew = Vec3(
            ar.columns.0.x * eOld.x + ar.columns.1.x * eOld.y + ar.columns.2.x * eOld.z,
            ar.columns.0.y * eOld.x + ar.columns.1.y * eOld.y + ar.columns.2.y * eOld.z,
            ar.columns.0.z * eOld.x + ar.columns.1.z * eOld.y + ar.columns.2.z * eOld.z
        )

        return AABB(minP: cNew - eNew, maxP: cNew + eNew)
    }
}

// MARK: - Ray Intersection
extension AABB {
    // Your AABB intersection logic is already robust for this use case,
    // but simplified to use the new `min`/`max` names.
    @inlinable
    func hit(_ ro: Vec3, _ rd: Vec3, _ invDir: Vec3, tMax: Scalar, eps: Scalar) -> Bool {
        let t1 = (minP - ro) * invDir // Changed minP to min
        let t2 = (maxP - ro) * invDir // Changed maxP to max
        let tmin = min(t1, t2)
        let tmax = max(t1, t2)

        let tNear = max(max(tmin.x, tmin.y), tmin.z)
        let tFar  = min(min(tmax.x, tmax.y), tmax.z)

        // Check for overlap and limits
        return !(tNear > tFar || tFar <= eps || tNear >= tMax)
    }
}
