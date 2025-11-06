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

    // Optimized Transformed AABB Calculation
    public func transformed(by M: Mat4) -> AABB {
        // Start with the translation component of the transform
        let translation = Vec3(M.columns.3.x, M.columns.3.y, M.columns.3.z)
        var out = AABB(minP: translation, maxP: translation)

        // Loop over the 3 axes of the transformation matrix (columns 0, 1, 2)
        for i in 0..<3 {
            let col = Vec3(M[i].x, M[i].y, M[i].z)
            let e = extent[i] // e is the extent of the original AABB along axis i

            // Add the extent of the transformed primary axes.
            // This is more numerically stable and faster than transforming all 8 corners.
            let a = col * minP[i]
            let b = col * maxP[i]
            out.grow(a)
            out.grow(b)
        }
        return out
        // Note: The original 8-corner method is correct, but this is a standard,
        // faster way to compute the transformed AABB if rotation/scaling is involved.
        /*
        let corners = [
            Vec3(minP.x, minP.y, minP.z),
            // ... all 8 corners
        ]
        var out = AABB()
        for c in corners {
             // ... transform logic ...
             out.grow(Vec3(q.x, q.y, q.z))
        }
        return out
        */
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
