//
//  Sphere.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

import Foundation
@preconcurrency import ParsingKit
import simd

public struct Sphere: Sendable, Decodable {
    public let id: String?
    /// Either a raw index into vertex array (as in your JSON) or 3D coordinates.
    /// We store both possibilities:
    public let centerIndex: Int?
    @FlexibleVec3<Vec3> public var center: Vec3 // in case a scene uses explicit coords
    @FlexibleDouble public var radius: Double
    public let materialID: String?
    public var material: Material?

    enum CodingKeys: String, CodingKey {
        case id = "_id"
        case center = "Center"
        case radius = "Radius"
        case materialID = "Material"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try? c.decode(String.self, forKey: .id)

        if let raw = try? c.pkString(forKey: .center), let idx = Int(raw.trimmingCharacters(in: .whitespaces)) {
            centerIndex = idx; _center = FlexibleVec3<Vec3>(wrappedValue: .init(0,0,0))
        } else {
            centerIndex = nil
            _center = (try? .init(from: c.superDecoder(forKey: .center))) ?? FlexibleVec3<Vec3>(wrappedValue: .init(0,0,0))
        }

        _radius = try .init(from: c.superDecoder(forKey: .radius))
        materialID = try? c.pkString(forKey: .materialID)
    }
}

extension Sphere: Object {

    public var materialIdx: Int {
        if let id = materialID, let idx = Int(id) {
            return idx - 1
        }

        return -1
    }

    @inlinable
    public func intersect(ray: inout Ray, backfaceCulling: Bool = false) -> Bool {
        let o = ray.origin
        let d = ray.direction
        let L = o - center

        // Quadratic coeffs for ||o + t d - c||^2 = r^2
        let a = simd_dot(d, d)
        let b = 2.0 * simd_dot(d, L)
        let c = simd_dot(L, L) - radius * radius

        let delta = b*b - 4.0*a*c
        if delta < 0.0 { return false }

        if delta == 0.0 {
            // One tangent root
            let t = -0.5 * b / a
            if t > 1e-6 {
                ray.t = t
                return true
            }
            return false
        }

        // Stable quadratic solution
        let sqrtDelta = sqrt(delta)
        let q = (b > 0.0) ? -0.5 * (b + sqrtDelta) : -0.5 * (b - sqrtDelta)

        // Two roots
        var t0 = q / a
        var t1 = c / q   // q != 0.0 because delta > 0

        // Ensure t0 <= t1
        if t0 > t1 { swap(&t0, &t1) }

        // Pick the nearest positive root above tMin
        let tHit = (t0 > 1e-6) ? t0 : ((t1 > 1e-6) ? t1 : .infinity)
        if tHit.isFinite {
            ray.t = tHit
            return true
        }
        return false
    }

    public func shadowIntersect(ray: inout Ray, distance: Double, backfaceCulling: Bool = false) -> Bool {
        var r = ray // don’t clobber caller’s t unless we hit and are within range
        if intersect(ray: &r), r.t < distance {
            ray.t = r.t
            return true
        }
        return false
    }

    public func getNormal(point: Vec3) -> Vec3 {
        simd_normalize((-center + point) / radius)
    }
}
