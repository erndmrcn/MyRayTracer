//
//  Triangle.swift
//  RayTracer
//
//  Created by Eren Demircan on 30.09.2025.
//

import ParsingKit
import simd

public struct Triangle: Decodable {
    public let id: String?
    public let indices: (Int, Int, Int) /// 1-based indices as in your JSON
    public let materialID: String?
    public var material: Material?
    public var vertices: [Vec3] = []

    @inline(__always) public var e1: Vec3 {
        vertices[1] - vertices[0]
    }

    @inline(__always) public var e2: Vec3 {
        vertices[2] - vertices[0]
    }

    enum CodingKeys: String, CodingKey {
        case id = "_id"
        case indices = "Indices"
        case materialID = "Material"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try? c.decode(String.self, forKey: .id)
        let ints = try c.pkVectorStrings(forKey: .indices).compactMap(Int.init)
        guard ints.count == 3 else { throw PKDecodingError.expectedVector("Triangle.Indices must have 3 ints") }
        indices = (ints[0], ints[1], ints[2])
        materialID = try? c.pkString(forKey: .materialID)
    }
}

extension Triangle: Object {
    
    public var materialIdx: Int {
        if let id = materialID, let idx = Int(id) {
            return idx - 1
        }

        return -1
    }

    @inlinable
    public func intersect(ray: inout Ray, backfaceCulling: Bool = false) -> Bool {
        let pvec = cross(ray.direction, e2)
        let det = dot(e1, pvec)

        if backfaceCulling {
            if det <= 1e-6 { return false }
        } else {
            if abs(det) < 1e-6 { return false }
        }

        let invDet: Double = 1.0 / det
        let tvec = ray.origin - vertices[0]

        let u = dot(tvec, pvec) * invDet
        if (u < 0 || u > 1) { return false }

        let qvec = simd_cross(tvec, e1)
        let v = dot(ray.direction, qvec) * invDet

        if (v < 0 || u + v > 1) { return false }

        let t = dot(e2, qvec) * invDet
        if t <= 1e-6 { return false }

        ray.t = t
        return true
    }

    @inlinable
    public func shadowIntersect(ray: inout Ray, distance: Double, backfaceCulling: Bool = false) -> Bool {
        var r = ray
        guard intersect(ray: &r, backfaceCulling: backfaceCulling) else { return false }
        return r.t > 1e-6 && r.t < distance
    }

    public func getNormal(point: Vec3) -> Vec3 {
        let vertex1 = vertices[0]
        let vertex2 = vertices[1]
        let vertex3 = vertices[2]
        
        let result: Vec3 = simd_cross(vertex2 - vertex1, vertex3 - vertex1)
        return simd_normalize(result)
    }
}
