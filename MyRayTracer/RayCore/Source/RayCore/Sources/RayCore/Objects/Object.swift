//
//  Object.swift
//  RayTracer
//
//  Created by Eren Demircan on 30.09.2025.
//

public protocol Object: Decodable {
    @inlinable func intersect(ray: inout Ray, backfaceCulling: Bool) -> Bool
    @inlinable func shadowIntersect(ray: inout Ray, distance: Double, backfaceCulling: Bool) -> Bool
    @inlinable func getNormal(point: Vec3) -> Vec3
    
    var materialIdx: Int { get }
    var material: Material? { get }
}

public struct Objects: Decodable {
    public var triangles: [Triangle]
    public var spheres: [Sphere]

    public var objects: [Object] {
        triangles + spheres
    }

    enum CodingKeys: String, CodingKey {
        case triangles = "Triangle"
        case spheres = "Sphere"
    }
}
