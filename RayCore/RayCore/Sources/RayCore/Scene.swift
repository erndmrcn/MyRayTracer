//
//  Scene.swift
//  RayTracer
//
//  Created by Eren Demircan on 30.09.2025.
//

import Foundation
@preconcurrency import ParsingKit

public struct SceneParser {
    public static func decode(from data: Data) throws -> Scene {
        let decoder = JSONDecoder()
        let root = try decoder.decode(SceneRoot.self, from: data)
        return root.scene
    }

    public static func decode(from url: URL) throws -> Scene {
        let data = try Data(contentsOf: url)
        return try decode(from: data)
    }
}

// MARK: - Root matches { "Scene": { ... } }
private struct SceneRoot: Decodable {
    let scene: Scene
    enum CodingKeys: String, CodingKey { case scene = "Scene" }
}

// MARK: - Scene Model
public struct Scene: @unchecked Sendable, Decodable {
    @FlexibleInt public var maxRecursionDepth: Int
    @FlexibleVec3<Vec3> public var backgroundColor: Vec3
    @FlexibleDouble public var shadowRayEpsilon: Double
    @FlexibleDouble public var intersectionTestEpsilon: Double

    public let cameras: Cameras
    public let lights: Lights
    public let materials: Materials
    public let vertexData: [[Double]] // each entry is [x, y, z]
    public var objects: Objects

    enum CodingKeys: String, CodingKey {
        case maxRecursionDepth = "MaxRecursionDepth"
        case backgroundColor = "BackgroundColor"
        case shadowRayEpsilon = "ShadowRayEpsilon"
        case intersectionTestEpsilon = "IntersectionTestEpsilon"
        case cameras = "Cameras"
        case lights = "Lights"
        case materials = "Materials"
        case vertexData = "VertexData"
        case objects = "Objects"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        _maxRecursionDepth = try .init(from: c.superDecoder(forKey: .maxRecursionDepth))
        _backgroundColor = try .init(from: c.superDecoder(forKey: .backgroundColor))
        _shadowRayEpsilon = try .init(from: c.superDecoder(forKey: .shadowRayEpsilon))
        _intersectionTestEpsilon = try .init(from: c.superDecoder(forKey: .intersectionTestEpsilon))

        lights = try c.decode(Lights.self, forKey: .lights)
        materials = try c.decode(Materials.self, forKey: .materials)
        cameras = try c.decode(Cameras.self, forKey: .cameras)

        let rawVertex = try c.decode(String.self, forKey: .vertexData)
        vertexData = pkParseVertexLines(rawVertex)

        objects = try c.decode(Objects.self, forKey: .objects)

        for (idx, object) in objects.triangles.enumerated() {
            objects.triangles[idx].vertices.append(Vec3(vertexData[object.indices.0 - 1]))
            objects.triangles[idx].vertices.append(Vec3(vertexData[object.indices.1 - 1]))
            objects.triangles[idx].vertices.append(Vec3(vertexData[object.indices.2 - 1]))

            objects.triangles[idx].material = materials.materials[object.materialIdx]
        }

        for (idx, sphere) in objects.spheres.enumerated() {
            if let vertexIdx = sphere.centerIndex {
                objects.spheres[idx].center = Vec3(vertexData[vertexIdx-1])
            }
            
            objects.spheres[idx].material = materials.materials[sphere.materialIdx]
        }
    }
}
