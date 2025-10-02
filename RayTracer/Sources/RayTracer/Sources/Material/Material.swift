//
//  Material.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

@preconcurrency import ParsingKit

public struct Materials: Sendable, Decodable {
    public let materials: [Material]

    enum CodingKeys: String, CodingKey {
        case material = "Material"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        if let arr = try? c.decode(Array<Material>.self, forKey: .material) {
            materials = arr
        } else if let single = try? c.decode(Material.self, forKey: .material) {
            materials = [single]
        } else {
            materials = []
        }
    }
}

public struct Material: Sendable, Decodable {
    @FlexibleVec3<Vec3> public var ambient: Vec3
    @FlexibleVec3<Vec3> public var diffuse: Vec3
    @FlexibleVec3<Vec3> public var specular: Vec3
    @FlexibleVec3<Vec3> public var mirrorReflectance: Vec3 = .zero
    @FlexibleDouble public var phongExponent: Double = 0

    public let id: String?
    public let type: String?

    enum CodingKeys: String, CodingKey {
        case id = "_id"
        case type = "_type"
        case ambient = "AmbientReflectance"
        case diffuse = "DiffuseReflectance"
        case specular = "SpecularReflectance"
        case phongExponent = "PhongExponent"
        case mirrorReflectance = "MirrorReflectance"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try? c.decode(String.self, forKey: .id)
        type = try? c.decode(String.self, forKey: .id)
        _ambient = try .init(from: c.superDecoder(forKey: .ambient))
        _diffuse = try .init(from: c.superDecoder(forKey: .diffuse))
        _specular = try .init(from: c.superDecoder(forKey: .specular))

        if c.contains(.mirrorReflectance),
           let sd = try? c.superDecoder(forKey: .mirrorReflectance) {
            _mirrorReflectance = try .init(from: sd)
        } // else: leave at default

        // Same idea for phongExponent (keep default if missing)
        if c.contains(.phongExponent),
           let sd = try? c.superDecoder(forKey: .phongExponent) {
            _phongExponent = try .init(from: sd)
        }
    }
}
