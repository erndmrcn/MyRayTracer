//
//  Light.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

import ParsingKit
import simd

public struct Lights: Decodable {
    @FlexibleVec3<Vec3> public var ambientLight: Vec3
    public let pointLights: [PointLight]

    enum CodingKeys: String, CodingKey {
        case ambientLight = "AmbientLight"
        case pointLights = "PointLight"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        _ambientLight = (try? .init(from: c.superDecoder(forKey: .ambientLight))) ?? FlexibleVec3<Vec3>(wrappedValue: .init(0,0,0))
        _ambientLight.wrappedValue = .init(x: _ambientLight.wrappedValue.x / 255.0, y: _ambientLight.wrappedValue.y / 255.0, z: _ambientLight.wrappedValue.z / 255.0)

        if var arr = try? c.decode(Array<PointLight>.self, forKey: .pointLights) {
            for idx in 0..<arr.count {
                arr[idx].intensity /= 255.0
            }
            pointLights = arr
        } else if var single = try? c.decode(PointLight.self, forKey: .pointLights) {
            single.intensity /= 255.0
            pointLights = [single]
        } else {
            pointLights = []
        }
    }
}

public struct PointLight: Decodable {
    public let id: String?
    @FlexibleVec3<Vec3> public var position: Vec3
    @FlexibleVec3<Vec3> public var intensity: Vec3

    enum CodingKeys: String, CodingKey {
        case id = "_id"
        case position = "Position"
        case intensity = "Intensity"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try? c.decode(String.self, forKey: .id)
        _position = try .init(from: c.superDecoder(forKey: .position))
        _intensity = try .init(from: c.superDecoder(forKey: .intensity))
    }

    @inline(__always)
        private func safeNormalize(_ v: Vec3) -> Vec3 {
            let len = simd_length(v)
            return len > 0 ? v / len : Vec3(0,0,0)
        }

    /// Combined (diffuse + specular) Blinn–Phong
    func shade(material: Material,
               ray: Ray,
               normal nIn: Vec3,
               at point: Vec3,
               twoSided: Bool = false,
               attenuateInverseSquare: Bool = true,
               shadowed: Bool = false) -> Vec3
    {
        if shadowed { return .zero }

        // View direction (toward camera)
        let v = safeNormalize(-ray.direction)

        // Optionally flip normal for two-sided shading
        var n = nIn
        if twoSided, simd_dot(n, v) < 0 { n = -n }

        // Light direction (toward light) and distance^2
        let L = position - point
        let dist2 = max(simd_length_squared(L), 1e-12)
        let l = safeNormalize(L)

        // Cosine term
        let nDotL = max(0.0, simd_dot(n, l))
        if nDotL == 0 { return .zero }

        // Attenuation
        let att: Double = attenuateInverseSquare ? (1.0 / dist2) : 1.0

        // ----- Diffuse (Lambert) -----
        let radiance = intensity * att
        var color = radiance * nDotL * material.diffuse

        // ----- Specular (Blinn–Phong) -----
        let h = safeNormalize(l + v)
        let nDotH = max(0.0, simd_dot(n, h))
        let spec = pow(nDotH, material.phongExponent)
        color += radiance * spec * material.specular

        return color
    }

    func diffuse(material: Material, at point: Vec3, normal nIn: Vec3,
                     attenuateInverseSquare: Bool = true) -> Vec3
        {
            let L = position - point
            let dist2 = max(simd_length_squared(L), 1e-12)
            let l = simd_normalize(L)
            let nDotL = max(0.0, simd_dot(nIn, l))
            let att: Double = attenuateInverseSquare ? (1.0 / dist2) : 1.0
            return intensity * att * nDotL * material.diffuse
        }

    func specular(material: Material, viewDir v: Vec3, at point: Vec3, normal nIn: Vec3,
                  attenuateInverseSquare: Bool = true) -> Vec3
    {
        let L = position - point
        let dist2 = max(simd_length_squared(L), 1e-12)
        let l = simd_normalize(L)
        let nDotL = max(0.0, simd_dot(nIn, l))
        if nDotL == 0 { return .zero }  // avoid backface highlights

        let att: Double = attenuateInverseSquare ? (1.0 / dist2) : 1.0
        let h = simd_normalize(l + v)
        let nDotH = max(0.0, simd_dot(nIn, h))
        let spec = pow(nDotH, material.phongExponent)
        return intensity * att * spec * material.specular
    }

}
