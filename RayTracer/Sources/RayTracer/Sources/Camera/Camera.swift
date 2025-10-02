//
//  Untitled.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

@preconcurrency import ParsingKit

public struct Cameras: Sendable, Decodable {
    public let cameras: [Camera]

    enum CodingKeys: String, CodingKey { case camera = "Camera" }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        if let arr = try? c.decode(Array<Camera>.self, forKey: .camera) {
            self.cameras = arr
        } else if let cam = try? c.decode(Camera.self, forKey: .camera) {
            self.cameras = [cam]
        } else {
            throw DecodingError.dataCorrupted(.init(codingPath: c.codingPath, debugDescription: "No camera found"))
        }
    }
}

public struct Camera: Sendable, Decodable {
    public let id: String?
    @FlexibleVec3<Vec3> public var position: Vec3
    @FlexibleVec3<Vec3> public var gaze: Vec3
    @FlexibleVec3<Vec3> public var up: Vec3
    @FlexibleDouble public var nearDistance: Double

    public let nearPlane: [Double]
    public let imageResolution: [Int]
    public let imageName: String

    enum CodingKeys: String, CodingKey {
        case id = "_id"
        case position = "Position"
        case gaze = "Gaze"
        case up = "Up"
        case nearPlane = "NearPlane"
        case nearDistance = "NearDistance"
        case imageResolution = "ImageResolution"
        case imageName = "ImageName"
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try? c.decode(String.self, forKey: .id)
        _position = try .init(from: c.superDecoder(forKey: .position))
        _gaze = try .init(from: c.superDecoder(forKey: .gaze))
        _up = try .init(from: c.superDecoder(forKey: .up))
        nearPlane = try c.pkVectorStrings(forKey: .nearPlane).compactMap(Double.init)
        _nearDistance = try .init(from: c.superDecoder(forKey: .nearDistance))
        imageResolution = try c.pkVectorStrings(forKey: .imageResolution).compactMap(Int.init)
        imageName = try c.decode(String.self, forKey: .imageName)
    }
}
