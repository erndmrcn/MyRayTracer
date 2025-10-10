//
//  RenderStats.swift
//  RayTracer
//
//  Created by Eren Demircan on 3.10.2025.
//

public struct RenderStats: Sendable, Codable {
    public let meshes: Int
    public let triangles: Int
    public let spheres: Int
    public let planes: Int
    public let rays: Int64
    public let milliseconds: Int

    public init(meshes: Int, triangles: Int, spheres: Int, planes: Int, rays: Int64, milliseconds: Int) {
        self.meshes = meshes
        self.triangles = triangles
        self.spheres = spheres
        self.planes = planes
        self.rays = rays
        self.milliseconds = milliseconds
    }
}
