//
//  RenderConfig.swift
//  RayTracer
//
//  Created by Eren Demircan on 3.10.2025.
//

public struct RenderConfig: Sendable {
    public var exposure: Float
    public var backfaceCulling: Bool
    public var ssp: Int
    public init(exposure: Float = 1.0, backfaceCulling: Bool = true, ssp: Int = 1) {
        self.exposure = exposure
        self.backfaceCulling = backfaceCulling
        self.ssp = ssp
    }
}

public struct CameraSpec: Sendable {
    public let index: Int
    public let id: String?
    public let imageName: String?
    public let width: Int
    public let height: Int
}

public struct SceneInfo: Sendable {
    public let cameras: [CameraSpec]
    public let meshes: Int
    public let triangles: Int
    public let spheres: Int
}
