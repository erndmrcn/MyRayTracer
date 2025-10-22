//
//  RenderResult.swift
//  RayTracer
//
//  Created by Eren Demircan on 3.10.2025.
//

import CoreGraphics

public struct RenderResult: Sendable {
    public let fileName: String
    public let image: CGImage
    public let camera: CameraSpec
    public let stats: RenderStats
}
