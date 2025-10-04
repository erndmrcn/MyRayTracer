//
//  RenderProgress.swift
//  RayTracer
//
//  Created by Eren Demircan on 3.10.2025.
//

public struct RenderProgress: Sendable {
    public let fraction: Double     // 0...1
    public let message: String?
    public init(_ fraction: Double, message: String? = nil) {
        self.fraction = fraction; self.message = message
    }
}
