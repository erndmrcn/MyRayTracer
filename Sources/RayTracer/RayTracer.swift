//
//  RayTracer.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

import Foundation
import simd
import CoreGraphics
import ImageIO
import UniformTypeIdentifiers
import CoreImage
import ParsingKit
#if canImport(SwiftUI)
import SwiftUI
#endif

public struct RenderConfig: Sendable {
    public var width: Int
    public var height: Int
    public var maxDepth: Int
    public var exposure: Float   // simple tone mapping knob (linear -> reinhard)
    public init(width: Int, height: Int, maxDepth: Int = 6, exposure: Float = 1.0) {
        self.width = width; self.height = height; self.maxDepth = maxDepth; self.exposure = exposure
    }
}

public final class RayTracerEngine: @unchecked Sendable {
    public init() {}

    // MARK: Entry points

    /// Render from raw JSON data (rootKey defaults to "Scene")
    public func renderCGImage(from jsonData: Data,
                              rootKey: String = "Scene",
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage
    {
        let scene = try await decodeScene(jsonData, rootKey: rootKey)
        return try await renderCGImage(scene: scene, config: config, progress: progress)
    }

    /// Render from file URL
    public func renderCGImage(from url: URL,
                              rootKey: String = "Scene",
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage
    {
        let data = try Data(contentsOf: url)
        return try await renderCGImage(from: data, rootKey: rootKey, config: config, progress: progress)
    }

    /// Main async renderer to CGImage
    public func renderCGImage(scene: Scene,
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage
    {
        // Heavy work off the main thread
        let rgba = try await renderRGBA8Async(scene: scene, config: config, progress: progress)
        return try makeCGImage(width: config.width, height: config.height, rgba: rgba)
    }

    /// Convenience wrappers
    public func renderCIImage(from url: URL,
                              rootKey: String = "Scene",
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CIImage
    {
        let cg = try await renderCGImage(from: url, rootKey: rootKey, config: config, progress: progress)
        return CIImage(cgImage: cg)
    }

    #if canImport(SwiftUI)
    public func renderSwiftUIImage(from url: URL,
                                   rootKey: String = "Scene",
                                   config: RenderConfig,
                                   progress: (@Sendable (Float) -> Void)? = nil) async throws -> Image
    {
        let cg = try await renderCGImage(from: url, rootKey: rootKey, config: config, progress: progress)
        return Image(decorative: cg, scale: 1, orientation: .up)
    }
    #endif

    // MARK: Core sync path you already have (linear RGB)
    /// Your existing *synchronous* core. Keep pure (no I/O, no UI).
    /// Implement the full render loop here.
    public func render(scene: Scene, config: RenderConfig,
                       progressRow: ((Int) -> Void)? = nil) -> [SIMD3<Float>]
    {
        // … your current render loop …
        // Call progressRow?(y) at the end of each scanline if you want progress.
        return [] // ← replace with actual pixels
    }
}

// MARK: - Async glue + helpers

extension RayTracerEngine {
    private func renderRGBA8Async(scene: Scene,
                                  config: RenderConfig,
                                  progress: (@Sendable (Float) -> Void)? = nil) async throws -> [UInt8]
    {
        try await withTaskCancellationHandler(operation: {
            try await Task.detached(priority: .userInitiated) { [config] in
                try Task.checkCancellation()

                // Synchronous linear render (Float RGB)
                var rgba = [UInt8](repeating: 0, count: config.width * config.height * 4)
                let linear = self.render(scene: scene, config: config) { y in
                    if let progress {
                        let p = Float(y + 1) / Float(max(config.height, 1))
                        DispatchQueue.main.async { progress(p) }
                    }
                    try? Task.checkCancellation()
                }

                // Tone map + sRGB encode into 8-bit
                var p = 0
                for c in linear {
//                    let mapped = reinhard(c * config.exposure) // simple tone-map
//                    let sr = srgb(mapped.x), sg = srgb(mapped.y), sb = srgb(mapped.z)
                    rgba[p+0] = UInt8(clamp01(c.x) * 255)
                    rgba[p+1] = UInt8(clamp01(c.y) * 255)
                    rgba[p+2] = UInt8(clamp01(c.z) * 255)
                    rgba[p+3] = 255
                    p += 4
                }
                return rgba
            }.value
        }) { }
    }

    private func decodeScene(_ data: Data, rootKey: String) async throws -> Scene {
        try await Task.detached {
            // If you use ParsingKit:
            // try ParsingKit.decode(Scene.self, from: data, rootKey: rootKey)
            // Replace with your decoder:
            return try ParsingKit.decode(Scene.self, from: data, rootKey: rootKey)
        }.value
    }
}

// MARK: - Image construction

@inline(__always) private func makeCGImage(width: Int, height: Int, rgba: [UInt8]) throws -> CGImage {
    let bytesPerPixel = 4, bitsPerComponent = 8, bytesPerRow = width * bytesPerPixel
    let cs = CGColorSpaceCreateDeviceRGB()
    let data = Data(rgba)
    guard let provider = CGDataProvider(data: data as CFData) else {
        throw NSError(domain: "RayCore.CG", code: -1, userInfo: [NSLocalizedDescriptionKey: "CGDataProvider failed"])
    }
    guard let img = CGImage(width: width, height: height,
                            bitsPerComponent: bitsPerComponent, bitsPerPixel: 32,
                            bytesPerRow: bytesPerRow, space: cs,
                            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                            provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent) else {
        throw NSError(domain: "RayCore.CG", code: -2, userInfo: [NSLocalizedDescriptionKey: "CGImage creation failed"])
    }
    return img
}

// MARK: - Color helpers (tone map + sRGB)

@inline(__always) private func clamp01(_ x: Float) -> Float { max(0, min(1, x)) }

@inline(__always) private func srgb(_ x: Float) -> Float {
    return x <= 0.0031308 ? 12.92 * x : 1.055 * powf(x, 1/2.4) - 0.055
}

@inline(__always) private func reinhard(_ c: SIMD3<Float>) -> SIMD3<Float> {
    // simple Reinhard: L / (1+L)
    let one = SIMD3<Float>(repeating: 1)
    return c / (one + c)
}
