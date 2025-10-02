//
//  RayTracer.swift
//  ray-tracer
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

    // MARK: - Core sync path
    public func render(scene: Scene,
                       config: RenderConfig,
                       progressRow: ((Int) -> Void)? = nil) -> [SIMD3<Float>] {

        // Use the first camera (you can iterate in your CLI/app if you want multiple)
        let cam = scene.cameras.cameras[0]

        let width  = cam.imageResolution[0]
        let height = cam.imageResolution[1]
        precondition(width == config.width && height == config.height,
                     "RenderConfig size must match scene camera resolution")

        typealias Vec3 = SIMD3<Double>

        // --- Camera basis (RIGHT-HANDED) ---
        let w: Vec3 = normalize(-cam.gaze)
        let v: Vec3 = normalize(cam.up)
        let u: Vec3 = normalize(simd_cross(v, w))

        let e  = cam.position
        let nd = Double(cam.nearDistance)
        let l  = Double(cam.nearPlane[0])
        let r  = Double(cam.nearPlane[1])
        let b  = Double(cam.nearPlane[2])
        let t  = Double(cam.nearPlane[3])

        // Output buffer (linear 0..1 RGB as Float)
        var out = [SIMD3<Float>](repeating: .zero, count: width * height)

        // Precompute deltas for raster
        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)

        // Main loop (scanlines outer so we can report progress per row)
        for j in 0..<height {
            // Near plane geometry
            let m: Vec3 = e + (-w) * nd           // center of near plane
            let q0: Vec3 = m + u * l + v * t      // top-left corner

            for i in 0..<width {
                // Pixel position on near plane
                let s_u = (Double(i) + 0.5) * du
                let s_v = (Double(j) + 0.5) * dv
                let s   = q0 + s_u * u - s_v * v

                // Primary ray
                var ray = Ray(origin: e, direction: s - e, t: 0)

                // Closest hit search
                var tMin = Double.infinity
                var hitObject: Object?

                for obj in scene.objects.objects {
                    if obj.intersect(ray: &ray, backfaceCulling: false), ray.t < tMin {
                        tMin = ray.t
                        hitObject = obj
                    }
                }

                let idx = j * width + i

                if let object = hitObject {
                    // Intersection point (IMPORTANT: same, unnormalized ray dir)
                    let p = ray.origin + ray.direction * tMin

                    // Start with ambient
                    var pixel = scene.lights.ambientLight * scene.materials.materials[object.materialIdx].ambient

                    // Shadows + direct lighting
                    for light in scene.lights.pointLights {
                        // Shadow ray
                        var wi = light.position - p
                        let dist = simd_length(wi)
                        wi = simd_normalize(wi)

                        let nGeom = shadedNormalAt(p, for: object)
                        let shadowOrigin = p + nGeom * scene.shadowRayEpsilon
                        var shadowRay = Ray(origin: shadowOrigin, direction: wi, t: 0)

                        var occluded = false
                        for blocker in scene.objects.objects {
                            if blocker.shadowIntersect(ray: &shadowRay, distance: dist, backfaceCulling: false) {
                                occluded = true
                                break
                            }
                        }
                        if occluded { continue }

                        // Add Blinn–Phong (your light.shade)
                        let mat = scene.materials.materials[object.materialIdx]
                        pixel += light.shade(material: mat,
                                             ray: ray,
                                             normal: shadedNormalAt(p, for: object),
                                             at: p)
                    }

                    // Clamp to [0,1] as in your main.swift (then we convert to Float)
                    var clamped = pixel
                    clamp01InPlace(&clamped)
                    out[idx] = SIMD3<Float>(Float(clamped.x), Float(clamped.y), Float(clamped.z))
                } else {
                    // Miss → background
                    let bg = scene.backgroundColor
                    out[idx] = SIMD3<Float>(Float(bg.x), Float(bg.y), Float(bg.z))
                }
            }

            // Progress per scanline
            progressRow?(j)
        }

        return out
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
//                    let sr = srgb(c.x), sg = srgb(c.y), sb = srgb(c.z)
                    rgba[p+0] = UInt8(clamp01(c.x) * 255)
                    rgba[p+1] = UInt8(clamp01(c.y) * 255)
                    rgba[p+2] = UInt8(clamp01(c.z) * 255)
                    rgba[p+3] = 255
                    p += 4
                }
                return rgba
            }.value
        }) {
            // on cancel
        }
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

// Uses your existing Object protocol + Sphere/Triangle types
@inline(__always)
private func shadedNormalAt(_ point: SIMD3<Double>, for object: Object) -> SIMD3<Double> {
    switch object {
    case let sphere as Sphere:
        return normalize(point - sphere.center)
    case let tri as Triangle:
        // flat shading (you can extend to smooth with barycentrics)
        return normalize(cross(tri.e1, tri.e2))
    default:
        return SIMD3<Double>(0,0,0)
    }
}

@inline(__always)
private func clamp01InPlace(_ c: inout SIMD3<Double>) {
    c.x = max(0.0, min(1.0, c.x))
    c.y = max(0.0, min(1.0, c.y))
    c.z = max(0.0, min(1.0, c.z))
}
