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

public final class RayTracerEngine: @unchecked Sendable {
    public init() {}

    public func inspect(sceneData: Data, format: SceneFormat) throws -> SceneInfo {
        let scene = try decodeScene(sceneData, format: format)
        let cams = scene.cameras.cameras
        let camSpecs: [CameraSpec] = cams.enumerated().map { idx, c in
            CameraSpec(index: idx, id: c.id, imageName: c.imageName, width: c.imageResolution[0], height: c.imageResolution[1])
        }
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)
        return SceneInfo(cameras: camSpecs, meshes: meshCount, triangles: triCount, spheres: sphereCount)
    }

    public func renderAll(
        sceneData: Data,
        format: SceneFormat,
        rootKey: String? = nil,
        config: RenderConfig,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> [RenderResult] {
        let scene = try decodeScene(sceneData, format: format, rootKey: rootKey)
        let cams = scene.cameras.cameras
        var results: [RenderResult] = []
        results.reserveCapacity(cams.count)

        for (idx, _) in cams.enumerated() {
            let ok = progress?(RenderProgress(
                Double(idx) / Double(max(cams.count, 1)),
                message: "Camera \(idx+1)/\(cams.count)"
            )) ?? true
            if !ok { break }

            let (rgba, stats, spec) = try await renderRGBA8Async(scene: scene, cameraIndex: idx, config: config) { p in
                let camFrac = (Double(idx) + p.fraction) / Double(max(cams.count, 1))
                return progress?(RenderProgress(camFrac, message: p.message)) ?? true
            }

            let cg = try makeCGImage(width: spec.width, height: spec.height, rgba: rgba)
            let name = (spec.imageName?.isEmpty == false) ? spec.imageName! : "Camera-\(spec.index)"
            results.append(RenderResult(fileName: name, image: cg, camera: spec, stats: stats))
        }
        // Final 100%
        _ = progress?(RenderProgress(1.0, message: "Done"))
        return results
    }

    /// Render from raw JSON data (rootKey defaults to "Scene")
    public func renderCGImage(from jsonData: Data,
                              rootKey: String = "Scene",
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage
    {
        let scene = try await decodeScene(jsonData, format: .auto, rootKey: rootKey)
        return try await renderCGImage(scene: scene, config: config, progress: progress)
    }

    /// Render from file URL
    public func renderCGImage(from url: URL,
                              rootKey: String = "Scene",
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage
    {
        let data = try Data(contentsOf: url)
        return try await renderCGImage(from: data, config: config, progress: progress)
    }

    public func renderCGImage(scene: Scene,
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage {
        let (rgba, _, camSpec) = try await renderRGBA8Async(scene: scene, cameraIndex: 0, config: config) { rp in
            progress?(Float(rp.fraction))
            return true
        }
        return try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
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

    public func render(
        sceneData: Data,
        format: SceneFormat,
        cameraIndex: Int,
        config: RenderConfig,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> RenderResult {
        let scene = try decodeScene(sceneData, format: format)
        let (rgba, stats, camSpec) = try await renderRGBA8Async(scene: scene, cameraIndex: cameraIndex, config: config, progress: progress)
        let cg = try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
        return RenderResult(fileName: scene.cameras.cameras[cameraIndex].imageName, image: cg, camera: camSpec, stats: stats)
    }


    @inline(__always)
    private func sniffFormat(_ data: Data) -> SceneFormat {
        guard let s = String(data: data.prefix(64), encoding: .utf8)?
            .trimmingCharacters(in: .whitespacesAndNewlines) else { return .json }
        if s.hasPrefix("<") { return .xml }
        return .json
    }

    private func decodeScene(_ data: Data, format: SceneFormat, rootKey: String) async throws -> Scene {
        try await Task.detached {
            switch format {
            case .json, .auto:
                return try ParsingKit.decode(Scene.self, from: data, rootKey: rootKey)
            case .xml:
                // If you already parse XML with ParsingKit, call that here.
                // Otherwise, throw for now (app shows error).
                throw NSError(domain: "RayTracer.Decode", code: -10,
                              userInfo: [NSLocalizedDescriptionKey: "XML not implemented yet"])
            }
        }.value
    }

    public func render(
        scene: Scene,
        cameraIndex: Int = 0,
        config: RenderConfig,
        progressRow: ((Int) -> Void)? = nil,
        raysTraced: inout Int64
    ) -> [Vec3] {

        let cam = scene.cameras.cameras[cameraIndex]
        let width  = cam.imageResolution[0]
        let height = cam.imageResolution[1]

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
        var out = [Vec3](repeating: .zero, count: width * height)

        // Precompute deltas for raster
        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)

        // Main loop (scanlines outer so we can report progress per row)
        for j in 0..<height {
            if Task.isCancelled { break }

            // Near plane geometry
            let m: Vec3 = e + (-w) * nd           // center of near plane
            let q0: Vec3 = m + u * l + v * t      // top-left corner

            for i in 0..<width {
                if Task.isCancelled { break }

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
                    out[idx] = Vec3(x: clamped.x, y: clamped.y, z: clamped.z)
                } else {
                    // Miss → background
                    let bg = scene.backgroundColor
                    out[idx] = Vec3(x: bg.x, y: bg.y, z: bg.z)
                }
            }

            // Progress per scanline
            progressRow?(j)

            if Task.isCancelled { break }
        }
        return out
    }
}

// MARK: - Async glue + helpers
extension RayTracerEngine {
    private func renderRGBA8Async(
        scene: Scene,
        cameraIndex: Int,
        config: RenderConfig,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> ([UInt8], RenderStats, CameraSpec) {

        // 1) choose camera + resolution
        let cams = scene.cameras.cameras
        guard cameraIndex >= 0 && cameraIndex < cams.count else {
            throw NSError(domain: "Ray", code: -10, userInfo: [NSLocalizedDescriptionKey: "Invalid camera index"])
        }
        let cam = cams[cameraIndex]
        let width = cam.imageResolution[0]
        let height = cam.imageResolution[1]
        let cameraSpec = CameraSpec(index: cameraIndex, id: cam.id, imageName: cam.imageName, width: width, height: height)

        // 2) static counts once
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)

        let started = DispatchTime.now()
        return try await withTaskCancellationHandler(operation: {
            try await Task.detached(priority: .userInitiated) {
                try Task.checkCancellation()

                var rays: Int64 = 0
                // Your synchronous renderer MUST use 'width/height' from 'cam'
                let linear = self.render(scene: scene,
                                         cameraIndex: cameraIndex,     // pass camera or cam fields
                                         config: config,
                                         progressRow: { y in
                                            if let progress {
                                                let cont = progress(RenderProgress(
                                                    Double(y + 1) / Double(max(height, 1)),
                                                    message: "Row \(y + 1)/\(height)"
                                                ))
                                                if !cont { withUnsafeCurrentTask { $0?.cancel() } }
                                            }
                                         },
                                         raysTraced: &rays)

                try Task.checkCancellation()

                // Pack RGBA8 (size = width * height * 4)
                var rgba = [UInt8](repeating: 0, count: width * height * 4)
                var p = 0
                for c in linear {
                    rgba[p+0] = UInt8(clamp01(c.x) * 255)
                    rgba[p+1] = UInt8(clamp01(c.y) * 255)
                    rgba[p+2] = UInt8(clamp01(c.z) * 255)
                    rgba[p+3] = 255
                    p += 4
                }

                let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - started.uptimeNanoseconds) / 1_000_000.0)
                let stats = RenderStats(meshes: meshCount, triangles: triCount, spheres: sphereCount, rays: rays, milliseconds: elapsedMs)
                return (rgba, stats, cameraSpec)
            }.value
        }, onCancel: {})
    }


    private func sceneMeshAndTriangleCounts(_ scene: Scene) -> (meshes: Int, tris: Int, spheres: Int) {
        var meshes = 0, spheres = 0, tris = 0
        for obj in scene.objects.objects {
            switch obj {
                case let m as Sphere:
                spheres += 1
//            case let m as Mesh:
//                meshes += 1
//                tris += m.triangles.count
            case _ as Triangle:
                tris += 1
            default: break
            }
        }
        return (meshes, tris, spheres)
    }

    // NEW: optional rootKey everywhere. Default is auto ("Scene" if present; else first top-level key).
    private func decodeScene(_ data: Data, format: SceneFormat, rootKey: String? = nil) throws -> Scene {
        switch format {
        case .xml:
            return try ParsingKit.decode(Scene.self, from: data, rootKey: rootKey ?? "Scene")
        case .json, .auto:
            let key = rootKey ?? detectJSONRootKey(data) ?? "Scene"
            return try ParsingKit.decode(Scene.self, from: data, rootKey: key)
        }
    }

    private func detectJSONRootKey(_ data: Data) -> String? {
        guard let obj = try? JSONSerialization.jsonObject(with: data),
              let dict = obj as? [String: Any] else { return nil }
        if let exact = dict.keys.first(where: { $0 == "Scene" }) { return exact }
        if let ci = dict.keys.first(where: { $0.lowercased() == "scene" }) { return ci }
        return dict.keys.first
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
@inline(__always) private func clamp01(_ x: Double) -> Double { max(0, min(1, x)) }

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

