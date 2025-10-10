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
@preconcurrency import ParsingKit
#if canImport(SwiftUI)
import SwiftUI
#endif

public typealias Scene = ParsingKit.Scene
public typealias Material = ParsingKit.Material

public final class RayTracerEngine: @unchecked Sendable {
    // Optional: if you initialize with a scene upfront
    private var renderer: Renderer?
    private var scene: Scene?

    public init(from url : URL? = nil, data: Data? = nil) {
        do {
            if let url = url {
                scene = try SceneLoader.load(.url(url))
            } else if let data = data {
                scene = try SceneLoader.load(.data(data, format: .auto))
            }
        } catch {
            print("Error decoding scene:", error)
        }

        if let scene {
            let rtContext = RTContext(scene: scene)
            self.renderer = Renderer(ctx: rtContext)
        } else {
            self.renderer = nil
        }
    }

    public func inspect(scene: Scene, format: SceneFormat) throws -> SceneInfo {
        let cams = scene.cameras
        let camSpecs: [CameraSpec] = cams.enumerated().map { idx, c in
            CameraSpec(index: idx, id: c.id, imageName: c.imageName, width: c.imageResolution.0, height: c.imageResolution.1)
        }
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)
        return SceneInfo(cameras: camSpecs, meshes: meshCount, triangles: triCount, spheres: sphereCount)
    }

    public func renderAll(
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> [RenderResult] {
        guard let scene else { return [] }
        let cams = scene.cameras
        var results: [RenderResult] = []
        results.reserveCapacity(cams.count)

        for (idx, _) in cams.enumerated() {
            let ok = progress?(RenderProgress(
                Double(idx) / Double(max(cams.count, 1)),
                message: "Camera \(idx+1)/\(cams.count)"
            )) ?? true
            if !ok { break }

            let (rgba, stats, spec) = try await renderRGBA8Async(cameraIndex: idx) { p in
                let camFrac = (Double(idx) + p.fraction) / Double(max(cams.count, 1))
                return progress?(RenderProgress(camFrac, message: p.message)) ?? true
            }

            let cg = try makeCGImage(width: spec.width, height: spec.height, rgba: rgba)
            let name = (spec.imageName?.isEmpty == false) ? spec.imageName! : "Camera-\(spec.index)"
            results.append(RenderResult(fileName: name, image: cg, camera: spec, stats: stats))
        }
        _ = progress?(RenderProgress(1.0, message: "Done"))
        return results
    }

    public func renderCGImage(scene: Scene,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage {
        let (rgba, _, camSpec) = try await renderRGBA8Async(cameraIndex: 0) { rp in
            progress?(Float(rp.fraction))
            return true
        }
        return try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
    }

    public func render(
        format: SceneFormat,
        cameraIndex: Int,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> RenderResult {
        guard let scene else { throw NSError(domain: "Render", code: -20, userInfo: [NSLocalizedDescriptionKey: "No scene loaded. Can't render."]) }
        let (rgba, stats, camSpec) = try await renderRGBA8Async(cameraIndex: cameraIndex, progress: progress)
        let cg = try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
        return RenderResult(fileName: scene.cameras[cameraIndex].imageName, image: cg, camera: camSpec, stats: stats)
    }
}

extension RayTracerEngine {
    // NOTE: not @MainActor; heavy work happens off the main thread.
    private func renderRGBA8Async(
        cameraIndex: Int,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> ([UInt8], RenderStats, CameraSpec) {
        guard let scene else { throw NSError(domain: "Render", code: -20, userInfo: [NSLocalizedDescriptionKey: "No scene loaded. Can't render."]) }
        // 1) choose camera + resolution
        let cams = scene.cameras
        guard cameraIndex >= 0 && cameraIndex < cams.count else {
            throw NSError(domain: "Ray", code: -10, userInfo: [NSLocalizedDescriptionKey: "Invalid camera index"])
        }
        let cam = cams[cameraIndex]
        let width = cam.imageResolution.0
        let height = cam.imageResolution.1
        let cameraSpec = CameraSpec(index: cameraIndex, id: cam.id, imageName: cam.imageName, width: width, height: height)

        // 2) static counts once (for stats)
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)

        let started = DispatchTime.now()

        // Build a fresh renderer for THIS scene (context stays immutable & thread-safe)
        let localRenderer = Renderer(ctx: RTContext(scene: scene))

        var rays: Int64 = 0
        let linear = try await localRenderer.render(
            scene: scene,
            cameraIndex: cameraIndex,
            progressRow: { y in
                if let progress {
                    let cont = progress(RenderProgress(
                        Double(y + 1) / Double(max(height, 1)),
                        message: "Row \(y + 1)/\(height)"
                    ))
                    if !cont {
                        withUnsafeCurrentTask { $0?.cancel() } // cooperative cancel
                    }
                }
            },
            raysTraced: &rays
        )

        // Pack to RGBA8 (kept same policy you had)
        let n = linear.count
        var rgba = [UInt8](repeating: 0, count: width * height * 4)
        for pix in 0..<n {
            let c = min(max(linear[pix], Vec3.zero), Vec3(255, 255, 255))
            let base = pix * 4
            rgba[base + 0] = UInt8(c.x)
            rgba[base + 1] = UInt8(c.y)
            rgba[base + 2] = UInt8(c.z)
            rgba[base + 3] = 255
        }

        let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - started.uptimeNanoseconds) / 1_000_000.0)
        let stats = RenderStats(meshes: meshCount, triangles: triCount, spheres: sphereCount, rays: rays, milliseconds: elapsedMs)
        return (rgba, stats, cameraSpec)
    }

    public func sceneMeshAndTriangleCounts(_ scene: Scene) -> (meshes: Int, tris: Int, spheres: Int) {
        var meshes = 0, spheres = 0, tris = 0
        let objs: [SceneObject] = scene.objects
        for obj in objs {
            switch obj {
            case _ as Sphere:
                spheres += 1
            case let m as Mesh:
                meshes += 1
                tris += m.faces.data.count
            case _ as Triangle:
                tris += 1
            default: break
            }
        }
        return (meshes, tris, spheres)
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
