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

// MARK: - Typealiases
public typealias Scene = ParsingKit.Scene
public typealias Material = ParsingKit.Material

// MARK: - Main Engine

public final class RayTracerEngine: @unchecked Sendable {
    private var renderer: Renderer?
    private var scene: Scene?

    // MARK: - Initialization
    public init(from url: URL? = nil, data: Data? = nil) {
        do {
            if let url {
                scene = try SceneLoader.load(.url(url))
                scene?.path = url
            } else if let data {
                scene = try SceneLoader.load(.data(data, format: .auto))
            }
        } catch {
            print("❌ Error decoding scene:", error)
        }

        if var scene {
            // ✅ Build TLAS/BLAS hierarchy (no legacy BVH)
            let rtContext = RTContext(scene: scene)
            self.renderer = Renderer(ctx: rtContext)
        } else {
            self.renderer = nil
        }
    }

    // MARK: - Scene Introspection
    public func inspect(scene: Scene, format: SceneFormat) throws -> SceneInfo {
        let cams = scene.cameras
        let camSpecs: [CameraSpec] = cams.enumerated().map { idx, c in
            CameraSpec(index: idx,
                       id: c.id,
                       imageName: c.imageName,
                       width: c.imageResolution.0,
                       height: c.imageResolution.1)
        }
        let (meshCount, triCount, sphereCount, planeCount) = sceneMeshAndTriangleCounts(scene)
        return SceneInfo(cameras: camSpecs,
                         meshes: meshCount,
                         triangles: triCount,
                         spheres: sphereCount,
                         planes: planeCount)
    }

    // MARK: - Batch Render
    public func renderAll(
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> [RenderResult] {
        guard let scene else { return [] }

        let cams = scene.cameras
        var results: [RenderResult] = []

        let totalRowsAll = max(1, cams.reduce(0) { $0 + $1.imageResolution.1 })
        var rowsOffset = 0

        print("\(cams.enumerated())")
        for (idx, cam) in cams.enumerated() {
            let height = max(1, cam.imageResolution.1)
            let baseOffset = rowsOffset
            let camCount = max(1, cams.count)

            let (rgba, stats, spec) = try await renderRGBA8Async(cameraIndex: idx) { p in
                let y = min(height - 1, max(0, Int(ceil(p.fraction * Double(height)) - 1)))
                let doneRowsGlobal = baseOffset + (y + 1)
                let globalFrac = Double(doneRowsGlobal) / Double(totalRowsAll)
                let msg = "Camera \(idx+1)/\(camCount) — Row \(y+1)/\(height)"
                return progress?(RenderProgress(globalFrac, message: msg)) ?? true
            }

            print("making CGImage")
            let cg = try makeCGImage(width: spec.width, height: spec.height, rgba: rgba)
            print("made CGImage")
            let name = (spec.imageName?.isEmpty == false) ? spec.imageName! : "Camera-\(spec.index)"
            results.append(RenderResult(fileName: name, image: cg, camera: spec, stats: stats))
            rowsOffset += height
        }

        _ = progress?(RenderProgress(1.0, message: "Done"))
        return results
    }

    // MARK: - Single Camera Render (returns CGImage)
    public func renderCGImage(scene: Scene,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage {
        let (rgba, _, camSpec) = try await renderRGBA8Async(cameraIndex: 0) { rp in
            progress?(Float(rp.fraction))
            return true
        }
        return try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
    }

    // MARK: - Single Camera Render (returns RenderResult)
    public func render(
        format: SceneFormat,
        cameraIndex: Int,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> RenderResult {
        guard let scene else {
            throw NSError(domain: "Render", code: -20,
                          userInfo: [NSLocalizedDescriptionKey: "No scene loaded. Can't render."])
        }
        let (rgba, stats, camSpec) = try await renderRGBA8Async(cameraIndex: cameraIndex,
                                                                progress: progress)
        let cg = try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
        return RenderResult(fileName: scene.cameras[cameraIndex].imageName,
                            image: cg,
                            camera: camSpec,
                            stats: stats)
    }
}

// MARK: - Rendering Core

extension RayTracerEngine {
    private func renderRGBA8Async(
        cameraIndex: Int,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> ([UInt8], RenderStats, CameraSpec) {
        guard let scene else {
            throw NSError(domain: "Render", code: -20,
                          userInfo: [NSLocalizedDescriptionKey: "No scene loaded. Can't render."])
        }
        guard let localRenderer = renderer else {
            throw NSError(domain: "Render", code: -21,
                          userInfo: [NSLocalizedDescriptionKey: "Renderer not initialized."])
        }

        let cams = scene.cameras
        guard cameraIndex >= 0 && cameraIndex < cams.count else {
            throw NSError(domain: "Ray", code: -10,
                          userInfo: [NSLocalizedDescriptionKey: "Invalid camera index"])
        }

        let cam = cams[cameraIndex]
        let width = cam.imageResolution.0
        let height = cam.imageResolution.1
        let cameraSpec = CameraSpec(index: cameraIndex,
                                    id: cam.id,
                                    imageName: cam.imageName,
                                    width: width,
                                    height: height)

        let (meshCount, triCount, sphereCount, planeCount) = sceneMeshAndTriangleCounts(scene)
        let started = DispatchTime.now()

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
                        withUnsafeCurrentTask { $0?.cancel() }
                    }
                }
            }
//            raysTraced: &rays
        )

        // Convert Vec3 color buffer → RGBA8
        var rgba = [UInt8](repeating: 0, count: width * height * 4)
        for pix in 0..<linear.count {
            let c = simd_clamp(linear[pix], Vec3.zero, Vec3(255, 255, 255))
            let base = pix * 4
            rgba[base + 0] = UInt8(c.x)
            rgba[base + 1] = UInt8(c.y)
            rgba[base + 2] = UInt8(c.z)
            rgba[base + 3] = 255
        }

        let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - started.uptimeNanoseconds) / 1_000_000.0)
        let stats = RenderStats(meshes: meshCount,
                                triangles: triCount,
                                spheres: sphereCount,
                                planes: planeCount,
                                rays: rays,
                                milliseconds: elapsedMs)
        return (rgba, stats, cameraSpec)
    }

    // MARK: - Scene Statistics Helper
    public func sceneMeshAndTriangleCounts(_ scene: Scene)
    -> (meshes: Int, tris: Int, spheres: Int, planes: Int) {
        var meshes = 0, spheres = 0, tris = 0, planes = 0
        for obj in scene.objects {
            switch obj {
            case _ as Sphere:
                spheres += 1
            case let m as Mesh:
                meshes += 1
                tris += m.faces.data.count / 3
            case _ as Triangle:
                tris += 1
            case _ as Plane:
                planes += 1
            default:
                break
            }
        }
        return (meshes, tris, spheres, planes)
    }
}

// MARK: - Image Conversion
@inline(__always)
private func makeCGImage(width: Int, height: Int, rgba: [UInt8]) throws -> CGImage {
    let bytesPerPixel = 4
    let bitsPerComponent = 8
    let bytesPerRow = width * bytesPerPixel
    let cs = CGColorSpaceCreateDeviceRGB()
    let data = Data(rgba)

    guard let provider = CGDataProvider(data: data as CFData) else {
        throw NSError(domain: "RayCore.CG", code: -1,
                      userInfo: [NSLocalizedDescriptionKey: "CGDataProvider failed"])
    }

    let maxValue = rgba.max()
    let minValue = rgba.min()

    print("max pixel value of the result: \(maxValue)")
    print("count of the max pixel value: \(rgba.count(where: { $0 == maxValue }))")
    print("min pixel value of the result: \(minValue)")
    print("count of the min pixel value: \(rgba.count(where: { $0 == minValue }))")

    guard let img = CGImage(width: width,
                            height: height,
                            bitsPerComponent: bitsPerComponent,
                            bitsPerPixel: 32,
                            bytesPerRow: bytesPerRow,
                            space: cs,
                            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                            provider: provider,
                            decode: nil,
                            shouldInterpolate: false,
                            intent: .defaultIntent)
    else {
        throw NSError(domain: "RayCore.CG", code: -2,
                      userInfo: [NSLocalizedDescriptionKey: "CGImage creation failed"])
    }

    return img
}
