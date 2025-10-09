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

import Foundation

public typealias Scene = ParsingKit.Scene
public typealias Material = ParsingKit.Material

@MainActor
public enum RTContext {
    // triangles
    static var triCount: Int = 0
    static var triIndex: UnsafePointer<Int32>!    // 3*triCount (i0,i1,i2,…)
    static var v0: UnsafePointer<Vec3>!
    static var v1: UnsafePointer<Vec3>!
    static var v2: UnsafePointer<Vec3>!
    static var e1: UnsafePointer<Vec3>!
    static var e2: UnsafePointer<Vec3>!
    static var triObj: UnsafePointer<Int32>!      // owner object id per tri
    static var triMat: UnsafePointer<Int32>!      // material id per tri

    // spheres
    static var sphCount: Int = 0
    static var sphCenter: UnsafePointer<Vec3>!
    static var sphRadius: UnsafePointer<Scalar>!
    static var sphObj: UnsafePointer<Int32>!      // owner object id per sphere
    static var sphMat: UnsafePointer<Int32>!      // material id per sphere

    static var planeCount: Int = 0
    static var planeCenter: UnsafePointer<Vec3>!
    static var planeNormal: UnsafePointer<Vec3>!
    static var planeObj: UnsafePointer<Int32>!      // owner object id per sphere
    static var planeMat: UnsafePointer<Int32>!      // material id per sphere

    // scene
    static var shadowRayEpsilon: Scalar = 0
    static var intersectionTestEpsilon: Scalar = 0
}

@MainActor
enum RuntimeStorage {
    static var positions = [Vec3]()
    static var triIdx    = [Int32]()
    static var v0 = [Vec3](),
               v1 = [Vec3](),
               v2 = [Vec3](),
               e1 = [Vec3](),
               e2 = [Vec3]()
    static var triObj = [Int32](), triMat = [Int32]()     // NEW
    static var sphCenter = [Vec3](), sphRadius = [Scalar]()
    static var sphObj = [Int32](), sphMat = [Int32]()     // NEW

    static var planeCenter = [Vec3]()
    static var planeNormal = [Vec3]()
    static var planeObj = [Int32](), planeMat = [Int32]()     // NEW

    // For non–hot-path lookups (optional):
    static var objects = [SceneObject]()                  // indexable by obj id
    static var materials = [Material]()            // indexable by mat id
    static var materialIndexById = [String: Int32]()      // "_id" -> idx
    static var objectIndexByIdentity = [ObjectIdentifier: Int32]()
}

@MainActor
func buildRuntime(from scene: Scene) {
    // Materials table (map Scene.Material._id -> index)
    RuntimeStorage.materials = scene.materials
    RTContext.intersectionTestEpsilon = scene.intersectionTestEpsilon
    RTContext.shadowRayEpsilon = scene.shadowRayEpsilon
    RuntimeStorage.materialIndexById.removeAll(keepingCapacity: true)
    for (i, m) in scene.materials.enumerated() {
        if let mid = m.id { RuntimeStorage.materialIndexById[mid] = Int32(i) }
    }

    // Objects table (stable IDs for owners)
    RuntimeStorage.objects = scene.objects
    RuntimeStorage.objectIndexByIdentity.removeAll(keepingCapacity: true)
    for (i, obj) in scene.objects.enumerated() {
        RuntimeStorage.objectIndexByIdentity[ObjectIdentifier(obj)] = Int32(i)
    }

    // Vertices
    let positions = scene.vertexData.data
    RuntimeStorage.positions = positions

    // Triangles (Triangle + Mesh.faces)
    var triIdx = [Int32]()
    var triObj = [Int32]()
    var triMat = [Int32]()
    triIdx.reserveCapacity(scene.objects.count * 6)
    triObj.reserveCapacity(scene.objects.count * 2)
    triMat.reserveCapacity(scene.objects.count * 2)

    func matIdx(_ materialId: String?) -> Int32 {
        guard let id = materialId, let mi = RuntimeStorage.materialIndexById[id] else { return -1 }
        return mi
    }

    for obj in scene.objects {
        let oid = RuntimeStorage.objectIndexByIdentity[ObjectIdentifier(obj)] ?? -1

        if let t = obj as? Triangle, t.indices.count >= 3 {
            let i0 = Int32(max(0, t.indices[0] - 1))
            let i1 = Int32(max(0, t.indices[1] - 1))
            let i2 = Int32(max(0, t.indices[2] - 1))
            triIdx += [i0, i1, i2]
            triObj.append(oid)
            triMat.append(matIdx(t.material))

        } else if let m = obj as? Mesh {
            let face = m.faces.data
            let n = (face.count / 3) * 3
            var k = 0
            while k < n {
                let i0 = Int32(max(0, face[k+0] - 1))
                let i1 = Int32(max(0, face[k+1] - 1))
                let i2 = Int32(max(0, face[k+2] - 1))
                triIdx += [i0, i1, i2]
                triObj.append(oid)
                triMat.append(matIdx(m.material))
                k += 3
            }
        }
    }

    let triCount = triIdx.count / 3

    // Precompute v0/e1/e2
    var v0 = [Vec3](repeating: .zero, count: triCount)
    var v1 = [Vec3](repeating: .zero, count: triCount)
    var v2 = [Vec3](repeating: .zero, count: triCount)
    var e1 = [Vec3](repeating: .zero, count: triCount)
    var e2 = [Vec3](repeating: .zero, count: triCount)
    for t in 0..<triCount {
        let i0 = Int(triIdx[3*t+0]), i1 = Int(triIdx[3*t+1]), i2 = Int(triIdx[3*t+2])
        let p0 = positions[i0], p1 = positions[i1], p2 = positions[i2]
        v0[t] = p0
        v1[t] = p1
        v2[t] = p2
        e1[t] = p1 - p0
        e2[t] = p2 - p0
    }

    // Spheres
    var sphC = [Vec3](), sphR = [Scalar](), sphObj = [Int32](), sphMat = [Int32]()
    var planeC = [Vec3](), planeN = [Vec3](), planeObj = [Int32](), planeMat = [Int32]()
    for (idx, obj) in scene.objects.enumerated() {
        if let s = obj as? Sphere {
            let oid = RuntimeStorage.objectIndexByIdentity[ObjectIdentifier(obj)] ?? -1
            (scene.objects[idx] as! Sphere).center = positions[s.centerIdx - 1]

            sphC.append(positions[s.centerIdx - 1])
            sphR.append(s.radius)
            sphObj.append(oid)
            sphMat.append(matIdx(s.material))
        } else if let p = obj as? Plane {
            let oid = RuntimeStorage.objectIndexByIdentity[ObjectIdentifier(obj)] ?? -1
            (scene.objects[idx] as! Plane).center = positions[p.centerIdx - 1]

            planeC.append(positions[p.centerIdx - 1])
            planeN.append(p.normal)
            planeObj.append(oid)
            planeMat.append(matIdx(p.material))
        }
    }

    // Retain & pin
    RuntimeStorage.triIdx = triIdx
    RuntimeStorage.v0 = v0
    RuntimeStorage.v1 = v1
    RuntimeStorage.v2 = v2
    RuntimeStorage.e1 = e1
    RuntimeStorage.e2 = e2
    RuntimeStorage.triObj = triObj; RuntimeStorage.triMat = triMat
    RuntimeStorage.sphCenter = sphC; RuntimeStorage.sphRadius = sphR
    RuntimeStorage.sphObj = sphObj; RuntimeStorage.sphMat = sphMat
    RuntimeStorage.planeCenter = planeC
    RuntimeStorage.planeObj = planeObj; RuntimeStorage.planeMat = planeMat
    RuntimeStorage.planeNormal = planeN

    RTContext.triCount = triCount
    RTContext.triIndex = RuntimeStorage.triIdx.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.v0 = RuntimeStorage.v0.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.v1 = RuntimeStorage.v1.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.v2 = RuntimeStorage.v2.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.e1 = RuntimeStorage.e1.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.e2 = RuntimeStorage.e2.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.triObj = RuntimeStorage.triObj.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.triMat = RuntimeStorage.triMat.withUnsafeBufferPointer { $0.baseAddress! }

    RTContext.sphCount = sphC.count
    RTContext.sphCenter = RuntimeStorage.sphCenter.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.sphRadius = RuntimeStorage.sphRadius.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.sphObj = RuntimeStorage.sphObj.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.sphMat = RuntimeStorage.sphMat.withUnsafeBufferPointer { $0.baseAddress! }

    RTContext.planeCount = planeC.count
    RTContext.planeCenter = RuntimeStorage.planeCenter.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.planeNormal = RuntimeStorage.planeNormal.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.planeObj = RuntimeStorage.planeObj.withUnsafeBufferPointer { $0.baseAddress! }
    RTContext.planeMat = RuntimeStorage.planeMat.withUnsafeBufferPointer { $0.baseAddress! }
}

@MainActor
public final class RayTracerEngine: @unchecked Sendable {
    public init() {}

    public func inspect(scene: Scene, format: SceneFormat) throws -> SceneInfo {
        let cams = scene.cameras
        let camSpecs: [CameraSpec] = cams.enumerated().map { idx, c in
            CameraSpec(index: idx, id: c.id, imageName: c.imageName, width: c.imageResolution.0, height: c.imageResolution.1)
        }
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)
        return SceneInfo(cameras: camSpecs, meshes: meshCount, triangles: triCount, spheres: sphereCount)
    }

    public func decodeScene(from url: URL? = nil, data: Data? = nil) throws -> Scene? {
        var scene: Scene?
        if let url = url {
            scene = try SceneLoader.load(.url(url))
        } else if let data = data {
            scene = try SceneLoader.load(.data(data, format: .auto))
        }

        if let scene = scene {
            Task {
                await MainActor.run {
                    buildRuntime(from: scene)
                }
            }
            return scene
        }

        return nil
    }

    public func renderAll(
        scene: Scene,
        rootKey: String? = nil,
        config: RenderConfig,
        url: URL,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> [RenderResult] {
        let cams = scene.cameras
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

    public func renderCGImage(scene: Scene,
                              config: RenderConfig,
                              progress: (@Sendable (Float) -> Void)? = nil) async throws -> CGImage {
        let (rgba, _, camSpec) = try await renderRGBA8Async(scene: scene, cameraIndex: 0, config: config) { rp in
            progress?(Float(rp.fraction))
            return true
        }
        return try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
    }

    public func render(
        scene: Scene,
        format: SceneFormat,
        cameraIndex: Int,
        config: RenderConfig,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> RenderResult {
        let (rgba, stats, camSpec) = try await renderRGBA8Async(scene: scene, cameraIndex: cameraIndex, config: config, progress: progress)
        let cg = try makeCGImage(width: camSpec.width, height: camSpec.height, rgba: rgba)
        return RenderResult(fileName: scene.cameras[cameraIndex].imageName, image: cg, camera: camSpec, stats: stats)
    }


    @inline(__always)
    private func sniffFormat(_ data: Data) -> SceneFormat {
        guard let s = String(data: data.prefix(64), encoding: .utf8)?
            .trimmingCharacters(in: .whitespacesAndNewlines) else { return .json }
        if s.hasPrefix("<") { return .xml }
        return .json
    }

    @MainActor
    public func render(
        scene: Scene,
        cameraIndex: Int = 0,
        config: RenderConfig,
        progressRow: ((Int) -> Void)? = nil,
        raysTraced: inout Int64
    ) -> [Vec3] {
        let cam = scene.cameras[cameraIndex]
        let width  = cam.imageResolution.0
        let height = cam.imageResolution.1

        let e  = cam.position
        let nd = Double(cam.nearDistance)
        let l  = Double(cam.nearPlane[0])
        let r  = Double(cam.nearPlane[1])
        let b  = Double(cam.nearPlane[2])
        let t  = Double(cam.nearPlane[3])

        let gazeNorm = normalize(cam.gaze)
        let w = -gazeNorm
        let upNorm = normalize(cam.up)
        let u = normalize(cross(upNorm, w))
        let v = normalize(cross(w, u))

        // Output buffer (linear 0..1 RGB as Float)
        var out: [Vec3] = [Vec3](repeating: .zero, count: width * height)

        // Precompute deltas for raster
        let du = (r - l) / Double(width)
        let dv = (t - b) / Double(height)

        for j in 0..<height {
            if Task.isCancelled { break }

            // Near plane geometry
            let m: Vec3 = e - w * nd           // center of near plane
            let q0: Vec3 = m + u * l + v * t      // top-left corner

            for i in 0..<width {
                if Task.isCancelled { break }

                // Pixel position on near plane
                let s_u = (Double(i) + 0.5) * du
                let s_v = (Double(j) + 0.5) * dv
                let s   = q0 + s_u * u - s_v * v

                // Primary ray
                var ray = makeRay(origin: e, dir: s - e)

                let triHit = intersectTriangles(ray: &ray)
                let sphHit = intersectSpheres(ray: &ray)
                let planeHit = intersectPlanes(ray: &ray)
                let idx = j * width + i

                if ray.kind != .none {
                    let mat = RuntimeStorage.materials[Int(ray.mat)]
                    let object = RuntimeStorage.objects[Int(ray.obj)]

                    // Intersection point (IMPORTANT: same, unnormalized ray dir)
                    let p = ray.origin + ray.dir * ray.tMax
                    // Face-forward the normal so N faces the camera (prevents “dark flips”)
                    var N = ray.normal
                    if dot(N, -ray.dir) < 0 { N = -N }

                    // Start with ambient (kept exactly as you had)
                    var pixel = scene.lights.ambient * mat.ambient

                    // Shadows + direct lighting
                    for light in scene.lights.points {
                        var w_i = light.position - p
                        let dist = length(w_i)
                        w_i = normalize(w_i)

                        // Shadow ray
                        var sRay = makeRay(origin: p + N * RTContext.shadowRayEpsilon , dir: w_i)
                        sRay.tMax = dist - RTContext.shadowRayEpsilon
                        let blocked = occluded(shadowRay: sRay, excludeObj: ray.obj, backfaceCulling: false)
                        if blocked { continue }

                        pixel += light.shade(material: mat, ray: ray, cameraPos: e, normal: N, at: p)
                    }
                    out[idx] = pixel
                } else {
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
    @MainActor
    private func renderRGBA8Async(
        scene: Scene,
        cameraIndex: Int,
        config: RenderConfig,
        progress: (@Sendable (RenderProgress) -> Bool)? = nil
    ) async throws -> ([UInt8], RenderStats, CameraSpec) {

        // 1) choose camera + resolution
        let cams = scene.cameras
        guard cameraIndex >= 0 && cameraIndex < cams.count else {
            throw NSError(domain: "Ray", code: -10, userInfo: [NSLocalizedDescriptionKey: "Invalid camera index"])
        }
        let cam = cams[cameraIndex]
        let width = cam.imageResolution.0
        let height = cam.imageResolution.1
        let cameraSpec = CameraSpec(index: cameraIndex, id: cam.id, imageName: cam.imageName, width: width, height: height)

        // 2) static counts once
        let (meshCount, triCount, sphereCount) = sceneMeshAndTriangleCounts(scene)

        let started = DispatchTime.now()
        return try await withTaskCancellationHandler(operation: {
            try await Task.detached(priority: .userInitiated) {
                try Task.checkCancellation()

                var rays: Int64 = 0
                // Your synchronous renderer MUST use 'width/height' from 'cam'
                let linear = await self.render(scene: scene,
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
                }, raysTraced: &rays)

                try Task.checkCancellation()

                let n = linear.count
                guard let maxIdx = linear.indices.max(by: { (i, j) in
                    let a = linear[i]; let b = linear[j]
                    return (a.x + a.y + a.z) < (b.x + b.y + b.z)
                }) else { fatalError("empty image") }

                print("DEBUG brightest HDR pixel @\(maxIdx):", linear[maxIdx])

                var rgba = [UInt8](repeating: 0, count: width * height * 4)

                for pix in 0..<n {
                    var c: Vec3 = min(max(linear[pix], Vec3.zero), Vec3(255, 255, 255))
                    let base = pix * 4
                    rgba[base + 0] = UInt8(c.x)
                    rgba[base + 1] = UInt8(c.y)
                    rgba[base + 2] = UInt8(c.z)
                    rgba[base + 3] = 255
                }

                let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - started.uptimeNanoseconds) / 1_000_000.0)
                let stats = RenderStats(meshes: meshCount, triangles: triCount, spheres: sphereCount, rays: rays, milliseconds: elapsedMs)
                return (rgba, stats, cameraSpec)
            }.value
        }, onCancel: {})
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

// MARK: - Color helpers (tone map + sRGB)
@inline(__always) private func clamp01(_ x: Float) -> Float { max(0, min(1, x)) }
@inline(__always) private func clamp01(_ x: Double) -> Double { max(0, min(1, x)) }

// Uses your existing Object protocol + Sphere/Triangle types
@inline(__always)
private func shadedNormalAt(_ point: Vec3, for object: SceneObject) -> SIMD3<Double> {
    switch object {
    case let sphere as Sphere:
        return (point - sphere.center) / sphere.radius
    case let tri as Triangle:
        // flat shading (you can extend to smooth with barycentrics)
        return normalize(cross(tri.e1, tri.e2))
    default:
        return SIMD3<Double>(0,0,0)
    }
}
