//
//  RTContext.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import Foundation
import simd
@preconcurrency import ParsingKit

// MARK: - Common aliases
public typealias Mat4   = simd_double4x4
public typealias Vec3   = SIMD3<Double>
public typealias Scalar = Double

extension Vec3: Hashable { }

// MARK: - Common structs
public struct BVHPrim: Sendable {
    public let type: UInt8
    public let index: Int
}

public enum ShadingMode: UInt8, Sendable {
    case flat
    case smooth
}

public struct PrimitiveInfo: Sendable {
    public let type: GeometryType
    public let primitiveIndex: Int
    public let bounds: AABB
    public var centroid: Vec3
    public let materialID: Int
    public var shadingMode: ShadingMode = .smooth
}

public enum GeometryType: UInt8, Sendable {
    case mesh
    case triangle
    case sphere
    case plane
}

// MARK: - Runtime containers
public struct InstanceRT: Sendable {
    public var baseIndex: Int
    public var transform: Mat4
    public var invTransform: Mat4
    public var worldBounds: AABB
    public var materialIndex: Int
}

// Analytic runtime wrapper (object reference + material)
public struct AnalyticRT: @unchecked Sendable {
    public let object: AnalyticRenderable
    public let materialIndex: Int
}

// MARK: - RTContext
public struct RTContext: Sendable {
    public var intersectionTestEpsilon: Scalar
    public var shadowRayEpsilon: Scalar

    public var materials: [Material]

    public var spheres: [Sphere]
    public var planes: [Plane]
    public var triangles: [Triangle]
    public var bvhPrims: [PrimitiveInfo]

    public var bvh: BVHBuilder?

    public init(scene: Scene) {
        self.intersectionTestEpsilon = scene.intersectionTestEpsilon
        self.shadowRayEpsilon        = scene.shadowRayEpsilon
        self.materials               = scene.materials
        self.triangles                = []
        self.spheres               = []
        self.planes               = []
        self.bvhPrims               = []

        var currentSphereIndex = 0
        var currentPlaneIndex = 0
        var currentTriangleIndex = 0

        for object in scene.objects {
            let matID = materialIndex(for: object.material, in: scene.materials)

            if var sph = object as? Sphere {
                sph.center = scene.vertexData.data[sph.centerIdx - 1]
                let bounds = AABB(minP: sph.center - Vec3(sph.radius), maxP: sph.center + Vec3(sph.radius))
                bvhPrims.append(PrimitiveInfo(
                    type: .sphere, primitiveIndex: currentSphereIndex, bounds: bounds,
                    centroid: sph.center, materialID: matID
                ))
                spheres.append(sph)
                currentSphereIndex += 1
            } else if var pln = object as? Plane {
                pln.center = scene.vertexData.data[pln.centerIdx - 1]
                let maxCoord = 1e5
                let bounds = AABB(minP: Vec3(repeating: -maxCoord), maxP: Vec3(repeating: maxCoord))
                bvhPrims.append(PrimitiveInfo(
                    type: .plane, primitiveIndex: currentPlaneIndex, bounds: bounds,
                    centroid: pln.center, materialID: matID
                ))
                planes.append(pln)
                currentPlaneIndex += 1

            } else if var tri = object as? Triangle {
                tri.v0 = scene.vertexData.data[tri.indices[0] - 1]
                tri.v1 = scene.vertexData.data[tri.indices[1] - 1]
                tri.v2 = scene.vertexData.data[tri.indices[2] - 1]
                tri.e1 = tri.v1 - tri.v0
                tri.e2 = tri.v2 - tri.v0
                tri.n0 = normalize(cross(tri.e1, tri.e2))
                tri.n1 = tri.n0
                tri.n2 = tri.n0

                let bounds = AABB(minP: min(tri.v0, min(tri.v1, tri.v2)), maxP: max(tri.v0, max(tri.v1, tri.v2)))
                tri.centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0

                bvhPrims.append(PrimitiveInfo(
                    type: .triangle, primitiveIndex: currentTriangleIndex, bounds: bounds,
                    centroid: tri.centroid, materialID: matID
                ))
                triangles.append(tri)
                currentTriangleIndex += 1

            } else if let m = object as? Mesh {
                let isSmooth = m.shadingMode == "smooth"

                var faceIndices: [Int] = []
                var meshPositions: [Vec3] = []
                var meshNormals: [Vec3] = []

                if let plyPath = m.faces.plyPath {
                    let cwd = scene.path?.deletingLastPathComponent()
                    ?? URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
                    let plyURL = cwd.appendingPathComponent(plyPath)

                    do {
                        let mesh = try PLYLoader.load(from: plyURL.path)
                        faceIndices = mesh.indices
                        meshPositions = mesh.positions
                        meshNormals = mesh.normals
                        print("number of vertices = \(meshPositions.count)")
                        print("number of normal = \(meshNormals.count)")
                        print("number of faces = \(faceIndices.count)")
                    } catch {
                        print("⚠️ Error loading PLY file at \(plyURL.path): \(error)")
                        continue
                    }
                } else {
                    faceIndices = m.faces.data
                    meshPositions = scene.vertexData.data
                }

                if !meshNormals.isEmpty {
                    let triangleCount = faceIndices.count / 3
                    for i in 0..<triangleCount {
                        let base = i * 3

                        let i0 = faceIndices[base + 0]
                        let i1 = faceIndices[base + 1]
                        let i2 = faceIndices[base + 2]

                        let v0 = meshPositions[i0]
                        let v1 = meshPositions[i1]
                        let v2 = meshPositions[i2]

                        let n0 = meshNormals[i0]
                        let n1 = meshNormals[i1]
                        let n2 = meshNormals[i2]

                        var tri = Triangle()
                        tri.v0 = v0
                        tri.v1 = v1
                        tri.v2 = v2
                        tri.e1 = v1 - v0
                        tri.e2 = v2 - v0
                        tri.centroid = (v0 + v1 + v2) / 3.0

                        tri.n0 = n0
                        tri.n1 = n1
                        tri.n2 = n2

                        triangles.append(tri)

                        let bounds = AABB(
                            minP: min(v0, min(v1, v2)),
                            maxP: max(v0, max(v1, v2))
                        )

                        bvhPrims.append(
                            PrimitiveInfo(
                                type: .triangle,
                                primitiveIndex: currentTriangleIndex,
                                bounds: bounds,
                                centroid: tri.centroid,
                                materialID: matID,
                                shadingMode: isSmooth ? .smooth : .flat
                            )
                        )

                        currentTriangleIndex += 1
                    }
                } else {
                    let fromPLY = m.faces.plyPath != nil
                    let offset = fromPLY ? 0 : 1
                    let indices1b = faceIndices
                    let pos = meshPositions
                    let triCount = indices1b.count / 3

                    var faceNormals = [Vec3](repeating: .zero, count: triCount)
                    for t in 0..<triCount {
                        let b = t * 3
                        let i0 = indices1b[b+0] - offset
                        let i1 = indices1b[b+1] - offset
                        let i2 = indices1b[b+2] - offset

                        let v0 = pos[i0]
                        let v1 = pos[i1]
                        let v2 = pos[i2]
                        faceNormals[t] = cross(v1 - v0, v2 - v0)
                    }

                    var vertexNormals = [Vec3](repeating: .zero, count: pos.count)
                    if isSmooth {
                        for t in 0..<triCount {
                            let b = t * 3
                            let fn = faceNormals[t]
                            vertexNormals[indices1b[b+0] - offset] += fn
                            vertexNormals[indices1b[b+1] - offset] += fn
                            vertexNormals[indices1b[b+2] - offset] += fn
                        }
                        for i in 0..<vertexNormals.count {
                            vertexNormals[i] = normalize(vertexNormals[i])
                        }
                    }

                    for t in 0..<triCount {
                        let b = t * 3
                        let i0 = indices1b[b+0] - offset
                        let i1 = indices1b[b+1] - offset
                        let i2 = indices1b[b+2] - offset

                        let v0 = pos[i0]
                        let v1 = pos[i1]
                        let v2 = pos[i2]

                        var tri = Triangle()
                        tri.v0 = v0; tri.v1 = v1; tri.v2 = v2
                        tri.e1 = v1 - v0; tri.e2 = v2 - v0
                        tri.centroid = (v0 + v1 + v2) / 3.0

                        if isSmooth {
                            tri.n0 = vertexNormals[i0]
                            tri.n1 = vertexNormals[i1]
                            tri.n2 = vertexNormals[i2]
                        } else {
                            let nFlat = normalize(faceNormals[t])
                            tri.n0 = nFlat; tri.n1 = nFlat; tri.n2 = nFlat
                        }

                        triangles.append(tri)

                        let bounds = AABB(minP: min(v0, min(v1, v2)),
                                          maxP: max(v0, max(v1, v2)))
                        bvhPrims.append(PrimitiveInfo(
                            type: .triangle,
                            primitiveIndex: currentTriangleIndex,
                            bounds: bounds,
                            centroid: tri.centroid,
                            materialID: matID,
                            shadingMode: isSmooth ? .smooth : .flat
                        ))
                        currentTriangleIndex += 1
                    }
                }
            }
        }

        bvh = BVHBuilder(primitives: bvhPrims)
    }
}

// MARK: - Helpers
private extension RTContext {
    func materialIndex(for id: String?, in materials: [Material]) -> Int {
        guard let id, let idx = Int(id) else { return 0 }
        return idx
    }

    @inline(__always)
    func findMatchingVertexIndex(in meshID: Int, v: Vec3, positions: [Vec3]) -> Int {
        var bestIndex = 0
        var bestDist = Double.greatestFiniteMagnitude
        for i in 0..<positions.count {
            let d2 = simd_length_squared(positions[i] - v)
            if d2 < bestDist {
                bestDist = d2
                bestIndex = i
            }
        }
        return bestDist < 1e-6 ? bestIndex : 0
    }
}
