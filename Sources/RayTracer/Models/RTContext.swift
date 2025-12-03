//
//  RTContext.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import Foundation
import simd
@preconcurrency import ParsingKit

// MARK: - Aliases
public typealias Mat4   = simd_double4x4
public typealias Mat3   = simd_double3x3
public typealias Vec3   = SIMD3<Double>
public typealias Scalar = Double

// MARK: - Helpers
public extension Vec3 { init(repeating v: Double) { self.init(v, v, v) } }
@inlinable public func min(_ a: Vec3, _ b: Vec3) -> Vec3 { simd.min(a, b) }
@inlinable public func max(_ a: Vec3, _ b: Vec3) -> Vec3 { simd.max(a, b) }

@inlinable
func normalTransformMatrix(from M: Mat4) -> simd_double3x3 {
    let m3 = simd_double3x3(
        SIMD3(M.columns.0.x, M.columns.0.y, M.columns.0.z),
        SIMD3(M.columns.1.x, M.columns.1.y, M.columns.1.z),
        SIMD3(M.columns.2.x, M.columns.2.y, M.columns.2.z)
    )
    return m3.inverse.transpose
}

// MARK: - BLAS / Instance / TLAS
public struct BLAS: Sendable {
    public let nodes: [BVHNode]
    public let primIdx: [UInt]
    public let prims: [PrimitiveInfo]
    public let root: UInt
    public init(nodes: [BVHNode], primIdx: [UInt], prims: [PrimitiveInfo], root: UInt) {
        self.nodes = nodes; self.primIdx = primIdx; self.prims = prims; self.root = root
    }
}
public struct Instance: Sendable {
    public let blas: BLAS
    public let localToWorld: Mat4
    public let worldToLocal: Mat4
    public let worldBounds: AABB
    public let materialOverride: Int?
    public var instanceMotion: Vec3 = .zero
    public var normalMatrix: Mat3

    init(blas: BLAS, localToWorld: Mat4, worldToLocal: Mat4, worldBounds: AABB, materialOverride: Int?, instanceMotion: Vec3 = .zero, normalMatrix: Mat3) {
        self.blas = blas
        self.localToWorld = localToWorld
        self.worldToLocal = worldToLocal
        self.worldBounds = worldBounds
        self.materialOverride = materialOverride
        self.instanceMotion = instanceMotion
        self.normalMatrix = normalMatrix
    }
}

public struct TLAS: Sendable {
    public var builder: BVHBuilder
    public let instances: [Instance]

    init(builder: BVHBuilder, instances: [Instance]) {
        self.builder = builder
        self.instances = instances
    }
}

// MARK: - RTContext (runtime owner of arrays + TLAS/BLAS)
public struct RTContext: Sendable {
    public var intersectionTestEpsilon: Scalar
    public var shadowRayEpsilon: Scalar

    public var materials: [Material]
    public var spheres: [Sphere]
    public var planes: [Plane]
    public var triangles: [Triangle]

    public var pointLights: [PointLight]
    public var areaLights: [AreaLight]

    // Global BVH for non-instanced prims (optional)
    public var bvhPrims: [PrimitiveInfo]
    public var bvh: BVHBuilder?

    // Instancing
    public var instances: [Instance] = []
    public var tlas: TLAS?

    public init(scene: Scene) {
        self.intersectionTestEpsilon = scene.intersectionTestEpsilon
        self.shadowRayEpsilon        = scene.shadowRayEpsilon
        self.materials               = scene.materials
        self.triangles               = []
        self.spheres                 = []
        self.planes                  = []
        self.bvhPrims                = []

        self.pointLights = scene.lights.points.compactMap { $0 as? PointLight }
        self.areaLights  = scene.lights.points.compactMap { $0 as? AreaLight }
        // --- Build analytic & mesh triangles into arrays and bvhPrims ---
        var currentSphereIndex = 0
        var currentPlaneIndex = 0
        var currentTriangleIndex = 0

        // Track mesh triangle ranges and metadata for BLAS building
        var meshTriRangeByID: [String: Range<Int>] = [:]
        var meshMaterialByID: [String: Int] = [:]

        // Track all processed objects (Mesh + MeshInstance) with their BLAS, material, and transform
        // This enables cascading instancing: MeshInstance can reference another MeshInstance
        var instanceByID: [String: (blas: BLAS, material: Int, transform: Mat4)] = [:]
        var instArray: [Instance] = []
        var meshInstances: [MeshInstance] = []

        for object in scene.objects {
            let matID = materialIndex(for: object.material, in: scene.materials)
            if var sph = object as? Sphere {
                // Keep local data (center index gives local position)
                let localCenter = scene.vertexData.data[sph.centerIdx - 1]

                // Compose object transform
                let M = scene.composeTransform(tokens: sph.transformTokens,
                                               reset: sph.resetTransform)

                // Compute world-space bounds from local-space sphere
                let s = decomposeUniformScale(M)
                let ctrW = transformPoint(M, localCenter)
                let worldB = AABB(minP: ctrW - Vec3(sph.radius * s),
                                  maxP: ctrW + Vec3(sph.radius * s))

                // Store sphere unchanged (for local intersection)
                sph.center = localCenter
                spheres.append(sph)

                // Build local-space primitive (untransformed)
                let prim = PrimitiveInfo(type: .sphere,
                                         primitiveIndex: currentSphereIndex,
                                         bounds: AABB(minP: sph.center - Vec3(sph.radius),
                                                      maxP: sph.center + Vec3(sph.radius)),
                                         centroid: sph.center,
                                         materialID: matID)

                // Build BLAS in local space
                let blas = BVHBuilder(primitives: [prim])

                // Create instance with transform (worldToLocal will handle scaling & translation)
                let instance = makeInstance(from: BLAS(nodes: blas.bvhNode,
                                                       primIdx: blas.primitiveIdx,
                                                       prims: [prim],
                                                       root: blas.rootNodeIdx),
                                            localToWorld: M,
                                            materialOverride: matID)
                instArray.append(instance)
                currentSphereIndex += 1
                continue
            } else if var pln = object as? Plane {
                let M = scene.composeTransform(tokens: pln.transformTokens,
                                               reset: pln.resetTransform)
                let Nm = normalTransformMatrix(from: M)

                // Keep plane local, apply transform only in instance
                let localCenter = scene.vertexData.data[pln.centerIdx - 1]
                pln.center = localCenter
                planes.append(pln)

                // Define local-space bounds
                let maxCoord = 1e5
                let localB = AABB(minP: Vec3(repeating: -maxCoord),
                                  maxP: Vec3(repeating: maxCoord))

                let prim = PrimitiveInfo(type: .plane,
                                         primitiveIndex: currentPlaneIndex,
                                         bounds: localB,
                                         centroid: .zero,
                                         materialID: matID)
                let blas = BVHBuilder(primitives: [prim])

                // Attach transform via instance (no pre-transform)
                let instance = makeInstance(from: BLAS(nodes: blas.bvhNode,
                                                       primIdx: blas.primitiveIdx,
                                                       prims: [prim],
                                                       root: blas.rootNodeIdx),
                                            localToWorld: M,
                                            materialOverride: matID)
                instArray.append(instance)
                currentPlaneIndex += 1
                continue
            } else if var tri = object as? Triangle {
                // Compose transform but DO NOT bake it into vertices
                let M = scene.composeTransform(tokens: tri.transformTokens,
                                               reset: tri.resetTransform)

                // Grab local-space vertices (untransformed)
                let v0 = scene.vertexData.data[tri.indices[0] - 1]
                let v1 = scene.vertexData.data[tri.indices[1] - 1]
                let v2 = scene.vertexData.data[tri.indices[2] - 1]

                // Keep triangle in local coordinates
                tri.v0 = v0; tri.v1 = v1; tri.v2 = v2
                tri.e1 = v1 - v0; tri.e2 = v2 - v0
                tri.centroid = (v0 + v1 + v2) / 3.0
                let n = normalize(cross(tri.e1, tri.e2))
                tri.n0 = n; tri.n1 = n; tri.n2 = n
                triangles.append(tri)

                // Build local-space bounding box
                let localB = AABB(minP: min(v0, min(v1, v2)),
                                  maxP: max(v0, max(v1, v2)))

                // Build a single-triangle BLAS in local space
                let prim = PrimitiveInfo(type: .triangle,
                                         primitiveIndex: currentTriangleIndex,
                                         bounds: localB,
                                         centroid: tri.centroid,
                                         materialID: matID,
                                         shadingMode: .flat)

                let blas = BVHBuilder(primitives: [prim])

                // Create an instance with transform only applied in TLAS
                let instance = makeInstance(from: BLAS(nodes: blas.bvhNode,
                                                       primIdx: blas.primitiveIdx,
                                                       prims: [prim],
                                                       root: blas.rootNodeIdx),
                                            localToWorld: M,
                                            materialOverride: matID)

                instArray.append(instance)
                currentTriangleIndex += 1
                continue
            } else if var mI = object as? MeshInstance {
                if mI.material == "" || mI.material == nil {
                    mI.material = scene.objects.first(where: { $0.id == mI.baseMeshID })?.material
                }
                meshInstances.append(mI)
                continue
            } else if let m = object as? Mesh {
                // Build triangles for this base mesh
                let start = triangles.count
                let isSmooth = (m.shadingMode == "smooth")

                var faceIndices: [Int] = []
                var meshPositions: [Vec3] = []
                var meshNormals: [Vec3] = []

                if let plyPath = m.faces.plyPath {
                    let cwd = scene.path?.deletingLastPathComponent()
                    ?? URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
                    let plyURL = cwd.appendingPathComponent(plyPath)
                    if let mesh = try? PLYLoader.load(from: plyURL.path) {
                        faceIndices  = mesh.indices
                        meshPositions = mesh.positions
                        meshNormals   = mesh.normals
                    } else {
                        continue
                    }
                } else {
                    faceIndices  = m.faces.data
                    meshPositions = scene.vertexData.data
                }

                if !meshNormals.isEmpty {
                    let triCount = faceIndices.count / 3
                    for i in 0..<triCount {
                        let b = i*3
                        let i0 = faceIndices[b+0], i1 = faceIndices[b+1], i2 = faceIndices[b+2]
                        let v0 = meshPositions[i0], v1 = meshPositions[i1], v2 = meshPositions[i2]
                        let n0 = meshNormals[i0],   n1 = meshNormals[i1],   n2 = meshNormals[i2]

                        var t = Triangle()
                        t.v0=v0; t.v1=v1; t.v2=v2
                        t.e1=v1-v0; t.e2=v2-v0
                        t.centroid = (v0+v1+v2)/3
                        t.n0=n0; t.n1=n1; t.n2=n2
                        t.motionBlur = m.motionBlur
                        triangles.append(t)

                        let blurV0 = m.motionBlur != .zero ? v0.withMotionBlur(m.motionBlur) : v0
                        let blurV1 = m.motionBlur != .zero ? v1.withMotionBlur(m.motionBlur) : v1
                        let blurV2 = m.motionBlur != .zero ? v2.withMotionBlur(m.motionBlur) : v2

                        let minP = min(min(v0, min(v1, v2)), min(blurV0, min(blurV1, blurV2)))
                        let maxP = max(max(v0, max(v1, v2)), max(blurV0, max(blurV1, blurV2)))
                        let bnd = AABB(minP: minP, maxP: maxP)
                        bvhPrims.append(PrimitiveInfo(type: .triangle,
                                                      primitiveIndex: currentTriangleIndex,
                                                      bounds: bnd,
                                                      centroid: t.centroid,
                                                      materialID: matID,
                                                      shadingMode: isSmooth ? .smooth : .flat))
                        currentTriangleIndex += 1
                    }
                } else {
                    // compute vertex normals if smooth
                    let fromPLY = m.faces.plyPath != nil
                    let offset = fromPLY ? 0 : 1
                    let idx1b = faceIndices
                    let pos = meshPositions
                    let triCount = idx1b.count / 3
                    var faceNormals = [Vec3](repeating: .zero, count: triCount)

                    for t in 0..<triCount {
                        let b = t*3
                        let i0 = idx1b[b+0] - offset
                        let i1 = idx1b[b+1] - offset
                        let i2 = idx1b[b+2] - offset
                        let v0 = pos[i0], v1 = pos[i1], v2 = pos[i2]
                        faceNormals[t] = normalize(cross(v1 - v0, v2 - v0))
                    }

                    var vtxNormals = [Vec3](repeating: .zero, count: pos.count)
                    if isSmooth {
                        for t in 0..<triCount {
                            let b = t*3
                            let fn = faceNormals[t]
                            vtxNormals[idx1b[b+0] - offset] += fn
                            vtxNormals[idx1b[b+1] - offset] += fn
                            vtxNormals[idx1b[b+2] - offset] += fn
                        }
                        for i in 0..<vtxNormals.count { vtxNormals[i] = normalize(vtxNormals[i]) }
                    }

                    for t in 0..<triCount {
                        let b = t*3
                        let i0 = idx1b[b+0] - offset
                        let i1 = idx1b[b+1] - offset
                        let i2 = idx1b[b+2] - offset
                        let v0 = pos[i0], v1 = pos[i1], v2 = pos[i2]

                        var tri = Triangle()
                        tri.v0=v0; tri.v1=v1; tri.v2=v2
                        tri.e1=v1-v0; tri.e2=v2-v0
                        tri.centroid = (v0+v1+v2)/3
                        tri.motionBlur = m.motionBlur
                        if isSmooth {
                            tri.n0 = vtxNormals[i0]; tri.n1 = vtxNormals[i1]; tri.n2 = vtxNormals[i2]
                        } else {
                            let n = normalize(faceNormals[t])
                            tri.n0=n; tri.n1=n; tri.n2=n
                        }
                        triangles.append(tri)

                        let blurV0 = m.motionBlur != .zero ? v0.withMotionBlur(m.motionBlur) : v0
                        let blurV1 = m.motionBlur != .zero ? v1.withMotionBlur(m.motionBlur) : v1
                        let blurV2 = m.motionBlur != .zero ? v2.withMotionBlur(m.motionBlur) : v2

                        let minP = min(min(v0, min(v1, v2)), min(blurV0, min(blurV1, blurV2)))
                        let maxP = max(max(v0, max(v1, v2)), max(blurV0, max(blurV1, blurV2)))
                        let bounds = AABB(minP: minP, maxP: maxP)
                        bvhPrims.append(PrimitiveInfo(type: .triangle,
                                                      primitiveIndex: currentTriangleIndex,
                                                      bounds: bounds,
                                                      centroid: tri.centroid,
                                                      materialID: matID,
                                                      shadingMode: isSmooth ? .smooth : .flat))
                        currentTriangleIndex += 1
                    }
                }

                let end = triangles.count
                let meshID = m.id ?? ""
                meshTriRangeByID[meshID] = start..<end
                meshMaterialByID[meshID] = matID

                // Build BLAS for base mesh and store it with its transform
                // This allows MeshInstance objects to reference this mesh
                if end > start {
                    let blas = buildBLASForMesh(prims: bvhPrims, triRange: start..<end)
                    let baseTransform = scene.composeTransform(tokens: m.transformTokens, reset: m.resetTransform)
                    instanceByID[meshID] = (blas: blas, material: matID, transform: baseTransform)
                }
            }
        }

        if !bvhPrims.isEmpty {
            self.bvh = BVHBuilder(primitives: bvhPrims)
        }

        for instObj in meshInstances {
            let instID = instObj.id ?? ""
            let baseMeshID = instObj.baseMeshID
            guard let baseData = instanceByID[baseMeshID] else { continue }
            let instanceTransform = scene.composeTransform(tokens: instObj.transformTokens,
                                                           reset: instObj.resetTransform,
                                                           base: baseData.transform)

            let matOverride = materialIndex(for: instObj.material, in: scene.materials)
            let instance = makeInstance(from: baseData.blas,
                                        localToWorld: instanceTransform,
                                        materialOverride: matOverride,
                                        instanceMotion: instObj.motionBlur 
            )
            instArray.append(instance)

            instanceByID[instID] = (blas: baseData.blas, material: matOverride, transform: instanceTransform)
        }

        for (meshID, _) in meshTriRangeByID {
            if let baseData = instanceByID[meshID] {
                let baseInstance = makeInstance(from: baseData.blas,
                                                localToWorld: baseData.transform,
                                                materialOverride: baseData.material)
                instArray.append(baseInstance)
            }
        }

        if !instArray.isEmpty {
            self.tlas = buildTLAS(instances: instArray)
            self.instances = instArray
        } else {
            print("⚠️ No instances to build TLAS")
        }
    }
}

// MARK: - Index helpers
private extension RTContext {
    func materialIndex(for id: String?, in materials: [Material]) -> Int {
        guard let id, let idx = Int(id) else { return -1 }
        return idx
    }
}

// MARK: - BLAS / TLAS builders
public func buildBLASForMesh(prims: [PrimitiveInfo], triRange: Range<Int>) -> BLAS {
    let subset = prims.filter { p in p.type == .triangle && triRange.contains(p.primitiveIndex) }
    var b = BVHBuilder(primitives: subset, maxLeaf: 2, split: .sah, binCount: 12)
    // ✅ use the actual root index
    return BLAS(nodes: b.bvhNode, primIdx: b.primitiveIdx, prims: subset, root: b.rootNodeIdx)
}

public func makeInstance(from base: BLAS, localToWorld M: Mat4, materialOverride: Int?, instanceMotion: Vec3 = .zero) -> Instance {
    // Recompute world bounds by transforming all leaf AABBs
    var minW = Vec3(repeating: .infinity)
    var maxW = Vec3(repeating: -.infinity)
    for p in base.prims {
        let localB = p.bounds
        let worldB = localB.transformed(by: M)
        minW = min(minW, worldB.minP)
        maxW = max(maxW, worldB.maxP)
    }
    let worldB = AABB(minP: minW, maxP: maxW)
    let nMat = normalTransformMatrix(from: M)
    return Instance(blas: base,
                    localToWorld: M,
                    worldToLocal: M.inverse,
                    worldBounds: worldB,
                    materialOverride: materialOverride,
                    instanceMotion: instanceMotion,
                    normalMatrix: nMat
    )
}

public func buildTLAS(instances: [Instance]) -> TLAS {
    var tlasPrims: [PrimitiveInfo] = []
    tlasPrims.reserveCapacity(instances.count)
    for (i, inst) in instances.enumerated() {
        let b = inst.worldBounds
        let c = (b.minP + b.maxP) * 0.5
        tlasPrims.append(PrimitiveInfo(type: .mesh,
                                       primitiveIndex: i,
                                       bounds: b,
                                       centroid: c,
                                       materialID: inst.materialOverride ?? 0,
                                       shadingMode: .smooth))
    }
    let builder = BVHBuilder(primitives: tlasPrims, maxLeaf: 2, split: .sah, binCount: 12)
    return TLAS(builder: builder, instances: instances)
}

// MARK: - Intersections (local space)
public extension RTContext {
    @inlinable @inline(__always)
    func intersectTriangle(ray: inout Ray, triangle: Triangle, isSmooth: Bool, matIdx: Int, eps: Scalar) {
        let offset = triangle.motionBlur * ray.time
        let origin = ray.origin - offset

        let pvec = cross(ray.dir, triangle.e2)
        let det  = dot(triangle.e1, pvec)
        if Swift.abs(det) < eps { return }
        let invDet = 1.0 / det
        let tvec = origin - triangle.v0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return }
        let q = cross(tvec, triangle.e1)
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { return }
        let t = dot(triangle.e2, q) * invDet
        if t <= max(eps, ray.tMin) || t >= ray.hit.t { return }

        let w = 1.0 - u - v
        var n: Vec3
        if isSmooth {
            let interp = w*triangle.n0 + u*triangle.n1 + v*triangle.n2
            n = normalize(interp)
        } else {
            n = normalize(cross(triangle.e1, triangle.e2))
        }
        let p = triangle.v0 + u*triangle.e1 + v*triangle.e2
        ray.hit.t = t
        ray.hit.p = p
        ray.hit.n = n
        ray.hit.kind = .triangle
        ray.hit.mat = matIdx
    }

    @inlinable @inline(__always)
    func intersectSphere(ray: inout Ray, sphere: Sphere, matIdx: Int, eps: Scalar) {
        let oc = ray.origin - sphere.center
        let a = dot(ray.dir, ray.dir)
        let b = 2.0 * dot(oc, ray.dir)
        let c = dot(oc, oc) - sphere.radius*sphere.radius
        let disc = b*b - 4*a*c
        if disc < 0 { return }
        let sd = sqrt(disc)
        var t = (-b - sd) / (2*a)
        if t < eps { t = (-b + sd) / (2*a) }
        if t <= max(eps, ray.tMin) || t >= ray.hit.t { return }
        let p = ray.origin + ray.dir * t
        let n = normalize(p - sphere.center)
        ray.hit.t = t; ray.hit.p = p; ray.hit.n = n; ray.hit.kind = .sphere; ray.hit.mat = matIdx
    }

    @inlinable @inline(__always)
    func intersectPlane(ray: inout Ray, plane: Plane, matIdx: Int, eps: Scalar) {
        let denom = dot(plane.normal, ray.dir)
        if Swift.abs(denom) < eps { return }
        let t = dot(plane.center - ray.origin, plane.normal) / denom
        if t <= max(eps, ray.tMin) || t >= ray.hit.t { return }
        let p = ray.origin + ray.dir * t
        let n = normalize(plane.normal)
        ray.hit.t = t; ray.hit.p = p; ray.hit.n = n; ray.hit.kind = .plane; ray.hit.mat = matIdx
    }
}

// MARK: - BLAS traversal (local)
public extension RTContext {
    @inlinable @inline(__always)
    func intersectBLAS(ray: inout Ray, inst: Instance, eps: Scalar) -> Int {
        let nodes = inst.blas.nodes
        let prims = inst.blas.prims
        let primIdx = inst.blas.primIdx

        // ✅ Optimization 1: Stack allocation (Zero malloc overhead)
        return withUnsafeTemporaryAllocation(of: UInt.self, capacity: 64) { stack in
            var visits = 0
            var sp = 0
            stack[sp] = inst.blas.root

            // Helper closure to keep scope clean
            @inline(__always)
            func hitAABB(_ n: BVHNode, _ r: Ray) -> Double {
                let t1 = (n.aabbMin - r.origin) * r.invDir
                let t2 = (n.aabbMax - r.origin) * r.invDir
                let tminv = simd.min(t1, t2)
                let tmaxv = simd.max(t1, t2)
                let tmin = max(max(tminv.x, tminv.y), tminv.z)
                let tmax = min(tmaxv.x, min(tmaxv.y, tmaxv.z))
                return (tmax >= max(tmin, eps)) ? tmin : .infinity
            }

            while sp >= 0 {
                visits &+= 1
                let idx = stack[sp]; sp &-= 1
                let node = nodes[Int(idx)]
                if hitAABB(node, ray) == .infinity { continue }

                if node.isLeaf {
                    let first = Int(node.leftFirst)
                    let count = Int(node.primitiveCount)
                    for k in 0..<count {
                        let pid = Int(primIdx[first + k])
                        let p = prims[pid]
                        switch p.type {
                        case .triangle:
                            let tri = triangles[p.primitiveIndex]
                            intersectTriangle(ray: &ray,
                                              triangle: tri,
                                              isSmooth: p.shadingMode == .smooth,
                                              matIdx: p.materialID,
                                              eps: eps)
                        case .sphere:
                            let s = spheres[p.primitiveIndex]
                            intersectSphere(ray: &ray, sphere: s, matIdx: p.materialID, eps: eps)
                        case .plane:
                            let pl = planes[p.primitiveIndex]
                            intersectPlane(ray: &ray, plane: pl, matIdx: p.materialID, eps: eps)
                        case .mesh:
                            break
                        }
                    }
                    continue
                }

                var L = node.leftFirst
                var R = L + 1
                let lNode = nodes[Int(L)], rNode = nodes[Int(R)]
                var d1 = hitAABB(lNode, ray), d2 = hitAABB(rNode, ray)
                if d1 > d2 { swap(&d1, &d2); swap(&L, &R) }
                if d2 != .infinity { sp &+= 1; stack[sp] = R }
                if d1 != .infinity { sp &+= 1; stack[sp] = L }
            }
            return visits
        }
    }
}

// MARK: - TLAS traversal (world → local → BLAS → world)
// In RTContext.swift

// MARK: - TLAS traversal (world -> local -> BLAS -> world)
public extension RTContext {
    @inlinable @inline(__always)
    func intersectTLAS(ray worldRay: inout Ray, tlas: TLAS, eps: Scalar) -> Int {
        let nodes = tlas.builder.bvhNode
        let prims = tlas.builder.primitives
        let pidx  = tlas.builder.primitiveIdx
        let insts = tlas.instances

        // ✅ Optimization 1: Stack allocation
        return withUnsafeTemporaryAllocation(of: UInt.self, capacity: 64) { stack in
            var visits = 0
            var sp = 0
            stack[sp] = tlas.builder.rootNodeIdx

            @inline(__always)
            func hitAABB(_ n: BVHNode, _ r: Ray) -> Double {
                let t1 = (n.aabbMin - r.origin) * r.invDir
                let t2 = (n.aabbMax - r.origin) * r.invDir
                let tminv = simd.min(t1, t2)
                let tmaxv = simd.max(t1, t2)
                let tmin = max(max(tminv.x, tminv.y), tminv.z)
                let tmax = min(tmaxv.x, min(tmaxv.y, tmaxv.z))
                return (tmax >= max(tmin, eps)) ? tmin : .infinity
            }

            while sp >= 0 {
                visits &+= 1
                let idx = stack[sp]; sp &-= 1
                let node = nodes[Int(idx)]
                if hitAABB(node, worldRay) == .infinity { continue }

                if node.isLeaf {
                    let first = Int(node.leftFirst)
                    let count = Int(node.primitiveCount)
                    for k in 0..<count {
                        let pid = Int(pidx[first + k])
                        let p = prims[pid]
                        guard p.type == .mesh else { continue }
                        let inst = insts[p.primitiveIndex]

                        // world -> local ray
                        let instOffset = inst.instanceMotion * worldRay.time
                        var rL = worldRay
                        rL.time = worldRay.time
                        rL.origin -= instOffset

                        let o4 = inst.worldToLocal * SIMD4(rL.origin.x, rL.origin.y, rL.origin.z, 1.0)
                        let d4 = inst.worldToLocal * SIMD4(rL.dir.x,    rL.dir.y,    rL.dir.z,    0.0)
                        let dirL = Vec3(d4.x, d4.y, d4.z)

                        rL.origin = Vec3(o4.x, o4.y, o4.z)
                        rL.dir    = dirL
                        rL.invDir = 1.0 / rL.dir

                        rL.tMax   = worldRay.tMax
                        rL.hit    = worldRay.hit
                        rL.hit.t  = worldRay.hit.t

                        // trace BLAS in local space
                        var tmp = rL
                        visits &+= intersectBLAS(ray: &tmp, inst: inst, eps: eps)

                        if tmp.hit.t < worldRay.hit.t {
                            // local hit -> world
                            let instOffset = inst.instanceMotion * worldRay.time
                            let hp = SIMD4(tmp.hit.p, 1.0)
                            let wp = inst.localToWorld * hp

                            // ✅ Optimization 2: Use precomputed normal matrix (Multiplier only)
                            var wn = normalize(inst.normalMatrix * tmp.hit.n)

                            // Check determinant to flip normal if scaled negatively
                            // (You can also precalculate the determinant sign if you want to be even faster,
                            //  but this is usually cheap enough)
                            let M3 = simd_double3x3(
                                SIMD3(inst.localToWorld.columns.0.x, inst.localToWorld.columns.0.y, inst.localToWorld.columns.0.z),
                                SIMD3(inst.localToWorld.columns.1.x, inst.localToWorld.columns.1.y, inst.localToWorld.columns.1.z),
                                SIMD3(inst.localToWorld.columns.2.x, inst.localToWorld.columns.2.y, inst.localToWorld.columns.2.z)
                            )
                            if simd_determinant(M3) < 0.0 {
                                wn = -wn
                            }

                            worldRay.hit.t    = tmp.hit.t
                            worldRay.hit.p    = Vec3(wp.x, wp.y, wp.z) + instOffset
                            worldRay.hit.n    = wn
                            worldRay.hit.kind = tmp.hit.kind
                            worldRay.hit.mat  = inst.materialOverride ?? tmp.hit.mat
                        }
                    }
                    continue
                }

                var L = node.leftFirst
                var R = L + 1
                let lNode = nodes[Int(L)], rNode = nodes[Int(R)]
                var d1 = hitAABB(lNode, worldRay), d2 = hitAABB(rNode, worldRay)
                if d1 > d2 { swap(&d1, &d2); swap(&L, &R) }
                if d2 != .infinity { sp &+= 1; stack[sp] = R }
                if d1 != .infinity { sp &+= 1; stack[sp] = L }
            }
            return visits
        }
    }

    // Optional: shadow traversal (short-circuit)
    @inlinable @inline(__always)
    func occludedTLAS(ray worldRay: Ray, tlas: TLAS, eps: Scalar) -> Bool {
        var stack: [UInt] = [tlas.builder.rootNodeIdx]
        let nodes = tlas.builder.bvhNode
        let prims = tlas.builder.primitives
        let pidx  = tlas.builder.primitiveIdx
        let insts = tlas.instances

        func hitAABB(_ n: BVHNode, _ r: Ray) -> Double {
            let t1 = (n.aabbMin - r.origin) * r.invDir
            let t2 = (n.aabbMax - r.origin) * r.invDir
            let tminv = simd.min(t1, t2)
            let tmaxv = simd.max(t1, t2)
            let tmin = max(max(tminv.x, tminv.y), tminv.z)
            let tmax = min(tmaxv.x, min(tmaxv.y, tmaxv.z))
            return (tmax >= max(tmin, eps)) ? tmin : .infinity
        }

        while let idx = stack.popLast() {
            let node = nodes[Int(idx)]
            if hitAABB(node, worldRay) == .infinity { continue }

            if node.isLeaf {
                let first = Int(node.leftFirst)
                let count = Int(node.primitiveCount)
                for k in 0..<count {
                    let pid = Int(pidx[first + k])
                    let p = prims[pid]
                    guard p.type == .mesh else { continue }
                    let inst = insts[p.primitiveIndex]

                    // world → local
                    var rL = worldRay
                    let instOffset = inst.instanceMotion * worldRay.time
                    rL.origin -= instOffset
                    rL.time = worldRay.time
                    let o4 = inst.worldToLocal * SIMD4(rL.origin.x, rL.origin.y, rL.origin.z, 1.0)
                    let d4 = inst.worldToLocal * SIMD4(rL.dir.x,    rL.dir.y,    rL.dir.z,    0.0)
                    let dirL = Vec3(d4.x, d4.y, d4.z)

                    rL.origin = Vec3(o4.x, o4.y, o4.z)
                    rL.dir    = dirL
                    rL.invDir = 1.0 / rL.dir

                    rL.tMax   = worldRay.tMax
                    rL.hit    = worldRay.hit
                    rL.hit.t  = worldRay.hit.t

                    // short BLAS check
                    if occludedBLAS(ray: rL, inst: inst, eps: eps) { return true }
                }
            } else {
                let L = node.leftFirst
                let R = L + 1
                stack.append(R); stack.append(L)
            }
        }
        return false
    }

    @inlinable @inline(__always)
    func occludedBLAS(ray r: Ray, inst: Instance, eps: Scalar) -> Bool {
        let nodes = inst.blas.nodes
        let prims = inst.blas.prims
        let primIdx = inst.blas.primIdx

        var stack: [UInt] = [inst.blas.root]

        func hitAABB(_ n: BVHNode, _ r: Ray) -> Double {
            let t1 = (n.aabbMin - r.origin) * r.invDir
            let t2 = (n.aabbMax - r.origin) * r.invDir
            let tminv = simd.min(t1, t2)
            let tmaxv = simd.max(t1, t2)
            let tmin = max(max(tminv.x, tminv.y), tminv.z)
            let tmax = min(tmaxv.x, min(tmaxv.y, tmaxv.z))
            return (tmax >= max(tmin, eps)) ? tmin : .infinity

        }

        while let idx = stack.popLast() {
            let node = nodes[Int(idx)]
            if hitAABB(node, r) == .infinity { continue }

            if node.isLeaf {
                let first = Int(node.leftFirst)
                let count = Int(node.primitiveCount)
                for k in 0..<count {
                    let pid = Int(primIdx[first + k])
                    let p = prims[pid]
                    switch p.type {
                    case .triangle:
                        if triShadowHit(r, triangle: triangles[p.primitiveIndex], eps: eps) { return true }
                    case .sphere:
                        if sphereShadowHit(r, sphere: spheres[p.primitiveIndex], eps: eps) { return true }
                    case .plane:
                        if planeShadowHit(r, plane: planes[p.primitiveIndex], eps: eps) { return true }
                    case .mesh: break
                    }
                }
            } else {
                let L = node.leftFirst
                let R = L + 1
                stack.append(R); stack.append(L)
            }
        }
        return false
    }

    @inlinable @inline(__always)
    func triShadowHit(_ ray: Ray, triangle: Triangle, eps: Double) -> Bool {
        let offset = triangle.motionBlur * ray.time
        let origin = ray.origin - offset

        let pvec = cross(ray.dir, triangle.e2)
        let det = dot(triangle.e1, pvec)
        if Swift.abs(det) < eps { return false }
        let invDet = 1.0 / det
        let tvec = origin - triangle.v0
        let u = dot(tvec, pvec) * invDet
        if u < 0.0 || u > 1.0 { return false }
        let q = cross(tvec, triangle.e1)
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { return false }
        let t = dot(triangle.e2, q) * invDet
        return (t > max(eps, ray.tMin) && t < ray.tMax)
    }
    
    @inlinable @inline(__always)
    func sphereShadowHit(_ ray: Ray, sphere: Sphere, eps: Scalar) -> Bool {
        let oc = ray.origin - sphere.center
        let a = dot(ray.dir, ray.dir)
        let b = 2.0 * dot(oc, ray.dir)
        let c = dot(oc, oc) - sphere.radius*sphere.radius
        let disc = b*b - 4*a*c
        if disc < 0 { return false }
        let sd = sqrt(disc)
        var t = (-b - sd) / (2*a)
        if t < max(eps, ray.tMin) { t = (-b + sd) / (2*a) }
        return (t > max(eps, ray.tMin) && t < ray.tMax)
    }

    @inlinable @inline(__always)
    func planeShadowHit(_ ray: Ray, plane: Plane, eps: Scalar) -> Bool {
        let denom = dot(plane.normal, ray.dir)
        if Swift.abs(denom) < eps { return false }
        let t = dot(plane.center - ray.origin, plane.normal) / denom
        return (t > max(eps, ray.tMin) && t < ray.tMax)
    }
}


@inlinable
func decomposeUniformScale(_ M: Mat4) -> Double {
    let sx = length(Vec3(M.columns.0.x, M.columns.0.y, M.columns.0.z))
    let sy = length(Vec3(M.columns.1.x, M.columns.1.y, M.columns.1.z))
    let sz = length(Vec3(M.columns.2.x, M.columns.2.y, M.columns.2.z))
    return (sx + sy + sz) / 3.0
}

// MARK: - TLAS Refit (for animation frames)
public extension RTContext {
    mutating func refitTLAS() {
        guard var tlas = self.tlas else {
            print("⚠️ No TLAS to refit")
            return
        }

        // Step 1: Recompute world bounds for all instances based on their updated transforms
        for i in 0..<instances.count {
            let inst = instances[i]
            var minW = Vec3(repeating: .infinity)
            var maxW = Vec3(repeating: -.infinity)
            for p in inst.blas.prims {
                let localB = p.bounds
                let worldB = localB.transformed(by: inst.localToWorld)
                minW = min(minW, worldB.minP)
                maxW = max(maxW, worldB.maxP)
            }
            let updatedBounds = AABB(minP: minW, maxP: maxW)
            let nMat = normalTransformMatrix(from: inst.localToWorld) // Do the math ONCE here
            instances[i] = Instance(
                blas: inst.blas,
                localToWorld: inst.localToWorld,
                worldToLocal: inst.localToWorld.inverse,
                worldBounds: updatedBounds,
                materialOverride: inst.materialOverride,
                normalMatrix: nMat

            )
        }

        // Step 2: Update TLAS primitive bounds and centroids
        for (i, inst) in instances.enumerated() {
            let b = inst.worldBounds
            let c = (b.minP + b.maxP) * 0.5
            tlas.builder.primitives[i].bounds = b
            tlas.builder.primitives[i].centroid = c
        }

        // Step 3: Refit BVH hierarchy (no rebuild)
        tlas.builder.refit(using: tlas.builder.primitives)

        // Step 4: Save back the updated TLAS
        self.tlas = tlas
    }
}

extension Vec3 {
    // Creates a coordinate system where 'self' is the Z-axis (the forward direction)
    func makeOrthonormalBasis() -> (u: Vec3, v: Vec3, w: Vec3) {
        let w = normalize(self)

        // Pick an arbitrary helper vector not parallel to W
        // If W is close to Y-axis, use X-axis, otherwise use Y-axis
        let helper = (Swift.abs(w.x) > 0.9) ? Vec3(x: 0, y: 1, z: 0) : Vec3(x: 1, y: 0, z: 0)

        let u = normalize(cross(w, helper))
        let v = cross(w, u) // v is automatically normalized since w and u are orthogonal unit vectors

        return (u, v, w)
    }

    static func random(min: Double, max: Double) -> Double {
        return Double.random(in: min...max)
    }
}

extension RTContext {
    @inlinable
    func orthonormalBasis(_ n: Vec3) -> (Vec3, Vec3) {
        let sign = n.z >= 0 ? 1.0 : -1.0
        let a = -1.0 / (sign + n.z)
        let b = n.x * n.y * a

        let tangent = normalize(Vec3(
            1.0 + sign * n.x * n.x * a,
            sign * b,
            -sign * n.x
        ))

        let bitangent = normalize(Vec3(
            b,
            sign + n.y * n.y * a,
            -n.y
        ))

        return (tangent, bitangent)
    }


}

public extension Vec3 {

    @inlinable
    /// only supports translations for now
    func withMotionBlur(_ motionVector: Vec3) -> Vec3 {
        self + motionVector
    }
}
