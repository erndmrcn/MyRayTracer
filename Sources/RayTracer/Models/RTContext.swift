//
//  RTContext.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import ParsingKit
import simd
import Foundation
//import CPly

public struct BVHPrim: Sendable {
    public let type: UInt8
    public let index: Int
}

public struct RTContext: Sendable {
    // Scalars
    let intersectionTestEpsilon: Scalar
    let shadowRayEpsilon: Scalar

    // Lookups
    let materialIndexById: [String: Int]
    let objectIndexByIdentity: [ObjectIdentifier: Int]

    // Geometry tables
    var positions: [Vec3]

    // Triangles
    let triIndex: [Int]
    let triObj: [Int]
    let triMat: [Int]
    let v0: [Vec3], v1: [Vec3], v2: [Vec3]
    let e1: [Vec3], e2: [Vec3]

    let triSmooth: [UInt8]

    let triN0: [Vec3]
    let triN1: [Vec3]
    let triN2: [Vec3]

    // Spheres
    let sphCenter: [Vec3], sphRadius: [Scalar], sphObj: [Int], sphMat: [Int]

    // Planes
    let planeCenter: [Vec3], planeNormal: [Vec3], planeObj: [Int], planeMat: [Int]

    var planeCount: Int {
        planeCenter.count
    }

    var hasSmoothNormals: Bool = false

    let bvhRoot: BVHNode?
    let bvhPrims: [BVHPrim]
}

extension RTContext {
    init(scene: Scene) {
        self.intersectionTestEpsilon = scene.intersectionTestEpsilon
        self.shadowRayEpsilon = scene.shadowRayEpsilon
        self.hasSmoothNormals = scene.cameras.contains(where: {$0.imageName.contains("smooth")})

        // Materials
        var matById: [String: Int] = [:]
        for (i, m) in scene.materials.enumerated() {
            if let id = m.id { matById[id] = Int(i) }
        }
        self.materialIndexById = matById

        // Objects
        var objByIdent: [ObjectIdentifier: Int] = [:]
        for (i, obj) in scene.objects.enumerated() {
            objByIdent[ObjectIdentifier(obj)] = Int(i)
        }
        self.objectIndexByIdentity = objByIdent

        // Vertices
        var positions = scene.vertexData.data

        // Triangles (Triangle + Mesh.faces)
        var triIdx: [Int] = []
        var triObj: [Int] = []
        var triMat: [Int] = []
        var triSmooth: [UInt8] = []

        triIdx.reserveCapacity(scene.objects.count * 6)
        triObj.reserveCapacity(scene.objects.count * 2)
        triMat.reserveCapacity(scene.objects.count * 2)
        triSmooth.reserveCapacity(scene.objects.count * 2)

        func isSmooth(_ mode: String?) -> Bool {
            guard let m = mode else { return false }
            return m.lowercased() == "smooth"
        }

        func matIdx(_ id: String?) -> Int {
            guard let id = id, let mi = matById[id] else { return -1 }
            return mi
        }

        for obj in scene.objects {
            let oid = objByIdent[ObjectIdentifier(obj)] ?? -1

            if let t = obj as? Triangle, t.indices.count >= 3 {
                let i0 = Int(max(0, t.indices[0] - 1))
                let i1 = Int(max(0, t.indices[1] - 1))
                let i2 = Int(max(0, t.indices[2] - 1))
                triIdx += [i0, i1, i2]
                triObj.append(oid)
                triMat.append(matIdx(t.material))
                triSmooth.append(0)
            } else if let m = obj as? Mesh {
                let smooth = isSmooth(m.shadingMode) ? UInt8(1) : UInt8(0)

                if let plyPath = m.faces.plyPath {
                    var cwd = scene.path?.deletingLastPathComponent()
                    if cwd == nil {
                        cwd = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
                    }


                    let plyURL = cwd!.appendingPathComponent(plyPath)
                    do {
                        let mesh = try PLYLoader.load(from: plyURL.path)

                        let offset = positions.count
                        positions.append(contentsOf: mesh.positions.map({Vec3($0)}))

                        for k in stride(from: 0, to: mesh.indices.count, by: 3) {
                            let f1 = offset + mesh.indices[k]
                            let f2 = offset + mesh.indices[k+1]
                            let f3 = offset + mesh.indices[k+2]

                            triIdx += [f1, f2, f3]
                            triObj.append(oid)
                            triMat.append(matIdx(m.material))
                            triSmooth.append(smooth)
                        }
                    } catch {
                        print("Error loading PLY file at \(plyURL.path): \(error)")
                    }
                } else {
                    let face = m.faces.data
                    let n = (face.count / 3) * 3
                    var k = 0
                    while k < n {
                        let i0 = max(0, face[k+0] - 1)
                        let i1 = max(0, face[k+1] - 1)
                        let i2 = max(0, face[k+2] - 1)
                        triIdx += [i0, i1, i2]
                        triObj.append(oid)
                        triMat.append(matIdx(m.material))
                        triSmooth.append(smooth)
                        k += 3
                    }
                }
            }
        }

        let triCount = triIdx.count / 3

        var v0 = [Vec3](repeating: .zero, count: triCount)
        var v1 = [Vec3](repeating: .zero, count: triCount)
        var v2 = [Vec3](repeating: .zero, count: triCount)
        var e1 = [Vec3](repeating: .zero, count: triCount)
        var e2 = [Vec3](repeating: .zero, count: triCount)

        var faceNormals = [Vec3](repeating: .zero, count: triCount)

        for t in 0..<triCount {
            let i0 = Int(triIdx[3*t+0])
            let i1 = Int(triIdx[3*t+1])
            let i2 = Int(triIdx[3*t+2])

            let p0 = positions[i0]
            let p1 = positions[i1]
            let p2 = positions[i2]

            v0[t] = p0
            v1[t] = p1
            v2[t] = p2

            let e10 = p1 - p0
            let e20 = p2 - p0
            e1[t] = e10
            e2[t] = e20

            faceNormals[t] = cross(e10, e20)
        }

        let vertexCount = positions.count
        var vertexNormals = [Vec3](repeating: .zero, count: vertexCount)

        for t in 0..<triCount {
            if triSmooth[t] != 0 {
                let i0 = Int(triIdx[3*t+0])
                let i1 = Int(triIdx[3*t+1])
                let i2 = Int(triIdx[3*t+2])

                let fn = faceNormals[t]
                let area = 0.5 * simd_length(fn)
                vertexNormals[i0] += area * simd_normalize(fn)
                vertexNormals[i1] += area * simd_normalize(fn)
                vertexNormals[i2] += area * simd_normalize(fn)
            }
        }

        for i in 0..<vertexCount {
            let n = vertexNormals[i]
            vertexNormals[i] = simd_length_squared(n) > 0 ? simd_normalize(n) : Vec3(0, 1, 0) // fallback
        }

        var n0s = [Vec3](repeating: .zero, count: triCount)
        var n1s = [Vec3](repeating: .zero, count: triCount)
        var n2s = [Vec3](repeating: .zero, count: triCount)

        for t in 0..<triCount {
            let i0 = triIdx[3*t+0]
            let i1 = triIdx[3*t+1]
            let i2 = triIdx[3*t+2]
            if triSmooth[t] != 0 {
                n0s[t] = vertexNormals[i0]
                n1s[t] = vertexNormals[i1]
                n2s[t] = vertexNormals[i2]
            } else {
                let fn = faceNormals[t]
                let gn = simd_length_squared(fn) > 0 ? simd_normalize(fn) : Vec3(0, 1, 0)
                n0s[t] = gn
                n1s[t] = gn
                n2s[t] = gn
            }
        }


        // Spheres & planes (no mutation back into scene)
        var sphC: [Vec3] = [], sphR: [Scalar] = [], sphObj: [Int] = [], sphMat: [Int] = []
        var planeC: [Vec3] = [], planeN: [Vec3] = [], planeObj: [Int] = [], planeMat: [Int] = []

        for obj in scene.objects {
            let oid = objByIdent[ObjectIdentifier(obj)] ?? -1
            if let s = obj as? Sphere {
                let c = positions[s.centerIdx - 1]
                sphC.append(c); sphR.append(s.radius)
                sphObj.append(oid); sphMat.append(matIdx(s.material))
            } else if let p = obj as? Plane {
                let c = positions[p.centerIdx - 1]
                planeC.append(c); planeN.append(p.normal)
                planeObj.append(oid); planeMat.append(matIdx(p.material))
            }
        }

        self.triIndex = triIdx
        self.triObj = triObj
        self.triMat = triMat
        self.v0 = v0; self.v1 = v1; self.v2 = v2
        self.e1 = e1; self.e2 = e2

        self.sphCenter = sphC; self.sphRadius = sphR
        self.sphObj = sphObj; self.sphMat = sphMat

        self.planeCenter = planeC; self.planeNormal = planeN
        self.planeObj = planeObj; self.planeMat = planeMat

        self.triSmooth = triSmooth
        self.triN0 = n0s
        self.triN1 = n1s
        self.triN2 = n2s

        self.positions = positions

        self.bvhRoot = nil
        self.bvhPrims = []
    }
}

extension RTContext {
    func buildBVH(maxLeaf: Int = 8, split: BVHSplitMode = .sah) -> RTContext {
        let builder = BVHBuilder(maxLeaf: maxLeaf, split: split)
        let start = DispatchTime.now()
        let (root, prims) = builder.buildFinite(
            triCount: v0.count,
            sphereCount: sphCenter.count,
            context: self
        )
        let elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - start.uptimeNanoseconds) / 1_000_000.0)
        print("BVH has built in \(elapsedMs) ms")

        return RTContext(
            intersectionTestEpsilon: intersectionTestEpsilon,
            shadowRayEpsilon: shadowRayEpsilon,
            materialIndexById: materialIndexById,
            objectIndexByIdentity: objectIndexByIdentity,
            positions: positions,
            triIndex: triIndex,
            triObj: triObj,
            triMat: triMat,
            v0: v0, v1: v1, v2: v2,
            e1: e1, e2: e2,
            triSmooth: triSmooth,
            triN0: triN0,
            triN1: triN1,
            triN2: triN2,
            sphCenter: sphCenter, sphRadius: sphRadius,
            sphObj: sphObj, sphMat: sphMat,
            planeCenter: planeCenter, planeNormal: planeNormal,
            planeObj: planeObj, planeMat: planeMat,
            bvhRoot: root,
            bvhPrims: prims
        )
    }
}
