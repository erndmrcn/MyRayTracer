//
//  RTContext.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import ParsingKit

public struct BVHPrim: Sendable {
    public let type: UInt8
    public let index: Int
}

public struct RTContext: Sendable {
    // Scalars
    let intersectionTestEpsilon: Scalar
    let shadowRayEpsilon: Scalar

    // Lookups
    let materialIndexById: [String: Int32]
    let objectIndexByIdentity: [ObjectIdentifier: Int32]

    // Geometry tables
    let positions: [Vec3]

    // Triangles
    let triIndex: [Int32]
    let triObj: [Int32]
    let triMat: [Int32]
    let v0: [Vec3], v1: [Vec3], v2: [Vec3]
    let e1: [Vec3], e2: [Vec3]

    // Spheres
    let sphCenter: [Vec3], sphRadius: [Scalar], sphObj: [Int32], sphMat: [Int32]

    // Planes
    let planeCenter: [Vec3], planeNormal: [Vec3], planeObj: [Int32], planeMat: [Int32]

    var planeCount: Int {
        planeCenter.count
    }

    let bvhRoot: BVHNode?
    let bvhPrims: [BVHPrim]
}

extension RTContext {
    init(scene: Scene) {
        self.intersectionTestEpsilon = scene.intersectionTestEpsilon
        self.shadowRayEpsilon = scene.shadowRayEpsilon

        // Materials
        var matById: [String: Int32] = [:]
        for (i, m) in scene.materials.enumerated() {
            if let id = m.id { matById[id] = Int32(i) }
        }
        self.materialIndexById = matById

        // Objects
        var objByIdent: [ObjectIdentifier: Int32] = [:]
        for (i, obj) in scene.objects.enumerated() {
            objByIdent[ObjectIdentifier(obj)] = Int32(i)
        }
        self.objectIndexByIdentity = objByIdent

        // Vertices
        let positions = scene.vertexData.data
        self.positions = positions

        // Triangles (Triangle + Mesh.faces)
        var triIdx: [Int32] = []
        var triObj: [Int32] = []
        var triMat: [Int32] = []
        triIdx.reserveCapacity(scene.objects.count * 6)
        triObj.reserveCapacity(scene.objects.count * 2)
        triMat.reserveCapacity(scene.objects.count * 2)

        func matIdx(_ id: String?) -> Int32 {
            guard let id = id, let mi = matById[id] else { return -1 }
            return mi
        }

        for obj in scene.objects {
            let oid = objByIdent[ObjectIdentifier(obj)] ?? -1

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
        var v0 = [Vec3](repeating: .zero, count: triCount)
        var v1 = [Vec3](repeating: .zero, count: triCount)
        var v2 = [Vec3](repeating: .zero, count: triCount)
        var e1 = [Vec3](repeating: .zero, count: triCount)
        var e2 = [Vec3](repeating: .zero, count: triCount)

        for t in 0..<triCount {
            let i0 = Int(triIdx[3*t+0]), i1 = Int(triIdx[3*t+1]), i2 = Int(triIdx[3*t+2])
            let p0 = positions[i0], p1 = positions[i1], p2 = positions[i2]
            v0[t] = p0; v1[t] = p1; v2[t] = p2
            e1[t] = p1 - p0; e2[t] = p2 - p0
        }

        // Spheres & planes (no mutation back into scene)
        var sphC: [Vec3] = [], sphR: [Scalar] = [], sphObj: [Int32] = [], sphMat: [Int32] = []
        var planeC: [Vec3] = [], planeN: [Vec3] = [], planeObj: [Int32] = [], planeMat: [Int32] = []

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

        self.bvhRoot = nil
        self.bvhPrims = []
    }
}

extension RTContext {
    func buildBVH(maxLeaf: Int = 8, split: BVHSplitMode = .sah) -> RTContext {
        let builder = BVHBuilder(maxLeaf: maxLeaf, split: split)
        let (root, prims) = builder.buildFinite(
            triCount: v0.count,
            sphereCount: sphCenter.count,
            context: self
        )

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
            sphCenter: sphCenter, sphRadius: sphRadius,
            sphObj: sphObj, sphMat: sphMat,
            planeCenter: planeCenter, planeNormal: planeNormal,
            planeObj: planeObj, planeMat: planeMat,
            bvhRoot: root,
            bvhPrims: prims
        )
    }
}
