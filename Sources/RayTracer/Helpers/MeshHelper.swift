//
//  MeshHelper.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.11.2025.
//

import simd
import ParsingKit
import Foundation

enum MeshHelper {
    static func materialIndex(for id: String?, in materials: [RayTracer.Material]) -> Int {
        guard let id, !id.isEmpty else { return 0 }
        return materials.firstIndex(where: { $0.id == id }) ?? 0
    }

    static func loadMeshData(_ mesh: Mesh, scene: RayTracer.Scene) -> ([Vec3],[Int],[Vec3]) {
        var positions: [Vec3] = []
        var indices: [Int] = []
        var normals: [Vec3] = []

        if let plyPath = mesh.faces.plyPath {
            do {
                let baseURL = scene.path?.deletingLastPathComponent() ??
                              URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
                let url = baseURL.appendingPathComponent(plyPath)
                let ply = try PLYLoader.load(from: url.path)
                positions = ply.positions.map { Vec3($0) }
                indices = ply.indices
                normals = computeVertexNormals(positions: positions, indices: indices)
            } catch {
                print("PLY load error: \(error)")
            }
        } else {
            let global = scene.vertexData.data
            let face = mesh.faces.data
            guard !face.isEmpty else { return ([], [], []) }

            var remap: [Int: Int] = [:]
            remap.reserveCapacity(face.count)

            for g1 in face {
                let g = max(0, g1 - 1)
                if let found = remap[g] {
                    indices.append(found)
                } else {
                    let newIdx = positions.count
                    remap[g] = newIdx
                    positions.append(global[g])
                    indices.append(newIdx)
                }
            }
            normals = computeVertexNormals(positions: positions, indices: indices)
        }
        return (positions, indices, normals)
    }

    static func triangulateSphere(center: Vec3, radius: Scalar,
                                  segments: Int = 16, rings: Int = 12) -> ([Vec3],[Int]) {
        var pos: [Vec3] = []
        var idx: [Int] = []
        let cols = segments + 1
        for y in 0...rings {
            let v = Scalar(y) / Scalar(rings)
            let phi = v * .pi
            for x in 0...segments {
                let u = Scalar(x) / Scalar(segments)
                let theta = u * 2 * .pi
                let px = radius * sin(phi) * cos(theta)
                let py = radius * sin(phi) * sin(theta)
                let pz = radius * cos(phi)
                pos.append(center + Vec3(px, py, pz))
            }
        }
        for y in 0..<rings {
            for x in 0..<segments {
                let i0 = y * cols + x
                let i1 = i0 + 1
                let i2 = i0 + cols
                let i3 = i2 + 1
                idx += [i0, i2, i1, i1, i2, i3]
            }
        }
        return (pos, idx)
    }

    static func triangulatePlane(center: Vec3, normal n: Vec3, size: Scalar) -> ([Vec3],[Int]) {
        let nn = simd_normalize(n)
        let a: Vec3 = Swift.abs(nn.x) > 0.5 ? Vec3(0,1,0) : Vec3(1,0,0)
        let tangent = simd_normalize(simd_cross(nn, a))
        let bitangent = simd_normalize(simd_cross(nn, tangent))
        let s = size * 0.5
        let p0 = center + (-tangent * s) + (-bitangent * s)
        let p1 = center + ( tangent * s) + (-bitangent * s)
        let p2 = center + ( tangent * s) + ( bitangent * s)
        let p3 = center + (-tangent * s) + ( bitangent * s)
        return ([p0,p1,p2,p3], [0,1,2,2,3,0])
    }

    static func computeVertexNormals(positions: [Vec3], indices: [Int]) -> [Vec3] {
        var nrm = [Vec3](repeating: .zero, count: positions.count)
        for t in stride(from: 0, to: indices.count, by: 3) {
            let i0 = indices[t], i1 = indices[t+1], i2 = indices[t+2]
            let n = simd_cross(positions[i1]-positions[i0], positions[i2]-positions[i0])
            nrm[i0] += n; nrm[i1] += n; nrm[i2] += n
        }
        for i in nrm.indices {
            let len2 = simd_length_squared(nrm[i])
            nrm[i] = (len2 > 0) ? simd_normalize(nrm[i]) : Vec3(0,1,0)
        }
        return nrm
    }
}
