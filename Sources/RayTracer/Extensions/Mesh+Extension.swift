//
//  Mesh+Extension.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.11.2025.
//

import ParsingKit
import Foundation
import simd

//extension Mesh: Renderable {
//    public func buildMeshBase(scene: Scene) -> MeshBase {
//        let idStr = id ?? UUID().uuidString
//        let (positions, indices, normals) = MeshHelper.loadMeshData(self, scene: scene)
////        let bounds = AABB(points: positions)
//        return MeshBase(
//            id: idStr,
//            type: .mesh,
//            materialIndex: MeshHelper.materialIndex(for: material, in: scene.materials),
//            positions: positions,
//            indices: indices,
//            normals: normals,
//            bounds: bounds,
//            blasRoot: nil,
//            blasPrims: []
//        )
//    }
//}
