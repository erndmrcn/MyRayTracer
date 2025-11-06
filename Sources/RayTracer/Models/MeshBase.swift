import ParsingKit
import simd

public struct MeshBase: Sendable {
    public let id: String
    public let type: GeometryType
    public var materialIndex: Int
    public var positions: [Vec3]
    public var indices: [Int]
    public var normals: [Vec3]
    public var bounds: AABB

    public var blasRoot: BVHNode?
    public var blasPrims: [BVHPrim] = []

//    static func empty(id: String, type: GeometryType) -> MeshBase {
//        MeshBase(id: id, type: type, materialIndex: 0,
//                 positions: [], indices: [], normals: [],
//                 bounds: AABB(), blasRoot: nil, blasPrims: [])
//    }
}

//extension MeshBase {
//    mutating func applyTransform(_ M: Mat4) {
//        for i in 0..<positions.count {
//            positions[i] = (M * Vec4(positions[i], 1)).xyz
//        }
//        for i in 0..<normals.count {
//            normals[i] = normalize((M.inverse.transpose * Vec4(normals[i], 0)).xyz)
//        }
//        bounds = AABB(points: positions)
//    }
//}
