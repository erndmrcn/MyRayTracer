//
//  PLYLoader.swift
//  RayTracer
//

import Foundation
import ParsingKit
import simd
import CPly

// ------------------------------------------------------------
// MARK: - Mesh Struct
// ------------------------------------------------------------
public struct PlyMesh {
    public var positions: [Vec3] = []
    public var normals: [Vec3] = []
    public var texcoords: [SIMD2<Float>] = []
    public var indices: [Int] = []
}

// ------------------------------------------------------------
// MARK: - Error Enum
// ------------------------------------------------------------
public enum PlyError: Error, CustomStringConvertible {
    case fileOpenFailed(String)
    case vertexDataMissing
    case faceDataMissing
    case triangulationNeedsVerts
    case corrupted

    public var description: String {
        switch self {
        case .fileOpenFailed(let path): return "Cannot open PLY file at: \(path)"
        case .vertexDataMissing:        return "Vertex element missing."
        case .faceDataMissing:          return "Face element missing."
        case .triangulationNeedsVerts:  return "Need vertex positions to triangulate faces."
        case .corrupted:                return "Corrupted or unsupported PLY file."
        }
    }
}

// ------------------------------------------------------------
// MARK: - PLY Property Type (matching C++ enum)
// ------------------------------------------------------------
private enum PLYPropertyType: Int32 {
    case char = 0, uchar, short, ushort, int, uint, float, double, none
}

// ------------------------------------------------------------
// MARK: - PLY Loader
// ------------------------------------------------------------
public enum PLYLoader {

    public static func load(from path: String) throws -> PlyMesh {
        print("➡️ Loading PLY:", path)

        guard let reader = ply_reader_create(path) else {
            throw PlyError.fileOpenFailed(path)
        }
        defer { ply_reader_destroy(reader) }

        guard ply_reader_valid(reader) else {
            throw PlyError.corrupted
        }

        var mesh = PlyMesh()
        var propIdxs = [UInt32](repeating: 0, count: 3)
        var gotVerts = false
        var gotFaces = false

        // --------------------------------------------------------
        // Iterate through all elements
        // --------------------------------------------------------
        while ply_reader_has_element(reader) {
            // -----------------------------
            // VERTICES
            // -----------------------------
            if ply_reader_element_is(reader, "vertex"),
               ply_reader_load_element(reader),
               ply_reader_find_pos(reader, &propIdxs) {

                let n = Int(ply_reader_num_rows(reader))

                // Positions
                var posF = [Float](repeating: 0, count: n * 3)
                posF.withUnsafeMutableBytes {
                    _ = ply_reader_extract_properties(
                        reader,
                        &propIdxs,
                        3,
                        PLYPropertyType.float.rawValue,
                        $0.baseAddress
                    )
                }

                mesh.positions.reserveCapacity(n)
                for i in 0..<n {
                    let x = Double(posF[3*i+0])
                    let y = Double(posF[3*i+1])
                    let z = Double(posF[3*i+2])
                    mesh.positions.append(Vec3(x, y, z))
                }

                // Optional normals
                if ply_reader_find_normals(reader, &propIdxs) {
                    var normF = [Float](repeating: 0, count: n * 3)
                    normF.withUnsafeMutableBytes {
                        _ = ply_reader_extract_properties(
                            reader,
                            &propIdxs,
                            3,
                            PLYPropertyType.float.rawValue,
                            $0.baseAddress
                        )
                    }
                    mesh.normals.reserveCapacity(n)
                    for i in 0..<n {
                        let nx = Double(normF[3*i+0])
                        let ny = Double(normF[3*i+1])
                        let nz = Double(normF[3*i+2])
                        mesh.normals.append(normalize(Vec3(nx, ny, nz)))
                    }
                }

                // Optional texcoords
                if ply_reader_find_texcoord(reader, &propIdxs) {
                    var uvF = [Float](repeating: 0, count: n * 2)
                    uvF.withUnsafeMutableBytes {
                        _ = ply_reader_extract_properties(
                            reader,
                            &propIdxs,
                            2,
                            PLYPropertyType.float.rawValue,
                            $0.baseAddress
                        )
                    }
                    mesh.texcoords.reserveCapacity(n)
                    for i in 0..<n {
                        mesh.texcoords.append(SIMD2<Float>(uvF[2*i+0], uvF[2*i+1]))
                    }
                }

                gotVerts = true
            }

            // -----------------------------
            // FACES
            // -----------------------------
            else if ply_reader_element_is(reader, "face"),
                    ply_reader_load_element(reader),
                    ply_reader_find_indices(reader, &propIdxs) {

                let idxProp = propIdxs[0]
                let needsTri = ply_reader_requires_triangulation(reader, idxProp)

                if needsTri && !gotVerts {
                    throw PlyError.triangulationNeedsVerts
                }

                if needsTri {
                    let numVerts = mesh.positions.count
                    var posF = [Float](repeating: 0, count: numVerts * 3)
                    for i in 0..<numVerts {
                        let p = mesh.positions[i]
                        posF[3*i+0] = Float(p.x)
                        posF[3*i+1] = Float(p.y)
                        posF[3*i+2] = Float(p.z)
                    }

                    let triCount = Int(ply_reader_num_triangles(reader, idxProp))
                    var triI32 = [Int32](repeating: 0, count: triCount * 3)

                    posF.withUnsafeBufferPointer { posBuf in
                        triI32.withUnsafeMutableBufferPointer { triBuf in
                            _ = ply_reader_extract_triangles(
                                reader,
                                idxProp,
                                posBuf.baseAddress,
                                UInt32(numVerts),
                                PLYPropertyType.int.rawValue,
                                triBuf.baseAddress
                            )
                        }
                    }
                    mesh.indices = triI32.map(Int.init)
                } else {
                    let total = Int(ply_reader_sum_of_list_counts(reader, idxProp))
                    var rawI32 = [Int32](repeating: 0, count: total)
                    rawI32.withUnsafeMutableBytes {
                        _ = ply_reader_extract_list_property(
                            reader,
                            idxProp,
                            PLYPropertyType.int.rawValue,
                            $0.baseAddress
                        )
                    }
                    mesh.indices = rawI32.map(Int.init)
                }

                gotFaces = true
            }

            ply_reader_next_element(reader)
        }

        if !gotVerts { throw PlyError.vertexDataMissing }
        if !gotFaces { throw PlyError.faceDataMissing }

        return mesh
    }
}
