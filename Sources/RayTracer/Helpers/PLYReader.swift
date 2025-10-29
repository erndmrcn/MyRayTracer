//
//  PLYLoader.swift
//  RayTracer
//
//  Safe Swift translation using C wrapper for miniply
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
    case char = 0
    case uchar = 1
    case short = 2
    case ushort = 3
    case int = 4
    case uint = 5
    case float = 6
    case double = 7
    case none = 8
}

// ------------------------------------------------------------
// MARK: - PLY Loader
// ------------------------------------------------------------
public enum PLYLoader {

    public static func load(from path: String) throws -> PlyMesh {
        print("Entered load")
        var mesh = PlyMesh()
        var gotVerts = false
        var gotFaces = false

        // Create reader using C wrapper
        guard let reader = ply_reader_create(path) else {
            throw PlyError.fileOpenFailed(path)
        }
        defer { ply_reader_destroy(reader) }

        print("reader created")
        guard ply_reader_valid(reader) else {
            throw PlyError.fileOpenFailed(path)
        }
        print("reader is valid")

        var propIdxs = [UInt32](repeating: 0, count: 3)

        // Iterate elements
        while ply_reader_has_element(reader) && (!gotVerts || !gotFaces) {
            print("reader has element")

            // -----------------------------
            // Vertices
            // -----------------------------
            if ply_reader_element_is(reader, "vertex"),
               ply_reader_load_element(reader),
               ply_reader_find_pos(reader, &propIdxs) {

                let n = Int(ply_reader_num_rows(reader))
                print("Found \(n) vertices")

                // positions
                var posF = [Float](repeating: 0, count: n * 3)
                posF.withUnsafeMutableBytes {
                    _ = ply_reader_extract_properties(reader, &propIdxs, 3,
                                                     PLYPropertyType.float.rawValue,
                                                     $0.baseAddress)
                }

                mesh.positions.reserveCapacity(n)
                for i in 0..<n {
                    let x = Double(posF[3*i+0])
                    let y = Double(posF[3*i+1])
                    let z = Double(posF[3*i+2])
                    mesh.positions.append(Vec3(x, y, z))
                }

                // optional texcoords
                if ply_reader_find_texcoord(reader, &propIdxs) {
                    var uvF = [Float](repeating: 0, count: n * 2)
                    uvF.withUnsafeMutableBytes {
                        _ = ply_reader_extract_properties(reader, &propIdxs, 2,
                                                         PLYPropertyType.float.rawValue,
                                                         $0.baseAddress)
                    }
                    mesh.texcoords.reserveCapacity(n)
                    for i in 0..<n {
                        mesh.texcoords.append(SIMD2<Float>(uvF[2*i+0], uvF[2*i+1]))
                    }
                }

                gotVerts = true
            }

            // -----------------------------
            // Faces
            // -----------------------------
            else if ply_reader_element_is(reader, "face"),
                    ply_reader_load_element(reader),
                    ply_reader_find_indices(reader, &propIdxs) {

                let idxProp = propIdxs[0]
                let needsTri = ply_reader_requires_triangulation(reader, idxProp)

                print("Found faces, needs triangulation: \(needsTri)")

                if needsTri && !gotVerts {
                    throw PlyError.triangulationNeedsVerts
                }

                if needsTri {
                    // Build a float-position buffer for triangulation helper
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
                    // Faces are already triangles; read concatenated list
                    let total = Int(ply_reader_sum_of_list_counts(reader, idxProp))
                    var rawI32 = [Int32](repeating: 0, count: total)
                    rawI32.withUnsafeMutableBytes {
                        _ = ply_reader_extract_list_property(reader, idxProp,
                                                            PLYPropertyType.int.rawValue,
                                                            $0.baseAddress)
                    }
                    mesh.indices = rawI32.map(Int.init)
                }

                gotFaces = true
            }

            if gotVerts && gotFaces { break }
            ply_reader_next_element(reader)
        }

        if !gotVerts { throw PlyError.vertexDataMissing }
        if !gotFaces { throw PlyError.faceDataMissing }

        print("Successfully loaded: \(mesh.positions.count) vertices, \(mesh.indices.count/3) triangles")
        return mesh
    }
}
