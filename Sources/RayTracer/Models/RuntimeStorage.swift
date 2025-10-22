//
//  Runtime.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import ParsingKit

struct RuntimeStorage: Sendable {
    let positions = [Vec3]()
    let triIdx    = [Int32]()
    let v0 = [Vec3](),
               v1 = [Vec3](),
               v2 = [Vec3](),
               e1 = [Vec3](),
               e2 = [Vec3]()
    let triObj = [Int32](), triMat = [Int32]()
    let sphCenter = [Vec3](), sphRadius = [Scalar]()
    let sphObj = [Int32](), sphMat = [Int32]()

    let planeCenter = [Vec3]()
    let planeNormal = [Vec3]()
    let planeObj = [Int32](), planeMat = [Int32]()

    let objects = [SceneObject]()
    let materials = [Material]()
    let materialIndexById = [String: Int32]()
    let objectIndexByIdentity = [ObjectIdentifier: Int32]()
}
