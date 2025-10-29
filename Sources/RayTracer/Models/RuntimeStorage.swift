//
//  Runtime.swift
//  RayTracer
//
//  Created by Eren Demircan on 10.10.2025.
//

import ParsingKit

struct RuntimeStorage: Sendable {
    let positions = [Vec3]()
    let triIdx    = [Int]()
    let v0 = [Vec3](),
               v1 = [Vec3](),
               v2 = [Vec3](),
               e1 = [Vec3](),
               e2 = [Vec3]()
    let triObj = [Int](), triMat = [Int]()
    let sphCenter = [Vec3](), sphRadius = [Scalar]()
    let sphObj = [Int](), sphMat = [Int]()

    let planeCenter = [Vec3]()
    let planeNormal = [Vec3]()
    let planeObj = [Int](), planeMat = [Int]()

    let objects = [SceneObject]()
    let materials = [Material]()
    let materialIndexById = [String: Int]()
    let objectIndexByIdentity = [ObjectIdentifier: Int]()
}
