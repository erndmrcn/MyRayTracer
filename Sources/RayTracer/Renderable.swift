//
//  Renderable.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.11.2025.
//

import ParsingKit

public protocol Renderable: AnyObject {
    func buildMeshBase(scene: Scene) -> MeshBase
}

public protocol AnalyticRenderable: AnyObject {
    func intersect(ray: inout Ray, eps: Scalar, materialIndex: Int)
}
