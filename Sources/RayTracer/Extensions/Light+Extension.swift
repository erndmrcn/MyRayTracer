//
//  Light+Extension.swift
//  RayTracer
//
//  Created by Eren Demircan on 7.10.2025.
//

import ParsingKit
import simd

extension PointLight {
    @inline(__always)
    private func safeNormalize(_ v: Vec3) -> Vec3 {
        let len = simd_length(v)
        return len > 0 ? v / len : .zero
    }

    func shade(material: Material, ray: Ray, cameraPos: Vec3, normal: Vec3, at p: Vec3) -> Vec3 {
        let N = normalize(normal)
        let V = normalize(cameraPos - p)            // or normalize(-ray.dir) for primary

        let toL   = position - p
        let r2    = max(length_squared(toL), 1e-6)
        let L     = normalize(toL)
        let ndotl = max(dot(N, L), 0.0)
        if ndotl == 0 { return .zero }

        // incoming radiance from the point light (pick your convention; here: intensity/r^2)
        let Li = intensity / r2

        // Phong specular: reflect the *incident* direction (-L)
        let H = safeNormalize(L + V)
        let spec = pow(max(dot(N, H), 0.0), material.phong)

        // dielectric: do not tint specular by base color
        let diffuse  = material.diffuse  * ndotl
        let specular = material.specular * spec

        return (diffuse + specular) * Li
    }
}
