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

//// Vector from surface point to light (unnormalized)
//let toLight = position - p
//let distanceSquared = length_squared(toLight)
//
//// Normalized light direction
//let L = normalize(toLight)
//
//// Light intensity with inverse square falloff
//// Intensity is already in format like 25550, so normalize it
//let Li = intensity / distanceSquared
//
//// ============ DIFFUSE (Lambertian) ============
//let NdotL = max(0.0, dot(normal, L))
//if NdotL <= 0.0 { return .zero }  // Light behind surface
//
//var diffuse = NdotL * material.diffuse
//
//// ============ SPECULAR (Blinn-Phong) ============
//// View direction (from surface to camera)
//let V = normalize(ray.origin - p)
//
//// Half vector between light and view
//let H = normalize(L + V)                         // not V - L
//let spec = pow(max(dot(normal, H), 0.0), material.phong)
//
//// Specular term
////        let NdotH = max(0.0, dot(normal, H))
////            let specular = pow(NdotH, material.phong)
//var specular = spec * material.specular * 2.0
//
//return (specular + diffuse) * Li
//}
// Alternative implementation without normalization in material colors
// (if you normalize materials in buildRuntime instead):
/*
extension PointLight {
    func shade(material: Material, ray: Ray, normal: Vec3, at p: Vec3) -> Vec3 {
        let toLight = position - p
        let distanceSquared = length_squared(toLight)
        let L = normalize(toLight)

        // If materials are already normalized (0-1 range)
        let Li = intensity / max(distanceSquared, 1e-12)

        let NdotL = max(0.0, dot(normal, L))
        if NdotL <= 0.0 { return .zero }

        var color = Li * NdotL * material.diffuse

        let V = normalize(-ray.dir)
        let H = normalize(L + V)
        let NdotH = max(0.0, dot(normal, H))

        if NdotH > 0.0 {
            color += Li * pow(NdotH, material.phong) * material.specular
        }

        return color
    }
}
*/
