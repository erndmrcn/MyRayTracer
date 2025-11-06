//
//  Ray.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.10.2025.
//

import ParsingKit
import simd

extension Ray {
    func transformed(by M: Mat4, inverse: Bool = false) -> Ray {
        let T = inverse ? M : simd_inverse(M)
        let o4 = T * Vec4(origin, 1)
        let d4 = T * Vec4(dir, 0)
        return Ray(origin: o4.xyz, dir: normalize(d4.xyz))
    }
}

extension Vec4 {
    var xyz: Vec3 {
        Vec3(x, y, z)
    }
}
