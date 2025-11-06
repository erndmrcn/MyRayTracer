//
//  Math.swift
//  RayTracer
//
//  Created by Eren Demircan on 2.11.2025.
//

import simd
import ParsingKit

@inline(__always)
func randomInUnitSphere() -> Vec3 {
    var p: Vec3
    repeat {
        p = 2.0 * Vec3(drand48(), drand48(), drand48()) - Vec3(1,1,1)
    } while simd_length_squared(p) >= 1.0
    return p
}

@inline(__always)
func clampVec(_ v: Vec3, _ minVal: Double, _ maxVal: Double) -> Vec3 {
    return Vec3(clamp(v.x, minVal, maxVal),
                clamp(v.y, minVal, maxVal),
                clamp(v.z, minVal, maxVal))
}

@inline(__always)
func clamp(_ x: Double, _ minVal: Double, _ maxVal: Double) -> Double {
    return Swift.max(minVal, Swift.min(maxVal, x))
}

@inline(__always)
func reflect(_ v: Vec3, _ n: Vec3) -> Vec3 {
    return v - 2.0 * dot(v, n) * n
}

@inline(__always)
func refract(_ v: Vec3, _ n: Vec3, eta: Double) -> (dir: Vec3, wasRefracted: Bool) {
    let cosi = clamp(-1.0, 1.0, dot(v, n))
    let etai = 1.0
    let etat = eta
    var nDir = n
    var iorRatio = etai / etat
    var cos = cosi
    if cosi < 0 {
        cos = -cosi
    } else {
        nDir = -nDir
        iorRatio = etat / etai
    }

    let k = 1.0 - iorRatio * iorRatio * (1.0 - cos * cos)
    if k < 0.0 {
        return (reflect(v, nDir), false)
    } else {
        return (iorRatio * v + (iorRatio * cos - sqrt(k)) * nDir, true)
    }
}

