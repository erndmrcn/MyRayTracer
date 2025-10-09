//
//  Object+Extension.swift
//  RayTracer
//
//  Created by Eren Demircan on 7.10.2025.
//

import ParsingKit
import simd

@MainActor @inline(__always)
func intersectPlanes(ray: inout Ray) -> Bool {
    var hit = false
    let eps = RTContext.intersectionTestEpsilon

    let nPlanes = RTContext.planeCount
    let C = RTContext.planeCenter!
    let N = RTContext.planeNormal!
    let O = RTContext.planeObj!
    let M = RTContext.planeMat!

    for i in 0..<nPlanes {
        let denom = dot(ray.dir, N[i])

        if abs(denom) < eps { continue }

        let ao = C[i] - ray.origin
        let nom = dot(ao, N[i])
        let t = nom/denom

        if t < eps || t >= ray.tMax { continue }

        hit = true
        ray.kind = .plane
        ray.prim = Int32(i)
        ray.obj  = O[i]
        ray.mat  = M[i]
        ray.tMax = t
        ray.normal = (denom < 0) ? RTContext.planeNormal[i] : -RTContext.planeNormal[i]
    }

    return hit
}

@MainActor @inline(__always)
func intersectTriangles(ray: inout Ray) -> Bool {
    var hit = false
    let eps = RTContext.intersectionTestEpsilon

    let nTris = RTContext.triCount
    let v0 = RTContext.v0!
    let e1 = RTContext.e1!
    let e2 = RTContext.e2!
    let triObj = RTContext.triObj!
    let triMat = RTContext.triMat!

    for t in 0..<nTris {
        // Moller-Trumbore against precomputed edges
        let p = cross(ray.dir, e2[t])
        let det = dot(e1[t], p)
        if det < eps { continue }
        let invDet = 1 / det

        let tvec = ray.origin - v0[t]
        let u = dot(tvec, p) * invDet
        if u < 0.0 || u > 1.0 { continue }

        let q = cross(tvec, e1[t])
        let v = dot(ray.dir, q) * invDet
        if v < 0.0 || u + v > 1.0 { continue }

        let tHit = dot(e2[t], q) * invDet
        if tHit <= eps || tHit >= ray.tMax { continue }

        ray.tMax = tHit
        ray.kind = .triangle
        ray.prim = Int32(t)
        ray.obj  = triObj[t]
        ray.mat  = triMat[t]
        ray.bary = SIMD3<Scalar>(1-u-v, u, v)
        #if SMOOTH_NORMAL
        // Suppose you have per-vertex normals parallel to positions, e.g.:
        let n0 = RuntimeStorage.vertexNormals[Int(RTContext.triIndex[3*t+0])]
        let n1 = RuntimeStorage.vertexNormals[Int(RTContext.triIndex[3*t+1])]
        let n2 = RuntimeStorage.vertexNormals[Int(RTContext.triIndex[3*t+2])]

        // Interpolate with barycentrics (w,u,v) = (1-u-v, u, v)
        let w = 1 - u - v
        let smooth = normalize(n0 * w + n1 * u + n2 * v)

        // Use smooth if available; fall back to geometric if not
        ray.normal = smooth
        #else
        ray.normal = normalize(cross(RTContext.e1[t], RTContext.e2[t]))
        #endif
        hit = true
    }

    return hit
}

@MainActor @inline(__always)
func intersectSpheres(ray: inout Ray) -> Bool {
    var hit = false
    let eps: Scalar = RTContext.intersectionTestEpsilon

    let n = RTContext.sphCount
    let C = RTContext.sphCenter!
    let R = RTContext.sphRadius!
    let O = RTContext.sphObj!
    let M = RTContext.sphMat!

    for i in 0..<n {
//        let b = dot(ray.dir, ray.origin - C[i])
//        let c = dot(ray.origin - C[i], ray.origin - C[i]) - R[i]*R[i]
//        let disc = b*b - c
//
//        if disc < RTContext.intersectionTestEpsilon { continue }
//        let t1 = -b - sqrt(disc)
//        let t2 = -b + sqrt(disc)
//        if t1 < RTContext.intersectionTestEpsilon && t2 < RTContext.intersectionTestEpsilon { continue }
//
//        let tHit = min(t1, t2)
//        if tHit < ray.tMax {
//            ray.tMax = tHit
//            ray.kind = .sphere
//            ray.prim = Int32(i)
//            ray.obj  = O[i]
//            ray.mat  = M[i]
//            ray.normal = (ray.origin + ray.dir * tHit - C[i]) / R[i]
//            hit = true
//        }
//        let oc = ray.origin - C[i]
//        let a = length_squared(ray.dir)
//        let h = dot(ray.dir, oc)
//        let c = length_squared(oc) - R[i]*R[i]
//        let disc = h*h - a*c
//        let b  = 2.0 * dot(oc, ray.dir)
//        if disc < 0 { continue }
//        let s = sqrt(disc)
//        let tHit = (h-s) / a

        let A = length_squared(ray.dir)
        let oc = ray.origin - C[i]
        let B = dot(ray.dir, oc)
        let c = length_squared(oc) - R[i] * R[i]

        let disc = B * B - A * c
        if disc < 0 { continue }
        let tHit = (-B - sqrt(disc)) / (A)

        if tHit > eps && tHit < ray.tMax {
            ray.tMax = tHit
            ray.kind = .sphere
            ray.prim = Int32(i)
            ray.obj  = O[i]
            ray.mat  = M[i]
            ray.normal = (ray.origin + ray.dir * tHit - C[i]) / R[i]
            hit = true
        }
    }
    return hit
}

@MainActor @inline(__always)
public func occluded(shadowRay rIn: Ray,
                     excludeObj: Int32 = -1,
                     backfaceCulling: Bool = false) -> Bool
{
    var r = rIn
    return trianglesShadow(&r, excludeObj: excludeObj, backfaceCulling: backfaceCulling) || spheresShadow(&r, excludeObj: excludeObj) || planeShadow(ray: &r, excludeObj: excludeObj)
}

@MainActor @inline(__always)
private func trianglesShadow(_ ray: inout Ray,
                             excludeObj: Int32,
                             backfaceCulling: Bool) -> Bool
{
    let eps = RTContext.intersectionTestEpsilon
    let nTris = RTContext.triCount
    let v0 = RTContext.v0!, e1 = RTContext.e1!, e2 = RTContext.e2!
    let triObj = RTContext.triObj!

    for t in 0..<nTris {
        // Möller–Trumbore (same as main, but early-out at first valid t)
        let p  = cross(ray.dir, e2[t])
        let det = dot(e1[t], p)

        if backfaceCulling {
            if det <= eps { continue }
        } else if det > -eps && det < eps { continue }

        let invDet = 1 / det
        let tVec = ray.origin - v0[t]
        let u = dot(tVec, p) * invDet
        if u < 0 || u > 1 { continue }

        let q = cross(tVec, e1[t])
        let v = dot(ray.dir, q) * invDet
        if v < 0 || u + v > 1 { continue }

        let tHit = dot(e2[t], q) * invDet
        if tHit > eps && tHit < ray.tMax { return true } // blocked
    }
    return false
}

@MainActor @inline(__always)
private func spheresShadow(_ ray: inout Ray,
                           excludeObj: Int32) -> Bool
{
    let eps = RTContext.intersectionTestEpsilon
    let n = RTContext.sphCount
    let C = RTContext.sphCenter!
    let R = RTContext.sphRadius!
    let O = RTContext.sphObj!

    for i in 0..<n {
        let oc = C[i] - ray.origin
        let a = length_squared(ray.dir)
        let h = dot(ray.dir, oc)
        let c = length_squared(oc) - R[i]*R[i]
        let disc = h*h - a*c
        let b  = -2.0 * dot(oc, ray.dir)
        if disc < 0 { continue }
        let s = sqrt(disc)
        let tHit = (h-s) / a

        if tHit > eps && tHit < ray.tMax {
            return true
        }
    }
    return false
}

@MainActor @inline(__always)
func planeShadow(ray: inout Ray, excludeObj: Int32) -> Bool {
    let eps = RTContext.intersectionTestEpsilon

    for p in 0..<RTContext.planeCount {
        let oid = RTContext.planeObj[p]

        let center = RTContext.planeCenter[p]
        let normal = RTContext.planeNormal[p]

        let denom = dot(ray.dir, normal)

        // Backface culling for shadows (optional optimization)
        if denom > 0 { continue }

        // Parallel check
        if abs(denom) < eps { continue }

        let t = dot(center - ray.origin, normal) / denom

        // Check if intersection is within shadow ray range
        if t >= eps && t < ray.tMax {
            return true  // Blocked!
        }
    }

    return false  // Clear path
}
