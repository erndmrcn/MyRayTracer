//
//  main.swift
//  ray-tracer
//
//  Created by Eren Demircan on 30.09.2025.
//

import Foundation
import ParsingKit
import simd

let fileName: String = "/Users/erendemircan/Desktop/RayTracer/ray-tracer/Sources/testScene.json"
let url = URL(fileURLWithPath: fileName)
let data = try Data(contentsOf: url)
let scene: Scene? = try? ParsingKit.decode(Scene.self, from: data, rootKey: "Scene")

guard let scene else { fatalError("Parser is nil") }

for camera in scene.cameras.cameras {
    // MARK: - Calculations
    let w: Vec3 = normalize(-camera.gaze)
    let v: Vec3 = normalize(cross(camera.up, w))
    let u: Vec3 = normalize(cross(v, w))

    print("u = \(u), v = \(v), w = \(w)")

    let width = scene.cameras.cameras[0].imageResolution[0]
    let height = scene.cameras.cameras[0].imageResolution[1]
    let e = scene.cameras.cameras[0].position
    let nearDistance = scene.cameras.cameras[0].nearDistance
    let nearPlane = scene.cameras.cameras[0].nearPlane
    let l = Double(nearPlane[0])
    let r = Double(nearPlane[1])
    let b = Double(nearPlane[2])
    let t = Double(nearPlane[3])

    var intersected = 0
    var colored = 0
    var shadowed = 0

    var imageBytes: [UInt8] = .init(repeating: 0, count: width*height*4)
    var p = 0
    for i in 0..<width {
        for j in 0..<height {
            let m: Vec3 = e + -w * Double(nearDistance)
            let q: Vec3 = m + u * l  + t * v
            let s_u: Double = (Double(i) + 0.5) * (r - l) / Double(width)
            let s_v: Double = (Double(j) + 0.5) * (t - b) / Double(height)
            let s = q + s_u * u - s_v * v
            var r = Ray(origin: e, direction: s - e, t: 0)

            var tMin = Double.infinity
            var closestObject: Object?

            var pixelColor: Vec3 = .zero
            for object in scene.objects.objects {
                if object.intersect(ray: &r, backfaceCulling: false) {
                    intersected += 1
                    if r.t < tMin {
                        tMin = r.t
                        closestObject = object
                    }
                }
            }

            if let closestObject {
                let intersectionPoint = r.origin + r.direction * tMin
                let material = scene.materials.materials[closestObject.materialIdx]
                var shadowRay: Ray

                // Ambient Light
                pixelColor = scene.lights.ambientLight * material.ambient
                for light in scene.lights.pointLights {
                    if let material = closestObject.material {
                        var w_i = light.position - intersectionPoint
                        let distance = simd_length(w_i)
                        w_i = simd_normalize(w_i)
                        let nGeom = shadedNormalAt(intersectionPoint, for: closestObject)
                        let shadowOrigin = intersectionPoint + nGeom * scene.shadowRayEpsilon
                        shadowRay = Ray(origin: shadowOrigin, direction: w_i, t: 0)
                        var didIntersect = false

                        for object in scene.objects.objects {
                            didIntersect = object.shadowIntersect(ray: &shadowRay, distance: distance, backfaceCulling: false)
                            if didIntersect {
                                shadowed += 1
                                break
                            }
                        }

                        if didIntersect {
                            continue;
                        }

                        colored += 1
                        pixelColor += light.shade(material: material, ray: r, normal: shadedNormalAt(intersectionPoint, for: closestObject), at: intersectionPoint)
                    }
                }
            } else {
                pixelColor = scene.backgroundColor
            }

            pixelColor.clamp(lowerBound: .zero, upperBound: .one)
            let r8 = UInt8(max(0.0, min(1.0, pixelColor.x)) * 255.0)
            let g8 = UInt8(max(0.0, min(1.0, pixelColor.y)) * 255.0)
            let b8 = UInt8(max(0.0, min(1.0, pixelColor.z)) * 255.0)

            imageBytes[p + 0] = r8
            imageBytes[p + 1] = g8
            imageBytes[p + 2] = b8
            imageBytes[p + 3] = 255
            p += 4
        }
    }

    print("Intersections: \(intersected)")
    print("Colored pixels: \(colored)")
    print("Shadowes pixels: \(shadowed)")

    if let cgImage = ImageHelper.makeCGImage(width: width, height: height, rgba: imageBytes) {
        do {
            try ImageHelper.savePNG(cgImage, to: camera.imageName)
        } catch {
            print("Error while writing the result \(error)")
        }
    }
}
func shadedNormalAt(_ point: Vec3, for object: Object) -> Vec3 {
    switch object {
    case let sphere as Sphere:
        return simd_normalize(point - sphere.center)

    case let tri as Triangle:
        return simd_normalize(simd_cross(tri.e1, tri.e2))

    default:
        return Vec3(0,0,0)
    }
}

extension Vec3 {
    mutating func clamp() {
        self.x = simd_clamp(self.x, 0, 1)
        self.y = simd_clamp(self.y, 0, 1)
        self.z = simd_clamp(self.z, 0, 1)
    }
}

