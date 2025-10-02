//
//  Image.swift
//  ray-tracer
//
//  Created by Eren Demircan on 1.10.2025.
//

import Foundation
import CoreGraphics
import ImageIO
import UniformTypeIdentifiers

public struct ImageHelper {
    public static func savePNG(_ image: CGImage, to path: String) throws {
        let url = URL(fileURLWithPath: path)
        guard let dest = CGImageDestinationCreateWithURL(url as CFURL,
                                                         UTType.png.identifier as CFString,
                                                         1, nil) else {
            throw NSError(domain: "PNG", code: 1, userInfo: [NSLocalizedDescriptionKey: "Create destination failed"])
        }
        CGImageDestinationAddImage(dest, image, nil)
        guard CGImageDestinationFinalize(dest) else {
            throw NSError(domain: "PNG", code: 2, userInfo: [NSLocalizedDescriptionKey: "Finalize failed"])
        }
    }

    public static func makeCGImage(width: Int, height: Int, rgba: [UInt8]) -> CGImage? {
        let bytesPerPixel = 4
        let bitsPerComponent = 8
        let bytesPerRow = width * bytesPerPixel
        let cs = CGColorSpaceCreateDeviceRGB()
        return rgba.withUnsafeBytes { ptr in
            guard let provider = CGDataProvider(data: Data(bytes: ptr.baseAddress!, count: rgba.count) as CFData)
            else { return nil }
            return CGImage(width: width, height: height,
                           bitsPerComponent: bitsPerComponent, bitsPerPixel: 32,
                           bytesPerRow: bytesPerRow, space: cs,
                           bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.noneSkipLast.rawValue),
                           provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
        }
    }
}
