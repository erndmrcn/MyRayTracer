import Foundation
import AVFoundation

public enum RenderBatchManager {
    static public func renderFolder(
        at folderURL: URL,
        outputDir: URL,
        makeVideo: Bool = true,
        fps: Int32 = 30
    ) async {
        let jsonFiles = (try? FileManager.default.contentsOfDirectory(at: folderURL,
                                                                     includingPropertiesForKeys: nil))?
            .filter { $0.pathExtension == "json" }
            .sorted {
                $0.lastPathComponent.localizedStandardCompare($1.lastPathComponent) == .orderedAscending
            } ?? []

        guard !jsonFiles.isEmpty else {
            print("❌ No JSON files found in folder: \(folderURL.path)")
            return
        }

        var renderedFrames: [(index: Int, image: CGImage)] = []
        renderedFrames.reserveCapacity(jsonFiles.count)

        var start = DispatchTime.now()
        for (i, file) in jsonFiles.enumerated() {
            print("▶️ Rendering frame \(i + 1)/\(jsonFiles.count): \(file.lastPathComponent)")
            let engine = RayTracerEngine(from: file)

            do {
                let cgImages = try await engine.renderAll { progress in
                    let pct = Int(progress.fraction * 100)
                    return progress.fraction == 1
                }

                if let img = cgImages.first?.image {
                    print("\n✅ Frame \(i) done (\(cgImages.first!.stats.milliseconds) ms)")
                    renderedFrames.append((i, img))
                }
            } catch {
                print("❌ Error rendering \(file.lastPathComponent): \(error)")
            }
        }
        var elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - start.uptimeNanoseconds) / 1_000_000.0)
        print("Rendered \(renderedFrames.count) frames in \(elapsedMs) ms")
        renderedFrames.sort { $0.index < $1.index }

        start = DispatchTime.now()
        if makeVideo, let firstImage = renderedFrames.first?.image {
            await createMP4(from: renderedFrames.map(\.image),
                            reference: firstImage,
                            fileName: String(folderURL.lastPathComponent),
                            outputDir: outputDir,
                            fps: fps)
        }
        elapsedMs = Int(Double(DispatchTime.now().uptimeNanoseconds - start.uptimeNanoseconds) / 1_000_000.0)
        print("Make video \(String(folderURL.lastPathComponent)) in \(elapsedMs) ms")
    }

    // MARK: - MP4 Encoding
    private static func createMP4(
        from images: [CGImage],
        reference first: CGImage,
        fileName: String,
        outputDir: URL,
        fps: Int32
    ) async {
        let width = first.width
        let height = first.height
        let outputURL = outputDir.appendingPathComponent("\(fileName).mp4")

        do {
            let writer = try AVAssetWriter(outputURL: outputURL, fileType: .mp4)
            let settings: [String: Any] = [
                AVVideoCodecKey: AVVideoCodecType.h264,
                AVVideoWidthKey: width,
                AVVideoHeightKey: height
            ]
            let input = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
            let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input,
                                                               sourcePixelBufferAttributes: nil)
            writer.add(input)
            writer.startWriting()
            writer.startSession(atSourceTime: .zero)

            let frameDuration = CMTime(value: 1, timescale: fps)
            var frameCount: Int64 = 0

            for img in images {
                autoreleasepool {
                    guard let pb = makePixelBuffer(from: img, width: width, height: height) else { return }
                    while !input.isReadyForMoreMediaData {
                        Thread.sleep(forTimeInterval: 0.01)
                    }
                    let time = CMTimeMultiply(frameDuration, multiplier: Int32(frameCount))
                    adaptor.append(pb, withPresentationTime: time)
                    frameCount += 1
                }
            }

            input.markAsFinished()
            await writer.finishWriting()
            print("🎥 Created animation: \(outputURL.path)")
        } catch {
            print("⚠️ Failed to create MP4: \(error)")
        }
    }

    private static func makePixelBuffer(from image: CGImage, width: Int, height: Int) -> CVPixelBuffer? {
        var pb: CVPixelBuffer?
        let attrs = [
            kCVPixelBufferCGImageCompatibilityKey: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey: true
        ] as CFDictionary

        guard CVPixelBufferCreate(kCFAllocatorDefault, width, height,
                                  kCVPixelFormatType_32ARGB, attrs, &pb) == kCVReturnSuccess,
              let px = pb else { return nil }

        CVPixelBufferLockBaseAddress(px, [])
        if let ctx = CGContext(data: CVPixelBufferGetBaseAddress(px),
                               width: width,
                               height: height,
                               bitsPerComponent: 8,
                               bytesPerRow: CVPixelBufferGetBytesPerRow(px),
                               space: CGColorSpaceCreateDeviceRGB(),
                               bitmapInfo: CGImageAlphaInfo.noneSkipFirst.rawValue) {
            ctx.draw(image, in: CGRect(x: 0, y: 0, width: width, height: height))
        }
        CVPixelBufferUnlockBaseAddress(px, [])
        return px
    }
}
