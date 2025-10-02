// swift-tools-version: 6.2
import PackageDescription

let package = Package(
    name: "RayTracer",
    platforms: [
        .iOS(.v15), .macOS(.v12)
    ],
    products: [
        .library(name: "RayTracer", targets: ["RayTracer"])
    ],
    dependencies: [
        .package(url: "https://github.com/erndmrcn/ParsingKit", branch: "main")
    ],
    targets: [
        .target(
            name: "RayTracer",
            dependencies: [
                .product(name: "ParsingKit", package: "ParsingKit")
            ]
        )
    ]
)
