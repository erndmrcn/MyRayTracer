// swift-tools-version: 6.2
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "RayTracer",
    platforms: [ .iOS(.v14), .macOS(.v12), .tvOS(.v14) ],
    products: [
        .library(
            name: "RayTracer",
            targets: ["RayTracer"]
        ),
    ],
    dependencies: [
        .package(url: "https://github.com/erndmrcn/ParsingKit", branch: "main")
    ],
    targets: [
        .target(
            name: "RayTracer",
            dependencies: [
                .product(name: "ParsingKit", package: "ParsingKit")
            ],
            path: "Sources/RayTracer"
        ),

    ]
)
