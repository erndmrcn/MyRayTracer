// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "RayCore",
    platforms: [
        .iOS(.v15), .macOS(.v12)
    ],
    products: [
        .library(
            name: "RayCore",
            targets: ["RayCore"]
        ),
    ],
    dependencies: [
        .package(url: "https://github.com/erndmrcn/ParsingKit", branch: "main")
    ],
    targets: [
        .target(
            name: "RayCore",
            dependencies: [
                .product(name: "ParsingKit", package: "ParsingKit")
            ],
            path: "RayCore/Source/RayCore"
        )
    ]
)
