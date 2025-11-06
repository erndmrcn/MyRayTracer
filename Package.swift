// swift-tools-version: 6.2
import PackageDescription

let package = Package(
    name: "RayTracer",
    platforms: [
        .iOS(.v14),
        .macOS(.v12),
        .tvOS(.v14)
    ],
    products: [
        .library(name: "CPly", targets: ["CPly"]),
        .library(name: "RayTracer", targets: ["RayTracer"])
    ],
    dependencies: [
//        .package(path: "/Users/erendemircan/ParsingKit")
        .package(url: "https://github.com/erndmrcn/ParsingKit", branch: "main")
    ],
    targets: [
        // MARK: - CPly (C++ target)
        .target(
            name: "CPly",
            path: "Sources/CPly",
            sources: ["wrapper.cpp", "miniply.cpp"],
            publicHeadersPath: "include",
            cxxSettings: [
                .headerSearchPath("include"),
                .headerSearchPath("include/CPly"),
                .headerSearchPath("include/miniply"),
                .headerSearchPath("include/module"),
                .unsafeFlags(["-stdlib=libc++"])
            ],
            linkerSettings: [
                            .linkedLibrary("c++")
                        ]
        ),
        .target(
            name: "RayTracer",
            dependencies: [
                "CPly",
                .product(name: "ParsingKit", package: "ParsingKit")
            ],
            path: "Sources/RayTracer",
            swiftSettings: [.interoperabilityMode(.Cxx)]
        )
    ],
    cxxLanguageStandard: .cxx17
)
