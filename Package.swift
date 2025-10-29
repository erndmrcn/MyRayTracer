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
        // ✅ Expose both modules to clients
        .library(name: "CPly", targets: ["CPly"]),
        .library(name: "RayTracer", targets: ["RayTracer"])
    ],
    dependencies: [
        .package(path: "/Users/erendemircan/ParsingKit")
    ],
    targets: [
        // MARK: - CPly (C++ target)
        .target(
            name: "CPly",
            path: "Sources/CPly",
            sources: ["wrapper.cpp", "miniply.cpp"],  // Specify the .cpp file
            publicHeadersPath: "include",
            cxxSettings: [
                // normal include paths
                .headerSearchPath("include"),
                .headerSearchPath("include/CPly"),
                .headerSearchPath("include/miniply"),
                .headerSearchPath("include/module"),
//                .languageStandard(.cxx17), // Or .cxx14 / .cxx11
                .unsafeFlags(["-stdlib=libc++"]) // Tell Clang to use the right s
            ],
            linkerSettings: [
                            .linkedLibrary("c++")
                        ]
        ),
        // MARK: - RayTracer (Swift target)
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
