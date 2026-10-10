// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "NavStar",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "NavStarCore", targets: ["NavStarCore"]),
        .executable(name: "navstar-sim", targets: ["NavStarSim"]),
    ],
    targets: [
        .target(name: "NavStarCore"),
        .executableTarget(name: "NavStarSim", dependencies: ["NavStarCore"]),
        .testTarget(name: "NavStarCoreTests", dependencies: ["NavStarCore"]),
    ]
)
