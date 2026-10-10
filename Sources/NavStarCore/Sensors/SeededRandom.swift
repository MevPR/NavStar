import Foundation

/// Deterministic SplitMix64 generator so simulations are reproducible.
public struct SeededRandom: RandomNumberGenerator {
    private var state: UInt64

    public init(seed: UInt64) { state = seed }

    public mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// Standard normal deviate (Box–Muller).
    public mutating func gaussian() -> Double {
        var u1 = Double(next() >> 11) / Double(1 << 53)
        let u2 = Double(next() >> 11) / Double(1 << 53)
        if u1 < 1e-300 { u1 = 1e-300 }
        return (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
    }

    public mutating func gaussian(sigma: Double) -> Double { sigma * gaussian() }

    public mutating func gaussianVector(sigma: Double) -> Vector3 {
        Vector3(gaussian(sigma: sigma), gaussian(sigma: sigma), gaussian(sigma: sigma))
    }
}
