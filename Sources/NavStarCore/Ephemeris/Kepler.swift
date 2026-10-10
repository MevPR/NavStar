import Foundation

public enum Kepler {
    /// Solves Kepler's equation M = E − e·sin(E) for the eccentric anomaly E
    /// (all angles in radians). Newton iteration with the iterate clamped to
    /// the admissible band |E − M| ≤ e, which keeps it stable for high
    /// eccentricities.
    public static func eccentricAnomaly(meanAnomaly m: Double, eccentricity e: Double, tolerance: Double = 1e-13) -> Double {
        var mean = m.truncatingRemainder(dividingBy: 2 * .pi)
        if mean < -.pi { mean += 2 * .pi }
        if mean > .pi { mean -= 2 * .pi }
        var eAnom = mean + e * sin(mean)
        for _ in 0..<60 {
            let delta = (eAnom - e * sin(eAnom) - mean) / (1 - e * cos(eAnom))
            eAnom = min(max(eAnom - delta, mean - e), mean + e)
            if abs(delta) < tolerance { break }
        }
        return eAnom
    }
}
