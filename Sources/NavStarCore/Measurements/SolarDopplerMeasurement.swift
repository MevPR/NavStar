import Foundation

/// Radial velocity relative to the Sun obtained from the Doppler shift of
/// solar absorption lines measured by an onboard spectrometer.
public struct SolarDopplerMeasurement: NavigationMeasurement {
    public let epoch: Epoch
    /// Measured radial velocity (positive = receding from the Sun), km/s.
    public let radialVelocity: Double
    /// 1σ in km/s.
    public let sigma: Double

    public init(epoch: Epoch, radialVelocity: Double, sigma: Double) {
        self.epoch = epoch
        self.radialVelocity = radialVelocity
        self.sigma = sigma
    }

    public var label: String { "SolarDoppler" }

    public func evaluate(state: NavigationState, context: NavigationContext) throws -> MeasurementEvaluation {
        let r = state.position
        let v = state.velocity
        let rn = r.norm
        let rHat = r / rn
        let predicted = v.dot(rHat)

        // ∂(v·r̂)/∂r = vᵀ(I − r̂ r̂ᵀ)/|r|,   ∂(v·r̂)/∂v = r̂ᵀ
        let dr = ((Matrix3.identity - Matrix3.outer(rHat, rHat)) * v) / rn
        var h = Matrix(rows: 1, cols: 6)
        for k in 0..<3 {
            h[0, k] = dr[k]
            h[0, 3 + k] = rHat[k]
        }
        return MeasurementEvaluation(residual: [radialVelocity - predicted], jacobian: h,
                                     noise: Matrix.diagonal([sigma * sigma]))
    }
}
