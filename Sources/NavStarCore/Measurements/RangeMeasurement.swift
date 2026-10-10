import Foundation

/// Distance to the surface of a body from a radar or laser altimeter, usable
/// during approach and orbit phases.
public struct RangeMeasurement: NavigationMeasurement {
    public let epoch: Epoch
    public let body: CelestialBody
    /// Measured distance to the body's mean surface, km.
    public let range: Double
    /// 1σ in km.
    public let sigma: Double

    public init(epoch: Epoch, body: CelestialBody, range: Double, sigma: Double) {
        self.epoch = epoch
        self.body = body
        self.range = range
        self.sigma = sigma
    }

    public var label: String { "Range(\(body.displayName))" }

    public func evaluate(state: NavigationState, context: NavigationContext) throws -> MeasurementEvaluation {
        let p = BearingMeasurement.retardedPosition(of: body, at: epoch, seenFrom: state.position, ephemeris: context.ephemeris)
        let d = p - state.position
        let dist = d.norm
        let u = d / dist
        let predicted = dist - body.meanRadius
        var h = Matrix(rows: 1, cols: 6)
        for k in 0..<3 { h[0, k] = -u[k] }
        return MeasurementEvaluation(residual: [range - predicted], jacobian: h,
                                     noise: Matrix.diagonal([sigma * sigma]))
    }
}
