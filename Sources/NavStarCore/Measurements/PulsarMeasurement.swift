import Foundation

/// Millisecond pulsar with a stored timing model (direction in ICRF and pulse
/// period). The onboard catalogue plays the role of the GNSS almanac.
public struct Pulsar: Hashable {
    public let name: String
    /// Unit vector towards the pulsar, EME2000/ICRF.
    public let direction: Vector3
    /// Pulse period in seconds.
    public let period: Double

    public init(name: String, rightAscensionDegrees ra: Double, declinationDegrees dec: Double, period: Double) {
        self.name = name
        let a = ra * Constants.degreesToRadians
        let d = dec * Constants.degreesToRadians
        self.direction = Vector3(cos(d) * cos(a), cos(d) * sin(a), sin(d))
        self.period = period
    }

    /// Bright, stable millisecond pulsars commonly proposed for XNAV.
    public static let catalog: [Pulsar] = [
        Pulsar(name: "PSR B1937+21", rightAscensionDegrees: 294.9107, declinationDegrees: 21.5831, period: 1.5578065e-3),
        Pulsar(name: "PSR B1821-24", rightAscensionDegrees: 276.1333, declinationDegrees: -24.8697, period: 3.0543e-3),
        Pulsar(name: "PSR J0437-4715", rightAscensionDegrees: 69.3158, declinationDegrees: -47.2525, period: 5.7574e-3),
        Pulsar(name: "PSR B1957+20", rightAscensionDegrees: 299.9031, declinationDegrees: 20.8042, period: 1.6074e-3),
    ]
}

/// X-ray pulsar navigation (XNAV) observation: the measured pulse arrival
/// phase, converted to seconds, relative to the arrival predicted at the
/// solar-system barycentre by the stored timing model.
///
/// Geometrically this is the projection of the barycentric position onto the
/// pulsar direction, divided by c, known only modulo the pulse period. The
/// cycle ambiguity is resolved with the filter's own prior and the
/// measurement is refused when the prior is not tight enough to do so.
public struct PulsarMeasurement: NavigationMeasurement {
    public let epoch: Epoch
    public let pulsar: Pulsar
    /// Observed phase delay in seconds, in [0, period).
    public let observedDelay: Double
    /// Timing 1σ in seconds.
    public let sigma: Double

    public init(epoch: Epoch, pulsar: Pulsar, observedDelay: Double, sigma: Double) {
        self.epoch = epoch
        self.pulsar = pulsar
        self.observedDelay = observedDelay
        self.sigma = sigma
    }

    public var label: String { "Pulsar(\(pulsar.name))" }

    /// Fraction of the ambiguity interval the 1σ prior must stay below.
    public static let ambiguityFraction = 1.0 / 6.0

    public func evaluate(state: NavigationState, context: NavigationContext) throws -> MeasurementEvaluation {
        let c = Constants.speedOfLight
        let n = pulsar.direction
        let barycentric = state.position - context.ephemeris.solarSystemBarycenterOffset(at: epoch)
        let predicted = n.dot(barycentric) / c

        let prr = state.covariance.block3(row: 0, col: 0)
        let sigmaAlong = n.dot(prr * n).squareRoot()
        let ambiguityRange = c * pulsar.period
        guard sigmaAlong < ambiguityRange * PulsarMeasurement.ambiguityFraction else {
            throw MeasurementError.unusable(
                String(format: "cycle ambiguity unresolved (prior %.0f km > %.0f km along %@)",
                       sigmaAlong, ambiguityRange * PulsarMeasurement.ambiguityFraction, pulsar.name))
        }

        let residual = wrapped(observedDelay - predicted, period: pulsar.period)
        var h = Matrix(rows: 1, cols: 6)
        for k in 0..<3 { h[0, k] = n[k] / c }
        return MeasurementEvaluation(residual: [residual], jacobian: h, noise: Matrix.diagonal([sigma * sigma]))
    }
}
