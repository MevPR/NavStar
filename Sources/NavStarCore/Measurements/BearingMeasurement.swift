import Foundation

/// Line-of-sight direction to a solar-system body observed by the optical
/// navigation camera (or the Sun sensor), already rotated into the inertial
/// frame with the star-tracker attitude.
///
/// The model includes the light-time delay and first-order stellar aberration.
/// The 3-component unit vector carries two degrees of freedom, so the residual
/// is expressed as two small angles in the tangent plane of the predicted
/// direction.
public struct BearingMeasurement: NavigationMeasurement {
    public let epoch: Epoch
    public let body: CelestialBody
    /// Observed apparent direction, unit vector in EME2000.
    public let observedDirection: Vector3
    /// Per-axis angular 1σ noise in radians.
    public let sigma: Double

    public init(epoch: Epoch, body: CelestialBody, observedDirection: Vector3, sigma: Double) {
        self.epoch = epoch
        self.body = body
        self.observedDirection = observedDirection.normalized
        self.sigma = sigma
    }

    public var label: String { "Bearing(\(body.displayName))" }

    public func evaluate(state: NavigationState, context: NavigationContext) throws -> MeasurementEvaluation {
        let (predicted, range) = BearingMeasurement.apparentDirection(
            to: body, from: state.position, velocity: state.velocity, epoch: epoch, ephemeris: context.ephemeris)
        guard range > body.meanRadius else {
            throw MeasurementError.unusable("estimated position lies inside \(body.displayName)")
        }
        let (e1, e2) = predicted.tangentBasis()
        let residual = [e1.dot(observedDirection), e2.dot(observedDirection)]

        // ∂u/∂r = −(I − u uᵀ)/ρ  for u = (p − r)/ρ
        let dudr = (-1.0 / range) * (Matrix3.identity - Matrix3.outer(predicted, predicted))
        let row1 = dudr.transposed * e1
        let row2 = dudr.transposed * e2
        var h = Matrix(rows: 2, cols: 6)
        for k in 0..<3 {
            h[0, k] = row1[k]
            h[1, k] = row2[k]
        }
        let noise = Matrix.diagonal([sigma * sigma, sigma * sigma])
        return MeasurementEvaluation(residual: residual, jacobian: h, noise: noise)
    }

    /// Apparent direction and geometric range from `r` to `body`, including
    /// light-time iteration and first-order aberration for observer velocity `v`.
    public static func apparentDirection(to body: CelestialBody, from r: Vector3, velocity v: Vector3,
                                         epoch: Epoch, ephemeris: any Ephemeris) -> (direction: Vector3, range: Double) {
        let p = retardedPosition(of: body, at: epoch, seenFrom: r, ephemeris: ephemeris)
        let d = p - r
        let apparent = (d.normalized + v / Constants.speedOfLight).normalized
        return (apparent, d.norm)
    }

    /// Body position at the emission time of the light received at `r` at `epoch`
    /// (fixed-point iteration of the light-time equation).
    public static func retardedPosition(of body: CelestialBody, at epoch: Epoch, seenFrom r: Vector3,
                                        ephemeris: any Ephemeris) -> Vector3 {
        var p = ephemeris.position(of: body, at: epoch)
        for _ in 0..<3 {
            let tau = (p - r).norm / Constants.speedOfLight
            p = ephemeris.position(of: body, at: epoch.adding(seconds: -tau))
        }
        return p
    }
}
