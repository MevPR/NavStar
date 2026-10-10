import Foundation

public enum UpdateResult: CustomStringConvertible {
    case accepted(mahalanobis: Double, residual: [Double])
    case rejected(reason: String)

    public var isAccepted: Bool {
        if case .accepted = self { return true }
        return false
    }

    public var description: String {
        switch self {
        case let .accepted(d, _): return String(format: "accepted (%.2fσ)", d)
        case let .rejected(reason): return "rejected: \(reason)"
        }
    }
}

/// Continuous-discrete extended Kalman filter for the translational state
/// plus an acceleration-bias state.
///
/// Propagation integrates the full nonlinear dynamics with RK4 and the
/// covariance with the second-order approximation of the state transition
/// matrix. Updates use the Joseph form to keep the covariance positive
/// definite, and an innovation (Mahalanobis) gate rejects outliers.
public final class ExtendedKalmanFilter {
    public private(set) var state: NavigationState
    public let dynamics: DynamicsModel
    /// 1σ of white unmodelled acceleration (km/s²) — integration and
    /// short-term force-model noise.
    public var processNoiseAcceleration: Double
    /// Steady-state 1σ of the Gauss–Markov acceleration bias (km/s²).
    public var biasSigma: Double
    /// Correlation time of the acceleration bias (s).
    public var biasCorrelationTime: Double
    /// Updates whose normalised innovation exceeds this many σ are rejected.
    public var gateSigma: Double?
    /// Maximum integration step in seconds.
    public var maxStep: Double = 120

    private let n = NavigationState.dimension

    public init(initialState: NavigationState, dynamics: DynamicsModel,
                processNoiseAcceleration: Double = 1e-11,
                biasSigma: Double = 1e-10, biasCorrelationTime: Double = 10 * 86_400,
                gateSigma: Double? = 5) {
        self.state = initialState
        self.dynamics = dynamics
        self.processNoiseAcceleration = processNoiseAcceleration
        self.biasSigma = biasSigma
        self.biasCorrelationTime = biasCorrelationTime
        self.gateSigma = gateSigma
    }

    /// Replaces the current estimate (used for cold-start initialisation).
    public func reset(to newState: NavigationState) { state = newState }

    /// Propagates the estimate and covariance forward to `target`, applying a
    /// known inertial-frame non-gravitational acceleration (km/s²) throughout.
    public func propagate(to target: Epoch, knownAcceleration: Vector3 = .zero) {
        while target - state.epoch > 1e-9 {
            let dt = min(maxStep, target - state.epoch)
            step(dt: dt, knownAcceleration: knownAcceleration)
        }
    }

    private func step(dt: Double, knownAcceleration: Vector3) {
        let r0 = state.position
        let t0 = state.epoch
        let bias = state.accelerationBias
        let tau = biasCorrelationTime

        // Linearised dynamics  ẋ = F x:  ṙ = v,  v̇ = G r + a_bias,  ȧ_bias = −a_bias/τ
        let g = dynamics.gravityGradient(position: r0, epoch: t0)
        var f = Matrix(rows: n, cols: n)
        f.setBlock(row: 0, col: 3, Matrix3.identity)
        f.setBlock(row: 3, col: 0, g)
        f.setBlock(row: 3, col: 6, Matrix3.identity)
        f.setBlock(row: 6, col: 6, (-1 / tau) * Matrix3.identity)
        let fdt = dt * f
        let phi = Matrix.identity(n) + fdt + 0.5 * (fdt * fdt)

        // Process noise: white acceleration on (r, v), driving noise on the bias.
        let q = processNoiseAcceleration * processNoiseAcceleration
        let qBias = 2 * biasSigma * biasSigma / tau
        var qMatrix = Matrix(rows: n, cols: n)
        qMatrix.setBlock(row: 0, col: 0, (q * dt * dt * dt * dt / 4) * Matrix3.identity)
        qMatrix.setBlock(row: 0, col: 3, (q * dt * dt * dt / 2) * Matrix3.identity)
        qMatrix.setBlock(row: 3, col: 0, (q * dt * dt * dt / 2) * Matrix3.identity)
        qMatrix.setBlock(row: 3, col: 3, (q * dt * dt) * Matrix3.identity)
        qMatrix.setBlock(row: 6, col: 6, (qBias * dt) * Matrix3.identity)

        let dyn = dynamics
        let extra = knownAcceleration + bias
        let (r1, v1) = Integrator.rk4Step(position: r0, velocity: state.velocity, epoch: t0, dt: dt) { r, t in
            dyn.acceleration(position: r, epoch: t, additional: extra)
        }
        state.position = r1
        state.velocity = v1
        state.accelerationBias = exp(-dt / tau) * bias
        state.epoch = t0.adding(seconds: dt)
        state.covariance = (phi * state.covariance * phi.transposed + qMatrix).symmetrized
    }

    /// Incorporates one measurement. If the measurement lies in the future the
    /// filter propagates to its epoch first; stale measurements are rejected.
    @discardableResult
    public func update(with measurement: any NavigationMeasurement, context: NavigationContext) -> UpdateResult {
        let lag = state.epoch - measurement.epoch
        if lag > 1.0 {
            return .rejected(reason: String(format: "measurement is %.0f s older than the filter state", lag))
        }
        if lag < 0 { propagate(to: measurement.epoch) }

        let evaluation: MeasurementEvaluation
        do {
            evaluation = try measurement.evaluate(state: state, context: context)
        } catch let error as MeasurementError {
            return .rejected(reason: error.description)
        } catch {
            return .rejected(reason: "\(error)")
        }

        // Measurements depend on position and velocity only; pad the Jacobian
        // with zero columns for the bias states.
        var h = Matrix(rows: evaluation.jacobian.rows, cols: n)
        h.setBlock(row: 0, col: 0, evaluation.jacobian)
        let rNoise = evaluation.noise
        let y = evaluation.residual
        let p = state.covariance
        let s = h * p * h.transposed + rNoise
        guard let sInv = s.inverse() else { return .rejected(reason: "singular innovation covariance") }

        let d2 = dot(y, sInv * y)
        let normalised = (d2 / Double(y.count)).squareRoot()
        if let gate = gateSigma, normalised > gate {
            return .rejected(reason: String(format: "innovation %.1fσ exceeds gate", normalised))
        }

        let k = p * h.transposed * sInv
        let dx = k * y
        state.position += Vector3(dx[0], dx[1], dx[2])
        state.velocity += Vector3(dx[3], dx[4], dx[5])
        state.accelerationBias += Vector3(dx[6], dx[7], dx[8])

        let ikh = Matrix.identity(n) - k * h
        state.covariance = (ikh * p * ikh.transposed + k * rNoise * k.transposed).symmetrized
        return .accepted(mahalanobis: normalised, residual: y)
    }
}
