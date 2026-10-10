import Foundation

/// Top-level onboard navigator: owns the attitude knowledge, the translational
/// Kalman filter and the ephemeris, and converts raw sensor readings into
/// filter measurements.
///
/// Typical flight loop:
/// 1. `updateAttitude(starTracker:)` or `propagateAttitude(bodyRate:dt:)`
/// 2. `advance(to:bodyAcceleration:)` with the accelerometer reading
/// 3. `ingest(_:)` for every optical / pulsar / Doppler / range observation
public final class Navigator {
    public enum Mode: String {
        case uninitialised, cruise
    }

    public let ephemeris: any Ephemeris
    public let filter: ExtendedKalmanFilter
    public private(set) var attitude: Quaternion
    public private(set) var mode: Mode
    /// Human-readable event log (bounded).
    public private(set) var events: [String] = []
    public var maxEvents = 200

    /// Accelerations below this (km/s²) are treated as sensor noise, not thrust.
    public var thrustThreshold: Double = 1e-7
    /// Per-axis 1σ of the star-tracker attitude (rad); added to every bearing's noise.
    public var attitudeSigma: Double
    /// Steady-state 1σ of the estimated acceleration bias (km/s²).
    public let accelerationBiasSigma: Double

    private var pendingColdStart: [BearingMeasurement] = []

    public init(initialState: NavigationState, ephemeris: any Ephemeris, attitude: Quaternion = .identity,
                attitudeSigma: Double = 3 * Constants.arcsecondsToRadians,
                processNoiseAcceleration: Double = 1e-11,
                accelerationBiasSigma: Double = 1e-10, accelerationBiasCorrelationTime: Double = 10 * 86_400,
                gateSigma: Double? = 5) {
        self.ephemeris = ephemeris
        self.filter = ExtendedKalmanFilter(initialState: initialState,
                                           dynamics: DynamicsModel(ephemeris: ephemeris),
                                           processNoiseAcceleration: processNoiseAcceleration,
                                           biasSigma: accelerationBiasSigma,
                                           biasCorrelationTime: accelerationBiasCorrelationTime,
                                           gateSigma: gateSigma)
        self.attitude = attitude
        self.attitudeSigma = attitudeSigma
        self.accelerationBiasSigma = accelerationBiasSigma
        self.mode = .cruise
    }

    public var state: NavigationState { filter.state }
    public var context: NavigationContext { NavigationContext(ephemeris: ephemeris) }

    // MARK: Attitude

    public func updateAttitude(starTracker q: Quaternion) { attitude = q.normalized }

    public func propagateAttitude(bodyRate: Vector3, dt: Double) {
        attitude = attitude.integrated(bodyRate: bodyRate, dt: dt)
    }

    // MARK: Translational state

    /// Propagates to `epoch`. `bodyAcceleration` is the accelerometer reading
    /// (km/s², body frame); it is rotated with the current attitude and fed to
    /// the dynamics when above the thrust threshold.
    public func advance(to epoch: Epoch, bodyAcceleration: Vector3 = .zero) {
        let inertial = bodyAcceleration.norm > thrustThreshold ? attitude.rotate(bodyAcceleration) : .zero
        filter.propagate(to: epoch, knownAcceleration: inertial)
    }

    /// Converts a camera/Sun-sensor body-frame direction into a bearing
    /// measurement; the attitude uncertainty is folded into the noise.
    public func makeBearing(to body: CelestialBody, bodyFrameDirection: Vector3, sigma: Double, epoch: Epoch) -> BearingMeasurement {
        let total = (sigma * sigma + attitudeSigma * attitudeSigma).squareRoot()
        return BearingMeasurement(epoch: epoch, body: body, observedDirection: attitude.rotate(bodyFrameDirection), sigma: total)
    }

    @discardableResult
    public func ingest(_ measurement: any NavigationMeasurement) -> UpdateResult {
        let result = filter.update(with: measurement, context: context)
        if case let .rejected(reason) = result {
            log("\(measurement.epoch) \(measurement.label) rejected: \(reason)")
        }
        return result
    }

    /// Re-initialises the filter from two sets of bearings (cold start).
    @discardableResult
    public func coldStart(first: [BearingMeasurement], second: [BearingMeasurement]) -> Bool {
        guard let state = InitialFix.coldStart(first: first, second: second, ephemeris: ephemeris,
                                               accelerationSigma: accelerationBiasSigma) else {
            log("cold start failed: insufficient geometry")
            return false
        }
        filter.reset(to: state)
        mode = .cruise
        log(String(format: "%@ cold start: position σ %.0f km, velocity σ %.1f m/s",
                   state.epoch.description, state.positionSigmaRSS, state.velocitySigmaRSS * 1000))
        return true
    }

    public func log(_ message: String) {
        events.append(message)
        if events.count > maxEvents { events.removeFirst(events.count - maxEvents) }
    }
}
