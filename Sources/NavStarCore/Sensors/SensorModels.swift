import Foundation

/// True spacecraft state used by the sensor models to synthesise readings.
/// In flight these structs are replaced by hardware drivers producing the same
/// output types; the filter side never sees the truth.
public struct SpacecraftTruth {
    public var epoch: Epoch
    public var position: Vector3
    public var velocity: Vector3
    /// Body → inertial attitude.
    public var attitude: Quaternion

    public init(epoch: Epoch, position: Vector3, velocity: Vector3, attitude: Quaternion) {
        self.epoch = epoch
        self.position = position
        self.velocity = velocity
        self.attitude = attitude
    }
}

/// Star tracker: delivers the body → inertial attitude by matching the star
/// field against the onboard catalogue.
public struct StarTrackerModel {
    /// Per-axis 1σ attitude noise, radians.
    public var sigma: Double
    public init(sigma: Double = 3 * Constants.arcsecondsToRadians) { self.sigma = sigma }

    public func measure(truth: SpacecraftTruth, rng: inout SeededRandom) -> Quaternion {
        let error = rng.gaussianVector(sigma: sigma)
        let angle = error.norm
        guard angle > 0 else { return truth.attitude }
        return (truth.attitude * Quaternion(axis: error, angle: angle)).normalized
    }
}

/// Inertial measurement unit: gyro rates and non-gravitational acceleration
/// in the body frame.
public struct InertialMeasurementUnitModel {
    public var gyroSigma: Double          // rad/s
    public var accelerometerSigma: Double // km/s²
    public var accelerometerBias: Vector3 // km/s², body frame

    public init(gyroSigma: Double = 1e-6, accelerometerSigma: Double = 1e-8, accelerometerBias: Vector3 = .zero) {
        self.gyroSigma = gyroSigma
        self.accelerometerSigma = accelerometerSigma
        self.accelerometerBias = accelerometerBias
    }

    public struct Reading {
        public let bodyRate: Vector3
        public let bodyAcceleration: Vector3
    }

    public func measure(trueBodyRate: Vector3, trueInertialAcceleration: Vector3,
                        truth: SpacecraftTruth, rng: inout SeededRandom) -> Reading {
        let rate = trueBodyRate + rng.gaussianVector(sigma: gyroSigma)
        let accelBody = truth.attitude.rotateInverse(trueInertialAcceleration)
            + accelerometerBias + rng.gaussianVector(sigma: accelerometerSigma)
        return Reading(bodyRate: rate, bodyAcceleration: accelBody)
    }
}

/// Optical navigation camera: centroid direction of a body against the star
/// background, reported in the camera (= body) frame.
public struct OpticalCameraModel {
    /// Per-axis centroiding 1σ, radians.
    public var sigma: Double
    /// Minimum angle between boresight target and Sun for a usable image.
    public var sunExclusionAngle: Double

    public init(sigma: Double = 5 * Constants.arcsecondsToRadians, sunExclusionAngle: Double = 30 * Constants.degreesToRadians) {
        self.sigma = sigma
        self.sunExclusionAngle = sunExclusionAngle
    }

    /// Returns the noisy body-frame direction, or `nil` if the Sun is too close.
    public func observe(_ body: CelestialBody, truth: SpacecraftTruth, ephemeris: any Ephemeris,
                        rng: inout SeededRandom) -> Vector3? {
        let (dirInertial, _) = BearingMeasurement.apparentDirection(
            to: body, from: truth.position, velocity: truth.velocity, epoch: truth.epoch, ephemeris: ephemeris)
        let sunDir = (-truth.position).normalized
        if body != .sun, dirInertial.angle(to: sunDir) < sunExclusionAngle { return nil }
        let noisy = (dirInertial + rng.gaussianVector(sigma: sigma)).normalized
        return truth.attitude.rotateInverse(noisy)
    }
}

/// Fine Sun sensor: body-frame direction to the Sun.
public struct SunSensorModel {
    public var sigma: Double
    public init(sigma: Double = 0.01 * Constants.degreesToRadians) { self.sigma = sigma }

    public func observe(truth: SpacecraftTruth, ephemeris: any Ephemeris, rng: inout SeededRandom) -> Vector3 {
        let (dirInertial, _) = BearingMeasurement.apparentDirection(
            to: .sun, from: truth.position, velocity: truth.velocity, epoch: truth.epoch, ephemeris: ephemeris)
        let noisy = (dirInertial + rng.gaussianVector(sigma: sigma)).normalized
        return truth.attitude.rotateInverse(noisy)
    }
}

/// X-ray detector folding photon arrival times on the pulsar's known period.
public struct PulsarDetectorModel {
    /// Timing 1σ in seconds after one observation interval.
    public var sigma: Double
    public init(sigma: Double = 5e-6) { self.sigma = sigma }

    public func observe(_ pulsar: Pulsar, truth: SpacecraftTruth, ephemeris: any Ephemeris,
                        rng: inout SeededRandom) -> PulsarMeasurement {
        let barycentric = truth.position - ephemeris.solarSystemBarycenterOffset(at: truth.epoch)
        let delay = pulsar.direction.dot(barycentric) / Constants.speedOfLight + rng.gaussian(sigma: sigma)
        var phase = delay.truncatingRemainder(dividingBy: pulsar.period)
        if phase < 0 { phase += pulsar.period }
        return PulsarMeasurement(epoch: truth.epoch, pulsar: pulsar, observedDelay: phase, sigma: sigma)
    }
}

/// Solar spectrometer measuring the Doppler shift of Fraunhofer lines.
public struct SolarDopplerModel {
    public var sigma: Double // km/s
    public init(sigma: Double = 0.001) { self.sigma = sigma }

    public func observe(truth: SpacecraftTruth, rng: inout SeededRandom) -> SolarDopplerMeasurement {
        let radial = truth.velocity.dot(truth.position.normalized) + rng.gaussian(sigma: sigma)
        return SolarDopplerMeasurement(epoch: truth.epoch, radialVelocity: radial, sigma: sigma)
    }
}

/// Radar / laser altimeter for close approaches.
public struct AltimeterModel {
    public var sigma: Double     // km
    public var maxRange: Double  // km
    public init(sigma: Double = 0.01, maxRange: Double = 50_000) {
        self.sigma = sigma
        self.maxRange = maxRange
    }

    public func observe(_ body: CelestialBody, truth: SpacecraftTruth, ephemeris: any Ephemeris,
                        rng: inout SeededRandom) -> RangeMeasurement? {
        let p = BearingMeasurement.retardedPosition(of: body, at: truth.epoch, seenFrom: truth.position, ephemeris: ephemeris)
        let dist = (p - truth.position).norm - body.meanRadius
        guard dist > 0, dist < maxRange else { return nil }
        return RangeMeasurement(epoch: truth.epoch, body: body, range: dist + rng.gaussian(sigma: sigma), sigma: sigma)
    }
}
