import Foundation
import NavStarCore

// MARK: - Simulation configuration

let ephemeris = OnboardEphemeris()
var rng = SeededRandom(seed: 20261001)
let t0 = Epoch(year: 2026, month: 11, day: 1)
let durationDays = 30
let truthStep = 600.0                    // s
let sunSensorInterval = 3_600            // s
let opticalInterval = 6 * 3_600          // s
let dopplerInterval = 6 * 3_600          // s
let pulsarInterval = 12 * 3_600          // s
let reportInterval = 86_400              // s
let opticalTargets: [CelestialBody] = [.earth, .mars, .jupiter]

// Mid-course correction: 1 h burn at day 15, 2 mm/s² along the velocity vector.
let burnStart = 15 * 86_400
let burnDuration = 3_600
let burnAcceleration = 2e-6              // km/s²

let starTracker = StarTrackerModel()
let imu = InertialMeasurementUnitModel(accelerometerBias: Vector3(2e-9, -1e-9, 1e-9))
let camera = OpticalCameraModel()
let sunSensor = SunSensorModel()
let pulsarDetector = PulsarDetectorModel()
let spectrometer = SolarDopplerModel()

// MARK: - Truth model

let truthDynamics = DynamicsModel(ephemeris: ephemeris)
let unmodelledAcceleration = rng.gaussianVector(sigma: 5e-11)   // e.g. solar-radiation-pressure mismodelling
let bodyRate = Vector3(0, 0, 2e-4)                             // slow roll, rad/s

var earthPosition = ephemeris.position(of: .earth, at: t0)
var earthVelocity = ephemeris.velocity(of: .earth, at: t0)
var truth = SpacecraftTruth(
    epoch: t0,
    position: earthPosition + 2.0e6 * earthPosition.normalized,
    velocity: earthVelocity + 3.0 * earthVelocity.normalized,
    attitude: Quaternion(axis: Vector3(1, 2, 3), angle: 0.7)
)

func thrust(at seconds: Int) -> Vector3 {
    guard seconds >= burnStart, seconds < burnStart + burnDuration else { return .zero }
    return burnAcceleration * truth.velocity.normalized
}

func advanceTruth(by dt: Double, elapsed: Int) {
    let extra = unmodelledAcceleration + thrust(at: elapsed)
    let (r, v) = Integrator.rk4Step(position: truth.position, velocity: truth.velocity, epoch: truth.epoch, dt: dt) { r, t in
        truthDynamics.acceleration(position: r, epoch: t, additional: extra)
    }
    truth.position = r
    truth.velocity = v
    truth.epoch = truth.epoch.adding(seconds: dt)
    truth.attitude = truth.attitude.integrated(bodyRate: bodyRate, dt: dt)
}

// MARK: - Navigator

let placeholder = NavigationState(position: truth.position, velocity: .zero, epoch: t0,
                                  positionSigma: 1e8, velocitySigma: 100)
let navigator = Navigator(initialState: placeholder, ephemeris: ephemeris)

func opticalBearings() -> [BearingMeasurement] {
    navigator.updateAttitude(starTracker: starTracker.measure(truth: truth, rng: &rng))
    var out: [BearingMeasurement] = []
    for body in opticalTargets {
        if let dir = camera.observe(body, truth: truth, ephemeris: ephemeris, rng: &rng) {
            out.append(navigator.makeBearing(to: body, bodyFrameDirection: dir, sigma: camera.sigma, epoch: truth.epoch))
        }
    }
    return out
}

func sunBearing() -> BearingMeasurement {
    let dir = sunSensor.observe(truth: truth, ephemeris: ephemeris, rng: &rng)
    return navigator.makeBearing(to: .sun, bodyFrameDirection: dir, sigma: sunSensor.sigma, epoch: truth.epoch)
}

var accepted = 0
var rejected = 0
var rejectionReasons: [String: Int] = [:]
var pulsarFirstAccepted: [String: Epoch] = [:]

func ingest(_ m: any NavigationMeasurement) {
    let result = navigator.ingest(m)
    switch result {
    case .accepted:
        accepted += 1
        if let p = m as? PulsarMeasurement, pulsarFirstAccepted[p.pulsar.name] == nil {
            pulsarFirstAccepted[p.pulsar.name] = m.epoch
            print("  \(m.epoch)  first accepted XNAV fix from \(p.pulsar.name)")
        }
    case let .rejected(reason):
        rejected += 1
        var key = reason.components(separatedBy: " (").first ?? reason
        if key.hasPrefix("innovation") { key = "innovation exceeds gate" }
        rejectionReasons["\(m.label): \(key)", default: 0] += 1
    }
}

func report(_ label: String) {
    let s = navigator.state
    let posErr = (s.position - truth.position).norm
    let velErr = (s.velocity - truth.velocity).norm * 1000
    print(String(format: "%@  %-20@  %10.1f km   %10.1f km   %8.2f m/s   %8.2f m/s   %5d / %-4d",
                 s.epoch.description, label, posErr, 3 * s.positionSigmaRSS, velErr, 3 * s.velocitySigmaRSS * 1000,
                 accepted, rejected))
}

// MARK: - Run

print("NavStar autonomous navigation simulation")
print("Start \(t0), heliocentric distance \(String(format: "%.4f", truth.position.norm / Constants.astronomicalUnit)) AU, speed \(String(format: "%.3f", truth.velocity.norm)) km/s")
print("Sensors: star tracker 3\", OpNav camera 5\" [\(opticalTargets.map(\.displayName).joined(separator: ", "))], Sun sensor 0.01°, XNAV 5 µs, solar Doppler 1 m/s, IMU with bias\n")

// Cold start: two bearing sets six hours apart, no prior state at all.
let firstSet = opticalBearings() + [sunBearing()]
var elapsed = 0
while elapsed < 6 * 3_600 {
    advanceTruth(by: truthStep, elapsed: elapsed)
    elapsed += Int(truthStep)
}
let secondSet = opticalBearings() + [sunBearing()]
guard navigator.coldStart(first: firstSet, second: secondSet) else {
    print("cold start failed")
    exit(1)
}
print("Cold start (bearings only, no prior):")
print("  " + (navigator.events.last ?? ""))
print(String(format: "  position error %.1f km, velocity error %.2f m/s\n",
             (navigator.state.position - truth.position).norm,
             (navigator.state.velocity - truth.velocity).norm * 1000))

print("Epoch (TDB)          Event                 pos error      3σ pos         vel error      3σ vel         acc / rej")
report("after cold start")

let endSeconds = durationDays * 86_400
while elapsed < endSeconds {
    let dt = truthStep
    let trueThrust = thrust(at: elapsed)
    advanceTruth(by: dt, elapsed: elapsed)
    elapsed += Int(dt)

    // Attitude from the star tracker, thrust from the accelerometers.
    navigator.updateAttitude(starTracker: starTracker.measure(truth: truth, rng: &rng))
    let reading = imu.measure(trueBodyRate: bodyRate, trueInertialAcceleration: trueThrust, truth: truth, rng: &rng)
    navigator.advance(to: truth.epoch, bodyAcceleration: reading.bodyAcceleration)

    if elapsed % sunSensorInterval == 0 { ingest(sunBearing()) }
    if elapsed % opticalInterval == 0 { for b in opticalBearings() { ingest(b) } }
    if elapsed % dopplerInterval == 0 { ingest(spectrometer.observe(truth: truth, rng: &rng)) }
    if elapsed % pulsarInterval == 0 {
        for pulsar in Pulsar.catalog {
            ingest(pulsarDetector.observe(pulsar, truth: truth, ephemeris: ephemeris, rng: &rng))
        }
    }
    if elapsed == burnStart + burnDuration { report("after MCC burn") }
    if elapsed % reportInterval == 0 { report("day \(elapsed / 86_400)") }
}

print("\nMeasurements accepted: \(accepted), rejected: \(rejected)")
for (reason, count) in rejectionReasons.sorted(by: { $0.value > $1.value }) {
    print("  \(count)× \(reason)")
}
let final = navigator.state
print(String(format: "\nFinal position error %.2f km (3σ %.2f km), velocity error %.3f m/s (3σ %.3f m/s)",
             (final.position - truth.position).norm, 3 * final.positionSigmaRSS,
             (final.velocity - truth.velocity).norm * 1000, 3 * final.velocitySigmaRSS * 1000))
print(String(format: "Estimated acceleration bias %@ km/s² (truth %@, 3σ %.2g km/s²)",
             final.accelerationBias.description, unmodelledAcceleration.description, 3 * final.accelerationSigma.norm))
