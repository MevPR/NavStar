import XCTest
@testable import NavStarCore

final class EstimationTests: XCTestCase {
    let eph = OnboardEphemeris()
    let t0 = Epoch(year: 2026, month: 11, day: 1)

    private func truthState() -> (Vector3, Vector3) {
        let pe = eph.position(of: .earth, at: t0)
        let ve = eph.velocity(of: .earth, at: t0)
        return (pe + 2.0e6 * pe.normalized, ve + 3.0 * ve.normalized)
    }

    private func perfectBearings(position r: Vector3, velocity v: Vector3, epoch: Epoch) -> [BearingMeasurement] {
        [CelestialBody.sun, .earth, .mars, .jupiter].map { body in
            let (dir, _) = BearingMeasurement.apparentDirection(to: body, from: r, velocity: v, epoch: epoch, ephemeris: eph)
            return BearingMeasurement(epoch: epoch, body: body, observedDirection: dir, sigma: body == .sun ? 1.7e-4 : 2.4e-5)
        }
    }

    func testTriangulationRecoversPosition() {
        let (r, v) = truthState()
        let fix = InitialFix.triangulate(bearings: perfectBearings(position: r, velocity: v, epoch: t0), ephemeris: eph, velocityHint: v)!
        XCTAssertEqual((fix.position - r).norm, 0, accuracy: 0.5)
        XCTAssertGreaterThan(fix.covariance.trace, 0)
    }

    func testColdStartEstimatesVelocity() {
        var (r, v) = truthState()
        let first = perfectBearings(position: r, velocity: v, epoch: t0)
        let dyn = DynamicsModel(ephemeris: eph)
        var t = t0
        for _ in 0..<360 {
            (r, v) = Integrator.rk4Step(position: r, velocity: v, epoch: t, dt: 60) { rr, tt in dyn.acceleration(position: rr, epoch: tt) }
            t = t.adding(seconds: 60)
        }
        let second = perfectBearings(position: r, velocity: v, epoch: t)
        let state = InitialFix.coldStart(first: first, second: second, ephemeris: eph)!
        XCTAssertEqual((state.position - r).norm, 0, accuracy: 1.0)
        XCTAssertEqual((state.velocity - v).norm, 0, accuracy: 1e-4)   // 0.1 m/s
        XCTAssertGreaterThan(state.velocitySigmaRSS, (state.velocity - v).norm)
    }

    func testFilterConvergesOnOpticalBearings() {
        var (r, v) = truthState()
        let dyn = DynamicsModel(ephemeris: eph)
        var rng = SeededRandom(seed: 7)
        let initial = NavigationState(position: r + Vector3(4000, -3000, 2000), velocity: v + Vector3(0.004, 0.003, -0.002),
                                      epoch: t0, positionSigma: 5000, velocitySigma: 0.005)
        let nav = Navigator(initialState: initial, ephemeris: eph)
        var t = t0
        let step = 600.0
        var elapsed = 0
        while elapsed < 3 * 86_400 {
            (r, v) = Integrator.rk4Step(position: r, velocity: v, epoch: t, dt: step) { rr, tt in dyn.acceleration(position: rr, epoch: tt) }
            t = t.adding(seconds: step)
            elapsed += Int(step)
            nav.advance(to: t)
            if elapsed % 21_600 == 0 {
                for body in [CelestialBody.earth, .mars, .jupiter] {
                    let sigma = 2.4e-5
                    let (dir, _) = BearingMeasurement.apparentDirection(to: body, from: r, velocity: v, epoch: t, ephemeris: eph)
                    let noisy = (dir + rng.gaussianVector(sigma: sigma)).normalized
                    let result = nav.ingest(BearingMeasurement(epoch: t, body: body, observedDirection: noisy, sigma: sigma))
                    XCTAssertTrue(result.isAccepted, "\(result)")
                }
            }
        }
        // Bearings alone constrain the position well across the Earth line of
        // sight but only weakly along it (Mars/Jupiter are far away), so the
        // test checks consistency with the covariance rather than a tight bound.
        let posErr = (nav.state.position - r).norm
        let velErr = (nav.state.velocity - v).norm
        XCTAssertLessThan(posErr, 3000)
        XCTAssertLessThan(posErr, 4 * nav.state.positionSigmaRSS)
        XCTAssertLessThan(velErr, 4 * nav.state.velocitySigmaRSS + 1e-4)
    }

    func testStaleMeasurementIsRejected() {
        let (r, v) = truthState()
        let nav = Navigator(initialState: NavigationState(position: r, velocity: v, epoch: t0, positionSigma: 100, velocitySigma: 0.01), ephemeris: eph)
        nav.advance(to: t0.adding(seconds: 600))
        let m = SolarDopplerMeasurement(epoch: t0, radialVelocity: 0, sigma: 1)
        XCTAssertFalse(nav.ingest(m).isAccepted)
    }
}
