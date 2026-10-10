import XCTest
@testable import NavStarCore

final class DynamicsTests: XCTestCase {
    func testCircularOrbitClosesAfterOnePeriod() {
        let dyn = DynamicsModel(ephemeris: OnboardEphemeris(), perturbingBodies: [])
        let a = Constants.astronomicalUnit
        let vc = (Constants.gmSun / a).squareRoot()
        var r = Vector3(a, 0, 0)
        var v = Vector3(0, vc, 0)
        var t = Epoch(secondsSinceJ2000: 0)
        let period = 2 * Double.pi * (a * a * a / Constants.gmSun).squareRoot()
        let dt = 300.0
        let steps = Int((period / dt).rounded())
        let h = period / Double(steps)
        for _ in 0..<steps {
            (r, v) = Integrator.rk4Step(position: r, velocity: v, epoch: t, dt: h) { rr, tt in dyn.acceleration(position: rr, epoch: tt) }
            t = t.adding(seconds: h)
        }
        XCTAssertEqual((r - Vector3(a, 0, 0)).norm, 0, accuracy: 1.0)   // km
        XCTAssertEqual(r.norm, a, accuracy: 0.01)
    }

    func testGravityGradientMatchesFiniteDifference() {
        let dyn = DynamicsModel(ephemeris: OnboardEphemeris())
        let t = Epoch(year: 2026, month: 11, day: 1)
        let r = Vector3(1.1e8, -0.8e8, 0.2e7)
        let analytic = dyn.gravityGradient(position: r, epoch: t)
        let h = 10.0
        for j in 0..<3 {
            var dr = Vector3.zero
            dr[j] = h
            let col = (dyn.acceleration(position: r + dr, epoch: t) - dyn.acceleration(position: r - dr, epoch: t)) / (2 * h)
            for i in 0..<3 {
                XCTAssertEqual(analytic[i, j], col[i], accuracy: 1e-6 * abs(analytic[i, j]) + 1e-22)
            }
        }
    }
}
