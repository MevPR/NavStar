import XCTest
@testable import NavStarCore

final class MeasurementTests: XCTestCase {
    let eph = OnboardEphemeris()
    let t = Epoch(year: 2026, month: 12, day: 15)
    lazy var context = NavigationContext(ephemeris: eph)
    lazy var state = NavigationState(position: Vector3(1.3e8, 0.9e8, 1.0e6), velocity: Vector3(-20, 25, 0.5),
                                     epoch: t, positionSigma: 1000, velocitySigma: 0.01)

    /// Compares the analytic Jacobian with central differences of the residual.
    private func checkJacobian(_ m: any NavigationMeasurement, scale: [Double], tolerance: Double) throws {
        let eval = try m.evaluate(state: state, context: context)
        for j in 0..<6 {
            var plus = state, minus = state
            let h = scale[j]
            if j < 3 { plus.position[j] += h; minus.position[j] -= h } else { plus.velocity[j - 3] += h; minus.velocity[j - 3] -= h }
            let yp = try m.evaluate(state: plus, context: context).residual
            let ym = try m.evaluate(state: minus, context: context).residual
            for i in 0..<yp.count {
                // residual = z − h(x)  ⇒  ∂residual/∂x = −H
                let numeric = -(yp[i] - ym[i]) / (2 * h)
                XCTAssertEqual(eval.jacobian[i, j], numeric, accuracy: tolerance * max(1, abs(numeric)), "entry (\(i),\(j)) of \(m.label)")
            }
        }
    }

    func testBearingJacobian() throws {
        let (dir, _) = BearingMeasurement.apparentDirection(to: .mars, from: state.position + Vector3(300, -200, 100),
                                                            velocity: state.velocity, epoch: t, ephemeris: eph)
        let m = BearingMeasurement(epoch: t, body: .mars, observedDirection: dir, sigma: 1e-5)
        try checkJacobian(m, scale: [1, 1, 1, 1e-3, 1e-3, 1e-3], tolerance: 3e-4)
    }

    func testBearingResidualIsZeroForPerfectObservation() throws {
        let (dir, _) = BearingMeasurement.apparentDirection(to: .earth, from: state.position, velocity: state.velocity, epoch: t, ephemeris: eph)
        let m = BearingMeasurement(epoch: t, body: .earth, observedDirection: dir, sigma: 1e-5)
        let eval = try m.evaluate(state: state, context: context)
        XCTAssertEqual(eval.residual[0], 0, accuracy: 1e-14)
        XCTAssertEqual(eval.residual[1], 0, accuracy: 1e-14)
    }

    func testDopplerJacobian() throws {
        let m = SolarDopplerMeasurement(epoch: t, radialVelocity: 5.0, sigma: 1e-3)
        try checkJacobian(m, scale: [10, 10, 10, 1e-3, 1e-3, 1e-3], tolerance: 1e-6)
    }

    func testRangeJacobian() throws {
        // The analytic Jacobian neglects the light-time derivative (body
        // velocity / c ≈ 1e-4), which is well below what the filter needs.
        let m = RangeMeasurement(epoch: t, body: .mars, range: 1e7, sigma: 1)
        try checkJacobian(m, scale: [1, 1, 1, 1, 1, 1], tolerance: 3e-4)
    }

    func testPulsarJacobianAndWrap() throws {
        let pulsar = Pulsar.catalog[0]
        let bary = state.position - eph.solarSystemBarycenterOffset(at: t)
        let trueDelay = pulsar.direction.dot(bary) / Constants.speedOfLight
        var phase = (trueDelay + 2e-6).truncatingRemainder(dividingBy: pulsar.period)
        if phase < 0 { phase += pulsar.period }
        var tight = state
        tight.covariance = Matrix.diagonal([100, 100, 100, 1e-6, 1e-6, 1e-6, 1e-20, 1e-20, 1e-20])
        let m = PulsarMeasurement(epoch: t, pulsar: pulsar, observedDelay: phase, sigma: 5e-6)
        let eval = try m.evaluate(state: tight, context: context)
        XCTAssertEqual(eval.residual[0], 2e-6, accuracy: 1e-12)
        for k in 0..<3 { XCTAssertEqual(eval.jacobian[0, k], pulsar.direction[k] / Constants.speedOfLight, accuracy: 1e-15) }

        // Prior of 1000 km along the line of sight cannot resolve a 467 km cycle.
        XCTAssertThrowsError(try m.evaluate(state: state, context: context))
    }
}
