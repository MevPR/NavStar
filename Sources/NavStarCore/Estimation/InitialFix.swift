import Foundation

/// Cold-start position/velocity determination from bearing measurements alone,
/// used when the filter has no usable prior (power-up, safe-mode recovery).
public enum InitialFix {
    public struct PositionFix {
        public let epoch: Epoch
        public let position: Vector3
        /// 3×3 position covariance derived from the weighted least-squares normal matrix (km²).
        public let covariance: Matrix3
    }

    /// Weighted least-squares triangulation: each bearing defines a line through
    /// the (light-time corrected) body position; the fix minimises the weighted
    /// squared perpendicular distances to all lines. Needs at least two bearings
    /// to bodies in different directions. `velocityHint` removes the stellar
    /// aberration when the observer velocity is already roughly known.
    public static func triangulate(bearings: [BearingMeasurement], ephemeris: any Ephemeris,
                                   velocityHint: Vector3 = .zero, iterations: Int = 4) -> PositionFix? {
        guard bearings.count >= 2, let epoch = bearings.first?.epoch else { return nil }
        var estimate = Vector3.zero
        var normal = Matrix3.zero
        var haveEstimate = false

        for _ in 0..<iterations {
            normal = Matrix3.zero
            var rhs = Vector3.zero
            for b in bearings {
                var p = ephemeris.position(of: b.body, at: b.epoch)
                var weight = 1.0
                if haveEstimate {
                    p = BearingMeasurement.retardedPosition(of: b.body, at: b.epoch, seenFrom: estimate, ephemeris: ephemeris)
                    let range = (p - estimate).norm
                    weight = 1 / (b.sigma * b.sigma * range * range)
                }
                let u = geometricDirection(apparent: b.observedDirection, velocity: velocityHint)
                let proj = Matrix3.identity - Matrix3.outer(u, u)
                normal = normal + weight * proj
                rhs += weight * (proj * p)
            }
            guard let inv = normal.inverse else { return nil }
            estimate = inv * rhs
            haveEstimate = true
        }
        guard let cov = normal.inverse else { return nil }
        return PositionFix(epoch: epoch, position: estimate, covariance: cov)
    }

    /// Exact inverse of the first-order aberration model
    /// `apparent = normalize(u + v/c)`: solves for the unit vector `u`.
    static func geometricDirection(apparent a: Vector3, velocity v: Vector3) -> Vector3 {
        let beta = v / Constants.speedOfLight
        let ab = a.dot(beta)
        let k = ab + (ab * ab + 1 - beta.normSquared).squareRoot()
        return (k * a - beta).normalized
    }

    /// Combines two position fixes into a full state. The velocity is found by
    /// shooting: the initial velocity is corrected until the propagated
    /// trajectory passes through the second fix, so gravitational curvature
    /// between the fixes is accounted for. The aberration correction is then
    /// refined with the velocity found and the whole procedure repeated.
    public static func coldStart(first: [BearingMeasurement], second: [BearingMeasurement],
                                 ephemeris: any Ephemeris, accelerationSigma: Double = 1e-10) -> NavigationState? {
        let dynamics = DynamicsModel(ephemeris: ephemeris)
        var hint0 = Vector3.zero
        var hint1 = Vector3.zero
        var solution: (fix0: PositionFix, fix1: PositionFix, v1: Vector3)?

        for _ in 0..<3 {
            guard let f0 = triangulate(bearings: first, ephemeris: ephemeris, velocityHint: hint0),
                  let f1 = triangulate(bearings: second, ephemeris: ephemeris, velocityHint: hint1) else { return nil }
            let dt = f1.epoch - f0.epoch
            guard dt > 0 else { return nil }
            let (v0, v1) = shoot(from: f0, to: f1, dynamics: dynamics)
            hint0 = v0
            hint1 = v1
            solution = (f0, f1, v1)
        }
        guard let (f0, f1, v1) = solution else { return nil }
        let dt = f1.epoch - f0.epoch

        // v1 ≈ (r1 − r0)/dt + known terms  ⇒  linear error propagation of the two fixes.
        var cov = Matrix(rows: NavigationState.dimension, cols: NavigationState.dimension)
        cov.setBlock(row: 0, col: 0, f1.covariance)
        cov.setBlock(row: 3, col: 3, (1 / (dt * dt)) * (f0.covariance + f1.covariance))
        cov.setBlock(row: 0, col: 3, (1 / dt) * f1.covariance)
        cov.setBlock(row: 3, col: 0, (1 / dt) * f1.covariance)
        cov.setBlock(row: 6, col: 6, (accelerationSigma * accelerationSigma) * Matrix3.identity)
        return NavigationState(position: f1.position, velocity: v1, epoch: f1.epoch, covariance: cov)
    }

    /// Simple shooting solution of the two-point boundary value problem.
    static func shoot(from f0: PositionFix, to f1: PositionFix, dynamics: DynamicsModel) -> (v0: Vector3, v1: Vector3) {
        let dt = f1.epoch - f0.epoch
        var v0 = (f1.position - f0.position) / dt
        var vEnd = v0
        for _ in 0..<5 {
            var r = f0.position
            var v = v0
            var t = f0.epoch
            while f1.epoch - t > 1e-9 {
                let h = min(60.0, f1.epoch - t)
                (r, v) = Integrator.rk4Step(position: r, velocity: v, epoch: t, dt: h) { rr, tt in
                    dynamics.acceleration(position: rr, epoch: tt)
                }
                t = t.adding(seconds: h)
            }
            vEnd = v
            let miss = f1.position - r
            v0 += miss / dt
            if miss.norm < 1e-3 { break }
        }
        return (v0, vEnd)
    }
}
