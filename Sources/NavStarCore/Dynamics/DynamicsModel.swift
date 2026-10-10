import Foundation

/// Heliocentric point-mass force model: Sun plus third-body perturbations
/// (direct and indirect terms) plus any known non-gravitational acceleration
/// (thrust measured by the accelerometers).
public struct DynamicsModel {
    public var ephemeris: any Ephemeris
    public var perturbingBodies: [CelestialBody]

    public init(ephemeris: any Ephemeris, perturbingBodies: [CelestialBody] = CelestialBody.planets) {
        self.ephemeris = ephemeris
        self.perturbingBodies = perturbingBodies
    }

    /// Total acceleration in km/s² at heliocentric position `r`.
    public func acceleration(position r: Vector3, epoch: Epoch, additional: Vector3 = .zero) -> Vector3 {
        let rn = r.norm
        var a = (-Constants.gmSun / (rn * rn * rn)) * r
        for body in perturbingBodies {
            let p = ephemeris.position(of: body, at: epoch)
            let d = p - r
            let dn = d.norm
            let pn = p.norm
            a += body.gm * (d / (dn * dn * dn) - p / (pn * pn * pn))
        }
        return a + additional
    }

    /// Jacobian ∂a/∂r (gravity gradient) used for covariance propagation.
    public func gravityGradient(position r: Vector3, epoch: Epoch) -> Matrix3 {
        var g = DynamicsModel.pointMassGradient(gm: Constants.gmSun, relative: -r)
        for body in perturbingBodies {
            let d = ephemeris.position(of: body, at: epoch) - r
            g = g + DynamicsModel.pointMassGradient(gm: body.gm, relative: d)
        }
        return g
    }

    /// ∂/∂r of gm·d/|d|³ where d = p − r:  gm·(3 d dᵀ/|d|⁵ − I/|d|³).
    static func pointMassGradient(gm: Double, relative d: Vector3) -> Matrix3 {
        let dn = d.norm
        let d3 = dn * dn * dn
        let d5 = d3 * dn * dn
        return (3 * gm / d5) * Matrix3.outer(d, d) - (gm / d3) * Matrix3.identity
    }
}
