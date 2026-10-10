import Foundation

public enum Integrator {
    /// Classical fourth-order Runge–Kutta step for the second-order system
    /// r̈ = a(r, t). Returns the state after `dt` seconds.
    public static func rk4Step(position r: Vector3, velocity v: Vector3, epoch t: Epoch, dt: Double,
                               acceleration a: (Vector3, Epoch) -> Vector3) -> (position: Vector3, velocity: Vector3) {
        let half = dt / 2
        let tHalf = t.adding(seconds: half)
        let tEnd = t.adding(seconds: dt)

        let k1v = a(r, t)
        let k1r = v
        let k2v = a(r + half * k1r, tHalf)
        let k2r = v + half * k1v
        let k3v = a(r + half * k2r, tHalf)
        let k3r = v + half * k2v
        let k4v = a(r + dt * k3r, tEnd)
        let k4r = v + dt * k3v

        let rNew = r + (dt / 6) * (k1r + 2 * k2r + 2 * k3r + k4r)
        let vNew = v + (dt / 6) * (k1v + 2 * k2v + 2 * k3v + k4v)
        return (rNew, vNew)
    }
}
