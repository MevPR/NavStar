import Foundation

/// Unit quaternion describing the rotation from the spacecraft body frame to
/// the inertial frame: `v_inertial = q.rotate(v_body)`.
public struct Quaternion: Equatable {
    public var w: Double
    public var x: Double
    public var y: Double
    public var z: Double

    public init(w: Double, x: Double, y: Double, z: Double) {
        self.w = w; self.x = x; self.y = y; self.z = z
    }

    public static let identity = Quaternion(w: 1, x: 0, y: 0, z: 0)

    /// Rotation of `angle` radians about `axis` (right-hand rule).
    public init(axis: Vector3, angle: Double) {
        let a = axis.normalized
        let s = sin(angle / 2)
        self.init(w: cos(angle / 2), x: a.x * s, y: a.y * s, z: a.z * s)
    }

    public var norm: Double { (w * w + x * x + y * y + z * z).squareRoot() }

    public var normalized: Quaternion {
        let n = norm
        guard n > 0 else { return .identity }
        return Quaternion(w: w / n, x: x / n, y: y / n, z: z / n)
    }

    public var conjugate: Quaternion { Quaternion(w: w, x: -x, y: -y, z: -z) }
    public var vectorPart: Vector3 { Vector3(x, y, z) }

    /// Hamilton product; `a * b` applies `b` first, then `a`.
    public static func * (a: Quaternion, b: Quaternion) -> Quaternion {
        Quaternion(
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        )
    }

    /// Rotates a vector from the body frame into the reference frame.
    public func rotate(_ v: Vector3) -> Vector3 {
        let qv = Quaternion(w: 0, x: v.x, y: v.y, z: v.z)
        let r = self * qv * conjugate
        return Vector3(r.x, r.y, r.z)
    }

    /// Rotates a vector from the reference frame into the body frame.
    public func rotateInverse(_ v: Vector3) -> Vector3 { conjugate.rotate(v) }

    public var rotationMatrix: Matrix3 {
        Matrix3(rows: rotate(.unitX), rotate(.unitY), rotate(.unitZ)).transposed
    }

    /// Propagates the attitude with a body-frame angular rate (rad/s) held
    /// constant over `dt` seconds — the gyro integration step of the IMU.
    public func integrated(bodyRate omega: Vector3, dt: Double) -> Quaternion {
        let angle = omega.norm * dt
        guard angle > 0 else { return self }
        return (self * Quaternion(axis: omega, angle: angle)).normalized
    }

    /// Smallest rotation angle (radians) taking `self` to `other`.
    public func angularDistance(to other: Quaternion) -> Double {
        let d = (conjugate * other).normalized
        return 2 * atan2(d.vectorPart.norm, abs(d.w))
    }
}
