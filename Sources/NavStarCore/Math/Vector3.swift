import Foundation

/// Three-dimensional Cartesian vector (double precision).
///
/// Throughout NavStar positions are expressed in km, velocities in km/s and
/// accelerations in km/s². The inertial reference frame is EME2000
/// (Earth mean equator and equinox of J2000, practically identical to ICRF).
public struct Vector3: Hashable, CustomStringConvertible {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public init(_ components: [Double]) {
        precondition(components.count == 3, "Vector3 needs exactly three components")
        self.init(components[0], components[1], components[2])
    }

    public static let zero = Vector3(0, 0, 0)
    public static let unitX = Vector3(1, 0, 0)
    public static let unitY = Vector3(0, 1, 0)
    public static let unitZ = Vector3(0, 0, 1)

    public var components: [Double] { [x, y, z] }

    public subscript(index: Int) -> Double {
        get {
            switch index {
            case 0: return x
            case 1: return y
            case 2: return z
            default: preconditionFailure("Vector3 index out of range")
            }
        }
        set {
            switch index {
            case 0: x = newValue
            case 1: y = newValue
            case 2: z = newValue
            default: preconditionFailure("Vector3 index out of range")
            }
        }
    }

    public var normSquared: Double { x * x + y * y + z * z }
    public var norm: Double { normSquared.squareRoot() }

    /// Unit vector in the same direction (returns `self` for the zero vector).
    public var normalized: Vector3 {
        let n = norm
        return n > 0 ? self / n : self
    }

    public func dot(_ other: Vector3) -> Double { x * other.x + y * other.y + z * other.z }

    public func cross(_ o: Vector3) -> Vector3 {
        Vector3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x)
    }

    /// Angle between two vectors in radians, numerically robust for small angles.
    public func angle(to other: Vector3) -> Double {
        atan2(cross(other).norm, dot(other))
    }

    /// Two orthonormal vectors spanning the plane perpendicular to `self`.
    /// Used to express direction residuals as two small angles.
    public func tangentBasis() -> (Vector3, Vector3) {
        let u = normalized
        let helper = abs(u.x) < 0.9 ? Vector3.unitX : Vector3.unitY
        let e1 = u.cross(helper).normalized
        let e2 = u.cross(e1).normalized
        return (e1, e2)
    }

    public static func + (a: Vector3, b: Vector3) -> Vector3 { Vector3(a.x + b.x, a.y + b.y, a.z + b.z) }
    public static func - (a: Vector3, b: Vector3) -> Vector3 { Vector3(a.x - b.x, a.y - b.y, a.z - b.z) }
    public static prefix func - (a: Vector3) -> Vector3 { Vector3(-a.x, -a.y, -a.z) }
    public static func * (s: Double, a: Vector3) -> Vector3 { Vector3(s * a.x, s * a.y, s * a.z) }
    public static func * (a: Vector3, s: Double) -> Vector3 { s * a }
    public static func / (a: Vector3, s: Double) -> Vector3 { Vector3(a.x / s, a.y / s, a.z / s) }
    public static func += (a: inout Vector3, b: Vector3) { a = a + b }
    public static func -= (a: inout Vector3, b: Vector3) { a = a - b }

    public var description: String {
        String(format: "(%.6g, %.6g, %.6g)", x, y, z)
    }
}
