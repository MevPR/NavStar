import Foundation

/// Dense 3×3 matrix stored row-major.
public struct Matrix3: Equatable {
    public var m: [Double]

    public init(_ m: [Double]) {
        precondition(m.count == 9, "Matrix3 needs nine entries")
        self.m = m
    }

    public init(rows r0: Vector3, _ r1: Vector3, _ r2: Vector3) {
        m = r0.components + r1.components + r2.components
    }

    public static let identity = Matrix3([1, 0, 0, 0, 1, 0, 0, 0, 1])
    public static let zero = Matrix3([Double](repeating: 0, count: 9))

    public subscript(r: Int, c: Int) -> Double {
        get { m[r * 3 + c] }
        set { m[r * 3 + c] = newValue }
    }

    public func row(_ i: Int) -> Vector3 { Vector3(m[i * 3], m[i * 3 + 1], m[i * 3 + 2]) }
    public func column(_ j: Int) -> Vector3 { Vector3(m[j], m[3 + j], m[6 + j]) }

    public var transposed: Matrix3 {
        Matrix3([m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]])
    }

    /// Outer product a bᵀ.
    public static func outer(_ a: Vector3, _ b: Vector3) -> Matrix3 {
        Matrix3([a.x * b.x, a.x * b.y, a.x * b.z,
                 a.y * b.x, a.y * b.y, a.y * b.z,
                 a.z * b.x, a.z * b.y, a.z * b.z])
    }

    public static func diagonal(_ d: Vector3) -> Matrix3 {
        Matrix3([d.x, 0, 0, 0, d.y, 0, 0, 0, d.z])
    }

    public static func * (a: Matrix3, b: Matrix3) -> Matrix3 {
        var out = [Double](repeating: 0, count: 9)
        for i in 0..<3 {
            for j in 0..<3 {
                var s = 0.0
                for k in 0..<3 { s += a.m[i * 3 + k] * b.m[k * 3 + j] }
                out[i * 3 + j] = s
            }
        }
        return Matrix3(out)
    }

    public static func * (a: Matrix3, v: Vector3) -> Vector3 {
        Vector3(a.row(0).dot(v), a.row(1).dot(v), a.row(2).dot(v))
    }

    public static func * (s: Double, a: Matrix3) -> Matrix3 { Matrix3(a.m.map { s * $0 }) }
    public static func + (a: Matrix3, b: Matrix3) -> Matrix3 { Matrix3(zip(a.m, b.m).map(+)) }
    public static func - (a: Matrix3, b: Matrix3) -> Matrix3 { Matrix3(zip(a.m, b.m).map(-)) }

    public var determinant: Double {
        m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    /// Inverse via adjugate, `nil` when singular.
    public var inverse: Matrix3? {
        let det = determinant
        guard abs(det) > 1e-300 else { return nil }
        let inv = 1.0 / det
        return Matrix3([
            (m[4] * m[8] - m[5] * m[7]) * inv, (m[2] * m[7] - m[1] * m[8]) * inv, (m[1] * m[5] - m[2] * m[4]) * inv,
            (m[5] * m[6] - m[3] * m[8]) * inv, (m[0] * m[8] - m[2] * m[6]) * inv, (m[2] * m[3] - m[0] * m[5]) * inv,
            (m[3] * m[7] - m[4] * m[6]) * inv, (m[1] * m[6] - m[0] * m[7]) * inv, (m[0] * m[4] - m[1] * m[3]) * inv,
        ])
    }

    public var trace: Double { m[0] + m[4] + m[8] }

    /// Right-handed rotation about the x axis by `angle` radians.
    public static func rotationX(_ angle: Double) -> Matrix3 {
        let c = cos(angle), s = sin(angle)
        return Matrix3([1, 0, 0, 0, c, -s, 0, s, c])
    }

    public static func rotationY(_ angle: Double) -> Matrix3 {
        let c = cos(angle), s = sin(angle)
        return Matrix3([c, 0, s, 0, 1, 0, -s, 0, c])
    }

    public static func rotationZ(_ angle: Double) -> Matrix3 {
        let c = cos(angle), s = sin(angle)
        return Matrix3([c, -s, 0, s, c, 0, 0, 0, 1])
    }
}
