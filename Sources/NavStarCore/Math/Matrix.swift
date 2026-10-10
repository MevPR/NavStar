import Foundation

/// Small dense matrix of arbitrary size, row-major. Sized for Kalman filter
/// algebra (state dimension 6, measurement dimension 1–3); not optimised for
/// large systems.
public struct Matrix: Equatable, CustomStringConvertible {
    public let rows: Int
    public let cols: Int
    public var data: [Double]

    public init(rows: Int, cols: Int, data: [Double]) {
        precondition(data.count == rows * cols, "Matrix data size mismatch")
        self.rows = rows
        self.cols = cols
        self.data = data
    }

    public init(rows: Int, cols: Int) {
        self.init(rows: rows, cols: cols, data: [Double](repeating: 0, count: rows * cols))
    }

    public init(_ m3: Matrix3) {
        self.init(rows: 3, cols: 3, data: m3.m)
    }

    public init(rowVector v: [Double]) { self.init(rows: 1, cols: v.count, data: v) }
    public init(columnVector v: [Double]) { self.init(rows: v.count, cols: 1, data: v) }

    public static func identity(_ n: Int) -> Matrix {
        var m = Matrix(rows: n, cols: n)
        for i in 0..<n { m[i, i] = 1 }
        return m
    }

    public static func diagonal(_ d: [Double]) -> Matrix {
        var m = Matrix(rows: d.count, cols: d.count)
        for (i, v) in d.enumerated() { m[i, i] = v }
        return m
    }

    public subscript(r: Int, c: Int) -> Double {
        get { data[r * cols + c] }
        set { data[r * cols + c] = newValue }
    }

    public var transposed: Matrix {
        var t = Matrix(rows: cols, cols: rows)
        for i in 0..<rows { for j in 0..<cols { t[j, i] = self[i, j] } }
        return t
    }

    public var diagonal: [Double] { (0..<min(rows, cols)).map { self[$0, $0] } }

    /// (A + Aᵀ)/2 — keeps covariance matrices exactly symmetric after round-off.
    public var symmetrized: Matrix {
        precondition(rows == cols)
        var s = self
        for i in 0..<rows {
            for j in (i + 1)..<rows {
                let v = 0.5 * (self[i, j] + self[j, i])
                s[i, j] = v
                s[j, i] = v
            }
        }
        return s
    }

    public mutating func setBlock(row: Int, col: Int, _ b: Matrix) {
        for i in 0..<b.rows { for j in 0..<b.cols { self[row + i, col + j] = b[i, j] } }
    }

    public mutating func setBlock(row: Int, col: Int, _ b: Matrix3) {
        setBlock(row: row, col: col, Matrix(b))
    }

    public func block(row: Int, col: Int, rows r: Int, cols c: Int) -> Matrix {
        var b = Matrix(rows: r, cols: c)
        for i in 0..<r { for j in 0..<c { b[i, j] = self[row + i, col + j] } }
        return b
    }

    public func block3(row: Int, col: Int) -> Matrix3 {
        Matrix3(block(row: row, col: col, rows: 3, cols: 3).data)
    }

    public static func * (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.cols == b.rows, "Matrix dimension mismatch in multiplication")
        var out = Matrix(rows: a.rows, cols: b.cols)
        for i in 0..<a.rows {
            for k in 0..<a.cols {
                let aik = a[i, k]
                if aik == 0 { continue }
                for j in 0..<b.cols { out[i, j] += aik * b[k, j] }
            }
        }
        return out
    }

    public static func * (a: Matrix, v: [Double]) -> [Double] {
        precondition(a.cols == v.count, "Matrix/vector dimension mismatch")
        var out = [Double](repeating: 0, count: a.rows)
        for i in 0..<a.rows {
            var s = 0.0
            for j in 0..<a.cols { s += a[i, j] * v[j] }
            out[i] = s
        }
        return out
    }

    public static func * (s: Double, a: Matrix) -> Matrix {
        Matrix(rows: a.rows, cols: a.cols, data: a.data.map { s * $0 })
    }

    public static func + (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.rows == b.rows && a.cols == b.cols)
        return Matrix(rows: a.rows, cols: a.cols, data: zip(a.data, b.data).map(+))
    }

    public static func - (a: Matrix, b: Matrix) -> Matrix {
        precondition(a.rows == b.rows && a.cols == b.cols)
        return Matrix(rows: a.rows, cols: a.cols, data: zip(a.data, b.data).map(-))
    }

    /// Gauss–Jordan inverse with partial pivoting; `nil` when singular.
    public func inverse() -> Matrix? {
        precondition(rows == cols, "Only square matrices can be inverted")
        let n = rows
        var a = self
        var inv = Matrix.identity(n)
        for col in 0..<n {
            var pivot = col
            var best = abs(a[col, col])
            for r in (col + 1)..<n where abs(a[r, col]) > best {
                best = abs(a[r, col])
                pivot = r
            }
            guard best > 1e-300 else { return nil }
            if pivot != col {
                for j in 0..<n {
                    let t = a[col, j]; a[col, j] = a[pivot, j]; a[pivot, j] = t
                    let u = inv[col, j]; inv[col, j] = inv[pivot, j]; inv[pivot, j] = u
                }
            }
            let p = 1.0 / a[col, col]
            for j in 0..<n { a[col, j] *= p; inv[col, j] *= p }
            for r in 0..<n where r != col {
                let f = a[r, col]
                if f == 0 { continue }
                for j in 0..<n {
                    a[r, j] -= f * a[col, j]
                    inv[r, j] -= f * inv[col, j]
                }
            }
        }
        return inv
    }

    public var description: String {
        (0..<rows).map { i in
            (0..<cols).map { j in String(format: "%12.5g", self[i, j]) }.joined(separator: " ")
        }.joined(separator: "\n")
    }
}

public func dot(_ a: [Double], _ b: [Double]) -> Double {
    precondition(a.count == b.count)
    return zip(a, b).reduce(0) { $0 + $1.0 * $1.1 }
}
