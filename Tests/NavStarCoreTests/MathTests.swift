import XCTest
@testable import NavStarCore

final class MathTests: XCTestCase {
    func testMatrixInverseRoundTrip() {
        let a = Matrix(rows: 3, cols: 3, data: [4, 1, 2, 1, 5, 3, 2, 3, 6])
        let inv = a.inverse()!
        let identity = a * inv
        for i in 0..<3 { for j in 0..<3 { XCTAssertEqual(identity[i, j], i == j ? 1 : 0, accuracy: 1e-12) } }
    }

    func testMatrix3InverseMatchesGeneral() {
        let m = Matrix3([2, 0, 1, 1, 3, 0, 0, 1, 4])
        let a = Matrix(m).inverse()!
        let b = m.inverse!
        for i in 0..<3 { for j in 0..<3 { XCTAssertEqual(a[i, j], b[i, j], accuracy: 1e-12) } }
        XCTAssertNil(Matrix3([1, 2, 3, 2, 4, 6, 0, 0, 1]).inverse)
    }

    func testQuaternionRotationMatchesMatrix() {
        let q = Quaternion(axis: Vector3(1, -2, 0.5), angle: 1.1)
        let v = Vector3(0.3, -0.7, 2.0)
        let a = q.rotate(v)
        let b = q.rotationMatrix * v
        XCTAssertEqual((a - b).norm, 0, accuracy: 1e-12)
        XCTAssertEqual(a.norm, v.norm, accuracy: 1e-12)
        XCTAssertEqual((q.rotateInverse(a) - v).norm, 0, accuracy: 1e-12)
    }

    func testQuaternionGyroIntegration() {
        var q = Quaternion.identity
        let rate = Vector3(0, 0, 0.1)
        for _ in 0..<100 { q = q.integrated(bodyRate: rate, dt: 0.1) }   // 1 rad about z
        let expected = Quaternion(axis: .unitZ, angle: 1.0)
        XCTAssertEqual(q.angularDistance(to: expected), 0, accuracy: 1e-9)
    }

    func testTangentBasisIsOrthonormal() {
        let u = Vector3(0.2, -0.9, 0.4)
        let (e1, e2) = u.tangentBasis()
        XCTAssertEqual(e1.norm, 1, accuracy: 1e-12)
        XCTAssertEqual(e2.norm, 1, accuracy: 1e-12)
        XCTAssertEqual(e1.dot(e2), 0, accuracy: 1e-12)
        XCTAssertEqual(e1.dot(u), 0, accuracy: 1e-12)
        XCTAssertEqual(e2.dot(u), 0, accuracy: 1e-12)
    }

    func testWrapping() {
        XCTAssertEqual(wrapped(0.9, period: 1.0), -0.1, accuracy: 1e-12)
        XCTAssertEqual(wrapped(-0.9, period: 1.0), 0.1, accuracy: 1e-12)
        XCTAssertEqual(wrapped(2.3, period: 1.0), 0.3, accuracy: 1e-12)
        XCTAssertEqual(wrapped(0.5, period: 1.0), -0.5, accuracy: 1e-12)
    }
}
