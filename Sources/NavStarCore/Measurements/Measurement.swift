import Foundation

/// Everything a measurement model needs besides the state itself.
public struct NavigationContext {
    public let ephemeris: any Ephemeris
    public init(ephemeris: any Ephemeris) { self.ephemeris = ephemeris }
}

/// Linearised measurement: residual y = z − h(x̂), Jacobian H = ∂h/∂x (m×6) and
/// measurement noise covariance R (m×m).
public struct MeasurementEvaluation {
    public let residual: [Double]
    public let jacobian: Matrix
    public let noise: Matrix

    public init(residual: [Double], jacobian: Matrix, noise: Matrix) {
        precondition(jacobian.rows == residual.count && jacobian.cols == 6)
        precondition(noise.rows == residual.count && noise.cols == residual.count)
        self.residual = residual
        self.jacobian = jacobian
        self.noise = noise
    }
}

public enum MeasurementError: Error, CustomStringConvertible {
    case unusable(String)

    public var description: String {
        switch self {
        case let .unusable(reason): return reason
        }
    }
}

/// A sensor observation expressed in a form the filter can consume.
public protocol NavigationMeasurement {
    var epoch: Epoch { get }
    var label: String { get }
    func evaluate(state: NavigationState, context: NavigationContext) throws -> MeasurementEvaluation
}

/// Wraps a value into [−period/2, period/2).
public func wrapped(_ value: Double, period: Double) -> Double {
    var r = value.truncatingRemainder(dividingBy: period)
    if r >= period / 2 { r -= period }
    if r < -period / 2 { r += period }
    return r
}
