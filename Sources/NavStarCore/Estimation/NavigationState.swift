import Foundation

/// Estimated heliocentric state (EME2000) with its 9×9 covariance, ordered
/// [x y z vx vy vz ax ay az]: position (km), velocity (km/s) and the
/// unmodelled-acceleration bias (km/s²) estimated as a first-order
/// Gauss–Markov process (solar radiation pressure mismodelling, outgassing,
/// thruster leaks).
public struct NavigationState {
    public static let dimension = 9

    public var position: Vector3
    public var velocity: Vector3
    public var accelerationBias: Vector3
    public var epoch: Epoch
    public var covariance: Matrix

    public init(position: Vector3, velocity: Vector3, accelerationBias: Vector3 = .zero, epoch: Epoch, covariance: Matrix) {
        precondition(covariance.rows == NavigationState.dimension && covariance.cols == NavigationState.dimension,
                     "Covariance must be \(NavigationState.dimension)×\(NavigationState.dimension)")
        self.position = position
        self.velocity = velocity
        self.accelerationBias = accelerationBias
        self.epoch = epoch
        self.covariance = covariance
    }

    /// Convenience initialiser with uncorrelated 1σ uncertainties.
    public init(position: Vector3, velocity: Vector3, epoch: Epoch,
                positionSigma: Double, velocitySigma: Double, accelerationSigma: Double = 1e-10) {
        let p2 = positionSigma * positionSigma
        let v2 = velocitySigma * velocitySigma
        let a2 = accelerationSigma * accelerationSigma
        self.init(position: position, velocity: velocity, epoch: epoch,
                  covariance: Matrix.diagonal([p2, p2, p2, v2, v2, v2, a2, a2, a2]))
    }

    public var vector: [Double] { position.components + velocity.components + accelerationBias.components }

    /// 1σ position uncertainty per axis (km).
    public var positionSigma: Vector3 {
        Vector3(covariance[0, 0].squareRoot(), covariance[1, 1].squareRoot(), covariance[2, 2].squareRoot())
    }

    /// 1σ velocity uncertainty per axis (km/s).
    public var velocitySigma: Vector3 {
        Vector3(covariance[3, 3].squareRoot(), covariance[4, 4].squareRoot(), covariance[5, 5].squareRoot())
    }

    /// 1σ acceleration-bias uncertainty per axis (km/s²).
    public var accelerationSigma: Vector3 {
        Vector3(covariance[6, 6].squareRoot(), covariance[7, 7].squareRoot(), covariance[8, 8].squareRoot())
    }

    /// Root-sum-square of the position 1σ values (km).
    public var positionSigmaRSS: Double { positionSigma.norm }
    public var velocitySigmaRSS: Double { velocitySigma.norm }
}
