import Foundation

/// Source of heliocentric body positions in the EME2000 frame (km, km/s).
/// On a real spacecraft this is stored onboard before launch, exactly like the
/// star catalogue of the star tracker; it needs no link to Earth.
public protocol Ephemeris {
    func position(of body: CelestialBody, at epoch: Epoch) -> Vector3
    func velocity(of body: CelestialBody, at epoch: Epoch) -> Vector3
    /// Position of the solar-system barycentre relative to the Sun (km).
    func solarSystemBarycenterOffset(at epoch: Epoch) -> Vector3
}

/// Approximate Keplerian elements of the major planets referred to the mean
/// ecliptic and equinox of J2000, valid for 1800–2050 (JPL, Standish & Williams).
struct ApproximateElements {
    let a: Double, aDot: Double          // semi-major axis, AU and AU/century
    let e: Double, eDot: Double          // eccentricity
    let i: Double, iDot: Double          // inclination, degrees
    let l: Double, lDot: Double          // mean longitude, degrees
    let peri: Double, periDot: Double    // longitude of perihelion, degrees
    let node: Double, nodeDot: Double    // longitude of ascending node, degrees
}

/// Analytic planetary ephemeris computed from stored orbital elements.
/// Accuracy is a few hundred to a few thousand km for the inner planets, which is
/// the dominant systematic error budget of the optical navigation (see Plan.md).
public struct OnboardEphemeris: Ephemeris {
    public init() {}

    static let table: [CelestialBody: ApproximateElements] = [
        .mercury: ApproximateElements(a: 0.38709927, aDot: 0.00000037, e: 0.20563593, eDot: 0.00001906,
                                      i: 7.00497902, iDot: -0.00594749, l: 252.25032350, lDot: 149472.67411175,
                                      peri: 77.45779628, periDot: 0.16047689, node: 48.33076593, nodeDot: -0.12534081),
        .venus: ApproximateElements(a: 0.72333566, aDot: 0.00000390, e: 0.00677672, eDot: -0.00004107,
                                    i: 3.39467605, iDot: -0.00078890, l: 181.97909950, lDot: 58517.81538729,
                                    peri: 131.60246718, periDot: 0.00268329, node: 76.67984255, nodeDot: -0.27769418),
        .earth: ApproximateElements(a: 1.00000261, aDot: 0.00000562, e: 0.01671123, eDot: -0.00004392,
                                    i: -0.00001531, iDot: -0.01294668, l: 100.46457166, lDot: 35999.37244981,
                                    peri: 102.93768193, periDot: 0.32327364, node: 0.0, nodeDot: 0.0),
        .mars: ApproximateElements(a: 1.52371034, aDot: 0.00001847, e: 0.09339410, eDot: 0.00007882,
                                   i: 1.84969142, iDot: -0.00813131, l: -4.55343205, lDot: 19140.30268499,
                                   peri: -23.94362959, periDot: 0.44441088, node: 49.55953891, nodeDot: -0.29257343),
        .jupiter: ApproximateElements(a: 5.20288700, aDot: -0.00011607, e: 0.04838624, eDot: -0.00013253,
                                      i: 1.30439695, iDot: -0.00183714, l: 34.39644051, lDot: 3034.74612775,
                                      peri: 14.72847983, periDot: 0.21252668, node: 100.47390909, nodeDot: 0.20469106),
        .saturn: ApproximateElements(a: 9.53667594, aDot: -0.00125060, e: 0.05386179, eDot: -0.00050991,
                                     i: 2.48599187, iDot: 0.00193609, l: 49.95424423, lDot: 1222.49362201,
                                     peri: 92.59887831, periDot: -0.41897216, node: 113.66242448, nodeDot: -0.28867794),
        .uranus: ApproximateElements(a: 19.18916464, aDot: -0.00196176, e: 0.04725744, eDot: -0.00004397,
                                     i: 0.77263783, iDot: -0.00242939, l: 313.23810451, lDot: 428.48202785,
                                     peri: 170.95427630, periDot: 0.40805281, node: 74.01692503, nodeDot: 0.04240589),
        .neptune: ApproximateElements(a: 30.06992276, aDot: 0.00026291, e: 0.00859048, eDot: 0.00005105,
                                      i: 1.77004347, iDot: 0.00035372, l: -55.12002969, lDot: 218.45945325,
                                      peri: 44.96476227, periDot: -0.32241464, node: 131.78422574, nodeDot: -0.00508664),
    ]

    /// Heliocentric position in the ecliptic J2000 frame, in AU.
    public func eclipticPosition(of body: CelestialBody, at epoch: Epoch) -> Vector3 {
        guard let el = OnboardEphemeris.table[body] else { return .zero }
        let t = epoch.julianCenturies
        let deg = Constants.degreesToRadians
        let a = el.a + el.aDot * t
        let e = el.e + el.eDot * t
        let inc = (el.i + el.iDot * t) * deg
        let l = (el.l + el.lDot * t) * deg
        let peri = (el.peri + el.periDot * t) * deg
        let node = (el.node + el.nodeDot * t) * deg

        let omega = peri - node
        let meanAnomaly = l - peri
        let eAnom = Kepler.eccentricAnomaly(meanAnomaly: meanAnomaly, eccentricity: e)

        let xp = a * (cos(eAnom) - e)
        let yp = a * (1 - e * e).squareRoot() * sin(eAnom)

        let cw = cos(omega), sw = sin(omega)
        let cn = cos(node), sn = sin(node)
        let ci = cos(inc), si = sin(inc)
        let x = (cw * cn - sw * sn * ci) * xp + (-sw * cn - cw * sn * ci) * yp
        let y = (cw * sn + sw * cn * ci) * xp + (-sw * sn + cw * cn * ci) * yp
        let z = (sw * si) * xp + (cw * si) * yp
        return Vector3(x, y, z)
    }

    public func position(of body: CelestialBody, at epoch: Epoch) -> Vector3 {
        guard body != .sun else { return .zero }
        let ecl = eclipticPosition(of: body, at: epoch)
        return Constants.astronomicalUnit * (Matrix3.rotationX(Constants.obliquityJ2000) * ecl)
    }

    /// Velocity by central differences (km/s); adequate for aberration and
    /// relative-motion terms, which need only ~1 m/s accuracy.
    public func velocity(of body: CelestialBody, at epoch: Epoch) -> Vector3 {
        guard body != .sun else { return .zero }
        let h = 30.0
        let p1 = position(of: body, at: epoch.adding(seconds: h))
        let p0 = position(of: body, at: epoch.adding(seconds: -h))
        return (p1 - p0) / (2 * h)
    }

    public func solarSystemBarycenterOffset(at epoch: Epoch) -> Vector3 {
        var weighted = Vector3.zero
        var total = Constants.gmSun
        for body in CelestialBody.planets {
            weighted += body.gm * position(of: body, at: epoch)
            total += body.gm
        }
        return weighted / total
    }
}
