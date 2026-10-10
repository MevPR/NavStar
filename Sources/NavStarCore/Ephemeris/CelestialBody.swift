import Foundation

/// Bodies the onboard ephemeris can place. `earth` denotes the Earth–Moon
/// barycentre, which is what the stored orbital elements describe.
public enum CelestialBody: String, CaseIterable, Hashable {
    case sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune

    /// Gravitational parameter GM in km³/s² (Earth entry includes the Moon).
    public var gm: Double {
        switch self {
        case .sun: return Constants.gmSun
        case .mercury: return 2.2032e4
        case .venus: return 3.24859e5
        case .earth: return 4.035032e5
        case .mars: return 4.282837e4
        case .jupiter: return 1.26686534e8
        case .saturn: return 3.7931187e7
        case .uranus: return 5.793939e6
        case .neptune: return 6.836529e6
        }
    }

    /// Mean radius in km, used for altimeter/range measurements.
    public var meanRadius: Double {
        switch self {
        case .sun: return 695_700
        case .mercury: return 2_439.7
        case .venus: return 6_051.8
        case .earth: return 6_371.0
        case .mars: return 3_389.5
        case .jupiter: return 69_911
        case .saturn: return 58_232
        case .uranus: return 25_362
        case .neptune: return 24_622
        }
    }

    public var displayName: String { rawValue.prefix(1).uppercased() + rawValue.dropFirst() }

    public static let planets: [CelestialBody] = [.mercury, .venus, .earth, .mars, .jupiter, .saturn, .uranus, .neptune]
}
