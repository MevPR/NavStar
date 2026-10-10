import XCTest
@testable import NavStarCore

final class EphemerisTests: XCTestCase {
    let eph = OnboardEphemeris()

    func testKeplerSolver() {
        for e in [0.0, 0.1, 0.5, 0.9, 0.99] {
            for m in stride(from: -3.0, through: 3.0, by: 0.37) {
                let ea = Kepler.eccentricAnomaly(meanAnomaly: m, eccentricity: e)
                XCTAssertEqual(ea - e * sin(ea), m, accuracy: 1e-11)
            }
        }
    }

    func testEarthAtJ2000() {
        let p = eph.eclipticPosition(of: .earth, at: Epoch(julianDate: 2451545.0))
        let lon = atan2(p.y, p.x) * Constants.radiansToDegrees
        XCTAssertEqual(lon, 100.37, accuracy: 0.1)          // heliocentric ecliptic longitude
        XCTAssertEqual(p.norm, 0.9833, accuracy: 0.001)     // near perihelion
        XCTAssertEqual(abs(p.z), 0, accuracy: 1e-4)
    }

    func testPlanetDistancesWithinOrbitalBounds() {
        let t = Epoch(year: 2026, month: 11, day: 1)
        let au = Constants.astronomicalUnit
        XCTAssertEqual(eph.position(of: .earth, at: t).norm / au, 1.0, accuracy: 0.02)
        let mars = eph.position(of: .mars, at: t).norm / au
        XCTAssertGreaterThan(mars, 1.38); XCTAssertLessThan(mars, 1.67)
        let jup = eph.position(of: .jupiter, at: t).norm / au
        XCTAssertGreaterThan(jup, 4.95); XCTAssertLessThan(jup, 5.46)
        XCTAssertEqual(eph.position(of: .sun, at: t).norm, 0)
    }

    func testEarthSpeed() {
        let v = eph.velocity(of: .earth, at: Epoch(year: 2026, month: 11, day: 1))
        XCTAssertEqual(v.norm, 29.8, accuracy: 0.5)
    }

    func testBarycenterOffsetIsDominatedByJupiter() {
        let t = Epoch(year: 2026, month: 11, day: 1)
        let offset = eph.solarSystemBarycenterOffset(at: t)
        XCTAssertLessThan(offset.norm, 0.012 * Constants.astronomicalUnit)
        XCTAssertGreaterThan(offset.norm, 0.001 * Constants.astronomicalUnit)
    }
}
