import XCTest
@testable import NavStarCore

final class EpochTests: XCTestCase {
    func testJ2000() {
        let e = Epoch(year: 2000, month: 1, day: 1, hour: 12)
        XCTAssertEqual(e.julianDate, 2451545.0, accuracy: 1e-9)
        XCTAssertEqual(e.secondsSinceJ2000, 0, accuracy: 1e-6)
        XCTAssertEqual(e.description, "2000-01-01 12:00:00")
    }

    func testCalendarConversion() {
        let e = Epoch(year: 2026, month: 11, day: 1)
        XCTAssertEqual(e.julianDate, 2461345.5, accuracy: 1e-9)
        XCTAssertEqual(e.description, "2026-11-01 00:00:00")
        XCTAssertEqual(e.adding(seconds: 3661).description, "2026-11-01 01:01:01")
        XCTAssertEqual(Epoch(year: 2024, month: 2, day: 29, hour: 23, minute: 59, second: 59).description, "2024-02-29 23:59:59")
    }
}
