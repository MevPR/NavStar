import Foundation

/// Instant on the onboard time scale, stored as seconds since J2000.0
/// (2000-01-01 12:00:00). The scale is treated as TDB for ephemeris
/// evaluation; the sub-second TDB/TT difference is far below the accuracy of
/// the onboard ephemeris and therefore ignored.
public struct Epoch: Comparable, Hashable, CustomStringConvertible {
    public static let j2000JulianDate = 2451545.0
    public static let secondsPerDay = 86400.0

    public let secondsSinceJ2000: Double

    public init(secondsSinceJ2000: Double) {
        self.secondsSinceJ2000 = secondsSinceJ2000
    }

    public init(julianDate: Double) {
        self.init(secondsSinceJ2000: (julianDate - Epoch.j2000JulianDate) * Epoch.secondsPerDay)
    }

    /// Proleptic Gregorian calendar date (Fliegel & Van Flandern algorithm).
    public init(year: Int, month: Int, day: Int, hour: Int = 0, minute: Int = 0, second: Double = 0) {
        let a = (14 - month) / 12
        let y = year + 4800 - a
        let m = month + 12 * a - 3
        let jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045
        let jd = Double(jdn) + (Double(hour) - 12) / 24 + Double(minute) / 1440 + second / 86400
        self.init(julianDate: jd)
    }

    public var julianDate: Double { Epoch.j2000JulianDate + secondsSinceJ2000 / Epoch.secondsPerDay }
    public var julianCenturies: Double { secondsSinceJ2000 / (Epoch.secondsPerDay * 36525) }
    public var daysSinceJ2000: Double { secondsSinceJ2000 / Epoch.secondsPerDay }

    public func adding(seconds: Double) -> Epoch { Epoch(secondsSinceJ2000: secondsSinceJ2000 + seconds) }
    public func adding(days: Double) -> Epoch { adding(seconds: days * Epoch.secondsPerDay) }

    /// Difference in seconds.
    public static func - (a: Epoch, b: Epoch) -> Double { a.secondsSinceJ2000 - b.secondsSinceJ2000 }
    public static func < (a: Epoch, b: Epoch) -> Bool { a.secondsSinceJ2000 < b.secondsSinceJ2000 }

    /// Calendar representation `YYYY-MM-DD HH:MM:SS`.
    public var description: String {
        let jd = julianDate + 0.5
        let z = floor(jd)
        let f = jd - z
        var a = z
        if z >= 2299161 {
            let alpha = floor((z - 1867216.25) / 36524.25)
            a = z + 1 + alpha - floor(alpha / 4)
        }
        let b = a + 1524
        let c = floor((b - 122.1) / 365.25)
        let d = floor(365.25 * c)
        let e = floor((b - d) / 30.6001)
        let dayFraction = b - d - floor(30.6001 * e) + f
        let day = Int(floor(dayFraction))
        let month = e < 14 ? Int(e) - 1 : Int(e) - 13
        let year = month > 2 ? Int(c) - 4716 : Int(c) - 4715
        var secs = (dayFraction - Double(day)) * 86400
        secs = (secs * 1000).rounded() / 1000
        let hh = Int(secs / 3600)
        let mm = Int((secs - Double(hh) * 3600) / 60)
        let ss = Int(secs - Double(hh) * 3600 - Double(mm) * 60)
        return String(format: "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hh, mm, ss)
    }
}
