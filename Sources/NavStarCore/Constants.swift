import Foundation

/// Physical constants used throughout the navigation system (km, s, rad).
public enum Constants {
    /// Astronomical unit in km (IAU 2012).
    public static let astronomicalUnit = 149_597_870.7
    /// Speed of light in km/s.
    public static let speedOfLight = 299_792.458
    /// Mean obliquity of the ecliptic at J2000 in radians (23.43928°).
    public static let obliquityJ2000 = 23.43928 * Double.pi / 180
    /// Solar gravitational parameter in km³/s².
    public static let gmSun = 1.327_124_400_18e11
    public static let degreesToRadians = Double.pi / 180
    public static let radiansToDegrees = 180 / Double.pi
    public static let arcsecondsToRadians = Double.pi / (180 * 3600)
}
