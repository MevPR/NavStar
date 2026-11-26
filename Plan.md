# NavStar – Plan for an Autonomous Navigation System in the Solar System

## 1. Task and Constraints

A spacecraft travels interplanetary space within the solar system and must
determine its own position and velocity at all times. There is **no** external
support: no Deep Space Network, no GNSS, no ephemeris updates, no time signals
from Earth.

Only knowledge loaded onboard before launch may be used – just like a nautical
chart on a ship:

| Onboard knowledge                   | Role                                                      |
|-------------------------------------|-----------------------------------------------------------|
| Star catalog                        | Attitude determination by the star tracker                |
| Planetary ephemerides               | Planet positions as a function of time                    |
| Pulsar timing models                | Direction and period of millisecond pulsars               |
| Physical constants, GM values       | Dynamics model                                            |
| Onboard atomic clock                | Time base for ephemerides and pulsar timing               |

Goals:

1. Estimate position and velocity in the heliocentric inertial frame EME2000
   with a realistic uncertainty (covariance).
2. Cold start without any prior information (power-up, safe-mode recovery).
3. Robustness against outliers and unmodeled forces.
4. Use of all available onboard sensors in a single common estimator.
5. Traceable verification through unit tests and an end-to-end simulation.

## 2. Onboard Sensors

| Sensor                          | Measured quantity                              | Typical accuracy (1σ)      | Contribution to navigation                               |
|---------------------------------|------------------------------------------------|----------------------------|----------------------------------------------------------|
| Star tracker                    | Attitude quaternion body → inertial            | 1–5″                       | Reference for all direction measurements                 |
| IMU (gyros, accelerometers)     | Angular rate, non-gravitational acceleration   | 1e-6 rad/s, 1e-8 km/s²     | Attitude propagation, thrust measurement during maneuvers |
| Optical navigation camera (OpNav) | Direction to planets against the star background | 2–10″                  | Primary position source, triangulation, cold start       |
| Sun sensor                      | Direction to the Sun                           | 0.01°                      | Coarse, always available direction; cold start           |
| X-ray pulsar detector (XNAV)    | Arrival phase of pulses from millisecond pulsars | 1–10 µs (0.3–3 km)       | Absolute position like a "galactic GPS"                  |
| Solar spectrometer              | Doppler shift of solar absorption lines        | 1 m/s                      | Radial velocity relative to the Sun                      |
| Radar/laser altimeter           | Distance to the surface of a body              | 10 m                       | Approach and orbit phase                                 |
| Onboard atomic clock            | Time                                           | 1e-13 relative stability   | Time base for ephemeris and pulsar evaluation            |

All measurements are translated into measurement models that compute a
prediction from the estimated state. The difference (innovation) drives the
filter.

## 3. System Architecture

```
 Star tracker ──► Attitude q (body→inertial) ──────────────────────────┐
 IMU ──┬─► Gyro ──► Attitude propagation between star tracker fixes    │
       └─► Acceleration ──► Thrust (body→inertial) ──► Propagation (RK4 + covariance)
 Camera ────────► Direction (body) ──q──► BearingMeasurement ─┐
 Sun sensor ────► Direction (body) ──q──► BearingMeasurement  │
 Pulsar detector ─────────────────────► PulsarMeasurement     ├──► EKF update (gating, Joseph form) ──► State + covariance
 Spectrometer ────────────────────────► SolarDopplerMeasurement │
 Altimeter ───────────────────────────► RangeMeasurement      ┘
 Onboard ephemerides + pulsar catalog ► Measurement models and dynamics model
```

Layers of the Swift package `NavStarCore`:

| Layer          | Folder                              | Contents                                                                |
|----------------|-------------------------------------|-------------------------------------------------------------------------|
| Math           | `Sources/NavStarCore/Math`          | `Vector3`, `Matrix3`, `Matrix` (n×m, inverse), `Quaternion`             |
| Time           | `Sources/NavStarCore/Time`          | `Epoch` (seconds since J2000, Julian date, calendar)                    |
| Ephemerides    | `Sources/NavStarCore/Ephemeris`     | `CelestialBody`, `Kepler`, `OnboardEphemeris` (JPL approximate elements) |
| Dynamics       | `Sources/NavStarCore/Dynamics`      | `DynamicsModel` (Sun + planets, gravity gradient), RK4 integrator       |
| Measurements   | `Sources/NavStarCore/Measurements`  | `NavigationMeasurement` protocol, bearing, pulsar, Doppler, range       |
| Sensors        | `Sources/NavStarCore/Sensors`       | Sensor models for the simulation (in flight: hardware drivers)          |
| Estimation     | `Sources/NavStarCore/Estimation`    | `NavigationState`, `ExtendedKalmanFilter`, `InitialFix` (cold start)    |
| Navigation     | `Sources/NavStarCore/Navigation`    | `Navigator` – facade with attitude, filter and measurement adapters     |
| Simulation     | `Sources/NavStarSim`                | Executable `navstar-sim`: truth model, sensors, evaluation              |
| Tests          | `Tests/NavStarCoreTests`            | 24 unit tests                                                           |

Design decisions:

- **Reference frame** heliocentric EME2000. The stored Keplerian elements are
  given in the J2000 ecliptic and are rotated by the obliquity of the ecliptic.
  Pulsar directions (ICRF) can therefore be used directly.
- **Ephemerides as a protocol** (`Ephemeris`): the analytic onboard ephemeris
  can be replaced by high-precision Chebyshev coefficients (JPL DE440) without
  changing the rest of the system.
- **Measurements as a protocol** (`NavigationMeasurement`): every measurement
  provides a residual, Jacobian and noise covariance. New sensors are added as
  a new struct; the filter remains unchanged.
- **Separation of truth and estimate**: the sensor models in `Sensors` generate
  measurements from the true state; the filter never sees the truth.

## 4. Mathematical Foundations

### 4.1 State

x = [r, v, a_b] with r ∈ ℝ³ (km), v ∈ ℝ³ (km/s) and a_b ∈ ℝ³ (km/s²), the
unmodeled acceleration bias (radiation pressure mismodeling, outgassing,
thruster leaks). 9 states in total with a 9×9 covariance P.

### 4.2 Dynamics

r̈ = −μ☉ r/|r|³ + Σᵢ μᵢ ( dᵢ/|dᵢ|³ − pᵢ/|pᵢ|³ ) + a_thrust + a_b,  dᵢ = pᵢ − r

ȧ_b = −a_b/τ + w (first-order Gauss–Markov process, τ = 10 days)

- Direct and indirect terms of all eight planets (Earth as the Earth–Moon
  barycenter).
- Thrust a_thrust comes from the accelerometers, rotated with the attitude from
  the star tracker.
- Integration with RK4 (step size ≤ 120 s), covariance with
  Φ ≈ I + FΔt + (FΔt)²/2 and the analytic gravity gradient
  ∂a/∂r = Σ μ (3 d dᵀ/|d|⁵ − I/|d|³).

### 4.3 Measurement Equations

| Measurement       | Model h(x)                                                        | Notes                                                               |
|-------------------|-------------------------------------------------------------------|---------------------------------------------------------------------|
| Bearing (OpNav, Sun) | u = (p(t−τ) − r)/ρ, then aberration u' = norm(u + v/c)         | Light time τ solved iteratively; residual as two angles in the tangent plane; ∂u/∂r = −(I − uuᵀ)/ρ |
| Pulsar (XNAV)     | Δt = n·(r − r_SSB)/c  mod P                                       | Barycentric correction from ephemerides; cycle ambiguity is only resolved when σ along n < cP/6 |
| Solar Doppler     | v_r = v · r̂                                                       | ∂/∂r = vᵀ(I − r̂r̂ᵀ)/|r|, ∂/∂v = r̂ᵀ                                   |
| Altimeter         | ρ = |p − r| − R_body                                              | ∂/∂r = −uᵀ                                                          |

### 4.4 Filter

Extended Kalman filter (EKF):

- Prediction: x ← f(x), P ← Φ P Φᵀ + Q with white acceleration noise for
  (r, v) and driving noise 2σ_b²/τ for the bias.
- Update: S = H P Hᵀ + R, K = P Hᵀ S⁻¹, x ← x + K y, P in Joseph form
  (I − KH) P (I − KH)ᵀ + K R Kᵀ for numerical positivity.
- Gating: measurements with a normalized innovation > 5σ are rejected and
  logged.
- Measurements from the future automatically trigger a propagation; stale
  measurements are rejected.

### 4.5 Cold Start

1. Weighted least-squares triangulation from ≥ 2 directions: each direction
   defines a line through the (light-time corrected) body position; the sought
   point minimizes the weighted sum of squared distances. The weights
   1/(σ·ρ)² are determined iteratively from the estimated range; the inverse of
   the normal matrix directly yields the position covariance.
2. Two fixes several hours apart; the velocity is determined by shooting
   (correcting the initial velocity so that the propagated trajectory hits the
   second fix), which accounts for the curvature of the trajectory.
3. With the velocity found, the aberration is inverted exactly and the
   procedure is repeated.

## 5. Operating Modes

| Mode         | Trigger                                | Behavior                                                                   |
|--------------|----------------------------------------|----------------------------------------------------------------------------|
| Cold start   | Power-up, loss of filter state         | Star tracker → attitude; camera + Sun sensor → triangulation; reset filter |
| Cruise       | Normal operation                       | Propagation with IMU attitude; OpNav every 6 h, Sun sensor hourly, XNAV every 12 h, Doppler every 6 h |
| Maneuver     | Acceleration above threshold           | Accelerometers feed thrust into the dynamics                               |
| Approach     | Target body < 50,000 km                | Altimeter active, OpNav densified on the target body                       |
| Safe mode    | Filter inconsistency (NEES test)       | Inflate covariance or cold start                                           |

## 6. Implementation Plan

| Phase | Contents                                                                | Status |
|-------|-------------------------------------------------------------------------|--------|
| 0     | Repository, Swift package, directory structure                          | ✅ |
| 1     | Vectors, matrices, quaternions, time computation                        | ✅ |
| 2     | Onboard ephemeris (JPL approximate elements 1800–2050), Kepler solver, barycenter | ✅ |
| 3     | Dynamics model (Sun + 8 planets), RK4, gravity gradient                 | ✅ |
| 4     | Measurement models: bearing, pulsar, Doppler, altimeter                 | ✅ |
| 5     | EKF with 9 states, gating, Joseph form; cold start                      | ✅ |
| 6     | Sensor models and end-to-end simulation Earth → interplanetary (30 days) | ✅ |
| 7     | Unit tests (math, ephemerides, Jacobians, filter consistency)           | ✅ |
| 8     | Extensions (see section 9)                                              | ⬜ |

## 7. Verification

### 7.1 Unit Tests (`swift test`, 24 tests)

- Matrix inversion, quaternion rotation and gyro integration, tangent basis, phase wrapping.
- Calendar conversion (J2000, leap year), Kepler solver up to e = 0.99.
- Earth at J2000: heliocentric longitude 100.4°, distance 0.9833 AU; planet distances within orbital limits; barycenter offset.
- Circular orbit closes after one period to < 1 km; gravity gradient against finite differences.
- Jacobians of all measurement models against finite differences; pulsar wrapping and ambiguity rejection.
- Noise-free triangulation < 0.5 km; cold-start velocity < 0.1 m/s; filter consistency (error < 4σ); rejection of stale measurements.

### 7.2 End-to-End Simulation (`swift run -c release navstar-sim`)

Scenario: start on 2026-11-01 near Earth with 3 km/s excess velocity, 30 days of
flight, unmodeled acceleration 9e-11 km/s², accelerometer bias, mid-course burn
on day 15. The truth is generated with the same dynamics model plus
perturbations; all measurements are noisy and go through the star tracker
attitude.

```text
NavStar autonomous navigation simulation
Start 2026-11-01 00:00:00, heliocentric distance 1.0060 AU, speed 33.004 km/s
Sensors: star tracker 3", OpNav camera 5" [Earth, Mars, Jupiter], Sun sensor 0.01°, XNAV 5 µs, solar Doppler 1 m/s, IMU with bias

Cold start (bearings only, no prior):
  2026-11-01 06:00:00 cold start: position σ 27931 km, velocity σ 1829.3 m/s
  position error 68601.9 km, velocity error 2859.42 m/s

Epoch (TDB)          Event                 pos error      3σ pos         vel error      3σ vel         acc / rej
2026-11-01 06:00:00  after cold start     68601.9 km      83793.0 km    2859.42 m/s    5487.84 m/s       0 / 0   
2026-11-02 00:00:00  day 1      2065.9 km      24603.3 km     119.29 m/s     324.64 m/s      27 / 8   
2026-11-03 00:00:00  day 2      4630.0 km      17256.4 km      37.98 m/s     112.74 m/s      63 / 16  
2026-11-04 00:00:00  day 3      4167.5 km      14109.3 km      14.76 m/s      62.05 m/s      99 / 24  
2026-11-05 00:00:00  day 4      3230.3 km      12217.3 km       9.56 m/s      40.57 m/s     135 / 32  
2026-11-06 00:00:00  day 5      3014.7 km      10915.4 km      12.00 m/s      29.14 m/s     171 / 40  
2026-11-07 00:00:00  day 6       604.6 km       3552.9 km       7.10 m/s      11.94 m/s     209 / 48  
2026-11-08 00:00:00  day 7       146.6 km       3234.6 km       0.79 m/s       4.77 m/s     249 / 56  
2026-11-09 00:00:00  day 8       470.6 km       3029.9 km       0.59 m/s       3.37 m/s     289 / 64  
2026-11-10 00:00:00  day 9       114.5 km       2868.2 km       0.44 m/s       2.78 m/s     329 / 72  
  2026-11-10 12:00:00  first accepted XNAV fix from PSR J0437-4715
  2026-11-10 12:00:00  first accepted XNAV fix from PSR B1957+20
  2026-11-11 00:00:00  first accepted XNAV fix from PSR B1937+21
  2026-11-11 00:00:00  first accepted XNAV fix from PSR B1821-24
2026-11-11 00:00:00  day 10         1.7 km         10.2 km       0.24 m/s       0.44 m/s     375 / 74  
2026-11-12 00:00:00  day 11         3.5 km          9.0 km       0.06 m/s       0.13 m/s     423 / 74  
2026-11-13 00:00:00  day 12         3.1 km          8.0 km       0.02 m/s       0.08 m/s     471 / 74  
2026-11-14 00:00:00  day 13         1.5 km          7.5 km       0.01 m/s       0.07 m/s     519 / 74  
2026-11-15 00:00:00  day 14         1.9 km          7.3 km       0.02 m/s       0.07 m/s     567 / 74  
2026-11-16 00:00:00  day 15         0.2 km          7.2 km       0.01 m/s       0.07 m/s     615 / 74  
2026-11-16 01:00:00  after MCC burn         0.3 km          7.4 km       0.01 m/s       0.07 m/s     616 / 74  
2026-11-17 00:00:00  day 16         0.8 km          7.2 km       0.01 m/s       0.06 m/s     663 / 74  
2026-11-18 00:00:00  day 17         2.1 km          7.1 km       0.02 m/s       0.06 m/s     711 / 74  
2026-11-19 00:00:00  day 18         1.0 km          7.0 km       0.00 m/s       0.06 m/s     759 / 74  
2026-11-20 00:00:00  day 19         1.8 km          7.0 km       0.01 m/s       0.06 m/s     807 / 74  
2026-11-21 00:00:00  day 20         1.9 km          6.9 km       0.01 m/s       0.06 m/s     855 / 74  
2026-11-22 00:00:00  day 21         2.2 km          6.9 km       0.01 m/s       0.06 m/s     903 / 74  
2026-11-23 00:00:00  day 22         1.1 km          6.9 km       0.01 m/s       0.06 m/s     951 / 74  
2026-11-24 00:00:00  day 23         1.0 km          6.9 km       0.00 m/s       0.06 m/s     999 / 74  
2026-11-25 00:00:00  day 24         0.8 km          6.9 km       0.00 m/s       0.06 m/s    1047 / 74  
2026-11-26 00:00:00  day 25         1.3 km          6.9 km       0.01 m/s       0.06 m/s    1095 / 74  
2026-11-27 00:00:00  day 26         1.4 km          6.9 km       0.01 m/s       0.06 m/s    1143 / 74  
2026-11-28 00:00:00  day 27         1.7 km          6.9 km       0.01 m/s       0.06 m/s    1191 / 74  
2026-11-29 00:00:00  day 28         1.6 km          6.9 km       0.01 m/s       0.06 m/s    1239 / 74  
2026-11-30 00:00:00  day 29         1.1 km          6.9 km       0.01 m/s       0.06 m/s    1287 / 74  
2026-12-01 00:00:00  day 30         2.1 km          6.9 km       0.01 m/s       0.06 m/s    1335 / 74  

Measurements accepted: 1335, rejected: 74
  19× Pulsar(PSR B1937+21): cycle ambiguity unresolved
  19× Pulsar(PSR B1821-24): cycle ambiguity unresolved
  18× Pulsar(PSR B1957+20): cycle ambiguity unresolved
  18× Pulsar(PSR J0437-4715): cycle ambiguity unresolved

Final position error 2.08 km (3σ 6.92 km), velocity error 0.009 m/s (3σ 0.057 m/s)
Estimated acceleration bias (2.46943e-11, -3.23387e-11, 3.05683e-11) km/s² (truth (1.92369e-11, -1.79788e-11, 5.77005e-11), 3σ 3.4e-10 km/s²)
```

Interpretation:

- The cold start without any prior knowledge yields a position to about
  70,000 km (3σ consistent); the uncertainty lies mainly along the line of
  sight to Earth.
- Doppler and bearing measurements reduce the error to a few 100 km within a
  week.
- As soon as the uncertainty along a pulsar direction drops below one sixth of
  the ambiguity distance (from day 10), the XNAV measurements are accepted and
  the position error drops to a few km.
- The acceleration bias is estimated as well; this keeps the covariance
  consistent and no pulsar update is wrongly rejected.

## 8. Error Budget and Risks

| Risk                                     | Impact                                         | Mitigation                                                   |
|------------------------------------------|------------------------------------------------|--------------------------------------------------------------|
| Accuracy of the analytic ephemeris       | Systematic errors of up to several 10,000 km in planet positions (JPL approximation: 20–40″) | Load DE440 Chebyshev coefficients before flight; the `Ephemeris` protocol is prepared for this. In the simulation this error is not visible because truth and filter use the same ephemeris. |
| Earth–Moon separation                    | EMB instead of Earth: up to 4,700 km offset    | Add a lunar ephemeris (phase 8)                              |
| Pulsar ambiguity                         | Wrong cycle = error of hundreds of km          | Accept only when σ < cP/6; 5σ gating                         |
| Camera Sun exclusion                     | No OpNav near the Sun                          | Sun sensor, pulsars and Doppler bridge the gap               |
| Clock drift                              | Pulsar timing and ephemeris time drift         | Clock bias and drift as states (phase 8)                     |
| Attitude error                           | Every attitude error becomes a direction error | Star tracker σ included in the measurement noise             |
| Unmodeled forces                         | Filter divergence                              | Gauss–Markov bias state; process noise                       |
| Nonlinearity with a poor prior           | EKF linearization invalid                      | Cold start provides a prior with correct covariance; alternatively UKF |

## 9. Extensions (Phase 8)

1. **High-precision ephemeris**: JPL DE440 as Chebyshev coefficients onboard, including the Moon and large asteroids.
2. **Clock states**: bias and drift of the onboard atomic clock in the filter, estimated via pulsar timing.
3. **Attitude filter**: multiplicative EKF (MEKF) for attitude and gyro bias instead of directly adopting the star tracker solution.
4. **Sensor bias**: accelerometer bias and scale factor as states during maneuvers.
5. **Relative navigation**: landmark and horizon tracking during approach, moons as OpNav targets.
6. **Square-root or UD filter** for numerical robustness on flight computers.
7. **Monte Carlo campaign** and NEES/NIS consistency statistics over many seeds.
8. **Flight software integration**: real-time scheduling, time-tagging of raw data, telemetry of filter states.

## 10. Usage

```bash
swift build                          # debug build
swift test                           # 24 unit tests
swift run -c release navstar-sim     # 30-day simulation
open Package.swift                   # open in Xcode
```

Minimal integration of the navigator:

```swift
import NavStarCore

let ephemeris = OnboardEphemeris()
let navigator = Navigator(initialState: priorState, ephemeris: ephemeris)

navigator.updateAttitude(starTracker: attitudeFromStarTracker)
navigator.advance(to: now, bodyAcceleration: accelerometerReading)
navigator.ingest(navigator.makeBearing(to: .mars, bodyFrameDirection: cameraVector,
                                       sigma: 5 * Constants.arcsecondsToRadians, epoch: now))
navigator.ingest(PulsarMeasurement(epoch: now, pulsar: Pulsar.catalog[0], observedDelay: phase, sigma: 5e-6))
print(navigator.state.position, navigator.state.positionSigmaRSS)
```
