# NavStar

Autonomous navigation system for a spacecraft in the solar system that relies
exclusively on onboard sensors: star tracker, IMU, optical navigation camera,
Sun sensor, X-ray pulsar detector (XNAV), solar spectrometer (Doppler) and
altimeter. No radio tracking, no GNSS, no data from Earth – only onboard
knowledge loaded before launch (star catalog, planetary ephemerides, pulsar
catalog).

The complete design, the mathematics, the error budget and the implementation
plan are described in [Plan.md](Plan.md).

## Structure

- `Sources/NavStarCore` – library: math, time, ephemerides, dynamics,
  measurement models, sensor models, Kalman filter with cold start, `Navigator`
  facade.
- `Sources/NavStarSim` – executable `navstar-sim`: 30-day simulation with a
  truth model, noisy sensors and error evaluation.
- `Tests/NavStarCoreTests` – 24 unit tests.

## Usage

```bash
swift build
swift test
swift run -c release navstar-sim
```

The package can be opened directly in Xcode (`Package.swift`).

## Method at a glance

1. The star tracker provides the attitude; the IMU propagates it and measures
   thrust.
2. The camera and Sun sensor provide directions to the Sun and the planets; a
   cold-start fix is computed from several directions by weighted triangulation.
3. An extended Kalman filter with 9 states (position, velocity, acceleration
   bias) propagates with an N-body model and processes bearing, pulsar, Doppler
   and range measurements with light-time and aberration correction.
4. Pulsar measurements are only used once the position uncertainty allows the
   cycle ambiguity to be resolved; after that the solution reaches km-level
   accuracy.
