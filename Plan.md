# NavStar – Plan für ein autonomes Navigationssystem im Sonnensystem

## 1. Aufgabe und Randbedingungen

Ein Raumgleiter bewegt sich interplanetar im Sonnensystem und muss seine Position
und Geschwindigkeit jederzeit selbst bestimmen. Es gibt **keine** Unterstützung von
außen: kein Deep Space Network, kein GNSS, keine Ephemeriden-Updates, keine
Zeitzeichen von der Erde.

Erlaubt ist ausschließlich Wissen, das vor dem Start an Bord geladen wurde – so wie
eine Seekarte auf einem Schiff:

| Bordwissen                          | Rolle                                                     |
|-------------------------------------|-----------------------------------------------------------|
| Sternkatalog                        | Lagebestimmung durch den Sternsensor                      |
| Planetenephemeriden                 | Position der Planeten als Funktion der Zeit               |
| Pulsar-Timing-Modelle               | Richtung und Periode von Millisekundenpulsaren            |
| Physikalische Konstanten, GM-Werte  | Dynamikmodell                                             |
| Bord-Atomuhr                        | Zeitbasis für Ephemeriden und Pulsar-Timing               |

Ziele:

1. Schätzung von Position und Geschwindigkeit im heliozentrischen Inertialsystem
   EME2000 mit realistischer Unsicherheit (Kovarianz).
2. Kaltstart ohne jede Vorinformation (Power-up, Safe-Mode-Recovery).
3. Robustheit gegen Ausreißer und unmodellierte Kräfte.
4. Verwertung aller verfügbaren Bordsensoren in einem gemeinsamen Schätzer.
5. Nachvollziehbare Verifikation durch Unit-Tests und eine End-to-End-Simulation.

## 2. Sensorik an Bord

| Sensor                          | Messgröße                                      | Typische Genauigkeit (1σ)  | Beitrag zur Navigation                                   |
|---------------------------------|------------------------------------------------|----------------------------|----------------------------------------------------------|
| Sternsensor (Star Tracker)      | Lage-Quaternion Body → Inertial                | 1–5″                       | Referenz für alle Richtungsmessungen                     |
| IMU (Gyros, Beschleunigungsmesser) | Drehrate, nicht-gravitative Beschleunigung  | 1e-6 rad/s, 1e-8 km/s²     | Lagefortschreibung, Schubmessung bei Manövern            |
| Optische Navigationskamera (OpNav) | Richtung zu Planeten vor Sternhintergrund   | 2–10″                      | Haupt-Positionsquelle, Triangulation, Kaltstart          |
| Sonnensensor                    | Richtung zur Sonne                             | 0,01°                      | Grobe, immer verfügbare Richtung; Kaltstart              |
| Röntgen-Pulsardetektor (XNAV)   | Ankunftsphase der Pulse von Millisekundenpulsaren | 1–10 µs (0,3–3 km)      | Absolute Position wie ein „galaktisches GPS“              |
| Solarspektrometer               | Dopplerverschiebung solarer Absorptionslinien  | 1 m/s                      | Radialgeschwindigkeit zur Sonne                          |
| Radar-/Laser-Altimeter          | Entfernung zur Oberfläche eines Körpers        | 10 m                       | Anflug- und Orbitphase                                   |
| Bord-Atomuhr                    | Zeit                                           | 1e-13 relative Stabilität  | Zeitbasis für Ephemeriden- und Pulsarauswertung          |

Alle Messungen werden in Messmodelle übersetzt, die aus dem geschätzten Zustand
eine Vorhersage berechnen. Die Differenz (Innovation) treibt den Filter.

## 3. Systemarchitektur

```
 Sternsensor ──► Lage q (Body→Inertial) ───────────────────────────────┐
 IMU ──┬─► Gyro ──► Lagefortschreibung zwischen Sternsensor-Fixes       │
       └─► Beschleunigung ──► Schub (Body→Inertial) ──► Propagation (RK4 + Kovarianz)
 Kamera ────────► Richtung (Body) ──q──► BearingMeasurement ─┐
 Sonnensensor ──► Richtung (Body) ──q──► BearingMeasurement  │
 Pulsardetektor ──────────────────────► PulsarMeasurement    ├──► EKF-Update (Gating, Joseph-Form) ──► Zustand + Kovarianz
 Spektrometer ────────────────────────► SolarDopplerMeasurement │
 Altimeter ───────────────────────────► RangeMeasurement     ┘
 Bord-Ephemeriden + Pulsarkatalog ────► Messmodelle und Dynamikmodell
```

Schichten des Swift-Pakets `NavStarCore`:

| Schicht        | Ordner                              | Inhalt                                                                  |
|----------------|-------------------------------------|-------------------------------------------------------------------------|
| Mathematik     | `Sources/NavStarCore/Math`          | `Vector3`, `Matrix3`, `Matrix` (n×m, Inverse), `Quaternion`             |
| Zeit           | `Sources/NavStarCore/Time`          | `Epoch` (Sekunden seit J2000, Julianisches Datum, Kalender)             |
| Ephemeriden    | `Sources/NavStarCore/Ephemeris`     | `CelestialBody`, `Kepler`, `OnboardEphemeris` (JPL-Näherungselemente)   |
| Dynamik        | `Sources/NavStarCore/Dynamics`      | `DynamicsModel` (Sonne + Planeten, Gravitationsgradient), RK4-Integrator |
| Messmodelle    | `Sources/NavStarCore/Measurements`  | `NavigationMeasurement`-Protokoll, Bearing, Pulsar, Doppler, Range      |
| Sensoren       | `Sources/NavStarCore/Sensors`       | Sensormodelle für die Simulation (im Flug: Hardwaretreiber)             |
| Schätzung      | `Sources/NavStarCore/Estimation`    | `NavigationState`, `ExtendedKalmanFilter`, `InitialFix` (Kaltstart)     |
| Navigation     | `Sources/NavStarCore/Navigation`    | `Navigator` – Fassade mit Lage, Filter und Messungs-Adaptern            |
| Simulation     | `Sources/NavStarSim`                | Executable `navstar-sim`: Wahrheitsmodell, Sensoren, Auswertung         |
| Tests          | `Tests/NavStarCoreTests`            | 24 Unit-Tests                                                           |

Design-Entscheidungen:

- **Referenzsystem** EME2000 heliozentrisch. Die gespeicherten Keplerelemente
  liegen in der Ekliptik J2000 vor und werden mit der Schiefe der Ekliptik
  gedreht. Pulsarrichtungen (ICRF) sind damit direkt verwendbar.
- **Ephemeriden als Protokoll** (`Ephemeris`): Die analytische Bordephemeride ist
  austauschbar gegen hochgenaue Chebyshev-Koeffizienten (JPL DE440), ohne den
  Rest des Systems zu ändern.
- **Messungen als Protokoll** (`NavigationMeasurement`): Jede Messung liefert
  Residuum, Jacobi-Matrix und Rauschkovarianz. Neue Sensoren werden durch eine
  neue Struktur hinzugefügt, der Filter bleibt unverändert.
- **Trennung Wahrheit/Schätzung**: Die Sensormodelle in `Sensors` erzeugen aus
  dem wahren Zustand Messungen; der Filter sieht die Wahrheit nie.

## 4. Mathematische Grundlagen

### 4.1 Zustand

x = [r, v, a_b] mit r ∈ ℝ³ (km), v ∈ ℝ³ (km/s) und a_b ∈ ℝ³ (km/s²), dem
unmodellierten Beschleunigungsbias (Strahlungsdruck-Fehlmodellierung, Ausgasung,
Triebwerkslecks). Insgesamt 9 Zustände mit 9×9-Kovarianz P.

### 4.2 Dynamik

r̈ = −μ☉ r/|r|³ + Σᵢ μᵢ ( dᵢ/|dᵢ|³ − pᵢ/|pᵢ|³ ) + a_schub + a_b,  dᵢ = pᵢ − r

ȧ_b = −a_b/τ + w (Gauss-Markov-Prozess erster Ordnung, τ = 10 Tage)

- Direkte und indirekte Terme aller acht Planeten (Erde als Erde-Mond-Baryzentrum).
- Schub a_schub kommt aus den Beschleunigungsmessern, gedreht mit der Lage aus
  dem Sternsensor.
- Integration mit RK4 (Schrittweite ≤ 120 s), Kovarianz mit
  Φ ≈ I + FΔt + (FΔt)²/2 und analytischem Gravitationsgradienten
  ∂a/∂r = Σ μ (3 d dᵀ/|d|⁵ − I/|d|³).

### 4.3 Messgleichungen

| Messung           | Modell h(x)                                                       | Besonderheiten                                                      |
|-------------------|-------------------------------------------------------------------|---------------------------------------------------------------------|
| Richtung (OpNav, Sonne) | u = (p(t−τ) − r)/ρ, dann Aberration u' = norm(u + v/c)      | Lichtlaufzeit τ iterativ; Residuum als zwei Winkel in der Tangentialebene; ∂u/∂r = −(I − uuᵀ)/ρ |
| Pulsar (XNAV)     | Δt = n·(r − r_SSB)/c  mod P                                       | Baryzentrumskorrektur aus Ephemeriden; Zyklus-Mehrdeutigkeit wird nur aufgelöst, wenn σ entlang n < cP/6 |
| Solar-Doppler     | v_r = v · r̂                                                       | ∂/∂r = vᵀ(I − r̂r̂ᵀ)/|r|, ∂/∂v = r̂ᵀ                                   |
| Altimeter         | ρ = |p − r| − R_Körper                                             | ∂/∂r = −uᵀ                                                          |

### 4.4 Filter

Erweiterter Kalman-Filter (EKF):

- Prädiktion: x ← f(x), P ← Φ P Φᵀ + Q mit weißem Beschleunigungsrauschen
  für (r, v) und Antriebsrauschen 2σ_b²/τ für den Bias.
- Update: S = H P Hᵀ + R, K = P Hᵀ S⁻¹, x ← x + K y, P in Joseph-Form
  (I − KH) P (I − KH)ᵀ + K R Kᵀ für numerische Positivität.
- Gating: Messungen mit normierter Innovation > 5σ werden verworfen und protokolliert.
- Messungen aus der Zukunft lösen automatisch eine Propagation aus, veraltete
  Messungen werden abgelehnt.

### 4.5 Kaltstart

1. Gewichtete Least-Squares-Triangulation aus ≥ 2 Richtungen: Jede Richtung
   definiert eine Gerade durch die (lichtlaufzeitkorrigierte) Körperposition,
   gesucht ist der Punkt minimaler gewichteter Abstandsquadratsumme. Die Gewichte
   1/(σ·ρ)² werden iterativ aus der geschätzten Entfernung bestimmt; die Inverse
   der Normalmatrix liefert direkt die Positionskovarianz.
2. Zwei Fixes im Abstand von Stunden; die Geschwindigkeit wird durch Shooting
   bestimmt (Anfangsgeschwindigkeit so korrigieren, dass die propagierte Bahn den
   zweiten Fix trifft), wodurch die Bahnkrümmung berücksichtigt ist.
3. Mit der gefundenen Geschwindigkeit wird die Aberration exakt invertiert und
   die Prozedur wiederholt.

## 5. Betriebsmodi

| Modus        | Auslöser                               | Verhalten                                                                  |
|--------------|----------------------------------------|----------------------------------------------------------------------------|
| Kaltstart    | Power-up, Verlust des Filterzustands   | Sternsensor → Lage; Kamera + Sonnensensor → Triangulation; Filter neu setzen |
| Cruise       | Normalbetrieb                          | Propagation mit IMU-Lage; OpNav alle 6 h, Sonnensensor stündlich, XNAV alle 12 h, Doppler alle 6 h |
| Manöver      | Beschleunigung über Schwellwert        | Beschleunigungsmesser liefern Schub in die Dynamik                         |
| Anflug       | Zielkörper < 50 000 km                 | Altimeter aktiv, OpNav auf Zielkörper verdichtet                           |
| Safe Mode    | Filterinkonsistenz (NEES-Test)         | Kovarianz aufweiten oder Kaltstart                                         |

## 6. Implementierungsplan

| Phase | Inhalt                                                                  | Status |
|-------|-------------------------------------------------------------------------|--------|
| 0     | Repository, Swift-Package, Verzeichnisstruktur                          | ✅ |
| 1     | Vektoren, Matrizen, Quaternionen, Zeitrechnung                          | ✅ |
| 2     | Bordephemeride (JPL-Näherungselemente 1800–2050), Kepler-Löser, Baryzentrum | ✅ |
| 3     | Dynamikmodell (Sonne + 8 Planeten), RK4, Gravitationsgradient           | ✅ |
| 4     | Messmodelle: Richtung, Pulsar, Doppler, Altimeter                       | ✅ |
| 5     | EKF mit 9 Zuständen, Gating, Joseph-Form; Kaltstart                     | ✅ |
| 6     | Sensormodelle und End-to-End-Simulation Erde → interplanetar (30 Tage)  | ✅ |
| 7     | Unit-Tests (Mathematik, Ephemeriden, Jacobi-Matrizen, Filterkonsistenz) | ✅ |
| 8     | Erweiterungen (siehe Abschnitt 9)                                       | ⬜ |

## 7. Verifikation

### 7.1 Unit-Tests (`swift test`, 24 Tests)

- Matrixinversion, Quaternion-Rotation und Gyro-Integration, Tangentialbasis, Phasenwrapping.
- Kalenderumrechnung (J2000, Schaltjahr), Kepler-Löser bis e = 0,99.
- Erde bei J2000: heliozentrische Länge 100,4°, Abstand 0,9833 AU; Planetenabstände in Bahngrenzen; Baryzentrumsversatz.
- Kreisbahn schließt nach einer Periode auf < 1 km; Gravitationsgradient gegen finite Differenzen.
- Jacobi-Matrizen aller Messmodelle gegen finite Differenzen; Pulsar-Wrapping und Mehrdeutigkeitsablehnung.
- Triangulation ohne Rauschen < 0,5 km; Kaltstart-Geschwindigkeit < 0,1 m/s; Filterkonsistenz (Fehler < 4σ); Ablehnung veralteter Messungen.

### 7.2 End-to-End-Simulation (`swift run -c release navstar-sim`)

Szenario: Start am 1.11.2026 nahe der Erde mit 3 km/s Überschuss, 30 Tage Flug,
unmodellierte Beschleunigung 9e-11 km/s², Beschleunigungsmesser-Bias,
Mid-Course-Burn an Tag 15. Die Wahrheit wird mit demselben Dynamikmodell plus
Störungen erzeugt; alle Messungen sind verrauscht und laufen über Sternsensor-Lage.

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

- Der Kaltstart ohne jedes Vorwissen liefert eine Position auf ca. 70 000 km
  (3σ-konsistent); die Unsicherheit liegt vor allem entlang der Sichtlinie zur Erde.
- Doppler und Richtungsmessungen bringen den Fehler innerhalb einer Woche auf
  wenige 100 km.
- Sobald die Unsicherheit entlang einer Pulsarrichtung unter ein Sechstel der
  Mehrdeutigkeitsstrecke fällt (ab Tag 10), werden die XNAV-Messungen akzeptiert
  und die Position springt auf wenige km.
- Der Beschleunigungsbias wird mitgeschätzt; dadurch bleibt die Kovarianz
  konsistent und kein Pulsar-Update wird fälschlich verworfen.

## 8. Fehlerbudget und Risiken

| Risiko                                   | Auswirkung                                     | Maßnahme                                                     |
|------------------------------------------|------------------------------------------------|--------------------------------------------------------------|
| Genauigkeit der analytischen Ephemeride  | Systematische Fehler bis einige 10 000 km bei Planetenpositionen (JPL-Näherung: 20–40″) | Vor dem Flug DE440-Chebyshev-Koeffizienten laden; `Ephemeris`-Protokoll ist dafür vorbereitet. In der Simulation ist dieser Fehler nicht sichtbar, weil Wahrheit und Filter dieselbe Ephemeride nutzen. |
| Erde–Mond-Trennung                       | EMB statt Erde: bis 4 700 km Versatz           | Mondephemeride ergänzen (Phase 8)                            |
| Pulsar-Mehrdeutigkeit                    | Falscher Zyklus = Fehler von Hunderten km      | Nur akzeptieren, wenn σ < cP/6; 5σ-Gating                     |
| Sonnenausschluss der Kamera              | Keine OpNav in Sonnennähe                      | Sonnensensor, Pulsare und Doppler überbrücken                |
| Uhrdrift                                 | Pulsar-Timing und Ephemeridenzeit driften      | Uhrbias und -drift als Zustände (Phase 8)                    |
| Lagefehler                               | Jeder Lagefehler wird zum Richtungsfehler      | Sternsensor-σ in das Messrauschen eingerechnet               |
| Unmodellierte Kräfte                     | Filterdivergenz                                | Gauss-Markov-Biaszustand; Prozessrauschen                    |
| Nichtlinearität bei schlechtem Prior     | EKF-Linearisierung ungültig                    | Kaltstart liefert Prior mit korrekter Kovarianz; alternativ UKF |

## 9. Erweiterungen (Phase 8)

1. **Hochgenaue Ephemeride**: JPL DE440 als Chebyshev-Koeffizienten an Bord, inkl. Mond und großer Asteroiden.
2. **Uhrzustände**: Bias und Drift der Bordatomuhr im Filter, geschätzt über Pulsar-Timing.
3. **Lagefilter**: Multiplikativer EKF (MEKF) für Lage und Gyro-Bias statt reiner Sternsensor-Übernahme.
4. **Sensorbias**: Beschleunigungsmesser-Bias und Skalenfaktor als Zustände während Manövern.
5. **Relativnavigation**: Landmarken- und Horizontverfolgung beim Anflug, Monde als OpNav-Ziele.
6. **Square-Root- oder UD-Filter** für numerische Robustheit auf Flugrechnern.
7. **Monte-Carlo-Kampagne** und NEES/NIS-Konsistenzstatistik über viele Seeds.
8. **Flugsoftware-Integration**: Echtzeit-Scheduling, Zeitstempelung der Rohdaten, Telemetrie der Filterzustände.

## 10. Nutzung

```bash
swift build                          # Debug-Build
swift test                           # 24 Unit-Tests
swift run -c release navstar-sim     # 30-Tage-Simulation
open Package.swift                   # in Xcode öffnen
```

Minimale Einbindung des Navigators:

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
