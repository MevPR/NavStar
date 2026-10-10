# NavStar

Autonomes Navigationssystem für einen Raumgleiter im Sonnensystem, das
ausschließlich bordeigene Sensoren nutzt: Sternsensor, IMU, optische
Navigationskamera, Sonnensensor, Röntgen-Pulsardetektor (XNAV),
Solarspektrometer (Doppler) und Altimeter. Keine Funkortung, kein GNSS, keine
Daten von der Erde – nur vor dem Start geladenes Bordwissen (Sternkatalog,
Planetenephemeriden, Pulsarkatalog).

Der vollständige Entwurf, die Mathematik, das Fehlerbudget und der
Implementierungsplan stehen in [Plan.md](Plan.md).

## Aufbau

- `Sources/NavStarCore` – Bibliothek: Mathematik, Zeit, Ephemeriden, Dynamik,
  Messmodelle, Sensormodelle, Kalman-Filter mit Kaltstart, `Navigator`-Fassade.
- `Sources/NavStarSim` – Executable `navstar-sim`: 30-Tage-Simulation mit
  Wahrheitsmodell, verrauschten Sensoren und Fehlerauswertung.
- `Tests/NavStarCoreTests` – 24 Unit-Tests.

## Verwendung

```bash
swift build
swift test
swift run -c release navstar-sim
```

Das Paket lässt sich direkt in Xcode öffnen (`Package.swift`).

## Kurzüberblick der Methode

1. Der Sternsensor liefert die Lage, die IMU schreibt sie fort und misst Schub.
2. Kamera und Sonnensensor liefern Richtungen zu Sonne und Planeten; aus mehreren
   Richtungen wird per gewichteter Triangulation ein Kaltstart-Fix bestimmt.
3. Ein erweiterter Kalman-Filter mit 9 Zuständen (Position, Geschwindigkeit,
   Beschleunigungsbias) propagiert mit einem N-Körper-Modell und verarbeitet
   Richtungs-, Pulsar-, Doppler- und Entfernungsmessungen mit Lichtlaufzeit-
   und Aberrationskorrektur.
4. Pulsarmessungen werden erst verwendet, wenn die Positionsunsicherheit die
   Zyklus-Mehrdeutigkeit zulässt; danach erreicht die Lösung km-Genauigkeit.
