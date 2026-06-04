# Falcon SITL Realism Roadmap (v1.16 → v1.25)

*From "hovers in a clean sim" to "flies in a realistic world." Each release adds
one physical-realism layer in BOTH the verifiable `no_std` SimBackend (a
CI-gated, proof-grade test) AND the gz Harmonic SITL (a built-in system for
physical fidelity), with a mechanical verification gate and a rivet trace.*

## Where we are (audited 2026-06-03)

**The gz world already has:** the MulticopterMotorModel (×N), and IMU /
magnetometer / NavSat(GPS) sensors with **basic Gaussian noise (mean/stddev
only)**. Estimator consistency (NEES), the adaptive-Q fix, geofence/battery
failsafes, and the pathology injector (vibration / bias-drift / GPS-dropout /
mag-interference) are all in the verified stack.

**The realism gaps (none modelled today):** wind · gusts · turbulence ·
aerodynamic drag · IMU **bias-instability** (random-walk dynamic bias — the
world omits `bias_mean`/`dynamic_bias_*`) · realistic GNSS noise + outage ·
barometer · realistic battery drain/voltage-sag · atmosphere/air-density
(thrust lapse) · ground effect.

**Tooling note:** the linked RotorS `GazeboWindPlugin` is **gz-classic** (ROS
Melodic). We are on **gz Harmonic (gz-sim 8)** — the equivalents are the
**built-in** systems below. Turbulence, GNSS dropout, motor→battery coupling,
and ground effect are **not** built in and need a small custom plugin / bridge.

## The gz Harmonic realism systems (verified catalog)

| Effect | gz system (built-in unless noted) | Key caveat |
|---|---|---|
| Wind + gusts | `gz-sim-wind-effects-system` (`WindEffects`) + world `<wind>` | force only on links with `<enable_wind>`; built-in sin+noise gusts |
| Aero drag/lift | `gz-sim-lift-drag-system` (`LiftDrag`) | one surface per instance; own `air_density` |
| IMU bias drift | `imu` sensor `<noise>` `bias_mean`/`dynamic_bias_*` | needs `gz-sim-imu-system` |
| Mag noise | `magnetometer` `<noise>` | needs world `<magnetic_field>` |
| GNSS noise | `navsat` `<noise>` position/velocity | outage = **custom**; horiz-noise-unit bug (gz-sensors #325) |
| Barometer | `air_pressure` sensor (`AirPressure`) | altitude derived downstream |
| Battery | `gz-sim-linearbatteryplugin-system` (`LinearBatteryPlugin`) | motor→battery coupling is **custom** (`<power_load>` is static) |
| Motor dynamics | `MulticopterMotorModel` time-constants + rotor drag | air density implicit/constant |
| Atmosphere | world `<atmosphere type="adiabatic">` | feeds baro only; does **not** lapse thrust/drag |
| Ground effect | **none built-in** | custom plugin (thrust vs height) |
| Turbulence (Dryden) | **none built-in** | custom wind-topic feeder |

## The releases

Each maps to a system requirement and ships the SimBackend test (the gate) +
the gz realism (the fidelity) + a rivet SWREQ/FV pair, in the established
release pattern.

| Ver | Layer | SimBackend (CI gate) | gz Harmonic (realism) | SYSREQ |
|---|---|---|---|---|
| **v1.16** | **Wind + gusts** | steady wind + gust **force**; bounded position-hold drift | `WindEffects` + `<wind>` + `<enable_wind>` | 017 |
| **v1.17** | **Aerodynamic drag** | body drag ∝ v²; velocity-bounded; no limit-cycle | `LiftDrag` on base_link | 017 |
| **v1.18** | **IMU bias-instability** | static + random-walk dynamic bias; IEKF stays consistent | `<noise>` `bias_mean`/`dynamic_bias_*` on IMU | 018 |
| **v1.19** | **GNSS noise + outage** | realistic position noise + intermittent fix; dead-reckon + reconverge | `navsat` `<noise>` + a dropout bridge | 018 |
| **v1.20** | **Barometer fusion** | baro altitude (noise) fused; altitude hold survives GPS-z loss | `air_pressure` sensor + `<atmosphere>` | 018 / 001 |
| **v1.21** | **Battery drain + sag** | charge drains w/ thrust, V sags under load → failsafe fires on real endurance | `LinearBatteryPlugin` + motor power-load bridge | 019 |
| **v1.22** | **Atmosphere / thrust lapse** | thrust scales with air density vs altitude; alt hold compensates | `<atmosphere>` + a density→thrust bridge | 017 |
| **v1.23** | **Motor dynamics** | first-order motor lag + rotor drag; ADRC ESO absorbs it | tune `MulticopterMotorModel` time constants | 002 |
| **v1.24** | **Ground effect** | thrust augmentation near ground; stable takeoff/landing | custom thrust-vs-height plugin | 017 |
| **v1.25** | **Turbulence (Dryden) + capstone** | Dryden/von Karman gust spectrum; control under turbulence | custom `/world/.../wind` topic feeder | 017 |

**v1.25** also re-runs the clean-room sweep and refreshes the readiness dossier
with the realism evidence — the "flies in weather" capstone (and a storm hero
shot).

## Discipline (unchanged)

- Every layer pairs with a mechanical gate; the SimBackend test is the
  *proof-grade* artifact, the gz run is the *fidelity* artifact.
- Honest framing: each release **holds or falsifies** — published either way
  (the ESO/position-loop may not catch every force component; that's a finding,
  not a failure).
- The flight crates stay `no_std`/`no_alloc`; gz + serde stay host-only.
- Traceability leads: **SYSREQ-FALCON-017/018/019** are filed; each release
  adds its `SWREQ-FALCON-*` + `FV-FALCON-*` before the code, per the loop.

## Suggested core vs stretch

If trimming to a tight set: **v1.16 (wind), v1.18 (IMU bias), v1.19 (GNSS),
v1.21 (battery), v1.25 (turbulence)** are the highest-impact five — they cover
the disturbance, the estimator, the sensors, the energy, and the worst-case
environment. v1.17/1.20/1.22/1.23/1.24 deepen fidelity.
