---
title: Simulation Dispersion Campaigns — the falcon validation deck
---

# Simulation dispersion campaigns

Falcon pairs **formal proofs** (Kani BMC on the leaves, Lean Lyapunov on the
controllers) with **simulation dispersion campaigns** — the empirical half. A
proof establishes soundness over symbolic inputs; a campaign establishes that
the *integrated* verified cascade behaves correctly across thousands of
randomised realistic conditions. This is the NASA/POST2 **dispersion-deck**
discipline (fix a seed per trial, sweep a dispersion vector, reduce to a
pass-rate + worst-case margins) — which PX4/ArduPilot approximate only via
field flights + EKF log-replay, and which is a differentiator here.

## Method (shared by every campaign)

- **Reproducibility.** Hierarchical `SplitMix64` seeding: trial *i* draws from
  `trial_rng(campaign_seed, i)`, so any failure is replayable from its index
  alone. The `campaign_seed` is checked in.
- **Recoverable envelope.** Initial conditions / dispersions are drawn from an
  explicit envelope the system *should* handle. A trial that fails inside the
  envelope is a real bug; conditions outside it are either excluded
  (rejection-sampling) or must engage a fail-safe (the three-way pattern). This
  keeps "physics said no" from being mistaken for a defect.
- **Reduce to a distribution.** Each campaign asserts `failures == 0` **and**
  reports worst-case margins; regression bounds sit just above the measured
  worst, so a change that erodes a margin trips CI long before it reaches the
  physical safety bound.
- **Trial counts (rule of three).** Zero failures in *N* trials bounds the
  failure rate at `3/N` (95% conf): 300 → ≥99%, 600 → ≥99.5%, 2000 → ≥99.85%.
  Deterministic *sweeps* need fewer (dense 1-D coverage); stochastic campaigns
  use more.
- **Where they run.** All campaigns run under the dedicated CI job
  **"Closed-loop simulation + Monte-Carlo campaigns"** (`cargo test -p
  falcon-sitl-gz` + the falcon-core fail-safe campaigns) and the workspace test.
  Their rivet artifact steps are marked `# bench-only` (they run in that gate,
  not the traceability sweep).

## The deck

### Nominal recovery (rigid-body attitude plant)

| campaign | trials | dispersion vector | invariants | worst margin |
|---|---|---|---|---|
| `motor_out_monte_carlo_campaign` | 2000 | which rotor fails, fail time, initial tilt/rate | FDI isolates the failed rotor; MIX-P08 holds; no tumble; settles upright | peak tilt 0.837 rad (<1.4), settle 0.097 (<0.5), detect 2 ms |
| `motor_out_dispersed_monte_carlo_campaign` | 1500 | + per-rotor actuator noise (≤5%) + wind-disturbance torque + gusts | same, under disturbance | peak tilt 1.20 rad (<1.4), settle 0.16 |
| `att_stab_monte_carlo_campaign` | 2000 | initial tilt (≤0.5 rad) + rate (≤1 rad/s) | recover to level | peak 0.554, final ~0 |
| `hexa_monte_carlo_campaign` | 2000 | initial tilt/rate, 6-rotor airframe | same geometric controller stabilises a hexa | peak 0.523, final ~0 |

### Fail-safes (three-way — validates the safety net; full FlightSupervisor)

| campaign | trials | regimes | invariants |
|---|---|---|---|
| `failsafe_runaway_terminate_campaign` | 120 | tilt >1.05 rad vs <1.05 rad | terminates + latches when it must; never false-terminates |
| `failsafe_wind_rtl_campaign` | 120 | tilt in [0.30,0.70] band vs below | RTL latches when it must; never false-latches |

### Estimator robustness (real relay-iekf IEKF)

| campaign | trials | dispersion vector | invariants | worst margin |
|---|---|---|---|---|
| `estimator_robustness_monte_carlo_campaign` | 600 | gyro/accel/GNSS noise σ + a GNSS-dropout window | tilt bounded; dead-reckons the dropout then reconverges; NEES bounded | tilt 0.016 rad, drift 9.5 m, reconverged 1.71 m, NEES 56.9 |
| `maneuvering_estimator_monte_carlo_campaign` | 500 | random horizontal maneuver + unmodelled accel scale + noise | consistent under motion (velocity-NEES) | pos 0.91 m, vel-NEES 11.2, tilt 0.064 |
| `gnss_spoof_monte_carlo_campaign` | 400 | legit-vs-spoof; spoof rate 4–8×σ, onset, noise | SpoofMonitor: no false alarm on noise; detects + latches a walk-off | worst detect latency 5 fixes |

## Key discipline notes captured

- **Gravity aiding is invalid under motion.** `update_gravity` assumes the
  accelerometer measures the gravity reaction; during a maneuver the specific
  force includes the maneuver acceleration, so aiding reads it as a tilt and
  diverges the filter. The maneuvering campaign omits it (gyro=0 keeps attitude
  level); the static campaign keeps it.
- **A spoof below the noise floor is undetectable — by design.** The
  `SpoofMonitor` drift is scaled to the GNSS noise (2σ); the spoof envelope
  keeps the walk-off above it (4–8×σ). This is the three-way distinction
  between a real detector bug and correct physics.
- **`no_std` crates can't format in tests.** falcon-core (no alloc) uses core
  panic formatting in its fail-safe campaign, not `Vec`/`format!`.

## Next slices

Mission/waypoint corridor tracking (full position+attitude cascade);
maneuvering-truth GNSS-dropout; a nightly heavy tier (10k/behavior); and
recordable `--scenario=rotor-out` Gazebo flights for release video.
