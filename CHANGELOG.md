# Changelog

All notable changes to relay + falcon are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Tags use a per-track prefix:
- `falcon-v<semver>` — the falcon dual-DNA flight stack
- (future) `relay-v<semver>` — the relay substrate itself

## [falcon-v0.3.0] — 2026-05-19

Rate controller + closed-loop SITL. The inner control loop closes:
EKF estimates attitude, rate PID drives commanded body rates, plant
integrates dynamics, closed loop converges. Pure-Rust SITL runs in
CI — no Gazebo install required.

### Added

- **`crates/relay-rate`** — body-rate PID stabilizer:
  - 3-axis PID with clamp-and-hold anti-windup. no_std + no_alloc;
    no transcendentals (just adds/multiplies/clamps).
  - Default gains tuned for a 500 g, 10-inch quadcopter at 1 kHz
    update; `RateGains::DEFAULT` with `i_max` and `torque_max`
    bounds per axis.
  - `RatePid::tick(time, measured_rate, setpoint_rate) -> torque`
    consumes gyro + setpoint, emits torque. dt derived from time
    deltas, clamped `[0.0001, 0.1]` s to defend against jumps.
  - `RatePid::reset()` for arm/disarm + setpoint discontinuity.
  - `RatePid::set_gains(g)` for TBL load.
  - 13 unit + proptest cases for RATE-P01 (integral bounded),
    RATE-P02 (no NaN/∞ propagation), RATE-P03 (step response
    convergence + Lyapunov surrogate), output-clamp, dt
    defensive bounding, reset semantics.
- **`examples/falcon-sitl-hover`** — closed-loop SITL bench. Pure
  Rust rigid-body plant (rotational dynamics + quaternion
  integration) driven by the v0.2 EKF + v0.3 rate PID. Three
  scenarios:
    - `step`: setpoint `[0.5, -0.3, 0.4] rad/s`; verify rise time,
      overshoot, steady-state error.
    - `disturbance`: at hover, inject a `1 rad/s` impulse about y;
      verify recovery time.
    - `hover`: vehicle starts with non-zero rates
      `[0.7, -0.5, 0.3]`; verify settle.
  CLI: `--scenario step|disturbance|hover|all`, `--noise σ`,
  `--quiet`. Returns 0 on PASS, 1 on FAIL. 5 unit + integration
  tests covering each scenario plus a noisy variant.
- **`artifacts/verification/FV-FALCON-RATE-001.yaml`** — v0.3
  verification artifact with extractable `fields.steps` (5 step
  commands).
- **`FEAT-FALCON-v0.3`** bumped `pending` → `approved` with
  achieved metrics inline.

### Achieved bench metrics

| scenario | convergence | overshoot | RMS-steady |
|---|---|---|---|
| step ([0.5, -0.3, 0.4] rad/s) | **0.139 s** | **1.2 %** | **0.0012 rad/s** |
| disturbance recovery (1 rad/s) | **0.141 s** | — | — |
| hover from [0.7, -0.5, 0.3] | **0.175 s** | — | — |

Loop wall time: ~400 µs for 5000 samples (one full 5 s trajectory).
No NaN/∞ in any scenario. Deterministic given a seed.

### Verification

- `cargo test --workspace`: 55 test suites green (was 52 in v0.2).
- `cargo test -p relay-rate`: 13/13 PASS including 2 proptest at
  256-default + 4096-fuzz.
- `cargo test -p falcon-sitl-hover`: 5/5 PASS.
- `cargo run -p falcon-sitl-hover --release`: PASS on all three
  scenarios.
- `python3 scripts/run-falcon-verification.py --markdown`: ✅ 5/5
  falcon FV artifacts pass, 18/18 steps green.
- `rivet validate`: 0 broken cross-references.

### Scope notes — what slipped

- **Gazebo Harmonic SITL** was originally scoped for v0.3 but
  pushed to v0.4. Pure-Rust SITL ships now because it runs in CI
  without Gazebo installation and produces byte-identical results
  given a seed. Real Gazebo lockstep arrives when the full
  attitude cascade (v0.4 with `relay-att`) makes a full hover
  meaningful.
- **Lean Lyapunov proof** of the rate loop → v0.4 with
  `rules_lean` wiring. The bench's empirical convergence is the
  v0.3 surrogate.
- **Kani bounded-overflow** on PID arithmetic → v0.4.
- **tokio-rs/loom** on a host bridge → v0.5 once the bridge
  exists.
- **rerun.io `.rrd` evidence** → v0.4 with the SITL hookup.

## [falcon-v0.2.0] — 2026-05-19

Real attitude estimator. Replaces the v0.1 stub with a Mahony
complementary filter on SO(3), validated by a deterministic
synthetic-IMU accuracy bench. No flight dynamics yet — that's v0.3.

### Added

- **`crates/relay-ekf`** — no_std + libm implementation of the
  Mahony, Hamel & Pflimlin (2008) complementary filter on SO(3)
  with gravity-only correction:
  - `Ekf::new()`, `Ekf::with_gains(EkfGains{kp, ki})`,
    `Ekf::set_initial_quaternion(q)`, `Ekf::tick(ImuSample)`.
  - Defaults Kp=2.0, Ki=0.05 (tuned for a 200 Hz–1 kHz consumer-
    grade IMU).
  - Bias estimate bounded ±0.5 rad/s under sustained excitation.
  - Pure-math helpers (`quat_mul`, `quat_conj`,
    `rotate_body_to_ned_inverse`, `cross`, `normalise`,
    `is_unit_quaternion`) exported for the controller layer.
  - 16 unit + proptest cases covering EKF-P01..P05 surrogates:
    unit-quaternion preservation per-tick + sequence, no NaN
    under adversarial accel, innovation monotone with tilt
    disagreement, static rest convergence, pure-yaw stability,
    bias bound.
- **`examples/falcon-ekf-bench`** — runnable accuracy bench:
  - 25-second deterministic synthetic trajectory at 200 Hz
    (rest at 20° pitch → roll → rest → yaw → rest).
  - Compares estimator vs ground truth, reports RMS attitude
    error in degrees + convergence time.
  - CLI: `cargo run -p falcon-ekf-bench --release` (deterministic);
    `--noise 0.2` for σ=0.2 m/s² IMU noise.
  - Acceptance budget: RMS-steady ≤ 5°, final ≤ 5°, no NaN.
  - Achieved on this release: RMS-steady **3.31°**, final
    **3.02°**, convergence **0.68 s**, peak 19.8°.
- **`artifacts/verification/FV-FALCON-EKF-001.yaml`** — v0.2
  verification artifact with extractable `fields.steps`:
  `cargo test -p relay-ekf` + release rerun + 4 k proptest fuzz
  + bench tests + bench binary smoke. Supersedes v0.1's
  `FV-FALCON-EKF-STUB-001` (which is preserved for history).

### Verification

- `cargo test --workspace`: 50 test suites green (was 49 in v0.1;
  one new — `falcon-ekf-bench`).
- `cargo test -p relay-ekf`: 16/16 PASS including 2 proptest cases
  at 256 default + 4096 fuzz mode.
- `cargo run -p falcon-ekf-bench --release`: PASS at v0.2 budget.
- `python3 scripts/run-falcon-verification.py --markdown` against
  the new gate: ✅ 4/4 falcon FV artifacts pass, 13/13 steps green.
- `rivet validate`: 0 broken cross-references.

### Known limitations

- **No magnetometer fusion yet** — gravity-only Mahony filter
  cannot observe heading directly. Small residual yaw drift is
  fundamental until v0.4 (`relay-att` with mag).
- **No Verus SMT proofs yet** on the EKF math. Deferred to v0.4
  with the `src/` Verus-annotated track + Bazel `verus_test`
  rules. v0.2 covers the same property classes via proptest at
  4 k cases.
- **No Lean WCET proof yet** on `Ekf::tick`. The estimator's wall
  time on a single tick is empirically ≤ 1 µs (5000 samples in
  333 µs on the bench runner), well inside a 1 ms IMU period;
  formal proof lands in v0.4.
- **No WASM-component compilation yet** — the EKF compiles as a
  plain `cargo` crate. wit-bindgen integration follows when the
  relay-substrate's P3 streams arrive (v0.3+).

## [falcon-v0.1.0] — 2026-05-19

The dual-DNA flight stack's first tagged release. Pre-product:
boots, exchanges MAVLink heartbeats with a ground control station,
proves the relay + falcon toolchain works end-to-end with the full
verification chain in CI.

### Added

- **WIT interfaces** (`wit/interfaces/`):
  - `relay-mavlink/protocol.wit` — MAVLink v2 protocol types
    (heartbeat, frame, codec-error).
  - `relay-control/dynamics.wit` — control-cascade typed streams
    (imu-sample, vehicle-state, rate/attitude/torque setpoints,
    motor-pwm). Records for v0.2–v0.5 forward-declared.
- **WIT worlds** (`wit/worlds/relay-falcon.wit`):
  falcon-quad (v0.1 first-ship target), falcon-hex (v1.0),
  falcon-coax (v1.0; Ingenuity-class).
- **Crates** (no_std + no_alloc):
  - `relay-mavlink` — MAVLink v2 codec with CRC-16/MCRF4XX
    (validated against the reference vector `0x6F91` on
    "123456789"), HEARTBEAT encode/decode (id=0,
    CRC_EXTRA=50), frame envelope construction and parsing,
    bounds-checked everywhere. 33 unit + proptest cases.
  - `relay-ekf-stub` — v0.1 placeholder state-estimator;
    emits identity attitude / zero position-velocity /
    healthy innovation. 9 unit + proptest cases. Real EKF
    arrives in v0.2.
- **Example** (`examples/falcon-hello/`):
  - `falcon-hello` binary with `--mode vehicle` and
    `--mode gcs`; exchanges MAVLink heartbeats over UDP
    loopback at user-configurable rate and duration.
  - 9 integration tests including
    `vehicle_and_gcs_exchange_heartbeats_over_udp` which
    proves the codec works over real sockets in-process.
- **Scripts** (`scripts/`):
  - `falcon-hello-demo.sh` — end-to-end smoke runner.
    Builds release binary, spawns vehicle + gcs as separate
    OS processes, asserts ≥ N heartbeats exchanged. Used
    as a step inside `FV-FALCON-WORLD-001`.
  - `run-falcon-verification.py` — extracts and runs every
    `fields.steps[].run` command from rivet-tagged
    verification artifacts; aggregates pass/fail per
    artifact; emits the Markdown comment template the
    GitHub-Actions sticky-comment poster consumes.
- **Rivet artifacts** (`artifacts/`):
  - `STKH-FALCON-001` stakeholder requirement —
    verified dual-DNA flight stack for safety-critical
    drones and smallsats across six standards.
  - `SYSREQ-FALCON-001..010` system requirements covering
    state estimation, rate / attitude / position control,
    mixing, MAVLink interop, airframe variants, dual-DNA
    composition, deterministic SITL, multi-domain credit
    packaging.
  - `SWARCH-FALCON-001` architecture decision capturing
    the Leeloo cross-DNA flows (TBL → controller gains,
    EVS ← EKF innovation, LC → SC RTL, mission via SC).
  - `SWDD-FALCON-{EKF, RATE, ATT, POS, MIX}-001` design
    descriptions with algorithms and verification mapping.
  - `SWREQ-FALCON-{EKF, RATE, ATT, POS, MIX, MAVLINK,
    WORLD}-P0*` software-level Verus/Lean/Rocq/Kani
    property requirements (~25 properties total).
  - `FEAT-FALCON-v0.1..v1.0` release-plan milestones,
    chained via `depends-on` so `rivet impact` answers
    "if I touch X, which release does that block?"
  - `FV-FALCON-{MAVLINK, EKF-STUB, WORLD}-001` v0.1
    verification artifacts. Each carries
    `fields.method: automated-test` (or
    `integration-test`) and `fields.steps:[{run: ...}]`
    so a verification gate can extract and execute them.
- **GitHub Actions** (`.github/workflows/`):
  - `ci.yml` — fmt + clippy + cargo test (linux / macos /
    windows) + falcon-hello-demo smoke.
  - `verification-gate.yml` — runs the rivet-driven
    falcon verification gate on every PR, posts a sticky
    Markdown comment with pass/fail per artifact. Filter
    overridable via `Verify-Filter:` line in the PR body.
  - `release.yml` — tag-triggered, builds the
    falcon-hello binary for x86_64 + aarch64 linux,
    x86_64 + aarch64 macOS, x86_64 Windows; cosign keyless
    signing (Fulcio OIDC) per archive; SHA-256 checksums;
    publishes a GitHub Release.
- **Top-level files**: `README.md`, `CHANGELOG.md`,
  `LICENSE`, and release notes at
  `.github/release-notes/v0.1.0.md`.

### Verification

- `cargo test --workspace`: 49 test suites, 0 failures
  (51 new tests added in this release).
- `bash scripts/falcon-hello-demo.sh`: PASS (14/14
  heartbeats exchanged over UDP, decoded payload fields
  byte-for-byte match the encoded values).
- `python3 scripts/run-falcon-verification.py --markdown`:
  ✅ 3/3 falcon verification artifacts passed, 8/8 steps
  executed.
- `rivet validate`: 0 broken cross-references across all
  falcon artifacts.

### Known limitations

The following are intentionally deferred per the falcon roadmap:

- **No Verus SMT proofs yet** on the new crates. Verification
  posture at v0.1 is extensive testing + proptest fuzz; formal
  proofs land in v0.2 alongside the real EKF.
- **No witness MC/DC coverage report** for the falcon WASM —
  WASM-component compilation is gated on the relay-substrate's
  P3 streams landing (v0.2 work).
- **No actual flight dynamics**. v0.1 emits constants from the
  EKF stub; v0.3 brings SITL hover; v0.6 brings hardware bring-up.
- **No SITL hookup** to Gazebo Harmonic. v0.3 wiring.
- **No Bazel `verus_test` rules** for the new crates. The cFS
  engines have them; falcon crates get them in v0.2 once the
  `src/` Verus-annotated track lands.

See [falcon/README.md](falcon/README.md) for the full v0.1 → v1.0
release plan.

---

Pre-falcon relay history (the cFS-isomorphic substrate that falcon
extends) is tracked in commits and rivet artifacts under
`artifacts/sysreq/STKH-RELAY-001.yaml` and downstream.
