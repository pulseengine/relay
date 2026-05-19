# Changelog

All notable changes to relay + falcon are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Tags use a per-track prefix:
- `falcon-v<semver>` — the falcon dual-DNA flight stack
- (future) `relay-v<semver>` — the relay substrate itself

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
