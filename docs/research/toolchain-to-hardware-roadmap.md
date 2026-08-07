# Falcon / Relay Roadmap: The Verified Component, and the Hand-off to Hardware

*Separation of concerns. **This repo (relay)** makes the verified flight core a
portable, wasmtime-runnable **WebAssembly Component Model** artifact and keeps
hardening it. **A separate integration project** consumes that component and
carries it to silicon — meld (fuse) → loom (optimize) → synth (transcode to
ARM/RISC-V + proofs) → gale (verified RTOS) → board — and coordinates everyone.*

This is the **single source of truth for what to build next.** Each open feature
below is a triggerable unit: a name, the concrete next slice, its mechanical
oracle, and any decision it needs. To start one, point at its ID (e.g. "do F1").

## The boundary

```
   ┌─ relay (this repo) ─────────────────┐   ┌─ falcon-integration (separate) ─┐
   │  falcon-core (verified cascade)      │   │  consumes pulseengine:falcon-flight@1.26    │
   │  → pulseengine:falcon-flight CM component  ──────┼──▶│  meld → loom → synth → gale     │
   │  (wasmtime-runnable, the hand-off)   │   │  → Cortex-M / RISC-V board       │
   │  + hardware-practical + autonomy     │   │  + guides integrators           │
   └──────────────────────────────────────┘   └─────────────────────────────────┘
```

The clean interface is the **component** (`wit/falcon-flight/flight.wit` +
`wasm/cm/flight`): a typed, portable, verified artifact. relay proves it *runs
and is correct*; the integration project owns *getting it onto hardware*.

## Shipped (main) — the arc so far

- **v0.20 → v1.0.0** — certified autonomy floor: IEKF estimator → geometric
  SE(3) + Lean Lyapunov → ADRC inner loop → control allocation → FDI/reconfig →
  trajectory → simplex shield. Each with a mechanical gate.
- **v1.1 → v1.15** — hardware-abstraction spine: FlightBackend / SimBackend /
  HardwareBackend / LinkBackend; relay-math (libm seam); WCET last-sorry closed.
- **v1.16 → v1.25** — SITL realism: wind, drag, IMU bias, GNSS, baro, battery,
  atmosphere, motor lag, ground effect, turbulence.
- **v1.26 → v1.27** — falcon-core as a wasmtime-runnable CM component;
  velocity-based touchdown controller.
- **v1.28 → v1.34** — MAVLink arc: bridge (COMMAND→Event, telemetry), landing
  integration, multi-waypoint missions, keep-out avoidance, gz ground-effect
  plugin, mission upload + download.
- **Hardening + security (this cycle, merged PRs, not yet tagged releases):**
  oracle-layer hardening (#111: 52 Kani harnesses relocated regen-safe, workspace
  clippy gate, EKF artifact over-claim fixed); **relay-sec** safety-protected
  comms slices #112 (anti-replay window) + #113 (Ascon-AEAD128 primitive);
  proptest float-sampler flake fixed (#114). Architecture-of-record:
  SWARCH-RELAY-SCHED-001 (scheduling layers) + SWARCH-RELAY-SEC-001 (security).

## The current feature roadmap (open work)

### Thread A — relay-sec: safety-protected comms (ACTIVE)

One shared, verified, no_std auth layer under the relay CCSDS stack, protecting
BOTH falcon (drone C2) and wohl (home-automation). NOT DDS/ROS2 — the powerful
companion/ground side converts CCSDS↔anything outside the verified boundary.
Full design: `SWARCH-RELAY-SEC-001`. Built oracle-first.

| ID | Slice | Status | Next oracle | Decision needed |
|---|---|---|---|---|
| — | anti-replay window (ReplayWindow) | ✅ #112 | Kani SEC-K01..K04 | — |
| — | Ascon-AEAD128 primitive (MAC-floor + AEAD) | ✅ #113 | NIST KATs + Kani SEC-K05/K06 | — |
| **F1** | **total Security-Header parser** (SPI∥channel_id∥counter), pre-auth, panic-free on garbage | next | Kani totality (like the window) | none — ready to trigger |
| **F2** | SA store + frame wrap/verify over relay-ccsds `sec_header_flag`; nonce = f(SPI,channel,counter) | after F1 | Kani totality + round-trip tests | none |
| **F3** | relay-ci verify-before-parse + relay-to wrap wiring (shared falcon+wohl) | after F2 | integration tests + rivet trace | none |
| **F4** | session establishment: ephemeral X25519 ECDH + Ed25519/sigil (PFS) + **rekey-on-reboot** invariant | after F3 | handshake KAT + invariant test | **wants a reviewer design pass** (highest-stakes) |
| **F5** | AEAD-upgrade profiles (wohl sensor data default-on) | after F2 | KAT + per-product config test | confirm per-product policy |

### Thread B — real driver suite (roadmap v1.31, OPEN — the hardware-boundary gap)

The PX4-coverage review's #1 honesty gap: every path to a real airframe is a
trait seam with no body. `falcon-imu-icm42688` is the only real driver (over an
abstract bus, axis-remap TODO, never on silicon).

| ID | Feature | Next oracle |
|---|---|---|
| **F6** | GNSS (UBX), magnetometer, barometer driver bodies over real embedded-hal; finish ICM-42688 SPI + axis-remap | mock-bus protocol tests (the icm42688 pattern) + on-silicon validation (the honest open item) |
| **F7** | ESC (DShot) + battery driver bodies | mock-bus protocol tests |

### Thread C — hygiene capstone (roadmap v1.32, PARTIAL)

| ID | Feature | Status | Oracle |
|---|---|---|---|
| — | oracle-layer hardening (Kani regen-safety, clippy gate) | ✅ #111 | — |
| **F8** | `Cargo.lock` tracked + `--locked` in CI (the untracked-lock dep-drift fragility that keeps biting) | open | CI builds with `--locked` |
| **F9** | brittle FV `verification-criteria` sweep (TOUCHDOWN/GZGE/ADRC exact-threshold text) | open | rivet validate, criteria read as properties |
| **F10** | EKF-crate retirement decision (relay-ekf is Mahony, used by the gz bench — held pending a scoped check) | open | confirm no live consumer, then remove |

### Thread D — SITL realism remainder (roadmap v1.28, PARTIAL)

| ID | Feature | Status |
|---|---|---|
| — | gz ground-effect plugin | ✅ v1.32 |
| **F11** | gz custom plugins: turbulence (Dryden) + motor→battery feeders, to match SimBackend realism | open |

### Boundary (separate integration project, NOT this repo)

meld → loom → synth → gale onto the user's board (G474 / ESP32-C3). gale (the
verified RTOS) is not present here; it is the documented hardware boundary and
hosts the proven RTOS scheduler beneath relay-sch (see SWARCH-RELAY-SCHED-001).

## Discipline (unchanged)

Every relay release pairs with a mechanical gate (a wasmtime run, a Kani proof,
a KAT, a test, a rivet check). Honest limits published. Flight crates stay
`no_std`/`no_alloc`. Traceability leads. The component is the contract — keep it
stable and typed so the integration project can depend on it.

## Recommended sequence (the user set: comms → hardening → drivers)

1. **F1 → F2 → F3** (relay-sec header parser → frame wrap/verify → wiring) — no
   decisions, all oracle-first, finishes the protected-frame path end-to-end.
2. **F4** (session keys / handshake) — after a reviewer design pass.
3. **F8 → F9** (lock hardening + FV sweep) — cheap hygiene that de-risks CI.
4. **F6 → F7** (driver suite) — the hardware on-ramp.
5. F5, F10, F11 as they fit.
