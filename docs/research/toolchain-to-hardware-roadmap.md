# Falcon Roadmap: The Verified Component, and the Hand-off to Hardware

*Separation of concerns. **This repo (relay)** makes the verified flight core a
portable, wasmtime-runnable **WebAssembly Component Model** artifact and keeps
hardening it. **A separate integration project** consumes that component and
carries it to silicon — meld (fuse) → loom (optimize) → synth (transcode to
ARM/RISC-V + proofs) → gale (verified RTOS) → board — and coordinates everyone.*

## The boundary

```
   ┌─ relay (this repo) ─────────────────┐   ┌─ falcon-integration (separate) ─┐
   │  falcon-core (verified cascade)      │   │  consumes falcon:flight@1.26    │
   │  → falcon:flight CM component  ──────┼──▶│  meld → loom → synth → gale     │
   │  (wasmtime-runnable, the hand-off)   │   │  → Cortex-M / RISC-V board       │
   │  + hardware-practical + autonomy     │   │  + guides integrators           │
   └──────────────────────────────────────┘   └─────────────────────────────────┘
```

The clean interface between them is the **component** (`wit/falcon-flight/
flight.wit` + `wasm/cm/flight`): a typed, portable, verified artifact. relay
proves it *runs and is correct*; the integration project owns *getting it onto
hardware*. Neither repo has to know the other's internals.

## v1.26 — DONE (this repo): the keystone

`falcon-core` (IEKF → geometric SE(3) → ADRC → mixer) builds as a Component
Model component and **runs in wasmtime**: `run-stabilization` recovers a tilted
body to level (0.023 rad), `run-position-hold` flies to a setpoint (0.13 m) —
the verified core executing as a portable component, gated by
`scripts/wasmtime-flight-test.sh`. This is the artifact the integration project
imports.

## The remaining relay releases (hardware-practical + autonomy)

| Ver | Layer | Thread |
|---|---|---|
| **v1.27** | **velocity-based touchdown controller** — the clean landing the v1.24 ground-effect float needs | hardware-practical |
| **v1.28** | gz **custom realism plugins** (turbulence / ground-effect / motor→battery feeders) so the gz SITL matches the SimBackend realism | hardware-practical |
| **v1.29** | **autonomy** — dynamic mission replanning + obstacle-aware geofence shapes | autonomy |
| **v1.30** | **autonomy** — multi-leg missions, sensor-driven avoidance, return-path planning | autonomy |
| **v1.31** | the **real driver suite** — GNSS / mag / ESC / barometer drivers (the `falcon-imu-icm42688` mock-bus pattern) | hardware-practical |
| **v1.32** | **hygiene + capstone** — EKF-crate retirement, `Cargo.lock`+`--locked` hardening, readiness re-verification | hygiene |

(meld / loom / synth / gale releases live in the **integration project**, not
here — though meld/loom/synth are installed locally and the v1.5 meld fusion
remains as a reference; gale is not present, the documented hardware boundary.)

## Discipline (unchanged)

Every relay release pairs with a mechanical gate (a wasmtime run, a Kani proof,
a test). Honest limits published. Flight crates stay `no_std`/`no_alloc`.
Traceability leads. The component is the contract; keep it stable and typed so
the integration project can depend on it.
