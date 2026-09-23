# Plan — best-in-class drone software, on gale, all in the WebAssembly component model

*A view, not the source of truth.* Scope lives in rivet (`release:` on each artifact); readiness is computed by `scripts/release-readiness.rs`. This page maps every open issue to the phase that owns it, so nothing is invisible to the plan.

**Goal set by the maintainer, 2026-09-23.** Supersedes the v1.139→v1.142 framing, which organised the work by release number. Release numbers are now an output; the phases below are the plan.

## The thesis, stated so it can be falsified

Four claims. Each is wrong if its test fails, and each has an owner phase.

1. **Every flight capability is a component.** No monolith on the flight path. Today `crates/falcon-core` is 6,090 lines carrying the supervisor, every failsafe and the FDI — with **0 Kani harnesses**, and it is not in the `kani.yml` matrix. It is the exact opposite of the thesis and must *dissolve*, not grow.
   *Wrong if:* a flight capability ships that is not a component, or `falcon-core` is still a monolith at the end of Phase 2.
2. **Composition is free.** A `wac`-composed, `meld`-fused, `synth`-compiled cascade computes **bit-identical** motor outputs to the native core on the same frames, on a real physics engine (`DIFFERENTIAL=1`).
   *Wrong if:* any non-zero per-motor difference, or the fused artifact cannot be built.
3. **It runs on gale on real silicon**, and no oracle can hide a fault that silicon would take.
   *Wrong if:* qemu passes something that BusFaults on a board — the exact shape of gale#398.
4. **The flight behaviour is genuinely good.** Best-in-class is not an architecture claim. Nothing best-in-class hovers with a 115°/s attitude ring.
   *Wrong if:* a healthy hover carries a limit cycle that a position-only verdict hides.

### The constraint that shapes everything

**Decomposition and memory packing are the same problem.** `synth`-dissolved objects address wasm linear memory as `[r11 + off]` with absolute wasm addresses near 1 MiB (gale#398). `fused.o` — *two* components — has data bases **1.1 MiB apart through one r11 base**, more RAM than any board here has (largest: WB55, 192 KB). gale's verdict: *"Not fixable in the embedder."*

Every component added makes this worse. So **`meld --pack-rebase` working on the flight object is a precondition for the whole thesis**, not a later optimisation. It is already proven on the iso-core path, and jess measured it at 5× on the M7.

And note what currently lowers to the M7 is the **legacy PID cascade**, not the law we fly. The one existing proof-point for "our stack runs as components on silicon" is about code we do not use.

## Phase 0 — make the ground true *(current)*

Everything downstream measures against the plant. Fix the plant's story first.

- **#270 — the attitude limit cycle is the root defect.** Filed 2026-07-14 at ~1 rad/s; measured 2026-09-23 at **1.90–2.12 rad/s** with motor commands thrashing saturation-to-saturation. The 1→2 rad/s FDI gate widening used to work around it **has been consumed**.
- **#398 — rotor-out does not recover on the gz plant.** Downstream of #270: the FDI gate is shut 60% of healthy-hover ticks, so detectability is decided by limit-cycle phase at the instant of failure (5/5: gate open ⇒ isolated in one tick; gate shut ⇒ 180° inversion). 12 trials: 11 FAIL / 1 PASS. `FV-FALCON-FAULT-003` was demoted `verified`→`implemented` when its own falsification clause was met (#479).
- Related plant-fidelity debt: #403, #434 (hold diverges at ≤10 Hz aiding), #435 (torque-free MockPhysics), #452 (accelerometer cannot show thrust), #290 (notch oracle non-deterministic), #477 (legacy `hover` scenario diverges).
- **Christof is the oracle here.** The bench wobble and the gz ring are to be worked as **one** defect, not two. His capture is the only real-vehicle datum that exists.

## Phase 1 — the seam returns state

`step(sensors, target) -> motor-pwm` returns nothing, so the verified MAVLink stack has nothing to read, no log exists, and no shadow flight can be compared.

**jess specified the shape on jess#167 (2026-09-22):** second export — *not* on `step`, so the M4 can read state without executing a control tick; **seqlock** with the tick count first *and* last; **OCRAM** (DTCM is M7-private and invisible to the M4); **32-byte aligned** and padded to a multiple, so a partial cache invalidate cannot tear it; fixed layout, validity bitmask, no `option<>`/lists; monotonic tick that never resets. Cadence deliberately **unspecified** — it is to be *measured* from the tick delta on the first shadow flight, not invented.

Correction jess supplied to our premise: there is **no flattening cliff on returns** (`MAX_FLAT_RESULTS` is 1; every export already returns a pointer). State-return size costs bytes, not a calling-convention change — do not design around it.

`SWREQ-FALCON-TRANSPORT-P01`. A `falcon-cascade` version bump ⇒ **announced before publishing** (the v1.133 path-move lesson).

## Phase 2 — the component model becomes real

`SWREQ-FALCON-OCI-P07`. The decomposition already exists in `wit/falcon-cascade/cascade.wit` — but it decomposes the **legacy PID** cascade. Per `scripts/audit-component-deps.rs`, `relay-geo` and `relay-adrc` — the flown middle stages — have **no component at all**.

- Componentize `relay-geo` + `relay-adrc`. Until these exist, no composition can produce the flown law.
- Reshape the cascade world to **import** stage interfaces (#393: it imports nothing, so `wac plug` has no socket — correct behaviour, and how this was found).
- Stop publishing the legacy stage components (#388, #411, #412, #419, #385). "Something labelled legacy beats nothing" is wrong when it is cosign-signed and an integrator has already fused it.
- #376 — per-tick reference vectors for the full cascade (jess's differential covers 2 of 5 stages).

## Phase 3 — fit on silicon

- **gale#398** — rebuild the flight object with `meld --pack-rebase` so every data address fits one RAM window; add the object-to-embedder **mechanical oracle** so qemu can never again pass what silicon faults on.
- #330 — the wit-bindgen fork: gale-owned arena instead of a per-component one. Directly reduces per-component memory, so it belongs here rather than in platform work.
- #407 — `//:falcon-cascade-coverage` fails to fuse (duplicate `cabi_realloc$2`) and is in no workflow.
- #214 (relay-hal seams + DroneCAN components), #177 (no_std shared-memory transport), #157 (the 7-item hardware register).
- Exit criterion: **one *flown* stage runs on the M7** — not a legacy one.

## Phase 4 — the safety layer, as components

**This phase is the first-flight blocker.** `relay_fsm::Mode` is `Disarmed | Armed | Takeoff | Loiter | Mission | Land | Rtl | Terminated` — **no Manual, no Stabilized, no Acro** — and the `FlightBackend` seam has **no RC channel at all**. There is no way for a human to take the sticks. No sane first-flight protocol permits that.

- `relay-rc` as a component + an RC channel in the seam + manual modes in the mode machine.
- `relay-fsafe` — the **verified** failsafe arbiter — replacing the subset `falcon-core` hand-rolls inline. Two implementations exist; the unused one is the verified one.
- #414 — the shipped component has no supervisor: no geofence, battery, runaway-cut or pre-arm. A host embedding it gets a stabiliser that will not stop for anything.
- `SWREQ-FALCON-ORPHAN-P01` covers the disposition of the rest. Flight-blocking orphans are `relay-rc`, `relay-fsafe`, `relay-arm`, `relay-mavlink`/`falcon-mavlink`, `falcon-param`. **Park** (do not wire) the parity-table filler: `relay-modextra`, `relay-mix-multi`, `relay-sensvote`, `relay-avoid`, `relay-offboard`, `relay-traj` — built to tick rows in a table that was itself wrong for 82 versions (#422).
- Kani the supervisor path, or dissolve it into proven component crates so it inherits their proofs.

## Phase 5 — evidence on a real vehicle

- **Shadow flight.** jess already runs a live read-only CRC-gated MAVLink feed off the FMU USB (ATTITUDE ~85 Hz) and can log PX4's estimate plus raw sensors **today** — and ours the moment Phase 1 lands.
- Then **PX4 Offboard, one loop at a time** (rate → attitude → position), PX4 able to take back instantly. This needs **no silicon bring-up**, and was never previously written down as an option.
- Then first supervised free flight, pilot on sticks, PX4 IO as failsafe.
- #362 — head-to-head control-loop benchmark against PX4 on the same hardware.

## Platform and integrity — runs alongside, never blocks a flight phase

- **Verification tracks that are dark or vacuous:** #405 (Verus has verified nothing since 2026-09-11 — and its 16 crates are all cFS-DNA, so **no flight-path crate has ever been under Verus**), #418, #202/#145 (witness MC/DC never produced on the real flight component), #216 (sigil never wired; release is cosign-only), #253, #265, #303.
- **Gates that can pass vacuously:** #410, #417, #415, #422.
- **CI flow:** #436, #449, #429, #350, #345, #373, #153, #222, #260.
- **Trace hygiene:** #261, #262. `verified` should be **computed** from a named test green on `main`, not hand-promoted.
- #372 — feedback wanted: needs and wishes for relay.

## How a release is closed

1. Every artifact in the release's rivet scope reaches `verified`.
2. **Independent review checkpoint**: the maintainer runs `/code-review ultra`. The loop cannot start it and stops to ask. `FV-RELAY-REVIEW-1xx` blocks readiness until then.
3. Readiness reports ready → **the loop says so and asks**. It never tags on its own. A tag means a signature and a partner pulling the result.
4. Dark or vacuous verification tracks are named in the release notes (`SWREQ-RELAY-VGATE-P04`).

## Releases in flight

- **falcon-v1.139.0 — SHIPPED** 2026-09-18 (`97ee07e`, signed, 20 assets; notes disclose Verus dark and 7 structural gaps).
- **falcon-v1.140.0 — 9/11.** Blocked on `FV-FALCON-FAULT-005` (Phase 0: rotor-out cannot be verified while #270 stands) and `FV-RELAY-REVIEW-140` (maintainer review). **Not tag-ready, and the loop cannot clear either item.**
- **falcon-v1.141.0** — Phases 1–2 (`TRANSPORT-P01`, `OCI-P07`, `ORPHAN-P01`, `SHOWCASE-P01`).
- **falcon-v1.142.0** — platform integrity, alongside.
