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

- **#270 — the attitude limit cycle is the root defect.** Filed 2026-07-14 at ~1 rad/s; measured 2026-09-23 at **~2.5 rad/s RMS** with motor commands thrashing saturation-to-saturation. The 1→2 rad/s FDI gate widening used to work around it **has been consumed**.
- **#398 — rotor-out does not recover on the gz plant.** Downstream of #270: the FDI gate is shut ~34% of pre-kill ticks (measured on the gate's own filtered signal), so detectability is decided by which side of the gate the failure lands on (5/5: gate open ⇒ isolated in one tick; gate shut ⇒ 180° inversion). 12 trials: 11 FAIL / 1 PASS. `FV-FALCON-FAULT-003` was demoted `verified`→`implemented` when its own falsification clause was met (#479).
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
- **falcon-v1.140.0 "Hold" — SHIPPED** 2026-10-09 12:30Z (annotated tag `7fe4e59` → commit `efc0dbf`; 20 assets, cosign-signed `SHA256SUMS`, six components at `ghcr.io/pulseengine/falcon/<stage>:1.140.0`). 9/9 artifacts verified. All ten per-commit tracks green on the released commit, and the nightly soak passed **on that commit itself** (final horizontal 0.010–0.030 m, peak 0.019–0.072 m against bars of 1.0 m / 2.0 m).
  - **The review checkpoint was DE-SCOPED by maintainer decision, 2026-10-09** — the explicit exception gate rule 2 above allows, exercised. `FV-RELAY-REVIEW-140` moved to v1.141. **falcon-v1.140.0 therefore ships without an independent review.** Logged rather than softened: the equivalent gate on v1.139 returned nine findings, including a `workflow_dispatch` input interpolated into a `run:` block on a self-hosted runner — **that precedent was surfaced after the decision, not before it**, so it did not inform the choice and the call may be revised now it is known.
  - **Rule 4 applies at tag time:** falcon-v1.140.0's release notes must state that it shipped with no independent review, the same treatment v1.139 gave its dark Verus track.
  - The review's **range is pinned** to `falcon-v1.139.0..falcon-v1.140.0` instead of inheriting the family's "previous tag..HEAD". Without that pin, v1.141's review would start at `falcon-v1.140.0` and the v1.140 diff would be covered by no review at all. The two artifacts still tile the history. Deferred by one release, not removed.
  - **Scope moved out, logged:** `FV-FALCON-FAULT-005` → v1.141 on 2026-09-23. It cannot be verified while #270 stands (the FDI gate is shut ~34% of pre-kill ticks), and the ring's primary cause is open. v1.140's own theme — the hold — is done and measured (estimator, endurance, battery, fleet, mixproof). Holding a finished release hostage to an open control investigation is the scope error, not the fix.
- **falcon-v1.141.0 "The verified seam"** — Phases 0–1, **narrowed** (#592, #604, and 2026-11-04 below): `SWREQ-FALCON-PREARM-P04`, `SWREQ-FALCON-SUPV-P02`, `SWREQ-RELAY-VGATE-P06`, `FV-RELAY-REVIEW-141`, **and `FV-RELAY-REVIEW-140`** (de-scoped from v1.140, above). `rivet query '(= release "falcon-v1.141.0")'` is the scope; this list mirrors it.
  - **HELD, 2026-10-27.** The review found the published `@0.12.0` supervised seam (#571) not shippable — 15 findings, 7 blockers, tracked in #609. Maintainer decision: fix the seam, then tag. The review ran as a local single-session `/code-review max` over `falcon-v1.140.0...21c9472` because `/code-review ultra` was unavailable over Remote Control; the release notes must say so.
  - **`SWREQ-FALCON-TRANSPORT-P01` moved to v1.142, 2026-11-04 (maintainer decision).** Its own falsification clause still holds: binding v2 (#614) has no State frame, so a host that speaks only the binding cannot see the mode or why an arm was refused. v1.141 ships binding v2 as it stands, with `FV-FALCON-TRANSPORT-002` and `-004` verified (#630); P01 moves together with the new `SWREQ-FALCON-TRANSPORT-P02` (the State frame), as ONE further wire revision whose layout is announced to integrators before it lands.
  - A v1.141 tag publishes the `@0.12.0` interface. It does not ship until **jess#167** has an answer on it, including the calibration record #609 adds to `supervisor-config`.
- **falcon-v1.142.0 RE-PLANNED, 2026-11-07 (maintainer): "Cleanup + keep promises".** Close shipped issues against v1.141 with evidence; re-scope every unverified requirement by one rule (#655 promoted 103, #657 moved 61 to v1.143); run the never-run artifacts (#601); the #415/#529 integrity sweeps; and only the two items announced to integrators, `SWREQ-FALCON-TRANSPORT-P01` + `SWREQ-FALCON-TRANSPORT-P02` (State frame) and known limit A. **Known limit A was deferred to v1.143 on 2026-11-08** after three designs were rejected on measurement (#638). Everything else listed for v1.142 below, and the Platform plan, moves to v1.143. `rivet query '(= release "falcon-v1.142.0")'` is the scope.
- **falcon-v1.142.0 "Components" (as planned before 2026-11-07, superseded above)** — Phase 2: `OCI-P07`, `ORPHAN-P01`, `CONFIG-P01`, `SHOWCASE-P01` (all moved here from v1.141); `SWREQ-FALCON-RATE-P05` (the ring) with `FV-FALCON-FAULT-005` behind it (#592); `SWREQ-FALCON-TRANSPORT-P01` + `SWREQ-FALCON-TRANSPORT-P02` (the binding returns the vehicle state; moved / added 2026-11-04); `SWREQ-FALCON-SUPERVISOR-P01` (#604) — which holds the `@0.12.0` release conditions, so its clauses are checked against v1.141's seam even though it is scoped here. Plus the platform-integrity items already scoped here, which ride alongside and never block.
  - **`SWREQ-RELAY-MANIFEST-P01` (added 2026-11-04, maintainer decision): the inventory manifest, slice 1.** Every thing relay ships or promises is listed once, as a rivet artifact type. A generated lock of what is on disk is diffed in CI, and three checks (ENUMERATE, PUBLISH-LISTS, LOCK-DIFF) each carry a negative control. The rollout is report-only: a check starts failing builds after one green tag, except ENUMERATE and LOCK-DIFF, which fail from day one. The lock ships as a release asset and feeds the review gate. The cross-repo part comes later: the shared schema in pulseengine.eu, with warte pulling each repo's manifest from its latest release; relay does not wait for it. Evidence: the 2026-11-04 inventory found the publish set hand-written in five places and the pre-arm row count stale in prose. Its cheap drifts are fixed as ordinary PRs before the v1.141 tag. relay-bp S1+S2 (v1.143) is unaffected.
- **falcon-v1.143.0 "The board means something"** — planned around `SWREQ-RELAY-BOARD-P01`, which moved to v1.144 with S1. **S1, owner decision 2026-11-19:** the ten proposed/draft items with no code leave v1.143. `CONFIG-P01`, `MANIFEST-P01` and `VGATE-P05` go to v1.145; `BOARD-P01` to v1.144; `OPSHELL-P01`, `SHOWCASE-P01`, `CIFLOW-P01`, `OCI-P03`, `OCI-P07` and `MATHF32-P06` to backlog (no release). v1.143 is now the implemented set (64) plus `FV`/`SWREQ` already verified, and is cut when each is promoted or moved with a reason. Also decided that day: S3 (the soak judges `RATE-P05`'s own bars) and S4 (the supervisor refuses more than 16 waypoints whole and keeps a 2 m margin inside the geofence). relay-bp S1 and S2 were slotted here and **moved to v1.145 on 2026-11-18** (below). One release spent on issue hygiene, because a tracker nobody trusts corrupts every planning decision made from it.
  - Measured 2026-09-24, 57 open: **nothing older than 6 months** (oldest 163 d, #4), **46 of 57 named in this plan**, **11 invisible to it**, **24 untouched 30+ days**, 6 untouched 100+.
  - So the board is not rotting the way a raw count suggests — most "stale" issues are *parked by design with a phase that owns them*. The real defects are narrower: the parked set has **no expiry**, the invisible set is invisible to every later decision, and some issues are **probably already fixed** (heavy cross-reference traffic, no closure: #262 has 16, #153 has 11, #8 has 9).
  - Every issue gets exactly one disposition; closure as fixed cites a **tagged release, never a merge**; re-verify before closing; audit the `closes #N` cascade that already falsely closed #413. Ships `scripts/board-health.rs` so decay is a number in CI rather than a feeling — without it this is a sweep that decays the week after.
  - **relay-bp S1 + S2 (decided for v1.143 on 2026-11-03, MOVED to v1.145 on 2026-11-18; Refs #626).** `SWREQ-RELAY-BP-P01`..`P07` (S1: codec, canonical CBOR subset, CRC types 1 and 2, EIDs, fragment rejection, external reference vectors) and `P10`..`P14` (S2: bounded store, lifetime/age, deterministic eviction, hop count, duplicate suppression). Status reports are deferred (`P15`). All `proposed` until the implementation PRs land; the two-commit rule applies (implementation PR sets at most `implemented`, promotion is a separate code-free `verify/` PR).
  - **Orphan dispositions (maintainer decision, 2026-11-13, `SWREQ-FALCON-ORPHAN-P01`).** RETIRED in v1.143: `relay-arm` (falcon-core spin-up and keep_warm supersede it; falcon-sitl-gz keeps `geo-hover`/`mission` on a bench-local spin-up gate that reproduces the sequencer tick for tick, and drops the `arming`, `arming-ungated` and `alt-rate` scenarios) and `relay-modextra` (POI yaw carried forward as backlog `SWREQ-FALCON-POIYAW-P01` in falcon-core). PARKED: `relay-traj` and precision landing. WIRED in v1.144: `relay-rc` and `relay-sensvote`; `relay-avoid`'s speed limiter with a forward rangefinder moved to v1.145 on 2026-11-18, and `ORPHAN-P01` closes with it. `relay-mix-multi` waits for a hex/octo Gazebo level.
- **Next releases, planned 2026-11-18** (drafted by Fable 5.2 from the issue tracker, the rivet scope, wohl, warte and the North Star; checked cold by a second Fable 5.2 pass whose 11 corrections were applied; maintainer decisions D1, D2, D4, D5 below). Every slice is built and tested at full scale locally first (Gazebo, campaigns, Renode, cross-builds, consumer builds on the build host), and goes to CI, merge and tag only after that. Every trial count names its backend: the dispersion campaigns fly the analytic `SimBackend` (`examples/falcon-sitl-gz/src/campaign.rs`), not Gazebo. New tools, checkers and gates are Rust (owner rule 2026-11-18: no new Python), as WASI components where they can run as one.
- **falcon-v1.144.0 "A pilot can take the sticks, and jess can check every tick"** — `rivet query '(= release "falcon-v1.144.0")'` is the scope:
  - `SWREQ-FALCON-REFVEC-P01` (#376): per-tick reference vectors at the #575 partition seam, published per release. First slice, never cut: it is jess's only open ask (jess#313 tier 1.2) and has slipped from v1.143; the v1.143 notes and jess#167 say so, with the stream layout.
  - `SWREQ-FALCON-MAVLINK-P08`: fix #675 (truncated payloads) and re-verify `FV-FALCON-CMDLONG-001`, then MAVLink2 signing (decided 2026-11-13). Vectors generated once with pymavlink and committed; Rust tests and a Rust MAVLink oracle; no Python in CI.
  - `SWREQ-FALCON-SIM-P05`: Gazebo X500 V2 level 1 (decided 2026-11-13), with a broken-model control and a falcon-quad baseline tolerance.
  - `SWREQ-FALCON-RC-P04`: relay-rc wired, with the pilot modes in `relay_fsm::Mode` (it has none today; RC-P01 is verified only on the leaf crate) and an RC-loss failsafe with a failsafe-disabled control. Two PRs.
  - `SWREQ-FALCON-SENSVOTE-P02`: relay-sensvote wired; starts on falcon-quad with three simulated IMUs, so it does not wait for the X500 world.
  - `SWREQ-RELAY-XTARGET-P01`: every `no_std` lib checked for `thumbv7em-none-eabihf` and `riscv32imac-unknown-none-elf` in CI (57 of 64 today; 7 named std crates excluded). Build evidence only.
  - `SWREQ-RELAY-CONSUMER-P01`: a relay job builds and tests wohl's ten crates and warte's decoder against each candidate. wohl and warte consume relay by **git dependency on the falcon-v1.x tag** plus the manifest lock (D4, 2026-11-18); their re-pin PRs after the tag are owned in those repos.
  - `SWREQ-SEC-P08`: relay-sec seals and opens a relay-ccsds frame at `no_std` for other consumers (wohl#49; warte#41 could reuse it).
  - Known limit A only if it slips from v1.143; it never holds the tag. Cut order: SEC-P08, then the Gazebo X500 part of RC-P04. REFVEC-P01 is never cut.
- **North Star bets for v1.144, 2026-11-19 night (work in progress; drafted by Fable 5.2, checked cold by a second Fable 5.2 pass against `north-star.toml` only, whose 9 revisions were applied).** The owner ruled the same night that North Star plans go straight to work, and that owner-only items queue as one decision. The three new items are rivet `draft`s in v1.144, and each names its objective. Nothing existing moved. Moving the cut-order items and revising D5 are in the queued decision. Measured on the shipped `falcon-cascade-v1.142.wasm` under varve layer 2026.11.16:
  - synth 0.90.0 lowers the flight core for `cortex-m7dp`: 37 of 37 functions, a 125,302 B ELF that has never been executed. `cortex-m7` reaches 33 of 37 (`controller#step` declined) and rv32imac 5 of 37.
  - meld fuses it with 0 imports.
  - loom drops the attestation (loom#535).
  - `wsc show-chain` cannot read meld's attestation (sigil#396), and `verify-chain` accepts a modified component (sigil#395).
  - kiln refuses the component because of its type-only import (kiln#697).
  - witness instruments 632 branches but reconstructs 0 decisions without DWARF.
  - The Renode ELF today is the native rustc build. jess#315's silicon run used the five legacy stage components, not this core.
  - **B1, `SWREQ-RELAY-CHAIN-P01` (O1, reach only).** CI lowers the candidate through meld, loom and synth, publishes the ELF and per-target reach, and emits `north-star-input.O1.json` (seam pulseengine.eu#393) with execution match and WCET as `ran: false`. Fails only on the `cortex-m7dp` target, with a declined-function control. v1.145 half: the lowered ELF replays `REFVEC-P01`'s control-output stream in Renode for 2000 ticks × 3 scenarios with 0 bit differences, with a 1-ULP mutation of the stream and one of the component as controls. Reported as an emulator input, not the O1 on-target number. Kill: if `step` differs from wasmtime on M7 and synth has no fix 14 days after the report, narrow to the estimator partition. If no `step` call with a real sensor frame runs in Renode by 2026-12-18, the match moves to v1.146 and the notes say so.
  - **B3, `SWREQ-FALCON-PLANT-P01` (input to O1, relay#705).** The plant becomes a library with `reset(seed)` / `step(motor_words, dt)` on wire v3, for jess's lockstep HIL rung (jess#317 B4). Counted only when jess's harness steps it. Controls: same seed means same bytes; a mutated mass or dt changes them. Kill: if native and wasm32 builds disagree, it ships native-only and says so.
  - **B4, `SWREQ-RELAY-LEDGER-P01` (O3, O5).** A shrink-only ledger of 69 `external_body` and 125 `kani::assume` (an added entry is red), plus a Rocq/Lean kernel axiom audit. Relay has none today and does not count under O3 until both checks are required. For O5, each checker counts only with a failing native-vs-wasip2 equivalence test in CI, run under kiln and signed. Until kiln#697-class gaps close, O5 reports 0 for relay. Kill: no kiln run by 2026-12-18 means the gate runs under wasmtime and O5 stays 0.
  - **B2 (O2, v1.145, the decided evidence bundle v0) gains MC/DC**, because witness 0.46.0 already reads this component. The indicator stays at 1 of 2 consumers while any leg reports `counts: false`, which the attestation leg will do until sigil#395 and sigil#396 are fixed. Controls run in CI: four removals, a swapped component, a bit flipped after attestation, decisions > 0, the run happened, and the report bound to the shipped digest. Kill: 0 reconstructed decisions on a DWARF sibling by 2027-01-08 means v0 ships without MC/DC, and the indicator does not move.
  - **B5 (no objective indicator; serves "from architecture model" in the North Star sentence, starts v1.145).** spar generates the flight `types` interface (28 hand-written records) from AADL with fields, gated both ways. It needs the spar pin moved off v0.18.0 (0.49.0 drifts the three gated WITs by casing) and spar enum/option fields. Kill: no spar enum/option support by 2027-01-29 means records only, behind a named waiver, and the 28 → 0 target is not met.
  - The O4 objective has no bet.
- **falcon-v1.145.0 "Not one board, not one consumer"** (artifacts authored when v1.144 is underway; `ORPHAN-P01` and relay-bp S1+S2 are already slotted here):
  - Forward rangefinder + `relay-avoid` speed limiter on the X500 world (**moved from v1.144, D2, 2026-11-18**); `ORPHAN-P01` closes with it.
  - `SWREQ-FALCON-CONFIG-P01`: airframe, sensor mounts and bus selection as data through the existing `configure-calibration` / `vehicle-config` records; no board name in relay (moved from v1.143 by the S1 decision, 2026-11-19).
  - RegBus consolidation into relay-hal (#214).
  - **Relay's own second board (D5, 2026-11-18): STM32H743 in Renode**, running the native rustc ELF of `embedded/falcon-cortex-m` (not the synth ELF) for 2000 ticks, bit-identical to the REFVEC-P01 vectors. Filed as emulator portability evidence; it does not count toward North Star O1. Prerequisite: install Renode on the build host.
  - #253 mirrors for the engines consumers use: relay-ccsds, relay-lc (including `wp_decide`), relay-sch, relay-hk, relay-ds; trust wording for relay-lc, relay-to and relay-ccsds.
  - #601 + #415 with the verification gate ported from `scripts/run-falcon-verification.py` to Rust under an equivalence test (same verdict per artifact over all 293); dispositions capped at 10 and listed for the owner.
  - Evidence bundle v0 (North Star O2): a Rust builder and a Rust verifier built as a `wasm32-wasip2` component, with controls for a deleted file, an altered component and a present-but-failing MC/DC report. MC/DC itself is a v1.146 candidate (needs witness to read component modules).
  - `SWREQ-RELAY-MANIFEST-P01` slice 1 (warte pulls it; moved from v1.143, S1) and `SWREQ-RELAY-VGATE-P05` with the gate port.
  - relay-bp S1 + S2, `P01`..`P07` and `P10`..`P14` (**moved from v1.143, D1, 2026-11-18**).
  - Hex level on native SITL with `relay-mix-multi` (no requester; first to cut).
- **falcon-v1.146.0** — relay-bp for warte: `P20`, `P30` + `P32`, `P40` + `P41` and `SYSREQ-RELAY-BP-001` (**moved from v1.144, D1, 2026-11-18**).
- **relay-bp — later, unslotted** (drafts, no `release:`, so they do not block any readiness number): contact-plan earliest-arrival routing (`P21`, `P22`), status reports (`P15`), the UDP convergence-layer binding (`P31`), BPSec RFC 9172/9173 (`P42`), fragmentation (`P50`) and the CCSDS 734.20 profile claim (`P51`).

### relay-bp decisions, 2026-11-03 (Refs #626)

1. **Replay protection on the DTN path** uses a DEDICATED relay-sec channel (own channel_id and security association) for bundled traffic, with a window sized to the backlog. The receiver keeps the old SA until the bundle lifetime expires, which covers rekey-on-reboot. (`P41`.)
2. **Primary-block integrity.** Source EID, creation timestamp and sequence number (and the destination) are bound INSIDE the sealed payload; the receiver rejects a bundle whose primary block does not match. (`P40`.)
3. **Profile: RFC 9171 BPv7, no fragmentation.** The node never fragments and explicitly rejects fragments (`P07`). Relay does NOT claim CCSDS 734.20 profile conformance until a fragmentation slice exists: that profile requires forwarding and delivering bundles up to 10 MiB, while a datagram-only link (UDP CLA, 65,507 B per datagram) cannot carry them without fragmentation.
4. **Slots** as decided then: v1.143 = S1 + S2, v1.144 = `P20` + CLA seam core + sealing profile, everything else later. **Revised 2026-11-18 (D1): S1 + S2 in v1.145, `P20`/`P30`/`P32`/`P40`/`P41` in v1.146**, because no relay-bp crate exists on main and v1.144 carries the flight kit.

Design rules that go with them: a fresh fixed-capacity `no_std` engine in relay-cfdp style (Verus source stripped to plain, Kani harnesses in a `kani_proofs.rs` sibling, crate added to the `kani.yml` matrix); `hardy-bpv7`, `bp7`, `minicbor` and `crc` appear ONLY as dev-dependency differential oracles. External vectors per the research notes: Wireshark bpv7 captures are downloaded in CI, not vendored, until their licence is checked; RFC 9173 A.1-A.4 carry an expected strict-mode verdict of "reject" and A.3 is marked disputed. The new artifacts are tagged `falcon`; the verification gate runs only falcon-tagged artifacts (#601), so every `FV-`/`SV-` artifact written for them must carry the tag too.
- **Phases 3–5** (silicon/gale, the safety layer, evidence on a vehicle) are not yet numbered. They get a release when Phase 2 lands and `meld --pack-rebase` has answered whether components fit real RAM.
