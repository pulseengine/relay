# Release plan — falcon v1.139 → v1.142

*A view, not the source of truth.* Release scope lives in rivet (`release:` on each artifact) and readiness is computed by `scripts/release-readiness.rs`. This page maps every open issue to where it will be worked, so that nothing is invisible to the plan. Approved by the maintainer 2026-09-17.

## How each release is closed

1. Every artifact in the release's rivet scope reaches `verified`: implementation PR at most `implemented`, then a separate code-free promotion PR (two-commit rule, CI-enforced).
2. **Independent review checkpoint** (v1.139, v1.140, v1.141): the maintainer runs `/code-review ultra` on the release candidate. The loop cannot start it. It stops and asks. Every finding is fixed, filed into a named release, or dismissed with a rationale, and `FV-RELAY-REVIEW-1xx` is promoted. Until then readiness reports the release **not ready**.
3. Readiness reports ready → **the loop says so and asks**. It never tags on its own. A tag means a signature and a partner pulling the result.
4. Dark or vacuous verification tracks are listed in the release notes (SWREQ-RELAY-VGATE-P04).

## Why this order

- **v1.139 first and short**: the fixes are merged. What remains is promotion, correcting published claims that users choose components by, and evidence that resolves.
- **v1.140 before v1.141**: the showcase flies a hold and an engine drop, so it cannot pass until the hold is bounded (#403/#434) and rotor-out recovers on the real plant (#398).
- **v1.142 platform work runs alongside**, never blocking a flight release.

## falcon-v1.139.0 — Integrity: honest gates, honest claims

Cuttable in days. Mostly promotion PRs for fixes already merged.

**Merged, awaiting promotion / tag**

- #413 — FAIL-UNSAFE: read_battery_v defaults to 16.0 V — a healthy pack — and nothing in the flight path overrides it — *FV-FALCON-BATT-003 — fix merged in #430; this issue closes WITH the tag*
- #429 — CI jobs wedge in_progress for 30x their normal duration, and the fleet monitor cannot see it (watches queued, not stuck) — *SWREQ-RELAY-FLEET-P01 — wedge alarm + Kani cap merged (#432); the stalling harness itself moved to SWREQ-RELAY-MIXPROOF-P01 in v1.140*
- #436 — The fleet monitor has never measured starvation: gh isn't installed on the light runners, and every green run since 2026-08-07 was an empty result — *SWREQ-RELAY-FLEET-P01 — gh installed on light (#437, in flight); cron cadence continues under CIFLOW-P01*
- #153 — Self-hosted runners lack `gh` (+ Node 20 deprecation): verification-gate PR comment can't post on smithy — *SWREQ-RELAY-FLEET-P01 — verification-gate PR comment gets gh (#437, in flight)*

**Published claims must match what ships — SWREQ-FALCON-CLAIMS-P01**

- #412 — Published OCI components advertise "Formally-verified geometric SO(3)" while wrapping the unverified legacy PID controller
- #411 — #388 is incomplete: the legacy 5-stage cascade still lives in wasm/cm/cascade/ and ships cosign-signed in v1.138.0
- #419 — Documentation integrity: the #388 correction never reached any document that tells someone what to DEPLOY
- #422 — docs/PX4-PARITY-ASSESSMENT.md is 82 versions stale and credits unreachable capability as delivered — it answers the wrong way on the exact question it exists for

**Evidence must resolve — SWREQ-RELAY-EVIDENCE-P01**

- #415 — Verification-artifact integrity sweep: 15 of 293 cite something that does not resolve; 6 verified artifacts execute nothing in CI

## falcon-v1.140.0 — Hold: a position hold that stays bounded

HOLD-P01 first: an hours-long hold is meaningless while a 60 s hold diverges.

**SWREQ-FALCON-HOLD-P01**

- #403 — Horizontal position loop diverges after ~26 s of hold — was hidden under the altitude limit cycle (#396) — *the blocker for flying longer than ~26 s*
- #434 — FlightCore's hold loop is UNSTABLE when position fixes arrive at ≤10 Hz — zero noise, deterministic, reproduces on SimBackend in <1 s (likely mechanism of #403) — *the <1 s oracle: hold loop unstable at <=10 Hz aiding; the fix must make it converge*

**SWREQ-FALCON-ENDURANCE-P01**

- #435 — MockPhysics applies NO torque — the analytic soak's 12 h hold is 1-D altitude on a vehicle that cannot tilt, and ENDURANCE-P01 draws a controller conclusion from it — *move the Tier-1 soak to a plant that can tilt; check horizontal hold*
- #257 — Verification fidelity: audit safety-behavior campaigns for idealized-harness blind spots — *same class as #435: audit campaigns for idealized-harness blind spots*

**FV-FALCON-FAULT-005 (re-verifies SWREQ-FALCON-FAULT-P02 on the gz plant)**

- #398 — Rotor-out recovery does not hold on the gz plant — vehicle descends with OR without ESC telemetry

**SWREQ-RELAY-MIXPROOF-P01 — the MIX-P06 proof stops stalling the Kani gate** (pulled forward from CIFLOW-P01 by maintainer decision, 2026-09-17)

- #429 — `verify_mix_priority_bound` stalled 13 of 42 CI executions (31%), three times on main on 2026-09-17 — *compositional proof: `scale_to_fit` contract + `stub_verified`; verified after 10 consecutive CI executions without a stall*

**Candidates — scoped at the start of v1.140 if HOLD work touches them**

- #270 — gz plant hovers with an attitude limit-cycle (~1 rad/s roll/pitch, motor thrash 0.1↔1.0) — *gz attitude limit-cycle*
- #290 — Notch closed-loop hover is a non-deterministic oracle: chaotically fragile under rotor-line vibration (kernel-agnostic; mechanism unknown) — *notch closed-loop hover is a non-deterministic oracle*

## falcon-v1.141.0 — Flying demo: showcase, wired supervisor, configuration

Needs v1.140's hold and rotor-out recovery.

**SWREQ-FALCON-ORPHAN-P01 + SWREQ-FALCON-CONFIG-P01 + SWREQ-FALCON-SHOWCASE-P01**

- #414 — FlightSupervisor — every failsafe, the mode machine and all 19 pre-arm rows — is absent from the shipped component — *wire FlightSupervisor into the flown path*
- #385 — falcon-flight embeds a simulator and cannot be driven — and it is the only loose .wasm we ship — *falcon-flight embeds a simulator and cannot be driven — wire, park or retire*

**Candidate**

- #277 — Blackbox TickRecord schema v2: log battery samples (v, i) for replay-exact BATTERY-P02 — *blackbox TickRecord v2 with battery samples, for replay-exact battery behaviour in the demo*

## falcon-v1.142.0 — Platform

Independent of the flight releases; must not hold them up.

**SWREQ-RELAY-VGATE-P05 — every cited verification track RUNS**

- #405 — Verus track has verified NOTHING since 2026-09-11 — toolchain cannot find core/std, all 19 proofs fail in 0.1s
- #418 — verus.yml PR trigger is path-filtered on LEAN paths — the Verus gate never runs on a PR that touches a Verus proof
- #410 — The required verification gate has no main backstop, and its scope filter is blind to wasm/ — the shipped components
- #407 — bazel: //:falcon-cascade-coverage fails to fuse — duplicate export cabi_realloc$2 — and is in no workflow
- #417 — All three required gates can pass vacuously for a PR that touches only the shipped wasm components

**SWREQ-RELAY-CIFLOW-P01 — green PRs merge; verdicts not decided by luck**

- #373 — CI: three green PRs sat unmergeable for 3 weeks — strict protection + auto-merge deadlocks, and a merge queue has two known gaps
- #350 — Verification gate has grown to meet its own 90-minute timeout — required check now fails ~half the time
- #345 — Verification gate: 'rivet validate' failed once, unreproducible — intermittent failure in a REQUIRED check

**SWREQ-FALCON-OCI-P03 / SWREQ-FALCON-MATHF32-P06 / SWREQ-FALCON-OPSHELL-P01**

- #330 — Consume pulseengine/wit-bindgen (cabi-realloc-extern): gale-owned arena instead of a per-component one — *wit-bindgen fork (OCI-P03)*
- #303 — Machine-checked FP kernel proof — remaining layers (approximation + argument reduction) — *machine-checked FP kernel proof (MATHF32-P06)*

## Backlog — deliberately NOT scheduled in v1.139–v1.142

Real, but not on this arc's critical path. Re-evaluated when the arc closes, or pulled forward on request (e.g. by jess).

**Hardware bring-up (with jess)**

- #157 — Hardware bring-up round (v1.57+): the 7-item flight-readiness register
- #214 — relay-hal: consolidate + complete the peripheral-abstraction seams (RegBus/Serial/Pwm/Adc/Can) + DroneCAN components for the i.MX RT1176 bring-up
- #177 — Software Bus: provide a no_std shared-memory/ipc_service transport backend behind the stream<T> seam (inter-core)
- #376 — v1.137: publish per-tick reference vectors for the full cascade — jess's on-target differential covers 2 of 5 stages

**Verification depth**

- #4 — End-to-end verification chain: Verus → Rocq → Lean → Kani for all five engines
- #145 — witness MC/DC on the real flight component (falcon_flight_component.wasm)
- #202 — witness MC/DC: harness can't invoke async-lift stream exports (composed pipeline coverage is 0/2713)
- #222 — Release standard: promote manual witness run to a CI gate, add scry
- #216 — Attestation chain: wire sigil (native Ed25519 endorsement) before cosign — release ships cosign-only (step-6 gap, v1.78–v1.81)
- #265 — Prove the ~30 external_body codec/CRC bodies with ordeal (upgrade proptest → certificate)
- #253 — approach: single-source the Verus/Kani engine (generation) + refine the Rocq model to the actual Rust
- #260 — relay-traj Kani harnesses hang CBMC — bound and enroll in the kani.yml matrix

**Traceability structure**

- #261 — rivet hygiene: decompose the architecture layer (5 swarch for 219 swreq)
- #262 — rivet hygiene: backfill release: fields + convert whole-crate verification steps to named tests

**Distribution**

- #289 — release.yml: flight wasm dropped out of the top-level SHA256SUMS.txt at v1.123 (bundle SHA256SUMS still covers it — consistency, not integrity)
- #306 — Roll out signed ghcr OCI publish + wasm.directory to the org's wasm-releasing repos

**Performance**

- #362 — PERF-P01 second half: benchmark PX4's control loop head-to-head on the same hardware
- #8 — Add criterion benchmarks for per-engine throughput (LC, SCH, SC, HS, CFDP)
- #7 — Add tokio-rs/loom harnesses for stream-channel backpressure

**Community**

- #372 — Feedback wanted: needs and wishes for relay — *open call for feedback — informational, not a work item*

---

Coverage check at generation: 49 of 49 open issues mapped, each exactly once. Issues closed while planning: #427 (fixed by #428), #380 and #384 (fixed in falcon-v1.138.0).
