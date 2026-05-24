# DO-178C DAL-A — falcon evidence crosswalk

**Status:** v0.12.0 scaffold. This is the first credit-bundle
mapping document in the v1.0 dossier campaign. One domain (DO-178C)
covered here; the other five (ISO 26262, IEC 61508, IEC 62304,
ECSS-Q-ST-80C, EN 50128) follow the same shape.

**Honest scope warning:** This is a *crosswalk*, not a certification
package. It maps the evidence falcon ships today to the DO-178C
Annex A objectives it supports. Gaps are listed explicitly; an
assessor reading this should treat the gaps as the work remaining
before a real DAL-A credit submission.

## What this document is

A two-column traceability table per DO-178C objective:

| Objective | Falcon evidence that supports it |
|---|---|

For each Annex A objective DAL-A requires, the right column lists
the rivet artifact, the formal property, or the test that
demonstrates compliance — or **GAP** if nothing is in place yet.

## Methodology summary

Per DO-178C §11.20, the supplier may take "Formal Methods" credit
under DO-333. Falcon's evidence pipeline composes four DO-333
technique classes:

| Class                              | Tool             | Property pattern |
|------------------------------------|------------------|------------------|
| Deductive verification (FM)        | Verus (SMT/Z3)   | Pre/post-condition + invariant proofs |
| Bounded model checking (FM)        | Kani             | Exhaustive over bounded input domain |
| Robust testing with UB detection   | miri             | UB-free over deterministic + arbitrary inputs |
| (Reserved) Abstract interpretation | MIRAI / Charon — honestly deferred (FV-FALCON-GEO-003) |

Each falcon engine is verified by **at least one** of these classes
per the rivet FV artifacts.

## Annex A — Software Development Process objectives

### Table A-3 — Verification of Outputs of Software Requirements Process

| Obj | Description | Falcon evidence |
|-----|-------------|-----------------|
| A-3.1 | Software high-level requirements comply with system requirements | rivet typed `derives-from` links: every `SWREQ-FALCON-*` carries `links.derives-from: SYSREQ-FALCON-*`; `rivet validate` enforces this |
| A-3.2 | High-level requirements are accurate and consistent | `rivet check` + `rivet validate` (PASS at v0.12.0) |
| A-3.3 | High-level requirements are compatible with target computer | SWREQ-FALCON-WORLD-P01 + bazel cross-compile targets (linux x86_64/aarch64, darwin x86_64/aarch64, windows x86_64; cortex-m7 via synth_compile) |
| A-3.4 | High-level requirements are verifiable | Every SWREQ has a `verifies` link from at least one FV-* artifact; `rivet coverage` reports the topology |
| A-3.5 | High-level requirements conform to standards | **GAP** — no formal SDP / SVP / SCMP document yet; this is v1.0 work |
| A-3.6 | High-level requirements are traceable to system requirements | rivet `derives-from` (same as A-3.1) |
| A-3.7 | Algorithms are accurate | Verified-engine pattern: every safety-critical algorithm has a Verus / Kani harness (relay-hs HS-P06/P07, relay-lc LC-P09/P10/K01-K05, relay-mavlink MAVLINK-P01-P03, relay-nid NID-V01-V03/K01-K05) |

### Table A-4 — Verification of Outputs of Software Design Process

| Obj | Description | Falcon evidence |
|-----|-------------|-----------------|
| A-4.1 | Low-level requirements comply with high-level requirements | rivet `verifies` / `implements` (every SWDD-* targets a SWREQ-*) |
| A-4.2 | Low-level requirements are accurate and consistent | `rivet check` |
| A-4.3 | Low-level requirements are compatible with target computer | cross-compile gate in CI (Test (ubuntu / macos / windows)) |
| A-4.4 | Low-level requirements are verifiable | every SWDD-* has an FV-* targeting it |
| A-4.5 | Low-level requirements conform to standards | **GAP** — coding-standard document deferred to v1.0 |
| A-4.6 | Low-level requirements traceable to high-level requirements | rivet topology |
| A-4.7 | Algorithms are accurate | same as A-3.7 |
| A-4.8 | Software architecture is compatible with high-level requirements | cFS-DNA pattern (detect → relay-sc → cascade) documented in rollout YAML; spar AADL model for the architecture is **GAP** |
| A-4.9 | Software architecture is consistent | **GAP** — pending spar AADL model + WIT regeneration pipeline |
| A-4.10 | Software architecture is compatible with target computer | bazel build proves it across 5 host triples + 1 embedded |
| A-4.11 | Software architecture is verifiable | partial — verified engines compose into the cFS-DNA pattern; full architectural verification = **GAP** until spar wired in |
| A-4.12 | Software architecture conforms to standards | **GAP** — no architectural standards document yet |
| A-4.13 | Software partitioning integrity is confirmed | partial — Wasm Component Model boundary provides static isolation; ARINC 653 / spatial partitioning analysis = **GAP** until spar wired in |

### Table A-5 — Verification of Outputs of Software Coding & Integration Process

| Obj | Description | Falcon evidence |
|-----|-------------|-----------------|
| A-5.1 | Source code complies with low-level requirements | every relay-* engine has Verus contracts (the source-of-truth `src/`) and a plain mirror cargo compiles; Verus discharges the contracts |
| A-5.2 | Source code complies with software architecture | the verified engines compose into the documented cFS-DNA pattern; SITL `falcon-sitl-hover` is the integration evidence |
| A-5.3 | Source code is verifiable | DO-333 §11.20 + the four FM technique classes |
| A-5.4 | Source code conforms to standards | partial — Rust forbids most CWEs at the type level; explicit `#![forbid(unsafe_code)]` on relay-mavlink, relay-nid, relay-hs, relay-lc, relay-sc, relay-ekf; **GAP** = formal Coding Standard document |
| A-5.5 | Source code is traceable to low-level requirements | rivet `verifies` links + the source-tree comment convention (every Verus property is captured in the doc-comment of its enclosing function/struct) |
| A-5.6 | Source code is accurate and consistent | covered by A-3.7 + A-5.1 |
| A-5.7 | Output of integration process is complete and correct | falcon-sitl-hover end-to-end test (17/17 at v0.10, still 17/17 at v0.12); `bazel test //:falcon-cascade-composed` integrates the Wasm-component chain |

### Table A-6 — Testing of Outputs of Integration Process

| Obj | Description | Falcon evidence |
|-----|-------------|-----------------|
| A-6.1 | Executable code complies with high-level requirements | falcon-sitl-hover scenarios trace each SYSREQ-FALCON-* |
| A-6.2 | Executable code is robust with high-level requirements | proptest fuzz on every codec (relay-mavlink, relay-nid); HITL stub-bench negative-controls; miri robust-testing |
| A-6.3 | Executable code complies with low-level requirements | engine-level cargo tests + Verus contracts (`cargo test --workspace` = 365+ tests at v0.11, growing with v0.12) |
| A-6.4 | Executable code is robust with low-level requirements | Kani harnesses + miri (FV-FALCON-GEO-002 / GEO-003 / HITL-001) |
| A-6.5 | Executable code is compatible with target computer | cross-compile gate (5 host triples + cortex-m7); cosign-signed release artifacts published with each tag |

### Table A-7 — Verification of Verification Process Results

| Obj | Description | Falcon evidence |
|-----|-------------|-----------------|
| A-7.1 | Test procedures are correct | every FV-* artifact carries the executable `steps:` field — `bazel test`, `cargo test`, `cargo kani`, `miri` invocations |
| A-7.2 | Test results are correct and discrepancies explained | release.yml uploads `rivet-snapshot-falcon-v*.json` as a release asset; failures fail-stop the CI lane (`Verification gate (rivet-driven)` job) |
| A-7.3 | Test coverage of high-level requirements achieved | `rivet coverage` topology + the explicit per-SYSREQ scenario in falcon-sitl-hover |
| A-7.4 | Test coverage of low-level requirements achieved | Verus + Kani + miri per the per-engine FV-* artifact |
| A-7.5 | Test coverage of software structure (modified condition / decision) achieved | partial — witness MC/DC on Wasm artifacts is the planned tooling; **GAP** until witness is wired |
| A-7.6 | Test coverage of software structure (decision coverage) achieved | partial — same as A-7.5 |
| A-7.7 | Test coverage of software structure (statement coverage) achieved | partial — `cargo llvm-cov` is the natural tool; **GAP** until it's wired in CI |
| A-7.8 | Test coverage of software structure (data coupling and control coupling) achieved | **GAP** — pending spar AADL + WIT regeneration pipeline |
| A-7.9 | Verification of additional code (object code, executable code) | cosign-keyless signing on every release artifact gives the bit-for-bit provenance chain |

## Gap summary (where v1.0 work concentrates)

1. **Process documents** (A-3.5, A-4.5, A-4.12, A-5.4): SDP / SVP / SCMP / Coding Standard / Architectural Standards. Pure documentation work, no tooling needed.
2. **Spar AADL pipeline** (A-4.8 / A-4.9 / A-4.11 / A-4.13 / A-7.8): falcon's WIT files are still hand-authored. Wiring spar → WIT closes the architectural traceability gap.
3. **Structural coverage** (A-7.5 / A-7.6 / A-7.7): witness MC/DC + `cargo llvm-cov` integration into CI. Tooling exists; needs the wiring.
4. **Abstract interpretation** (FV-FALCON-GEO-003): honestly substituted with miri at v0.12; full DO-333 abstract-interpretation class deferred until MIRAI / Charon ship stable-rustc-tracking releases.

## How to read this

For an assessor walking the dossier, the value of this document is the **explicit gap list**. Each gap is named with the objective it would close. v1.0 = every cell either has evidence or has a deliberate, documented deferral. v0.12 = the first row of cells.
