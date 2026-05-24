# IEC 61508 SIL-4 — falcon evidence crosswalk

**Status:** v0.13.0 scaffold. Same shape as
[DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md).

IEC 61508-3 (software requirements for E/E/PE safety-related
systems) at SIL-4 is the highest safety integrity level for the
general industrial / process domain. The technique requirements are
largely a superset of ISO 26262 ASIL-D + DO-178C DAL-A, so most
falcon evidence already in place applies.

## Methodology — IEC 61508-3 Annex A/B technique tables

IEC 61508-3 Tables A.4 / A.5 / B.5 / B.8 specify the techniques
required for SIL-4. The recommendations are HR (Highly
Recommended) or R (Recommended). Falcon's stack covers:

| Table / Technique | Tool / Evidence |
|---|---|
| A.4 Software design — Formal methods (HR for SIL-4) | Verus (SMT/Z3) — discharges LC-P09/P10, HS-P06/P07, NID-V01..V03 contracts |
| A.5 Software architecture — Modelling (HR) | spar AADL (`spar/falcon_*.aadl`) → instance → analyze (EMV2 fault tree) |
| A.5 Software architecture — Defensive programming (HR) | cFS-DNA pattern: detect (HS / LC) → relay-sc RTL → cascade; encoded in `examples/falcon-sitl-hover` scenarios |
| A.5 Software architecture — Failure analysis (HR) | spar EMV2: 7 minimal cut sets, 7 single-point failures, STPA bridge |
| B.5 Software design — Suitable programming language (HR) | Rust + `#![forbid(unsafe_code)]` on all safety crates |
| B.5 Software design — Strongly typed programming language (HR) | Rust type system enforces this at compile time |
| B.5 Software design — Defensive programming (HR) | Verus pre/post-conditions reject pre-condition violations at compile time |
| B.8 Software verification — Static analysis (HR) | Kani + miri |
| B.8 Software verification — Dynamic analysis and testing (HR) | cargo test + proptest fuzz + falcon-sitl-hover + falcon-hitl-rfspoof |
| B.8 Software verification — Structural coverage (HR) | witness `wasm_module_coverage` rule wired; smoke subject green; cascade target **GAP** pending WASI (see FV-FALCON-COV-001) |
| B.8 Software verification — Performance modelling (HR) | spar bin-packing schedulability: 20% utilisation, 80% headroom at 200 Hz on STM32H743 |

## Lifecycle (§7 + §8)

| Clause | Activity | Falcon evidence |
|---|---|---|
| §7.4.2 SW safety requirements | rivet `SWREQ-FALCON-*` typed traceability |
| §7.4.3 SW safety architecture | `spar/falcon_*.aadl` + `artifacts/spar/falcon-quad-analysis.json` |
| §7.4.4 Module / unit design + implementation | every relay-* crate's `src/` (verus) + `plain/` (cargo) |
| §7.4.5 Code verification | cargo test (387 tests), Kani, miri, witness (where module is in scope) |
| §7.4.6 Integration testing | falcon-sitl-hover (17 scenarios) + falcon-hitl-rfspoof (10 tests) |
| §7.4.7 Validation | end-to-end SITL: cFS-DNA RTL fires on EKF fault, geofence breach, etc.; live-bench HITL validation is **GAP** (HackRfBench + MavlinkBench are *drivers*, the bench-run is the user's job) |
| §7.4.8 Modification | every change passes through rivet validate + clean-room verification (Explore subagent, cold context) |
| §7.4.9 Verification of SW safety lifecycle | `Verification gate (rivet-driven)` GitHub Actions job; release.yml uploads rivet snapshot per release |

## Gap summary

1. **SIL-4 process documents** — Safety Plan, Verification Plan, Software Configuration Management Plan. v1.0 documentation.
2. **Diverse software** (§7.4.4 Note 4, R for SIL-4) — the relay-stack is single-version; redundant/diverse N-version implementations are out of scope.
3. **Cascade-target witness coverage** — same as ISO 26262 gap (#3) and DO-178C gap (#3); same WASI cause.
4. **Tool qualification per IEC 61508-3 §7.4.4.7** — Verus/Kani/miri/witness/spar — T3 by classification; T2 needs a separate qualification record. v1.0 work.
5. **Other dual-tree refactors** — same gap as ISO 26262 (#5).
