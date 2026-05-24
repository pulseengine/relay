# ISO 26262 ASIL-D — falcon evidence crosswalk

**Status:** v0.13.0 scaffold. Same shape as
[DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md). One row per
ISO 26262 Part 6 work product, mapped to falcon evidence or **GAP**.

ISO 26262-6 (software development) is the closest analogue to
DO-178C for automotive applications. Falcon's relay-stack engines
target ASIL-D — the strictest integrity level.

## Methodology summary

| ISO 26262 method               | Falcon tool             | Same as DO-178C |
|--------------------------------|-------------------------|-----------------|
| Formal verification (§9 RT.1f) | Verus (SMT/Z3)          | DO-333 §11.20 deductive FM |
| Static code analysis (§9 RT.1d) | Kani (bounded MC)       | DO-333 §11.20 bounded model checking |
| Software unit testing (§9 RT.1c) | cargo test + proptest | DO-178C §6.4.4 unit testing |
| Robust testing with UB (§9 RT.1d) | miri                  | DO-178C §11.20 (substituted, see FV-FALCON-GEO-003) |
| Structural coverage (§9 RT.1e) | witness (MC/DC on Wasm) | DO-178C A-7.5 |
| Architecture analysis (§7 RT.1a-d) | spar (EMV2 fault tree) | DO-178C A-4.8 |

## Part 6 — Product development at the software level

### §7 Initiation / SW safety requirements

| Work product | Falcon evidence |
|---|---|
| §7.4.1 SW safety requirements | rivet `SWREQ-FALCON-*` (32 entries; every SWREQ traces to a SYSREQ via `derives-from`) |
| §7.4.2 SW safety requirements verification | `rivet validate` + `rivet check` (PASS at v0.13.0) |

### §8 Software architectural design

| Work product | Falcon evidence |
|---|---|
| §8.4.2 SW architectural design specification | `spar/falcon_system.aadl` + `spar/falcon_cascade.aadl` + `spar/falcon_types.aadl` (v0.13); spar instance + render produces `artifacts/spar/falcon-quad-architecture.svg` |
| §8.4.3 Safety analysis (DFA) | `spar analyze` EMV2 fault tree: **7 minimal cut sets, 7 single-point failures** (each controller); STPA bridge: 1 hazard + 2 loss scenarios → `artifacts/spar/falcon-quad-analysis.json` |
| §8.4.5 Verification of the SW architectural design | spar bin-packing schedulability (per-thread WCET ≤ 1 ms, period 5 ms → 20% utilisation, 80% headroom) — same JSON |

### §9 Software unit design and implementation

| Work product | Falcon evidence |
|---|---|
| §9.4.2 SW unit design specification | every relay-* crate has Verus-annotated `src/` (source of truth) + plain mirror for cargo (relay-hs, relay-lc, relay-nid follow this pattern; rest TBD per v1.0) |
| §9.4.3 Implementation | `cargo build --workspace` |
| §9.4.4 Verification methods (Table 6) | Verus (RT.1f formal) + Kani (RT.1d bounded MC) + miri (RT.1d robust) + cargo test + proptest |

### §10 Software unit verification

| Work product | Falcon evidence |
|---|---|
| §10.4.2 SW unit verification methods (Table 8) | cargo unit tests (387 at v0.12), Kani LC-K01..K05 / NID-K01..K05, miri (13 tests UB-free), proptest fuzz |
| §10.4.3 Coverage at SW unit level (MC/DC) | witness `wasm_module_coverage` rule wired (`//:geofence-subject-coverage` LCOV pipeline green; cascade target `tags=["manual"]` until WASI gap closed — see FV-FALCON-COV-001) |

### §11 Software integration and verification

| Work product | Falcon evidence |
|---|---|
| §11.4.2 SW integration | `bazel build //:falcon-cascade-fused` (meld_fuse composes 5 controllers + cascade wrapper into one core module) |
| §11.4.3 Integration testing | `cargo test -p falcon-sitl-hover` (8 SITL scenarios, 17/17 passing) |
| §11.4.4 SW integration verification | `cargo test -p falcon-hitl-rfspoof` (10 tests across stub + mavlink backends); v0.12 MavlinkBench wires the verified chain end-to-end against a real MAVLink telemetry source |

## Gap summary

1. **ASIL-D process documents** (Safety Plan, SW Development Plan, SW Verification Plan) — pure documentation work, none yet.
2. **Coding guidelines compliance** (§5.5.3) — Rust forbids most CWEs at the type level and `#![forbid(unsafe_code)]` covers the falcon stack, but a formal Coding Guidelines document tying these to MISRA-C:2012-style rule numbers does not exist yet.
3. **Cascade-target witness coverage** — wasi:io@0.2.6 imports in the Rust-compiled cascade block the witness embedded runtime; closing this needs `--harness <wasmtime+WASI>` mode or a WASI-free Rust core-module build path.
4. **Tool qualification** (ISO 26262-8 §11) — Verus/Kani/miri/witness/spar are TCL3 by classification (output influences safety, no detection); a formal Tool Qualification document for each is v1.0 work.
5. **Other dual-tree refactors** — relay-sc, relay-hk, relay-sch, relay-cs, relay-ds still plain-only; same situation relay-nid was in before v0.11.
