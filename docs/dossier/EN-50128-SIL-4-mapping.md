# EN 50128 SIL-4 — falcon evidence crosswalk

**Status:** v0.13.0 scaffold. Same shape as
[DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md).

EN 50128 (railway applications — Software for railway control and
protection systems) at SIL-4 is the highest safety integrity level
for rail. The technique requirements have a one-to-one mapping with
IEC 61508-3 SIL-4 in most cases — EN 50128 is the rail-domain
adaptation of IEC 61508. Falcon evidence applies as-is.

## EN 50128:2011 Table A.4 — Software design and implementation

EN 50128 names "Highly Recommended" (HR) and "Recommended" (R)
techniques per SIL level. For SIL-4:

| Technique | EN 50128 ref | Falcon evidence |
|---|---|---|
| Formal Methods | A.4 / D.28 | Verus (SMT/Z3) — `relay-{hs,lc,nid,sc,…}` contracts |
| Modelling (Semi-Formal Methods) | A.4 / D.51 | spar AADL — `spar/falcon_*.aadl`; render in `artifacts/spar/falcon-quad-architecture.svg` |
| Structured Methodology | A.4 / D.59 | rivet typed traceability (SYSREQ → SWREQ → SWDD → FV chains; `rivet validate` enforces) |
| Defensive Programming | A.4 / D.15 | Verus `requires`/`ensures` reject pre/post violations at proof time; cFS-DNA RTL pattern catches runtime drift |
| Strongly Typed Programming Language | A.4 / D.60 | Rust + `#![forbid(unsafe_code)]` |
| Use of Validated Translators (Compilers) | A.4 / D.61 | rustc stable + LLVM; cosign-signed release artifacts pin build provenance — formal translator qualification per T1/T2/T3 is **GAP** (v1.0 work) |

## EN 50128 Table A.5 — Software verification

| Technique | EN 50128 ref | Falcon evidence |
|---|---|---|
| Formal Proof | A.5 / D.29 | Verus + Kani per engine |
| Static Analysis | A.5 / D.55 | Kani + miri |
| Dynamic Analysis & Testing | A.5 / D.18 | cargo test (387) + proptest fuzz + falcon-sitl-hover (17 scenarios) + falcon-hitl-rfspoof (10) |
| Software Module Testing | A.5 / D.50 | cargo unit tests + Verus per-function contracts |
| Coverage (Statement + Branch + Compound + MC/DC) | A.5 / D.43 | witness `wasm_module_coverage` wired; LCOV pipeline green on demo subject; cascade-run **GAP** by WASI (FV-FALCON-COV-001); per-engine statement coverage via `cargo llvm-cov` is **GAP** |
| Equivalence Classes & Boundary Value Tests | A.5 / D.20 | Kani harnesses LC-K01..K05 + NID-K01..K05 are equivalence-class enumerations over bounded input domains |
| Performance Modelling | A.5 / D.45 | spar bin-packing — 20% utilisation at 200 Hz on STM32H743 |

## EN 50128 lifecycle activities

| Phase | EN 50128 ref | Falcon evidence |
|---|---|---|
| Planning | §6 | docs/dossier (this directory) + rivet rollout YAML |
| Software Requirements | §7.2 | rivet SYSREQ / SWREQ (32 SWREQs at v0.13) |
| Software Architecture & Design | §7.3 | spar AADL; rivet SWDD-FALCON-* |
| Software Component Design | §7.4 | every relay-* crate's `src/` |
| Component Implementation & Testing | §7.5 | cargo test + Verus + Kani + miri + proptest |
| Software Integration Testing | §7.6 | bazel build //:falcon-cascade-fused + falcon-sitl-hover |
| Overall Software Testing / Final Validation | §7.7 | falcon-hitl-rfspoof + clean-room verification per release |
| Software Deployment | §7.8 | release.yml: 5 host triples + cortex-m7 ELF, cosign-signed, rivet-snapshot uploaded |
| Software Maintenance | §9 | rivet `status: in-progress` / `blocked` tracking; GitHub Issues |

## Gap summary

1. **EN 50128 plans** — Software Quality Assurance Plan, Software Configuration Management Plan, Software Verification Plan, Software Validation Plan — all v1.0 documentation work.
2. **T1 / T2 / T3 tool classification per tool** (§6.7.4) — falcon's toolchain (rustc, cargo, bazel, Verus, Kani, miri, witness, spar, meld, loom, synth) needs an explicit T-level assignment + qualification document per tool. v1.0.
3. **Cascade-target witness coverage** — same WASI gap as the other four domains.
4. **Diversity / N-version programming** (D.16, R for SIL-4) — out of scope.
5. **Failure analysis (FMEA / FTA)** (D.26, HR) — spar EMV2 fault tree covers FTA partially (7 minimal cut sets, 7 single-point failures); FMEA-shape document would be v1.0.
