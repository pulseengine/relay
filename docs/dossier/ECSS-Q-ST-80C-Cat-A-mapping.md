# ECSS-Q-ST-80C Cat A — falcon evidence crosswalk

**Status:** v0.13.0 scaffold. Same shape as
[DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md).

ECSS-Q-ST-80C (Software product assurance) Category A applies to
space software whose failure could cause loss of mission or
catastrophic consequences. Falcon's "spacecraft heritage" comes
from the NASA cFS DNA inside relay-sc / relay-hs / relay-hk /
relay-sch — the verified Rust replacement for that exact set of
cFS apps.

## Mapping note — ECSS heritage in the falcon design

The relay-sc / relay-hs / relay-hk / relay-sch crate names are not
arbitrary. They are direct verified-Rust analogues of:

| ECSS / cFS function | Relay crate | What's verified |
|---|---|---|
| Command Storage (cFS SC) | `relay-sc` | RTS dispatch, ATS time-tagging |
| Health & Safety monitor (cFS HS) | `relay-hs` | HS-P06/P07 latch + EkfHealthMonitor |
| Housekeeping (cFS HK) | `relay-hk` | telemetry aggregation |
| Scheduler (cFS SCH) | `relay-sch` | message-bus dispatch |

This makes the ECSS-Q-ST-80C mapping the *natural* fit — the rest
of the dossier crosswalks adapt evidence sideways; this one applies
it head-on.

## §5 — Process requirements (selected)

| §  | Activity | Falcon evidence |
|----|----------|-----------------|
| §5.4.2 SW design | spar AADL + Verus-annotated `src/` per engine |
| §5.4.3 SW implementation | cargo build --workspace; bazel build //:falcon-cascade-fused; synth_compile to Cortex-M7 ELF |
| §5.4.4 SW validation | falcon-sitl-hover end-to-end SITL (17 scenarios across step / hover / mission / fault / untethered / geofence) |
| §5.4.5 SW V&V — formal proofs (Cat A: required) | Verus contracts on relay-sc / relay-hs / relay-lc / relay-nid; `bazel test //:relay_*_verus_test` |
| §5.4.6 Testing | cargo test (387 tests) + proptest fuzz + Kani + miri + witness |
| §5.5 Configuration management | git + signed tags + cosign-signed release artifacts |
| §5.6 Problem reporting | GitHub Issues; rivet artifact status tracking |

## Category A–specific (vs Cat B/C/D)

| Cat A requirement | Falcon evidence |
|---|---|
| Independent V&V (§5.4.7 Cat A: required) | Clean-room verification: Explore subagent cold-context audit of each release (8/8 confirmed at v0.12; will repeat at v0.13) |
| Formal methods (§5.4.5 Cat A: required) | Verus + Kani; coverage table in `FV-FALCON-*.yaml` |
| Tool qualification (§5.4.8) | rules_verus / rules_wasm_component / rules_lean / rules_rocq_rust ship versioned MODULE.bazel pins; spar / witness / kani-verifier installed via deterministic cargo install — formal Tool Qualification documents are **GAP** |
| Code coverage (§5.4.6, Cat A: 100% statement + branch where practical) | witness MC/DC pipeline wired; cascade-target run-stage gapped by WASI (FV-FALCON-COV-001); per-engine statement coverage via `cargo llvm-cov` is **GAP** |

## Gap summary

1. **SPA Plan + SCMP + SVVP** — ECSS-Q-ST-80C requires these formal documents; pure documentation work, none yet.
2. **Tool Qualification dossier per tool** (§5.4.8) — Verus/Kani/miri/witness/spar — separate document per tool; v1.0 work.
3. **Cascade-target witness coverage** — same WASI gap as the other domains.
4. **Radiation analysis** — flight software at Cat A typically requires a SEU/SET tolerance review; out of scope for falcon-quad (terrestrial UAS) but in scope for any future falcon-space variant (Ingenuity-class falcon-coax was a stretch v1.0 goal).
5. **Other dual-tree refactors** — relay-sc / relay-hk / relay-sch still plain-only despite being the closest ECSS heritage match; bringing these to the verified-engine pattern is the highest-leverage v1.x work for this domain.
