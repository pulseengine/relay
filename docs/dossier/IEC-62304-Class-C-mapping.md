# IEC 62304 Class C — falcon evidence crosswalk

**Status:** v0.13.0 scaffold. Same shape as
[DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md).

IEC 62304 (medical device software lifecycle processes) Class C
covers software that could contribute to a hazardous situation
resulting in death or serious injury. Falcon's safety-critical
mission profile (drone autonomy under EW threats) is a structural
analogue to Class C — a single fault that the geofence/EKF watchdog
fails to catch is similarly life-threatening.

## Mapping note — what IEC 62304 emphasises that the others don't

The medical standard's distinguishing requirement is the **Risk
Management File** (§4.3) linked to ISO 14971. Falcon's
`spar analyze` EMV2 fault-tree + STPA-bridge output is the falcon
analogue to a risk-management document (one hazard, two loss
scenarios, seven sub-hazards from seven minimal cut sets — all
mechanically derived from the AADL model, not hand-authored).

## Section-by-section

| §  | Activity | Falcon evidence |
|----|----------|-----------------|
| §5.1 Software development planning | docs/dossier (this directory) + the rivet rollout YAML |
| §5.2 SW requirements analysis | rivet SWREQ-FALCON-* (32 entries) |
| §5.3 SW architectural design | spar AADL (`spar/falcon_*.aadl`); render: `artifacts/spar/falcon-quad-architecture.svg` |
| §5.4 SW detailed design | every relay-* crate's `src/` (Verus-annotated, contracts visible in doc comments + `requires`/`ensures` clauses) |
| §5.5 SW unit implementation + verification | cargo test (387), Verus contracts, Kani harnesses, miri |
| §5.6 SW integration + integration testing | `bazel build //:falcon-cascade-fused` (meld_fuse); falcon-sitl-hover (17 scenarios) |
| §5.7 SW system testing | falcon-hitl-rfspoof (10 tests) — stub backend automated, mavlink + hackrf backends wired for bench |
| §5.8 SW release | GitHub release per tag (falcon-v0.7.0 → v0.13.0); cosign-signed artifacts across 5 host triples + cortex-m7; rivet-snapshot uploaded with each |
| §7 SW risk management | spar EMV2 fault tree (7 minimal cut sets) + STPA bridge (1 hazard, 2 loss scenarios) → `artifacts/spar/falcon-quad-analysis.json`; cFS-DNA RTL pattern is the system-level mitigation |
| §8 SW configuration management | git (DCO-signed tags); release.yml pins each release artifact's SHA256 + cosign signature |
| §9 SW problem resolution | GitHub Issues; rivet artifact status field (approved / in-progress / blocked) |

## Class C–specific requirements

| Class-C requirement | Falcon evidence |
|---|---|
| §5.5.5 SW unit verification of unit acceptance criteria | every SWREQ has at least one FV-* targeting it; rivet `verifies` link enforced |
| §5.7.4 Regression testing | CI re-runs the full workspace test suite + bazel build on every PR; `Verification gate (rivet-driven)` enforces rivet validate |
| §7.1.3 Risk control measures verification | cFS-DNA RTL chain end-to-end test (`falcon-sitl-hover deterministic_geofence_catches_spoof_and_triggers_rtl`); same chain proven by HITL stub + mavlink backends |

## Gap summary

1. **Software Safety Classification document** (§4.3) — pure documentation.
2. **Linkage to ISO 14971 risk management file** — would need a medical-device-grade risk file separate from the spar EMV2 output.
3. **Cascade-target witness coverage** — same as DO-178C / ISO 26262 / IEC 61508 gap (WASI).
4. **Cybersecurity considerations** (IEC 81001-5-1, increasingly referenced for medical-adjacent) — falcon ships cosign keyless signing on releases; SBOM generation is **GAP**.
5. **Field-deployment update process** — falcon doesn't ship update infrastructure today; v1.0 needs to either provide one (via the existing `synth_compile` ARM ELF pipeline + a download/verify path) or out-of-scope it explicitly.
