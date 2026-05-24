# Verus — Tool Qualification record (falcon v0.14.3)

**Tool:** Verus  (SMT-based formal verifier for Rust)
**Version pinned:** see `rules_verus` MODULE.bazel reference
**Source:** https://github.com/verus-lang/verus
**Underlying solver:** Z3 (SMT)

## Use in falcon

Discharges pre/post-condition + invariant contracts on the
verified-engine source tree (`crates/relay-*/src/`). Run via
Bazel: `bazel test //:relay_*_verus_test`. Properties discharged
include:

- LC-P09 / LC-P10 (Geofence monotone latch + transition-only)
- HS-P06 / HS-P07 (EkfHealthMonitor monotone latch)
- SC-P01..P05 (CommandStore invariants)
- NID-V01..V03 (from_code totality + Option contracts)
- MAVLINK-V01..V05 (encoder size + decoder Option-totality)

Plus the per-engine HS-/HK-/SCH-/CS-/DS- properties recorded in
the per-engine `SWREQ-*-P01.yaml` rivet artifacts.

## Cross-standard classification

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | **T2** | Generates artifacts (proof certificates) consumed by other tools (the verification gate); no direct safety-code generation; errors detectable via Kani cross-confirmation (HS-P07 ↔ HS-P08/P09; LC-P09/P10 ↔ LC-K01..K05). |
| ISO 26262-8 §11           | **TCL3** by classification (output influences safety + errors not always detected by review alone) → **qualified to TCL3** via the cross-confirmation argument below. |
| ECSS-Q-ST-80C §5.4.8      | **Category B** — used in development of Cat A software; not a runtime/embedded tool; output review-able. |
| EN 50128 §6.7.4           | **T2** — same rationale as IEC 61508-3. |

## Qualification approach

Falcon takes a **technique-class-diversity** approach to Verus
qualification. Every Verus-discharged contract is cross-confirmed
by at least one independent technique class:

| Verus property         | Independent confirmation                                  |
|------------------------|-----------------------------------------------------------|
| LC-P09 / LC-P10        | Kani LC-K01..K05 (FV-FALCON-GEO-002) + miri (COV-003)     |
| HS-P06 / HS-P07        | Kani HS-P08 / HS-P09 (FV-FALCON-UAM-001)                  |
| SC-P01..P05            | cargo proptest fuzz + falcon-sitl-hover end-to-end        |
| NID-V01..V03           | Kani NID-K01..K05 + proptest fuzz (FV-FALCON-NID-001/002) |
| MAVLINK-V01..V05       | proptest fuzz (round-trip + arbitrary-input) per message  |

A Verus soundness bug that affected a falcon-claimed property would
have to also fool the corresponding independent checker — which
operates on a different program representation (Kani: SAT-encoded
bounded MC; miri: concrete interpreter with UB detection; proptest:
randomised concrete execution; SITL: signal-level scenario). The
**diverse-confirmation argument** is what lets a TCL3-classed tool
ship in a safety-credit submission without a full DO-330-style
Tool Qualification Report.

## Validation evidence

- Verus's own test suite passes (vetted by the Verus team on every
  merge; falcon does not re-run it).
- `bazel test //:relay_*_verus_test` runs the per-engine
  verifications on every PR (via the `Verification gate
  (rivet-driven)` GitHub Actions job).
- Each FV-FALCON-* rivet artifact lists the Verus contracts +
  the bazel invocation steps that discharge them.
- Versioned via Bazel MODULE.bazel `local_path_override` to a
  pinned sibling repo; reproducible by commit SHA.

## Honestly out of scope

- A formal DO-330 TQR (Tool Qualification Report) with an
  assessor-signed validation against a known-bad test corpus.
  v1.0 work; current operational evidence + cross-confirmation
  argument substitutes at v0.14.3.
- Verus's own internal soundness audit. Falcon relies on the
  upstream Verus project for that.
