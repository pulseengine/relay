# Kani — Tool Qualification record (falcon v0.14.3)

**Tool:** Kani  (bounded model checker for Rust)
**Version pinned:** kani-verifier 0.63.0 (`cargo install --list`)
**Source:** https://github.com/model-checking/kani
**Underlying solver:** CBMC (bit-precise; SAT-based)

## Use in falcon

Discharges per-harness properties via exhaustive enumeration over
bounded input domains. Harnesses live under `#[cfg(kani)] mod
kani_proofs` in each engine's plain mirror:

- LC-K01..K05 (Geofence::check — monotone latch, transition-only,
  already-latched silent, inside never trips, outside always trips)
- HS-P08 / HS-P09 (EkfHealthMonitor — monotone window + alert
  bound)
- NID-K01..K05 (Network ID codec — round-trip + decoder never
  panics + bit-packed wire-format round-trip)
- LC-P04..P06 (Watchpoint table — bounded output, compare totality,
  disabled-never-fires)

## Cross-standard classification

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | **T2** | Generates verification artifacts; output review-able via CBMC counter-example trace; cross-confirmed by Verus contracts on same properties. |
| ISO 26262-8 §11           | **TCL3** classified; qualified-to-TCL3 via diverse confirmation. |
| ECSS-Q-ST-80C §5.4.8      | **Category B** — used in development. |
| EN 50128 §6.7.4           | **T2** — same as IEC. |

## Qualification approach

Same **technique-class-diversity** principle as Verus, with the
roles swapped: Kani confirms Verus contracts and vice versa.

| Kani property        | Independent confirmation                                  |
|----------------------|-----------------------------------------------------------|
| LC-K01..K05          | Verus LC-P09 / LC-P10 (FV-FALCON-GEO-001) + miri (COV-003)|
| HS-P08 / HS-P09      | Verus HS-P06 / HS-P07 (relay-hs verus_test)               |
| NID-K01..K05         | Verus NID-V01..V03 (FV-FALCON-NID-002) + proptest fuzz    |

In addition, Kani's CBMC backend produces a **concrete counter-
example** when a property fails — this is independently review-able
by reading the failing-state assignment, distinct from Verus's SMT
"unsat" → proof-accepted result.

## Validation evidence

- `cargo kani -p relay-{lc,hs,nid}` runs each harness pack with the
  pinned kani-verifier version.
- Per-FV `steps:` field lists the exact invocation per harness.
- Operational history: every harness pack landed has been
  re-verified at each release tag (falcon-v0.9.1 onward).
- Harnesses themselves are `#[cfg(kani)]` so they're invisible to
  cargo and don't affect cargo test results — clean separation.

## Honestly out of scope

- A formal TQR with an assessor-signed validation against
  intentional-fault test cases. v1.0 work.
- Bound-completeness audit (Kani is *bounded*; unbounded harnesses
  are NOT formal proofs). Each falcon harness's bounded scope is
  documented in its FV artifact.
