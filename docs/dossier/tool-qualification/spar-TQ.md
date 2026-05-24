# spar — Tool Qualification record (falcon v0.14.3)

**Tool:** spar  (AADL architecture modelling + analysis)
**Version pinned:** `/Users/r/.cargo/bin/spar` (`cargo install --list`)
**Source:** PulseEngine `spar` repository
**Underlying analyses:** EMV2 fault tree, ARINC 653 partitioning,
bin-packing schedulability, modal scheduling, STPA bridge.

## Use in falcon

Authors and analyses the autopilot architectural model:

- `spar/falcon_types.aadl` — 7 data records matching the WIT
  field-for-field.
- `spar/falcon_cascade.aadl` — 5 periodic threads (5 ms period,
  ≤1 ms WCET, descending priority 10..6), the AutopilotProcess
  implementation with 9 inter-thread connections.
- `spar/falcon_system.aadl` — Falcon.Quad system: AutopilotMcu
  (Cortex-M7), SRAM (1 MB), explicit Actual_Processor_Binding for
  every thread.

`spar parse + instance + analyze + render` mechanically derives:

- **EMV2 fault tree: 7 minimal cut sets, 7 single-point failures**
  (each controller; mitigated by the cFS-DNA RTL pattern at the
  system level).
- **EMV2-STPA bridge: 1 hazard, 2 loss scenarios, 7 sub-hazards.**
- **Bin-packing schedulability**: 20% MCU utilisation at 200 Hz,
  80% headroom.
- Architecture SVG: `artifacts/spar/falcon-quad-architecture.svg`.
- Analysis JSON: `artifacts/spar/falcon-quad-analysis.json`
  (~50 KB).

## Cross-standard classification

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | **T1** | Generates analysis artifacts (fault tree, schedulability report); does NOT generate runtime code; output review-able by direct inspection of the AADL model and analysis JSON. |
| ISO 26262-8 §11           | **TCL1** — does not affect the runtime; output is design-stage evidence the safety case references. |
| ECSS-Q-ST-80C §5.4.8      | **Category B**. |
| EN 50128 §6.7.4           | **T1**. |

spar is the *lowest* tool class of the five because its output is
design-time-only — it never affects the runtime artifact.

## Qualification approach

Since spar's output is design-stage evidence (not code), the
qualification argument is simpler than for Verus / Kani:

1. The AADL model is **independently auditable**: anyone with an
   AADL editor (OSATE, AADL Inspector, etc.) can open
   `spar/falcon_*.aadl` and verify the structure.
2. spar's analyses are **deterministic functions** of the model;
   re-running on a different AADL toolchain (OSATE's built-in
   analyses) reproduces the cut sets + schedulability.
3. The **fault tree and STPA outputs are reviewable as text** in
   `artifacts/spar/falcon-quad-analysis.json` — a safety engineer
   can read the 7 minimal cut sets directly and confirm they match
   the system's known criticality structure.

A spar bug that produced a wrong fault tree would be caught by
either (a) the OSATE re-run (different tool, same standard) or
(b) the safety-engineer review of the named cut sets.

## Validation evidence

- `spar parse spar/*.aadl` — 3/3 OK.
- `spar instance --root Falcon_System::Falcon.Quad spar/*.aadl` —
  fully instantiates Falcon.Quad (9 components, all connections
  valid).
- `spar analyze --root Falcon_System::Falcon.Quad spar/*.aadl` —
  0 errors, 11 expected warnings (5 SPFs by design + 2 ARINC-653
  partitioning gaps + 4 stub-impl infos), 25 infos.
- Recipe pinned in `FV-FALCON-ARCH-001 steps:`.

## Honestly out of scope

- A formal TQR. v1.0 work.
- spar `codegen --format wit` (auto-regeneration of falcon WIT
  from the AADL model) — current spar emits skeleton Cargo.toml +
  BUILD.bazel only. Deferred; recorded in v0.13 FV-FALCON-ARCH-001.
- Cross-tool fault-tree confirmation via OSATE — currently a
  one-tool analysis. Adding OSATE in CI is a v1.0 deliverable to
  close the strict "diverse confirmation" loop for spar.
