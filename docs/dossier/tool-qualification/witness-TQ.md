# witness — Tool Qualification record (falcon v0.14.3)

**Tool:** witness  (MC/DC structural-coverage tool for WebAssembly)
**Version pinned:** witness 0.22.0 (via `rules_wasm_component`
toolchain pin in `MODULE.bazel`)
**Source:** PulseEngine `rules_wasm_component` repository
**Underlying runtime:** witness's embedded `wasmtime`-based runner
+ optional `--harness <cmd>` mode (not yet used by falcon)

## Use in falcon

Instruments Wasm core modules with branch counters, executes them,
emits LCOV + branch-counter JSON.

Falcon's witness pipeline today:

- **//:geofence-subject-coverage** (v0.13) — hand-written WAT
  module mirroring `Geofence::check`'s decision shape; smoke target
  proving the instrument → run → LCOV pipeline works end-to-end.
- **//:geofence-subject-rs-coverage** (v0.14.1) — real Rust subject
  calling `relay_lc::engine::Geofence::check` via three
  black_box-wrapped exports; WASI-free `wasm32-unknown-unknown`
  build.
- **//:falcon-cascade-coverage** (v0.13, `tags=["manual"]`) — full
  cascade target; instrument-stage succeeds, run-stage blocked by
  the upstream WASI gap (deferred to v0.14.x via `--harness
  wasmtime+WASI`).

## Cross-standard classification

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | **T2** | Generates structural-coverage artifacts; output (LCOV + branch-counter JSON) is review-able and reproducible per the bazel build. |
| ISO 26262-8 §11           | **TCL2** — coverage tool; output influences acceptance but does not generate safety code; errors detectable by re-running. |
| ECSS-Q-ST-80C §5.4.8      | **Category B**. |
| EN 50128 §6.7.4           | **T2**. |

## Qualification approach

witness's role is **measurement**, not proof. A measurement-only
tool failure mode is undercount (the LCOV report claims branches
weren't hit when they were, or vice versa). Falcon's
cross-confirmation:

| witness claim                    | Independent confirmation                                  |
|----------------------------------|-----------------------------------------------------------|
| Branch IDs identified in module  | wasm-tools print + manual inspection of the instrumented module |
| Branch hit counts                | The smoke subject (WAT) has a known decision shape; the v0.14.1 Rust subject has three distinct entry points each exercising a known branch — observed counts match expectation |
| LCOV format conformance          | Standard LCOV consumers (lcov, genhtml) parse the output |

A witness undercount would not affect the *correctness* of the
verified safety chain — it would just produce overly-pessimistic
coverage evidence. The technique-class diversity already gives
falcon belt-and-braces evidence (Verus + Kani prove the property;
witness only reports on the test suite's structural reach).

## Validation evidence

- `bazel build //:geofence-subject-coverage` (smoke; v0.13) — green;
  produces 4-branch JSON + LCOV.
- `bazel build //:geofence-subject-rs-coverage` (Rust subject;
  v0.14.1) — green; produces 2-branch JSON + LCOV (the
  Geofence::check call was fully inlined into the wrapper by LLVM —
  honest disposition in FV-FALCON-COV-003).
- Recipe pinned in `FV-FALCON-COV-001 / -002 / -003` `steps:`.
- witness toolchain version pinned in
  `rules_wasm_component/checksums/tools/witness.json`.

## Honestly out of scope

- A formal TQR. v1.0 work.
- Cascade-target run-stage: blocked by WASI gap. Until the
  `--harness wasmtime+WASI` runner lands, the cascade target is
  `tags=["manual"]` and produces instrument-stage evidence only.
- Source-line LCOV records: empty (`TN:wasm-bytecode` header only)
  because release-profile wasm carries no DWARF. Adding `debug =
  true` would emit DWARF but balloons the module; tracked as a
  future tightening.
