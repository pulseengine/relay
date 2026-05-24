# witness-wasi-harness — subprocess harness for the witness MC/DC tool

Closes the cascade-target WASI gap recorded in
[`FV-FALCON-COV-001`](../../artifacts/verification/FV-FALCON-COV-001.yaml)
(v0.13). Implements the [`witness-harness-v1`](https://github.com/pulseengine/witness)
JSON snapshot protocol over a wasmtime + WASI preview1 runtime
that's permissive enough to load Rust-compiled wit-bindgen wasm
modules.

## Why this exists

witness's embedded `wasmtime` runner is WASI-free. Falcon's cascade
core module (produced by `meld_fuse` of the five leaf components)
carries **24 WASI imports across 10 namespaces** — Rust's std panic
glue + wit-bindgen runtime pull these in even when the cascade
itself never calls them. The embedded runtime can't satisfy them,
so the cascade `run` stage failed at instantiation in v0.13/v0.14:

```text
Error: wasm runtime error: unknown import: `wasi:io/error@0.2.6::
[resource-drop]error` has not been defined
```

This harness uses wasmtime's `Linker::func_new` to install no-op
stubs for every unresolved import, then invokes the cascade
exports under the standard WASI preview1 environment so the
instrumented module runs to completion.

## How witness invokes the harness

Per `witness/crates/witness/src/run.rs` (the `run_via_harness`
path), witness spawns the harness via `sh -c` with three env vars:

| Env var          | Meaning                                                     |
|------------------|-------------------------------------------------------------|
| `WITNESS_MODULE` | Path to the instrumented `.wasm` core module                |
| `WITNESS_MANIFEST` | Path to the branch manifest JSON (info; we don't read it) |
| `WITNESS_OUTPUT` | Where we write the counter snapshot                         |

The harness writes a `witness-harness-v1` snapshot — a JSON object
with `schema: "witness-harness-v1"` and a `counters` map from
branch ID (decimal string) to hit count. Witness merges this with
the manifest to produce the final `witness-run.json`.

The on-wire shape lives in
[`witness-core/src/run_record.rs`](https://github.com/pulseengine/witness/blob/main/crates/witness-core/src/run_record.rs)
as `HarnessSnapshot` — v1 (counters only) and v2 (counters + per-
row brval/brcnt/trace memory). We emit v1; v2 with full MC/DC
truth-table reconstruction is a follow-up.

## Build + use

```bash
# Build
cargo build -p witness-wasi-harness --release

# Run witness via subprocess harness mode
WITNESS=/path/to/witness-bin       # rules_wasm_component toolchain
HARNESS=target/release/witness-wasi-harness

$WITNESS run path/to/instrumented.wasm \
  --output run.json \
  --harness "$HARNESS"
```

## Smoke test — geofence_subject_rs (v0.14.1)

Hit-count parity with v0.14.1's embedded-mode run:

```text
witness-wasi-harness: invoked 3 exports
witness-wasi-harness: collected 2 counters
                  hits: 1, 0           # identical to embedded mode
```

## Cascade run (v0.15.1)

```text
witness-wasi-harness: invoke falcon:cascade/controller@0.7.0#step
   failed: memory fault at wasm address 0x100000 in linear memory
   of size 0x100000: wasm trap: out of bounds memory access
witness-wasi-harness: invoked 14 exports
witness-wasi-harness: collected 5972 counters
   → 81/5972 branches hit
```

**Honest dispositions:**

1. **WASI gap is closed.** The cascade now instantiates and runs
   end-to-end under the harness — no missing imports, no
   instantiation failures. This is the v0.13/v0.14 deferred
   item: closed.

2. **81/5972 branches hit, not "high coverage".** Zero-args
   invocation walks a single execution path through 14 of the 15
   cascade exports; calling them with realistic record-shaped
   args would exercise more branches. v0.16 candidate: extend
   the harness to read `--invoke-with-args`-style specs from
   `WITNESS_HARNESS_INVOKES` so we can drive the cascade with
   the same inputs the SITL uses.

3. **`controller#step` tripped a trace-buffer OOB** inside
   witness's own instrumentation (`__witness_trace_record`
   writes past the 1 MB linear-memory boundary because the
   cascade has 5972+ branches). This is a witness limitation,
   not a harness bug — fixable upstream by sizing the trace
   buffer proportionally to the branch count, or by us setting
   `WITNESS_TRACE_BYTES` if witness exposes the knob.

4. **Stubbed imports don't model real WASI semantics.** If the
   cascade *called* `wasi:cli/exit::exit` the harness would
   silently return zero; nothing else relies on these imports
   at runtime, so the stubbed-as-zero stance is sound for
   coverage measurement but not for execution validation.
