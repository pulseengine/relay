"""Wrapper macro: witness MC/DC coverage on Wasm modules that need WASI.

The vanilla `wasm_module_coverage` rule from rules_wasm_component uses
witness's embedded WASI-free runtime, which fails on modules that pull
in any wasi:* imports (Rust panic glue, wit-bindgen runtime — the
falcon cascade has 24 of these across 10 namespaces; see v0.13–v0.15
FV-FALCON-COV-001..004 evidence).

This macro wraps witness's three-stage pipeline behind a `genrule`:

  instrument → run --harness=//host/witness-wasi-harness → lcov

`//host/witness-wasi-harness` (v0.15.1) implements witness's
`witness-harness-v1` subprocess-harness ABI on wasmtime + WASI
preview1, stubbing any unresolved component-model imports so dense
Rust-compiled modules can be MC/DC-instrumented end-to-end.

Closes the `tags = ["manual"]` workaround on //:falcon-cascade-coverage
that's been carried since v0.13.

Inputs:
  name:   target name (LCOV output is `<name>.lcov.info`).
  module: a meld_fuse target OR a `.wasm` file label.
  invokes: optional list of scripted invocation specs in
    `WITNESS_HARNESS_INVOKES` format — semicolon-separated
    `func:val1,val2,…`. Defaults to "" (auto-discover only).

Outputs:
  <name>_instrumented.wasm
  <name>_witness-run.json
  <name>.lcov.info
"""

load("@rules_wasm_component//providers:providers.bzl", "MeldFusedInfo")

def _resolve_module(ctx, dep):
    """Return the .wasm file from either MeldFusedInfo or a plain wasm."""
    if MeldFusedInfo in dep:
        return dep[MeldFusedInfo].fused_wasm
    wasm_files = [f for f in dep[DefaultInfo].files.to_list() if f.extension == "wasm"]
    if len(wasm_files) != 1:
        fail("wasm_module_coverage_wasi: target '{}' must produce exactly one .wasm (found {})".format(
            dep.label,
            len(wasm_files),
        ))
    return wasm_files[0]

def _impl(ctx):
    witness = ctx.toolchains["@rules_wasm_component//toolchains:witness_toolchain_type"].witness
    harness = ctx.executable._harness
    module = _resolve_module(ctx, ctx.attr.module)

    name = ctx.label.name
    instrumented = ctx.actions.declare_file(name + "_instrumented.wasm")
    manifest = ctx.actions.declare_file(name + "_instrumented.wasm.witness.json")
    run_data = ctx.actions.declare_file(name + "_witness-run.json")
    lcov = ctx.actions.declare_file(name + ".lcov.info")

    # Stage 1 — instrument.
    instrument_args = ctx.actions.args()
    instrument_args.add("instrument")
    instrument_args.add(module)
    instrument_args.add("-o", instrumented)
    ctx.actions.run(
        inputs = [module],
        outputs = [instrumented, manifest],
        executable = witness,
        arguments = [instrument_args],
        mnemonic = "WitnessInstrumentWasi",
        progress_message = "Instrumenting %s (WASI-aware)" % module.short_path,
        tools = [witness],
    )

    # Stage 2 — run via subprocess harness (host/witness-wasi-harness).
    # We invoke `witness run` with `--harness <abs path to harness binary>`;
    # witness spawns the harness with WITNESS_MODULE / WITNESS_MANIFEST /
    # WITNESS_OUTPUT and reads back the v1 snapshot the harness writes.
    run_args = ctx.actions.args()
    run_args.add("run")
    run_args.add(instrumented)
    run_args.add("--manifest", manifest)
    run_args.add("-o", run_data)
    run_args.add("--harness", harness.path)
    env = {}
    if ctx.attr.invokes:
        env["WITNESS_HARNESS_INVOKES"] = ctx.attr.invokes
    ctx.actions.run(
        inputs = [instrumented, manifest, harness],
        outputs = [run_data],
        executable = witness,
        arguments = [run_args],
        mnemonic = "WitnessRunWasi",
        progress_message = "Running %s under wasmtime+WASI harness" % instrumented.short_path,
        tools = [witness, harness],
        env = env,
    )

    # Stage 3 — lcov.
    # The current witness CLI takes run + manifest as named flags
    # (`witness lcov --run <RUN> --manifest <MANIFEST>`); the earlier positional
    # form (`witness lcov <run>`) was rejected after a witness version bump in
    # rules_wasm_component.
    lcov_args = ctx.actions.args()
    lcov_args.add("lcov")
    lcov_args.add("--run", run_data)
    lcov_args.add("--manifest", manifest)
    lcov_args.add("-o", lcov)
    ctx.actions.run(
        inputs = [run_data, manifest],
        outputs = [lcov],
        executable = witness,
        arguments = [lcov_args],
        mnemonic = "WitnessLcovWasi",
        progress_message = "Emitting LCOV for %s" % run_data.short_path,
        tools = [witness],
    )

    return [DefaultInfo(files = depset([lcov, run_data, instrumented, manifest]))]

wasm_module_coverage_wasi = rule(
    implementation = _impl,
    attrs = {
        "module": attr.label(mandatory = True, allow_files = [".wasm"]),
        "invokes": attr.string(default = ""),
        "_harness": attr.label(
            default = "//host/witness-wasi-harness",
            executable = True,
            cfg = "exec",
        ),
    },
    toolchains = ["@rules_wasm_component//toolchains:witness_toolchain_type"],
)
