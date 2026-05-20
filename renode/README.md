# renode — falcon MCU emulation

Emulates the falcon control layer on an STM32H743 (Cortex-M7) — the
FMU-class MCU from the [falcon roadmap](../falcon/README.md) — so the
v0.6 pipeline can be exercised without physical hardware.

## Where Renode runs

| host | status |
|---|---|
| Linux (CI, ubuntu-latest) | ✅ via `renode-bazel-rules` hermetic portable Renode |
| macOS arm64 (this dev box) | ◐ no portable build at builds.renode.io; the `pulseengine/renode-bazel-rules` mac port is in progress |

The emulation therefore runs **in CI on Linux**, where
`renode-bazel-rules` fetches a hermetic portable Renode
(`renode-1.15.3+...linux-portable-dotnet`). The `.resc` script here
is host-agnostic — once the mac port of the rules lands it runs
locally too.

## Pipeline that feeds this

```
relay-* control crates
  → cargo build --target wasm32-unknown-unknown   (core modules)
  → wasm-tools component embed + new              (WASM components)
  → meld fuse --memory shared --address-rebase    (one single-memory module)
  → wasm-opt -Os                                  (Binaryen)
  → synth compile --cortex-m                      (ARM Cortex-M ELF)
  → Renode: LoadELF on emulated STM32H743         (this directory)
```

Run the host side of the pipeline:

```sh
bash scripts/falcon-wasm-pipeline.sh
# → target/falcon-pipeline/falcon-fused.elf
```

Then, on a host with Renode:

```sh
renode renode/falcon-cortex-m.resc
# (renode) start
```

## Files

- `falcon-cortex-m.resc` — Renode machine script: STM32H743 platform,
  loads the synth ELF, opens USART1 as the telemetry analyzer.

## v0.6 status

The `.resc` script is committed and CI-ready. Actual emulation runs
in the Linux CI job once `renode-bazel-rules` is wired into the
build (tracked with the `rules_wasm_component` Bazel integration).
v0.6 ships the host pipeline (proven: meld fuse → wasm-opt → synth
ARM ELF) plus this staged emulation harness.
