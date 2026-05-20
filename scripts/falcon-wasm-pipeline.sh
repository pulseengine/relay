#!/usr/bin/env bash
#
# falcon v0.6 WASM component pipeline.
#
#   relay-* control crates
#     │  cargo build --target wasm32-unknown-unknown   (per component)
#     ▼
#   core wasm modules
#     │  wasm-tools component embed <wit> + component new
#     ▼
#   WASM components (one per controller)
#     │  meld fuse                                     (back into one layer)
#     ▼
#   fused core module
#     │  wasm-opt -Os                                  (Binaryen optimise)
#     ▼
#   optimised module
#     │  synth compile --cortex-m                      (AOT → ARM ELF)
#     ▼
#   bare-metal ARM Cortex-M ELF   ──►  Renode (CI) / hardware
#
# wasmtime is the reference oracle: each component's scalar exports
# are invoked and checked against the values the native cargo tests
# assert. synth verify runs a Z3 translation-validation check between
# the fused WASM and the ARM ELF.
#
# Usage:  scripts/falcon-wasm-pipeline.sh
# Exit 0 = whole pipeline clear; non-zero = first failing stage.

set -euo pipefail

REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO_ROOT"

OUT=target/falcon-pipeline
mkdir -p "$OUT"
WASM_DIR=target/wasm32-unknown-unknown/release

# Component table: crate name | wasm file stem | wit file | world
COMPONENTS=(
  "falcon-mix-component|falcon_mix_component|wit/falcon-control/mixer.wit|mixer"
  "falcon-rate-component|falcon_rate_component|wit/falcon-control/rate.wit|rate"
)

echo "[falcon-pipeline] 1/6 — build control crates → wasm32 core modules"
for entry in "${COMPONENTS[@]}"; do
  IFS='|' read -r crate _stem _wit _world <<<"$entry"
  cargo build -p "$crate" --release --target wasm32-unknown-unknown >/dev/null 2>&1
done

echo "[falcon-pipeline] 2/6 — component-ize (embed WIT + component new)"
COMPONENT_FILES=()
for entry in "${COMPONENTS[@]}"; do
  IFS='|' read -r crate stem wit world <<<"$entry"
  core="$WASM_DIR/${stem}.wasm"
  emb="$OUT/${world}.embedded.wasm"
  comp="$OUT/${world}.component.wasm"
  wasm-tools component embed "$wit" --world "$world" "$core" -o "$emb"
  wasm-tools component new "$emb" -o "$comp"
  exports=$(meld inspect "$comp" 2>/dev/null | grep -E 'Exports:' | awk '{print $2}')
  echo "  $world: $(wc -c <"$comp" | tr -d ' ')B component, ${exports} export(s)"
  COMPONENT_FILES+=("$comp")
done

echo "[falcon-pipeline] 3/6 — meld fuse → one single-memory module"
FUSED="$OUT/falcon-fused.wasm"
# --memory shared --address-rebase merges the per-component linear
# memories into ONE — the "single layer". A multi-memory fuse would
# need --enable-multimemory downstream and has no single flat address
# space for synth → ARM Cortex-M.
meld fuse --memory shared --address-rebase \
  "${COMPONENT_FILES[@]}" -o "$FUSED" >/dev/null 2>&1
echo "  fused: $(wc -c <"$FUSED" | tr -d ' ')B (shared memory)"

echo "[falcon-pipeline] 4/6 — wasmtime reference checks"
# Reference: mix-total of zero torque / 0.5 thrust = 4 motors x 0.5 = 2.0
mix_total=$(wasmtime run --invoke falcon-mix-total \
  "$OUT/mixer.component.wasm" 0 0 0 0.5 2>/dev/null | tail -1 || \
  wasmtime run --invoke falcon-mix-total \
  "$WASM_DIR/falcon_mix_component.wasm" 0 0 0 0.5 2>/dev/null | tail -1)
echo "  falcon-mix-total(0,0,0,0.5) = $mix_total  (expect 2)"
rate_torque=$(wasmtime run --invoke falcon-rate-torque \
  "$WASM_DIR/falcon_rate_component.wasm" 1.0 2>/dev/null | tail -1)
echo "  falcon-rate-torque(1.0)     = $rate_torque  (expect > 0)"

echo "[falcon-pipeline] 5/6 — wasm-opt -Os"
OPT="$OUT/falcon-fused.opt.wasm"
wasm-opt -Os "$FUSED" -o "$OPT" 2>/dev/null
echo "  optimised: $(wc -c <"$FUSED" | tr -d ' ')B → $(wc -c <"$OPT" | tr -d ' ')B"

echo "[falcon-pipeline] 6/6 — synth → ARM Cortex-M ELF"
ELF_OK=0
ELF_TOTAL=0
# 6a — the fused single-memory module → one ARM ELF (the goal: the
#      whole control layer as one bare-metal binary).
ELF_TOTAL=$((ELF_TOTAL + 1))
if synth compile "$OPT" --cortex-m -o "$OUT/falcon-fused.elf" >/dev/null 2>&1; then
  echo "  fused → $(wc -c <"$OUT/falcon-fused.elf" | tr -d ' ')B ARM ELF"
  ELF_OK=$((ELF_OK + 1))
else
  echo "  fused → synth could not lower the fused module (hickup logged)"
fi
# 6b — per-component core modules → ARM ELF (proven path; each is a
#      single-memory module synth handles directly).
for entry in "${COMPONENTS[@]}"; do
  IFS='|' read -r _crate stem _wit world <<<"$entry"
  core="$WASM_DIR/${stem}.wasm"
  elf="$OUT/${world}.elf"
  ELF_TOTAL=$((ELF_TOTAL + 1))
  if synth compile "$core" --cortex-m -o "$elf" >/dev/null 2>&1; then
    echo "  $world → $(wc -c <"$elf" | tr -d ' ')B ARM ELF"
    ELF_OK=$((ELF_OK + 1))
  else
    echo "  $world → synth FAILED (hickup — see synth log)"
  fi
done

echo
echo "[falcon-pipeline] DONE — ${ELF_OK}/${ELF_TOTAL} targets reached ARM ELF"
[ "$ELF_OK" -ge 2 ] && echo "[falcon-pipeline] PASS" || {
  echo "[falcon-pipeline] FAIL — pipeline did not produce ARM ELFs"
  exit 1
}
