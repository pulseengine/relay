#!/usr/bin/env bash
# meld-fuse-cascade.sh — fuse the VERIFIED falcon control cascade (the five
# Component Model leaves, incl. the verified IEKF) into a single WebAssembly
# module with `meld` — the PulseEngine fusion step (CLAUDE.md: "Meld fuses
# components, wires streams at build time").  This is the embedded artifact:
# one module → loom → synth → gale on a real target.
#
# meld is NOT provisioned in the CI bazel toolchain, so this runs locally /
# on a bench (like the gz flights), not inside the gate. Output: the fused
# module + meld stats + a meld inspect of the result.
#
#   ./scripts/meld-fuse-cascade.sh [out.wasm]
set -euo pipefail
cd "$(dirname "$0")/.."

OUT="${1:-/tmp/falcon-fused.wasm}"
VENDOR="--vendor_dir=vendor/bazel"

echo "── building the five verified cascade components ──"
bazel build $VENDOR \
  //:falcon-iekf //:falcon-position //:falcon-attitude //:falcon-rate //:falcon-mixer

# Resolve each component's .wasm output via cquery (platform-independent).
declare -a INPUTS=()
for t in falcon-iekf falcon-position falcon-attitude falcon-rate falcon-mixer; do
  f=$(bazel cquery $VENDOR --output=files "//:$t" 2>/dev/null | grep -E '\.wasm$' | head -1)
  [ -n "$f" ] || { echo "no wasm for //:$t"; exit 1; }
  INPUTS+=("$f")
done

echo "── meld fuse → $OUT ──"
meld fuse "${INPUTS[@]}" -o "$OUT" --stats

echo "── meld inspect $OUT ──"
meld inspect "$OUT" | head -30

echo "fused module: $OUT  ($(wc -c < "$OUT") bytes)"
