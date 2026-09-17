#!/usr/bin/env bash
# meld-fuse-cascade.sh — fuse the five per-stage Component Model leaves
# (iekf, position, attitude, rate, mixer) into a single WebAssembly module with
# `meld` — the PulseEngine fusion step (CLAUDE.md: "Meld fuses components,
# wires streams at build time").
#
# NOT THE FLIGHT CORE (#388, #419). Three of these leaves wrap LEGACY
# controllers the flight core does not fly (relay-pos, relay-att, relay-rate;
# no Kani or Verus in CI) — falcon-core flies geometric SE(3) + ADRC, shipped as
# the single `falcon-cascade` component. The fused output exercises the
# meld -> loom -> synth -> gale pipeline; it is NOT a deployable flight artifact.
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
