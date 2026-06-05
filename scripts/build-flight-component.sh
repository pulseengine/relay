#!/usr/bin/env bash
# Build the falcon flight Component-Model artifact and name it for release:
# `falcon-flight-v<MAJOR.MINOR>.wasm` (+ `.sha256`) — the load-bearing hand-off
# the separate hardware-integration project (jess: meld → loom → synth → kiln)
# consumes. The verified IEKF → geometric SE(3) → ADRC → mixer cascade as a
# portable, wasmtime-runnable component.
#
# This emission is MANUAL/bench (the CI gate runner lacks the wasm32-wasip2
# component toolchain — cargo-component — like the gz / meld steps). Run it at
# release time and attach the output to the GitHub release; see the usage below.
# (Restores the asset that was emitted for v1.26/v1.27 but dropped for
# v1.28–v1.32 — issue #100.)
#
# Usage:
#   scripts/build-flight-component.sh <MAJOR.MINOR>            # build + sha into dist/
#   scripts/build-flight-component.sh <MAJOR.MINOR> <tag>      # …then attach to <tag>
#     e.g. scripts/build-flight-component.sh 1.33 falcon-v1.33.0
#
# Requires: cargo-component, wasmtime.
set -euo pipefail
MM="${1:?usage: build-flight-component.sh <MAJOR.MINOR> [release-tag]}"
TAG="${2:-}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$HERE/dist"
mkdir -p "$OUT"

echo "== building falcon flight component (wasm32-wasip2) =="
( cd "$HERE/wasm/cm/flight" && cargo component build --release >/dev/null )
SRC=$(ls "$HERE"/wasm/cm/flight/target/wasm32-wasip*/release/falcon_flight_component.wasm | head -1)

echo "== verifying it runs in wasmtime (the release gate) =="
STAB=$(wasmtime run --invoke 'run-stabilization()' "$SRC" 2>/dev/null)
POS=$(wasmtime run --invoke 'run-position-hold()' "$SRC" 2>/dev/null)
awk -v s="$STAB" -v p="$POS" 'BEGIN{ if (s<0.1 && p<0.6) printf("   PASS: stab %.4f<0.1 rad, pos %.4f<0.6 m\n", s, p); else { printf("   FAIL: stab %s pos %s\n", s, p); exit 1 } }'

ART="$OUT/falcon-flight-v$MM.wasm"
cp "$SRC" "$ART"
( cd "$OUT" && shasum -a 256 "falcon-flight-v$MM.wasm" > "falcon-flight-v$MM.wasm.sha256" )
echo "== artifact: $ART ($(wc -c < "$ART") bytes) =="
cat "$OUT/falcon-flight-v$MM.wasm.sha256"

if [ -n "$TAG" ]; then
  echo "== attaching to release $TAG =="
  gh release upload "$TAG" "$ART" "$OUT/falcon-flight-v$MM.wasm.sha256" --clobber
fi
