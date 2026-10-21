#!/usr/bin/env bash
# The cascade / cascade-split TWIN DIFFERENTIAL (#575, SWREQ-FALCON-SPLIT-P01).
#
# Builds the monolithic `cascade` component exactly as scripts/build-components.sh
# publishes it (no_std, wasm32-unknown-unknown, 8 KiB shadow stack, relocations),
# builds the three split components, composes the split with `wac plug`, and
# flies both through tests/cascade-sitl-wasm in twin mode: one input stream,
# every motor command and every published flight-state field compared BIT FOR
# BIT, every tick, through an event-driven scenario that visits every mode and
# raises the failsafe bits (see tests/cascade-sitl-wasm/src/twin.rs).
#
# Both sides are SUPERVISED: both are components exporting `pilot`. There is no
# native mirror in this comparison to fall out of step with.
#
# BENCH-ONLY: needs cargo-component and wac (like build-components.sh, the CI
# gate runner has neither). The split components build `std` for now — their
# no_std build waits on #574 (arena reclamation).
#
# Usage:
#   scripts/split-differential.sh              # build, compose, compare
#   OUT=<dir> scripts/split-differential.sh    # keep the .wasm files in <dir>
#
# Honours CARGO_TARGET_DIR (one shared target dir keeps the disk cost down).
set -euo pipefail

HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${OUT:-$(mktemp -d)}"
mkdir -p "$OUT"
TARGET=wasm32-unknown-unknown

# artifact <crate-dir> — the built .wasm, selected BY NAME (build-components.sh
# explains why a glob over a shared target dir picks the wrong crate).
artifact() {
  local dir="$1" name
  name=$(grep -m1 '^name *= *"' "$dir/Cargo.toml" | sed -E 's/.*"(.*)".*/\1/')
  name="${name//-/_}.wasm"
  for base in "${CARGO_TARGET_DIR:-}" "$dir/target"; do
    if [ -n "$base" ] && [ -f "$base/$TARGET/release/$name" ]; then
      echo "$base/$TARGET/release/$name"
      return
    fi
  done
  echo "ERROR: no $name built for $dir" >&2
  exit 1
}

echo "== monolithic cascade (the published build: no_std, 8 KiB stack)"
( cd "$HERE/wasm/cm/cascade" && \
  RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=--emit-relocs -C link-arg=-zstack-size=8192 -C link-arg=--export=__heap_base -C link-arg=--export=__data_end" \
  cargo component build --release --target "$TARGET" --no-default-features )
cp "$(artifact "$HERE/wasm/cm/cascade")" "$OUT/cascade.wasm"

for c in cascade-estimator cascade-control cascade-split; do
  echo "== $c (std; no_std waits on #574)"
  ( cd "$HERE/wasm/cm/$c" && cargo component build --release --target "$TARGET" )
  cp "$(artifact "$HERE/wasm/cm/$c")" "$OUT/$c.wasm"
done

echo "== wac plug"
wac plug "$OUT/cascade-split.wasm" \
  --plug "$OUT/cascade-estimator.wasm" \
  --plug "$OUT/cascade-control.wasm" \
  -o "$OUT/cascade-split-composed.wasm"

echo "== harness"
( cd "$HERE/tests/cascade-sitl-wasm" && cargo build --release )
HARNESS="${CARGO_TARGET_DIR:-$HERE/tests/cascade-sitl-wasm/target}/release/cascade-sitl-wasm"

echo "== twin: cascade (A, drives the plant) vs cascade-split (B)"
DIFF_AGAINST="$OUT/cascade-split-composed.wasm" "$HARNESS" "$OUT/cascade.wasm"
