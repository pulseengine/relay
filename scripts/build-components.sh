#!/usr/bin/env bash
# Build ALL falcon Component-Model components and emit a versioned bundle into
# dist/falcon-components-vMM/:
#
#   falcon-flight-vMM.wasm        — falcon:flight (standalone, wasmtime-runnable)
#   falcon-iekf-vMM.wasm          — falcon:cascade ekf-component (IEKF SE₂(3))
#   falcon-ekf-vMM.wasm           — falcon:cascade ekf-component (Mahony legacy)
#   falcon-attitude-vMM.wasm      — falcon:cascade attitude-component
#   falcon-rate-vMM.wasm          — falcon:cascade rate-component
#   falcon-position-vMM.wasm      — falcon:cascade position-component
#   falcon-mixer-vMM.wasm         — falcon:cascade mixer-component
#   falcon-cascade-vMM.wasm       — falcon:cascade orchestrator
#   manifest.json                 — name + file + sha256 + bytes + toolchain
#   SHA256SUMS                    — canonical checksums for all .wasm artifacts
#
# The flight component is also smoke-tested in wasmtime after building:
# run-stabilization < 0.1 rad, run-position-hold < 0.6 m — the same gate
# scripts/build-flight-component.sh uses.
#
# This emission is MANUAL/bench (the CI gate runner lacks the wasm32-wasip2
# component toolchain — cargo-component — like the gz / meld steps). Run it at
# release time and attach the dist/ directory to the GitHub release.
#
# Usage:
#   scripts/build-components.sh <MAJOR.MINOR>           # build all + bundle into dist/
#   scripts/build-components.sh <MAJOR.MINOR> <tag>     # …then attach to <tag>
#     e.g. scripts/build-components.sh 1.56 falcon-v1.56.0
#
# Requires: cargo-component (0.21.1+), wasmtime.
# Compatible with bash 3 (macOS system bash).
set -euo pipefail

MM="${1:?usage: build-components.sh <MAJOR.MINOR> [release-tag]}"
TAG="${2:-}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
BUNDLE_DIR="$HERE/dist/falcon-components-v$MM"
mkdir -p "$BUNDLE_DIR"

RUSTC_VER=$(rustc --version 2>/dev/null | cut -d' ' -f2)
CC_VER=$(cargo component --version 2>/dev/null | awk '{print $NF}')
TOOLCHAIN_STR="rustc=$RUSTC_VER cargo-component=$CC_VER"

# ---------------------------------------------------------------------------
# build_component <dir>
# Builds the component in wasm/cm/<dir>, finds the output .wasm, copies it
# to the bundle dir as falcon-<dir>-vMM.wasm, and prints the destination path.
# ---------------------------------------------------------------------------
build_component() {
  local dir="$1"
  local wasm_dir="$HERE/wasm/cm/$dir"
  printf "  building %-16s..." "$dir" >&2
  ( cd "$wasm_dir" && cargo component build --release --target wasm32-wasip2 --no-default-features 2>&1 ) \
    | grep -E "^error" | sed 's/^/    /' >&2 || true
  # Prefer the wasip2 path; cargo-component <0.20 emits wasip1.
  local src
  src=$(ls "$wasm_dir"/target/wasm32-wasip2/release/*.wasm 2>/dev/null | head -1 || true)
  if [ -z "$src" ]; then
    src=$(ls "$wasm_dir"/target/wasm32-wasip1/release/*.wasm 2>/dev/null | head -1 || true)
  fi
  if [ -z "$src" ]; then
    echo " ERROR: no .wasm found" >&2
    exit 1
  fi
  # Artifact slug: strip leading "falcon-" from dir if present (avoids
  # "falcon-falcon-mixer") so all 8 names are falcon-<slug>-vMM.wasm.
  local slug="${dir#falcon-}"
  local dest="$BUNDLE_DIR/falcon-$slug-v$MM.wasm"
  cp "$src" "$dest"
  local bytes
  bytes=$(wc -c < "$dest" | tr -d ' ')
  printf " %s bytes\n" "$bytes" >&2
  # stdout: only the dest path (captured by callers)
  echo "$dest"
}

smoke_test_flight() {
  local wasm="$1"
  # Install, don't skip: the release job installs wasmtime so this smoke-test
  # RUNS and validates the component before shipping; dev hosts have it too. This
  # branch is only a FALLBACK for an ad-hoc bundle build on a box without wasmtime
  # — it warns loudly rather than aborting (caught: v1.57.0 release exit 127 when
  # wasmtime wasn't installed). To validate locally: `cargo install wasmtime-cli`.
  if ! command -v wasmtime >/dev/null 2>&1; then
    echo "== skipping flight smoke-test (wasmtime not on PATH) =="
    return 0
  fi
  echo "== smoke-testing flight component in wasmtime =="
  local stab pos
  stab=$(wasmtime run --invoke 'run-stabilization()' "$wasm" 2>/dev/null)
  pos=$(wasmtime run --invoke 'run-position-hold()' "$wasm" 2>/dev/null)
  awk -v s="$stab" -v p="$pos" \
    'BEGIN{ if (s<0.1 && p<0.6) printf("   PASS: stab %.4f<0.1 rad, pos %.4f<0.6 m\n", s, p); \
             else { printf("   FAIL: stab %s pos %s\n", s, p); exit 1 } }'
}

# ---------------------------------------------------------------------------
# Build all 8 components in order
# ---------------------------------------------------------------------------
echo "== building all 8 falcon CM components (wasm32-wasip2) =="

FLIGHT_WASM=$(build_component "flight")
build_component "iekf"         > /dev/null
build_component "ekf"          > /dev/null
build_component "attitude"     > /dev/null
build_component "rate"         > /dev/null
build_component "position"     > /dev/null
build_component "falcon-mixer" > /dev/null
build_component "cascade"      > /dev/null

# P3 STREAM artifacts (bazel-built, not cargo-component): the composed
# flight-control stream pipeline + its Meld-lowered runnable CORE module. Added
# to the bundle so the bundle SHA256SUMS (cosign-signed in release.yml) attests
# them — closing the FV-RELAY-STREAM-014 gap (v1.77). The cascade CI job already
# builds these targets, so a build failure here is loud, not silent.
if command -v bazel >/dev/null 2>&1; then
  echo "== building P3 stream artifacts (bazel) =="
  bazel build //:falcon-cascade-stream-composed //:falcon-cascade-stream-fused
  for tgt in falcon-cascade-stream-composed falcon-cascade-stream-fused; do
    src=$(bazel cquery --output=files "//:$tgt" 2>/dev/null | grep -E '\.wasm$' | head -1)
    [ -n "$src" ] && [ -f "$src" ] || { echo " ERROR: no .wasm for //:$tgt" >&2; exit 1; }
    cp "$src" "$BUNDLE_DIR/$tgt-v$MM.wasm"
    echo "  $tgt-v$MM.wasm  <-  $src"
  done
else
  echo "== WARNING: bazel not on PATH — P3 stream artifacts NOT bundled =="
fi

# Helper: artifact filename from dir name (strips leading "falcon-")
artifact_name() { local s="${1#falcon-}"; echo "falcon-$s-v$MM.wasm"; }

# Re-list with sizes
echo ""
echo "artifacts:"
for dir in flight iekf ekf attitude rate position falcon-mixer cascade; do
  fname=$(artifact_name "$dir")
  f="$BUNDLE_DIR/$fname"
  printf "  %-34s  %s bytes\n" "$fname" "$(wc -c < "$f" | tr -d ' ')"
done

# ---------------------------------------------------------------------------
# Smoke-test the flight component (it has wasmtime-invokable exports)
# ---------------------------------------------------------------------------
smoke_test_flight "$FLIGHT_WASM"

# ---------------------------------------------------------------------------
# Emit SHA256SUMS
# ---------------------------------------------------------------------------
echo "== writing SHA256SUMS =="
( cd "$BUNDLE_DIR" && shasum -a 256 ./*.wasm > SHA256SUMS )
cat "$BUNDLE_DIR/SHA256SUMS"

# ---------------------------------------------------------------------------
# Emit manifest.json (hand-written JSON — avoids jq dependency)
# ---------------------------------------------------------------------------
echo "== writing manifest.json =="
MANIFEST="$BUNDLE_DIR/manifest.json"
{
  printf '{\n'
  printf '  "bundle_version": "%s",\n' "$MM"
  printf '  "toolchain": "%s",\n' "$TOOLCHAIN_STR"
  printf '  "components": [\n'
  first=1
  for dir in flight iekf ekf attitude rate position falcon-mixer cascade; do
    slug="${dir#falcon-}"
    fname="falcon-$slug-v$MM.wasm"
    dest="$BUNDLE_DIR/$fname"
    sha=$(shasum -a 256 "$dest" | awk '{print $1}')
    bytes=$(wc -c < "$dest" | tr -d ' ')
    if [ "$first" = "1" ]; then first=0; else printf ',\n'; fi
    printf '    { "name": "falcon-%s", "file": "%s", "sha256": "%s", "bytes": %s, "toolchain": "%s" }' \
      "$slug" "$fname" "$sha" "$bytes" "$TOOLCHAIN_STR"
  done
  printf '\n  ]\n}\n'
} > "$MANIFEST"
echo "  manifest.json written"

echo ""
echo "== bundle: $BUNDLE_DIR =="
ls -lh "$BUNDLE_DIR"

# ---------------------------------------------------------------------------
# Optionally attach to a GitHub release
# ---------------------------------------------------------------------------
if [ -n "$TAG" ]; then
  echo "== attaching bundle to release $TAG =="
  UPLOAD_FILES=()
  for dir in flight iekf ekf attitude rate position falcon-mixer cascade; do
    UPLOAD_FILES+=("$BUNDLE_DIR/$(artifact_name "$dir")")
  done
  UPLOAD_FILES+=("$BUNDLE_DIR/SHA256SUMS" "$BUNDLE_DIR/manifest.json")
  gh release upload "$TAG" "${UPLOAD_FILES[@]}" --clobber
fi
