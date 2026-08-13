#!/usr/bin/env bash
# Build ALL falcon Component-Model components and emit a versioned bundle into
# dist/falcon-components-vMM/:
#
#   falcon-flight-vMM.wasm        — pulseengine:falcon-flight (standalone, wasmtime-runnable)
#   falcon-iekf-vMM.wasm          — pulseengine:falcon-cascade ekf-component (IEKF SE₂(3))
#   falcon-ekf-vMM.wasm           — pulseengine:falcon-cascade ekf-component (Mahony legacy)
#   falcon-attitude-vMM.wasm      — pulseengine:falcon-cascade attitude-component
#   falcon-rate-vMM.wasm          — pulseengine:falcon-cascade rate-component
#   falcon-position-vMM.wasm      — pulseengine:falcon-cascade position-component
#   falcon-mixer-vMM.wasm         — pulseengine:falcon-cascade mixer-component
#   falcon-cascade-vMM.wasm       — pulseengine:falcon-cascade orchestrator
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
# Components that are `no_std` build for wasm32-unknown-unknown, NOT
# wasm32-wasip2 (v1.131). `target_env = "p2"` is the wasip2 RUST TARGET, which
# links wasi-libc — and wasi-libc's cabi_realloc goes through malloc, which
# emits `memory.grow`. `memory.grow` is what makes
# `meld fuse --memory shared --address-rebase` reject a component (gale#89,
# meld#299), so it blocks the single-address-space MCU lowering these
# components exist for. A component does NOT need the wasip2 target to be a
# valid Component-Model P2 component: falcon-rate built for
# wasm32-unknown-unknown is a component (0061736d0d000100) with ZERO wasi
# imports. gale provides gust:os/gust:hal, not WASI, so nothing on this path
# should be linking wasi-libc at all.
#
# Add a component here as it is converted to no_std (jess's per-stage lowering
# report is the priority order). A std component still needs wasip2.
NOSTD_COMPONENTS=" flight iekf ekf attitude rate position falcon-mixer cascade "

component_target() {
  case "$NOSTD_COMPONENTS" in
    *" $1 "*) echo "wasm32-unknown-unknown" ;;
    *)        echo "wasm32-wasip2" ;;
  esac
}

build_component() {
  local dir="$1"
  local wasm_dir="$HERE/wasm/cm/$dir"
  local target
  target=$(component_target "$dir")
  printf "  building %-16s (%s)..." "$dir" "$target" >&2
  # FAIL LOUD (v1.130): this used to end in `|| true`, so a build error was
  # printed and then IGNORED — the script fell through to a stale artifact from
  # a previous build and shipped it. That is exactly how falcon-rate:1.129.0 was
  # published as a raw CORE MODULE: the no_std wasip2 build failed
  # ("module does not export a function named `cabi_realloc`"), the failure was
  # swallowed, and the fallback picked up a pre-componentization module.
  # A build failure must stop the bundle, not degrade it silently.
  # RELOCATION METADATA (v1.134, OCI-P05). meld's shared-memory fusion places
  # each module at a non-zero base and REBASES its absolute addresses. It can
  # only do that if the module carries `linking` / `reloc.*` sections. Without
  # them it refuses — correctly, since silently mis-rebasing would produce a
  # component that links and then misbehaves at runtime:
  #
  #   component 'mixer.wasm' module 0 is placed at a non-zero shared-memory base
  #   but carries no relocation metadata (linking/reloc.*); its absolute
  #   addresses cannot be rebased safely.
  #
  # Measured by jess on the v1.133 components (jess#167): ALL SIX had
  # reloc-sections=0, so `meld fuse --memory shared --address-rebase` rejected
  # the cascade. Zero `memory.grow` (OCI-P02) was NECESSARY but NOT SUFFICIENT;
  # this is the second, independent blocker.
  #
  # It must be on the FINAL link. `wasm-ld -r` (an unresolved relocatable
  # object) is NOT sufficient: its stored values are addends, not final
  # addresses, so a consumer cannot rebase from them.
  if ! ( cd "$wasm_dir" && RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=--emit-relocs" cargo component build --release --target "$target" --no-default-features 2>&1 ) \
       | tee /tmp/falcon-build-$dir.log | grep -E "^error" | sed 's/^/    /' >&2; then
    : # grep found no "^error" lines — that is the success path
  fi
  if grep -qE "^error" /tmp/falcon-build-$dir.log 2>/dev/null; then
    echo " ERROR: cargo component build failed for $dir (see above)" >&2
    exit 1
  fi
  # Prefer the wasip2 path; cargo-component <0.20 emits wasip1.
  # Select the artifact BY NAME, never `ls *.wasm | head -1` (v1.130). The old
  # glob picked whatever sorted first in the target dir — with a shared
  # CARGO_TARGET_DIR (a common CI disk-saving setting) that is a DIFFERENT
  # crate's component: building `rate` picked up falcon_flight_component.wasm
  # because "flight" < "rate". It would then be published under the wrong name.
  # cargo turns - into _ for the artifact filename.
  local crate_name artifact src
  crate_name=$(grep -m1 '^name *= *"' "$wasm_dir/Cargo.toml" | sed -E 's/.*"(.*)".*/\1/')
  artifact="${crate_name//-/_}.wasm"
  src=""
  for tgt in "$target" wasm32-wasip2 wasm32-wasip1; do
    if [ -f "$wasm_dir/target/$tgt/release/$artifact" ]; then
      src="$wasm_dir/target/$tgt/release/$artifact"; break
    fi
    # honour a shared CARGO_TARGET_DIR if one is set
    if [ -n "${CARGO_TARGET_DIR:-}" ] && [ -f "$CARGO_TARGET_DIR/$tgt/release/$artifact" ]; then
      src="$CARGO_TARGET_DIR/$tgt/release/$artifact"; break
    fi
  done
  if [ -z "$src" ]; then
    echo " ERROR: no .wasm found" >&2
    exit 1
  fi
  # Artifact slug: strip leading "falcon-" from dir if present (avoids
  # "falcon-falcon-mixer") so all 8 names are falcon-<slug>-vMM.wasm.
  local slug="${dir#falcon-}"
  local dest="$BUNDLE_DIR/falcon-$slug-v$MM.wasm"
  cp "$src" "$dest"
  # ASSERT the payload really is a Component, not a core module (v1.130).
  # jess found falcon-rate:1.129.0 shipped as a core module — `meld fuse`
  # rejected it outright. The OCI config mediaType says "component" regardless
  # of the bytes, so metadata cannot catch this; check the 8-byte header.
  #   core module : 00 61 73 6d 01 00 00 00
  #   component   : 00 61 73 6d 0d 00 01 00
  magic=$(od -An -tx1 -N8 "$dest" | tr -d ' \n')
  if [ "$magic" != "0061736d0d000100" ]; then
    echo " ERROR: $dest is NOT a wasm component (header $magic)" >&2
    echo "        expected 0061736d0d000100; a core module is 0061736d01000000" >&2
    exit 1
  fi
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
