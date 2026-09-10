#!/usr/bin/env bash
# Compose the five published stage components into the published cascade
# socket, producing a SELF-CONTAINED component a host can instantiate.
#
# This is what makes the shipped wasm runnable at all: `falcon-cascade` on its
# own imports the five stage interfaces and cannot be instantiated. `wac plug`
# satisfies them from the leaf components — the same composition BUILD.bazel
# performs, but over the PUBLISHED no_std artifacts rather than the bazel
# std-linked ones (which carry 18 wasi imports and 6 memory.grow).
#
# Usage:
#   scripts/compose-cascade.sh <dir-with-published-components> [out.wasm]
# where <dir> holds falcon-{cascade,iekf,position,attitude,rate,mixer}-vX.Y.wasm
# e.g. unpacked from the release's falcon-components-vX.Y.tar.gz.
set -euo pipefail

DIR="${1:?usage: compose-cascade.sh <dir-with-published-components> [out.wasm]}"
OUT="${2:-$DIR/composed-cascade.wasm}"

command -v wac >/dev/null 2>&1 || { echo "ERROR: wac not on PATH (cargo install wac-cli)" >&2; exit 1; }

# The glob is anchored on `-v<digit>` deliberately. `falcon-cascade-*.wasm`
# also matches falcon-cascade-stream-composed and falcon-cascade-stream-fused,
# and `ls` sorts the stream variants FIRST — so the naive glob silently plugs
# the five stages into the wrong socket and wac dies with a confusing
# "invalid leading byte for component defined type". That is exactly the
# artifact ambiguity jess raised on #202, and it cost a debugging cycle here
# within minutes of writing the script. manifest.json's `kind` field (v1.136)
# is the machine-readable answer; this glob is the shell-level one.
find_one() {  # $1 = stage name
  local f n
  n=$(ls "$DIR"/falcon-"$1"-v[0-9]*.wasm 2>/dev/null | wc -l | tr -d ' ')
  [ "$n" = "1" ] || { echo "ERROR: expected exactly 1 falcon-$1-v*.wasm in $DIR, found $n" >&2; exit 1; }
  f=$(ls "$DIR"/falcon-"$1"-v[0-9]*.wasm)
  echo "$f"
}

SOCKET=$(find_one cascade)
# iekf, NOT ekf: BUILD.bazel's wac_plug plugs the VERIFIED IEKF into the ekf
# socket ("the verified stack is the composed stack"). Composing falcon-ekf
# here would silently produce a different vehicle than the one we verify.
PLUGS=()
for s in iekf position attitude rate mixer; do PLUGS+=(--plug "$(find_one "$s")"); done

wac plug "$SOCKET" "${PLUGS[@]}" -o "$OUT"

echo "composed -> $OUT"
echo "  header      : $(xxd -p -l8 "$OUT")   (0061736d0d000100 = component)"
if command -v wasm-tools >/dev/null 2>&1; then
  echo "  memory.grow : $(wasm-tools print "$OUT" | grep -c 'memory.grow')"
  echo "  wasi imports: $(wasm-tools component wit "$OUT" | grep -c 'wasi:')"
fi
