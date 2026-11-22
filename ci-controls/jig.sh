#!/bin/sh
# Runs jig for a planted control: $JIG when set (a local build), else the
# pinned release, checked against the sha256 from its cosign-verified
# SHA256SUMS.txt (jig v0.1.4, x86_64 linux musl, 2026-11-22). Same version
# and sum as the jig record commit ci.yml pins (required.yml JIG_SHA256).
set -eu
if [ -n "${JIG:-}" ]; then exec "$JIG" "$@"; fi
v=v0.1.4
sum=3ceb9ead350265d3c8d24bc7c2013466744a01bb32a54fb7b6764fec949fac45
a="jig-$v-x86_64-unknown-linux-musl.tar.gz"
d="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/relay-controls-jig-$v"
if [ ! -x "$d/jig" ]; then
  mkdir -p "$d"
  curl -fsSL -o "$d/$a" "https://github.com/pulseengine/jig/releases/download/$v/$a"
  (cd "$d" && echo "$sum  $a" | sha256sum -c - >/dev/null && tar -xzf "$a")
fi
exec "$d/jig" "$@"
