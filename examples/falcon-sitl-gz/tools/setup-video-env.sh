#!/usr/bin/env bash
# setup-video-env.sh — stand up everything the falcon release-video tooling needs.
#
# WHY THIS EXISTS: the video scripts (gen-release-card.py, make-release-video.sh,
# narrate-release-videos.sh) assume a Python with Pillow exists at
# /tmp/plotenv/bin/python — but nothing ever created it, and /tmp gets wiped, so
# the pipeline broke silently. This script is the missing setup step:
#
#   1. creates a STABLE venv (under the repo's target/, which is git-ignored and
#      survives reboots — NOT /tmp) and installs Pillow into it;
#   2. verifies ffmpeg, gz, and the TTS (Speaches) server are reachable;
#   3. prints a clear PASS / FAIL for each prerequisite.
#
# The other scripts pick up this venv automatically: they default PYTHON to
# $REPO/target/video-venv/bin/python (this script's VENV), falling back to the
# legacy /tmp/plotenv only if that's what already exists.
#
# Usage:
#   examples/falcon-sitl-gz/tools/setup-video-env.sh          # set up + verify
#   examples/falcon-sitl-gz/tools/setup-video-env.sh --check  # verify only, no install
#
# Env:
#   VENV         override the venv path (default: $REPO/target/video-venv)
#   SPEACHES_URL TTS server to probe   (default: http://192.168.178.28:8000)
#   PIP_ARGS     extra args to pip install (e.g. --index-url ... for a mirror)
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
VENV="${VENV:-$REPO/target/video-venv}"
SPEACHES_URL="${SPEACHES_URL:-http://192.168.178.28:8000}"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# track overall status: 0 = all required deps OK
RC=0
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; RC=1; }
warn() { printf '  \033[33mWARN\033[0m  %s\n' "$1"; }

echo "== falcon video env setup =="
echo "   repo:  $REPO"
echo "   venv:  $VENV"
echo "   tts:   $SPEACHES_URL"
echo

# ── 1. Python venv + Pillow ───────────────────────────────────────────────
PY3="$(command -v python3 || true)"
if [ -z "$PY3" ]; then
  fail "python3 not found on PATH (need it to build the venv)"
else
  if [ "$CHECK_ONLY" -eq 0 ]; then
    if [ ! -x "$VENV/bin/python" ]; then
      echo "-- creating venv at $VENV"
      "$PY3" -m venv "$VENV" || fail "python3 -m venv failed"
    fi
    if [ -x "$VENV/bin/python" ]; then
      echo "-- installing Pillow into the venv"
      # gen-release-card.py imports PIL (Pillow). That's the only third-party dep
      # across the video tooling; everything else is stdlib (json, urllib).
      "$VENV/bin/python" -m pip install --quiet --upgrade pip >/dev/null 2>&1 || true
      if ! "$VENV/bin/python" -m pip install --quiet ${PIP_ARGS:-} Pillow >/tmp/video-pip.log 2>&1; then
        warn "pip install Pillow failed (network blocked?) — see /tmp/video-pip.log"
        warn "re-run this script when network is available; venv is ready otherwise"
      fi
    fi
  fi

  # verify Pillow imports from the venv (the real gate)
  if [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -c 'import PIL' >/dev/null 2>&1; then
    PILV="$("$VENV/bin/python" -c 'import PIL; print(PIL.__version__)' 2>/dev/null)"
    pass "Pillow $PILV importable from venv python"
  else
    fail "Pillow NOT importable from $VENV/bin/python (run without --check to install)"
  fi
fi

# ── 2. ffmpeg ─────────────────────────────────────────────────────────────
if command -v ffmpeg >/dev/null 2>&1; then
  FFV="$(ffmpeg -version 2>/dev/null | head -1 | awk '{print $3}')"
  pass "ffmpeg $FFV ($(command -v ffmpeg))"
else
  fail "ffmpeg not found (needed to encode/overlay/mux video)"
fi

# ── 3. gz (Gazebo) — OPTIONAL: only needed to capture fresh flight footage ──
if command -v gz >/dev/null 2>&1; then
  GZV="$(gz sim --versions 2>/dev/null | head -1)"
  pass "gz sim $GZV ($(command -v gz)) — fresh-footage capture available"
else
  warn "gz not found — fresh flight capture (record_headless.sh) unavailable;"
  warn "      composing from existing relvids/ footage still works"
fi

# ── 4. TTS / Speaches server — OPTIONAL: only needed for narration ─────────
if curl -s -m 8 "$SPEACHES_URL/v1/models" 2>/dev/null | grep -q "Kokoro"; then
  pass "TTS server reachable at $SPEACHES_URL (Kokoro model present)"
elif curl -s -m 8 "$SPEACHES_URL/v1/models" >/dev/null 2>&1; then
  warn "TTS server reachable but Kokoro model not advertised at $SPEACHES_URL"
else
  warn "TTS server NOT reachable at $SPEACHES_URL — videos render silent;"
  warn "      set SPEACHES_URL or skip narration"
fi

echo
if [ "$RC" -eq 0 ]; then
  echo "== READY. Use this python in the video scripts:"
  echo "     export PYTHON=$VENV/bin/python"
  echo "   (make-release-video.sh / make-achievements-video.sh default to it.)"
else
  echo "== NOT READY — fix the FAIL lines above (PASS = required dep is satisfied)."
fi
exit "$RC"
