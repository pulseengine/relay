#!/usr/bin/env bash
# kani.yml's change detector: does this change need the Kani matrix?
# Writes run=true|false (and reason= when it skips) to $GITHUB_OUTPUT.
#
# FAILS CLOSED. The matrix is skipped only when EVERY changed path is known to
# be irrelevant to the Kani harnesses (all live under crates/<engine>/plain/).
# Anything unclassified runs the matrix, so a new kind of input (a .rs under
# docs/, a supply-chain/ file, a new top-level directory) is gated by default.
# The previous detector was an allowlist (run only on crates/, Cargo.*,
# kani.yml) and failed open for exactly those inputs (temper#86,
# detector_unprobed).
#
# Probe contract (pulseengine/temper docs/gate-probe.md, .github/gate-probe.json):
# when GATE_PROBE_FILES is set it names a file listing the changed paths, one
# per line, and is the ONLY input; no git, no event. Otherwise the paths come
# from the git diff the workflow describes below.
set -euo pipefail
out=${GITHUB_OUTPUT:?GITHUB_OUTPUT must be set}

# 0 = known-irrelevant to Kani, 1 = relevant. If unsure, relevant.
is_known_irrelevant() {
  local p=$1
  case "$p" in
    crates/*|Cargo.toml|Cargo.lock|*/Cargo.toml|rust-toolchain|rust-toolchain.toml|.cargo/*|.github/workflows/kani.yml|scripts/kani-scope.sh) return 1 ;;
  esac
  case "$p" in
    *.md|*.yaml|*.yml|*.png|*.svg|*.jpg|*.jpeg|*.gif|*.mp4|*.webm|*.txt|*.csv|*.aadl|*.wit) ;;
    *) return 1 ;;
  esac
  # A file a crate names (include_str!/include_bytes!) can change what a crate
  # compiles: falcon-hitl's wire.rs includes docs/TRANSPORT-BINDING.md. grep
  # exit 1 = not named; anything else (0 named, 2 could not tell) = relevant.
  local rc=0
  grep -rqF -- "${p##*/}" crates || rc=$?
  [ "$rc" -eq 1 ] || return 1
  return 0
}

classify() {
  local changed=$1 p rel=0
  if [ -z "$changed" ]; then
    echo "empty change list — fail-safe to run"
    echo "run=true" >>"$out"; return
  fi
  while IFS= read -r p; do
    [ -n "$p" ] || continue
    if ! is_known_irrelevant "$p"; then
      echo "Kani-relevant (or unclassified): $p"
      rel=1; break
    fi
  done <<<"$changed"
  if [ "$rel" -eq 1 ]; then
    echo "run=true" >>"$out"
  else
    echo "no Kani-relevant files changed (every changed path is known Kani-irrelevant) — SKIP the matrix (kani-gate reports pass)"
    printf 'changed:\n%s\n' "$changed"
    echo "run=false" >>"$out"
    echo "reason=out-of-scope" >>"$out"
  fi
}

if [ -n "${GATE_PROBE_FILES:-}" ]; then
  classify "$(cat "$GATE_PROBE_FILES")"
  exit 0
fi

# QUEUE-ONLY: a pull request never runs the matrix — the merge queue does, on
# the commit that will land. Decided before any diffing.
if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ]; then
  echo "pull_request — Kani DEFERRED to the merge queue (merge_group runs the matrix)"
  echo "run=false" >>"$out"
  echo "reason=deferred" >>"$out"
  exit 0
fi
# Three ways a base can be unusable, each of which must SWEEP rather than skip.
case "${BASE_SHA:-}" in
  "")
    echo "no base sha (manual dispatch) — run the full matrix (backstop)"
    echo "run=true" >>"$out"; exit 0 ;;
  0000000000000000000000000000000000000000)
    echo "all-zeros base (branch created / no predecessor) — full matrix (backstop)"
    echo "run=true" >>"$out"; exit 0 ;;
esac
# A force-push can leave `before` pointing at an unreachable commit.
if ! git cat-file -e "${BASE_SHA}^{commit}" 2>/dev/null; then
  echo "base $BASE_SHA is not in this history (force-push?) — full matrix (backstop)"
  echo "run=true" >>"$out"; exit 0
fi
echo "scoping against base $BASE_SHA (event: ${GITHUB_EVENT_NAME:-?})"
if ! CHANGED=$(git diff --name-only "$BASE_SHA"...HEAD); then
  echo "diff failed — fail-safe to run"
  echo "run=true" >>"$out"; exit 0
fi
classify "$CHANGED"
