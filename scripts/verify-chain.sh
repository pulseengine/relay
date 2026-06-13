#!/usr/bin/env bash
# verify-chain.sh — run/enumerate the four-track verification chain (#4) for an
# engine: Verus (Z3), Kani (CBMC), Rocq, Lean. Four independent techniques
# confirming the same engine. See docs/VERIFICATION-CHAIN.md.
#
#   scripts/verify-chain.sh relay-lc     # one engine
#   scripts/verify-chain.sh --all        # all five
#
# Kani runs locally if cargo-kani is present. Verus/Rocq are Bazel targets (need
# the Bazel toolchain) — named, and run if `bazel` is on PATH. Lean proofs are
# listed. Each track reports RUN(pass/fail) / PRESENT(not run here) / ABSENT.
# Exit non-zero only if a track that ACTUALLY RAN failed.
set -uo pipefail
cd "$(dirname "$0")/.."

ENGINES=(relay-lc relay-sch relay-sc relay-hs relay-cfdp)
[ "${1:-}" = "--all" ] && targets=("${ENGINES[@]}") || targets=("${1:-relay-lc}")

have() { command -v "$1" >/dev/null 2>&1; }
rc=0

for e in "${targets[@]}"; do
  short="${e#relay-}"
  echo "══ $e ════════════════════════════════════════════════"

  # ── Kani (runs locally) ───────────────────────────────────────────────
  if have cargo-kani; then
    if cargo kani -p "$e" >/tmp/vc-kani-$short.log 2>&1; then
      sum=$(grep -E "successfully verified harnesses" /tmp/vc-kani-$short.log | tail -1)
      echo "  Kani  : RUN  ✅ ${sum:-passed}"
    else
      echo "  Kani  : RUN  ❌ FAILED (see /tmp/vc-kani-$short.log)"; rc=1
    fi
  else
    n=$(grep -cE "kani::proof" "crates/$e/plain/src/kani_proofs.rs" 2>/dev/null || echo 0)
    echo "  Kani  : PRESENT ($n harnesses in plain/src/kani_proofs.rs) — cargo-kani not installed"
  fi

  # ── Verus (Bazel target) ──────────────────────────────────────────────
  vt="relay_${short}_verus_test"
  if grep -q "$vt" BUILD.bazel 2>/dev/null; then
    if have bazel; then
      if bazel test "//:$vt" >/tmp/vc-verus-$short.log 2>&1; then
        echo "  Verus : RUN  ✅ //:$vt"
      else echo "  Verus : RUN  ❌ //:$vt (see /tmp/vc-verus-$short.log)"; rc=1; fi
    else
      echo "  Verus : PRESENT (//:$vt) — bazel not on PATH"
    fi
  else
    echo "  Verus : ABSENT (no $vt in BUILD.bazel)"
  fi

  # ── Rocq (Bazel target) ───────────────────────────────────────────────
  if [ -f "proofs/rocq/${short}_proofs.v" ]; then
    if have bazel; then
      if bazel test "//proofs/rocq:${short}_proofs_test" >/tmp/vc-rocq-$short.log 2>&1; then
        echo "  Rocq  : RUN  ✅ //proofs/rocq:${short}_proofs_test"
      else echo "  Rocq  : RUN  ❌ (see /tmp/vc-rocq-$short.log)"; rc=1; fi
    else
      echo "  Rocq  : PRESENT (proofs/rocq/${short}_proofs.v) — bazel not on PATH"
    fi
  else
    echo "  Rocq  : ABSENT"
  fi

  # ── Lean (system-level proofs) ────────────────────────────────────────
  lean_n=$(ls proofs/lean/*.lean 2>/dev/null | wc -l | tr -d ' ')
  echo "  Lean  : PRESENT ($lean_n system-level proofs in proofs/lean/ — WCET/backpressure/dynamics)"
done

echo "═══════════════════════════════════════════════════════"
[ "$rc" = 0 ] && echo "No RUN track failed." || echo "A track that ran FAILED — see logs above."
exit "$rc"
