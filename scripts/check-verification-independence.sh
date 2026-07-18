#!/usr/bin/env bash
# Two-commit rule (v1.116, rivet hygiene): a PR that changes IMPLEMENTATION
# code must not, in the same PR, promote any rivet artifact to a terminal
# verification status (`verified` / `accepted`). Verification sign-off has to
# arrive in a SEPARATE PR that runs the cited evidence — otherwise the party
# claiming "verified" is the same change being verified (the traceability-audit
# finding: zero independence in status transitions).
#
# Usage: check-verification-independence.sh <base-ref>
#   <base-ref> — the merge base to diff against (CI passes the PR base SHA).
#
# Break-glass: set ALLOW_SAME_PR_VERIFIED=1 (e.g. via a workflow re-run with
# the env var) for a genuine emergency; the override itself is visible in the
# CI log, which is the point.
set -euo pipefail

BASE="${1:?usage: check-verification-independence.sh <base-ref>}"

if [ "${ALLOW_SAME_PR_VERIFIED:-0}" = "1" ]; then
    echo "::warning::two-commit rule OVERRIDDEN via ALLOW_SAME_PR_VERIFIED=1"
    exit 0
fi

# Implementation code touched by this PR (flight code, not artifacts/docs).
code_changed=$(git diff --name-only "$BASE"...HEAD -- 'crates/' 'examples/' | grep -E '\.rs$' || true)

# Artifact lines this PR ADDS that set a terminal verification status.
verified_added=$(git diff "$BASE"...HEAD -- 'artifacts/' | grep -E '^\+\s*status:\s*(verified|accepted)\b' || true)

if [ -n "$code_changed" ] && [ -n "$verified_added" ]; then
    echo "::error::TWO-COMMIT RULE: this PR changes implementation code AND promotes artifact status to verified/accepted in the same change."
    echo ""
    echo "Code changed:"
    echo "$code_changed" | sed 's/^/  /' | head -20
    echo ""
    echo "Terminal-status lines added:"
    echo "$verified_added" | sed 's/^/  /' | head -10
    echo ""
    echo "Fix: land the implementation with status at most 'implemented'; promote to"
    echo "'verified' in a follow-up PR that runs the cited evidence (the verification"
    echo "gate executes the artifact's steps there). Independence is the point."
    exit 1
fi

echo "two-commit rule: OK (code-changed=$([ -n "$code_changed" ] && echo yes || echo no), terminal-status-added=$([ -n "$verified_added" ] && echo yes || echo no))"
