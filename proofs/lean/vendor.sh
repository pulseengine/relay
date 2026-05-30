#!/usr/bin/env bash
# Vendor the Lean toolchain + Mathlib oleans + external repos into
# vendor/bazel/ so the Lean proofs can be re-checked WITHOUT network.
# Regeneratable from the pinned versions (see proofs/lean/README.md).
#
# After running, build offline by adding to .bazelrc:
#   common --vendor_dir=vendor/bazel
set -euo pipefail
cd "$(dirname "$0")/../.."
echo "Vendoring Lean/Mathlib into vendor/bazel/ (multi-GB, slow first time)…"
bazel vendor //proofs/lean:all --vendor_dir=vendor/bazel
echo "Done. Enable offline builds with:  common --vendor_dir=vendor/bazel"
