# Lean 4 proofs

Pure mathematical proofs underpinning Relay's safety claims, checked by the
Lean 4 kernel against Mathlib. Built with Bazel via `rules_lean`.

| Proof | Establishes |
|---|---|
| `WcetAnalysis.lean` | Per-engine worst-case execution-time bounds |
| `BackpressureSafety.lean` | Bounded stream transformers ⇒ no buffer overflow |
| `CompositionalWcet.lean` | Pipeline WCET = Σ stage WCETs + handoff |
| `GeometricLyapunov.lean` | Geometric SE(3) controller: closed-loop `V̇ = −k_Ω‖ω‖² ≤ 0` (v0.23) |

## Pinned versions (reproducibility)

These are fixed inputs — the build is deterministic from them:

- **Lean toolchain:** `4.27.0` (`MODULE.bazel` → `lean.toolchain(version = "4.27.0")`),
  downloaded from `github.com/leanprover/lean4/releases` with a per-platform
  SHA-256 pinned in `rules_lean` (`require_hashes = True`).
- **Mathlib:** revision `v4.27.0` — `rules_lean` ties the Mathlib rev to the
  Lean toolchain version. Pre-built oleans are fetched via `lake exe cache
  get` (Lean's reservoir cache); it falls back to building from source.

## Build / verify

```bash
bazel test //proofs/lean:all          # kernel-check every proof
bazel build //proofs/lean:geometric_lyapunov   # just the v0.23 Lyapunov core
```

First run downloads the Lean toolchain (~hundreds of MB) and Mathlib oleans
(~several GB) — slow. Subsequent runs reuse the repository cache
(`~/.cache/relay-bazel`, see `.bazelrc`).

## Offline vendoring ("redo the bazel stuff if needed")

To make the build reproducible **without network** (and re-runnable if the
upstream Lean/Mathlib release ever moves), vendor the fetched external repos
— including the Mathlib oleans — into `vendor/bazel/`:

```bash
./proofs/lean/vendor.sh        # = bazel vendor //proofs/lean:all --vendor_dir=vendor/bazel
```

`vendor/bazel/` is **gitignored** (multi-GB Mathlib oleans are not committed)
but is fully regeneratable from the pinned versions above. Once populated,
build offline by adding to `.bazelrc`:

```
common --vendor_dir=vendor/bazel
```

This is the "redo" path: the pinned versions + `vendor.sh` reconstruct the
exact toolchain + Mathlib the proofs were checked against.

## Scope note — `GeometricLyapunov.lean`

Mathlib (Feb 2026) has ODE flows but no Lyapunov/LaSalle theory, so the
proof is the **algebraic core** (`V̇` collapses to `−k_Ω‖ω‖² ≤ 0`,
positive-definite `V`); the dynamic LaSalle integration is cited to Lee 2010
Prop. 1. It is backed by a runnable numerical certificate
(`crates/relay-geo` → `lyapunov_decrease_certificate`). See
`docs/research/v0.23-lean-lyapunov-sota.md`.
