# Lean 4 proofs

Pure mathematical proofs underpinning Relay's safety claims, checked by the
Lean 4 kernel against Mathlib. Built with Bazel via `rules_lean`.

| Proof | Establishes |
|---|---|
| `WcetAnalysis.lean` | Per-engine worst-case execution-time bounds |
| `BackpressureSafety.lean` | Bounded stream transformers ⇒ no buffer overflow |
| `CompositionalWcet.lean` | Pipeline WCET = Σ stage WCETs + handoff |
| `GeometricLyapunov.lean` | Geometric SE(3) controller: closed-loop `V̇ = −k_Ω‖ω‖² ≤ 0` (v0.23) |
| `PositionLyapunov.lean` | Translational + full-state `V̇ = −k_Ω‖ω‖² − k_v‖e_v‖² ≤ 0` (v0.38) |
| `StrictLyapunov.lean` | Strict (cross-term) Lyapunov: `c_hi·V̇ ≤ −c_D·V` — the exponential-decay/Grönwall inequality (v1.105) |
| `LyapunovConvergence.lean` | `V̇ ≤ −γV ⇒ V(t) ≤ V(0)e^(−γt) → 0` — the deferred "trajectory ⇒ converges" step, via integrating factor (v1.107) |

## Pinned versions (reproducibility)

These are fixed inputs — the build is deterministic from them:

- **Lean toolchain:** `4.29.1` (`MODULE.bazel` → `lean.toolchain(version = "4.29.1")`),
  downloaded from `github.com/leanprover/lean4/releases` with a per-platform
  SHA-256 pinned in `rules_lean` (`require_hashes = True`). This MUST match
  `rules_lean`'s own `MODULE.bazel` pin — the extension collects tags from
  all modules and `rules_lean` contributes the Mathlib tag, so a mismatched
  toolchain here forces an Elan-based toolchain switch mid-build that fails.
- **Mathlib:** revision `v4.29.1` (pinned in `rules_lean`'s `MODULE.bazel`,
  `lean.mathlib(rev = "v4.29.1")`). Pre-built oleans are fetched via `lake
  exe cache get`; it falls back to building from source. `lake update`
  clones mathlib4 (large) — `rules_lean` allows up to 3600 s for it.

## Build / verify

```bash
bazel test //proofs/lean:all          # kernel-check every proof
bazel build //proofs/lean:geometric_lyapunov   # just the v0.23 Lyapunov core
```

First run downloads the Lean toolchain (~hundreds of MB) and Mathlib oleans
(~several GB) — slow. Subsequent runs reuse the repository cache
(`~/.cache/relay-bazel`, see `.bazelrc`); a cached incremental rebuild of a
single proof is ~10 s.

`GeometricLyapunov.lean` has been **kernel-verified locally** (the
`geometric_lyapunov_test` target passes, zero `sorry`/`axiom`).

## Offline vendoring ("redo the bazel stuff if needed")

To make the build reproducible **without network** (and re-runnable if the
upstream Lean/Mathlib release ever moves), vendor the fetched external repos
— including the Mathlib oleans — into `vendor/bazel/`:

```bash
./proofs/lean/vendor.sh        # = bazel vendor //proofs/lean:all --vendor_dir=vendor/bazel
```

`vendor/bazel/` is **gitignored** (it vendors the *entire* transitive dep
set — ~24 GB apparent, though bazel hardlinks to the repository cache so
real extra disk is far less) but is fully regeneratable from the pinned
versions above. Once populated, build offline by adding to `.bazelrc`:

```
common --vendor_dir=vendor/bazel
```

This is the "redo" path: the pinned versions + `vendor.sh` reconstruct the
exact toolchain + Mathlib the proofs were checked against. **Confirmed
working** — `bazel build //proofs/lean:geometric_lyapunov --vendor_dir=
vendor/bazel` builds from the vendored copy without re-fetching.

## Scope note — `GeometricLyapunov.lean`

Mathlib (Feb 2026) has ODE flows but no Lyapunov/LaSalle theory, so the
proof is the **algebraic core** (`V̇` collapses to `−k_Ω‖ω‖² ≤ 0`,
positive-definite `V`); the dynamic LaSalle integration is cited to Lee 2010
Prop. 1. It is backed by a runnable numerical certificate
(`crates/relay-geo` → `lyapunov_decrease_certificate`). See
`docs/research/v0.23-lean-lyapunov-sota.md`.
