//! Relay Limit Checker — formally verified flight software component.
//!
//! LC is now a *composition* of verified primitives plus LC-specific
//! glue (watchpoint table, sensor-id matching, bounded output).
//!
//! Verification tracks:
//! - **Verus (this crate)**: SMT-backed proofs of functional correctness,
//!   bounded output, comparison totality, persistence semantics.
//! - **Rocq (plain/ directory)**: Theorem-prover-backed proofs via coq_of_rust.
//! - **Kani (plain/ directory)**: Bounded model checking harnesses.
//!
//! Architecture:
//!   src/engine.rs       ← Verus-annotated (single source of truth)
//!   plain/src/engine.rs ← Cargo-buildable mirror (kept in sync by verus_strip_test)
//!   src/c_api.rs        ← cFS-compatible C API (secondary packaging)
//!
//! Cross-crate import: relay-primitives lives in `../relay-primitives`.
//! For Verus (Bazel), we re-import its modules via `#[path]` so the
//! verus_test sees them as submodules of `relay_lc`. For Cargo, the
//! plain/src/lib.rs uses a normal `pub use relay_primitives::...;`.

#[path = "../../relay-primitives/src/compare.rs"]
pub mod compare;

#[path = "../../relay-primitives/src/persistence.rs"]
pub mod persistence;

pub mod engine;

#[cfg(feature = "c-api")]
pub mod c_api;
