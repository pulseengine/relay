//! Relay Limit Checker — formally verified flight software component.
//!
//! Verified replacement for NASA cFS LC (lc_watch.c).
//! Same approach as Gale (verified Zephyr kernel), but for the application layer.
//!
//! Verification tracks:
//! - **Verus (this crate)**: SMT-backed proofs of functional correctness,
//!   bounded output, comparison totality, persistence semantics.
//! - **Rocq (plain/ directory)**: Theorem-prover-backed proofs via coq_of_rust.
//! - **Kani (plain/ directory)**: Bounded model checking harnesses.
//!
//! Architecture:
//!   src/engine.rs       ← Verus-annotated (single source of truth)
//!   plain/src/engine.rs ← verus-strip output (cargo test, Kani, coq_of_rust)
//!   src/c_api.rs        ← cFS-compatible C API (secondary packaging)

pub mod engine;

#[cfg(feature = "c-api")]
pub mod c_api;
