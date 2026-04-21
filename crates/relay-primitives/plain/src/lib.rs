//! Relay Primitives — verified domain-agnostic kernels.
//!
//! This crate contains the small, pure, formally verified building blocks
//! that every Relay stream transformer composes from. Nothing in here is
//! cFS-specific, spacecraft-specific, or even space-specific. These are
//! the universal primitives: integrity checks, comparison, hysteresis,
//! rate division, time gating, wire-format codecs.
//!
//! Each primitive is:
//!   - A pure function (no mutation in the decision path)
//!   - Verus-annotated with ensures clauses
//!   - no_std, no_alloc, no trait objects, no closures
//!   - Reusable across spacecraft, drones, ECUs, PLCs, medical, industrial
//!
//! Transformers (the wrappers that lift primitives into stream<T> → stream<U>)
//! live in the `relay-transformers` crate.
//!
//! Compositional proofs (WCET(A ∘ B) ≤ WCET(A) + WCET(B) + overhead,
//! mem(A ∘ B) ≤ mem(A) + mem(B) + buffer) live in proofs/rocq and proofs/lean.
#![no_std]
pub mod crc32;
pub mod compare;
pub mod persistence;
pub mod rate_divide;
pub mod time_gate;
pub mod ccsds;
pub mod merge;
pub mod filter;
