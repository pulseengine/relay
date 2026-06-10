//! Kani harnesses for relay-mix-multi (plain-only sibling).
//!
//! The clamp is the airframe-agnostic safety floor: every rotor command passes
//! through clamp01, so whatever the geometry's f32 allocation computes, the
//! motor never sees a negative / over-unity / NaN value. Kani proves the clamp
//! over all f32; the full per-geometry allocation range is proptest-gated (the
//! f32 matrix multiply is intractable for CBMC).
#![cfg(kani)]

use crate::*;

/// MIXMULTI-K01 — the mixer output clamp is in [0,1] for ANY allocated value
/// (incl. NaN/∞). This is the invariant every rotor command of every geometry
/// inherits: no negative thrust, never over-unity.
#[kani::proof]
fn verify_clamp01_in_unit_interval() {
    let x: f32 = kani::any();
    let c = clamp01(x);
    assert!(c >= 0.0 && c <= 1.0);
    if x.is_nan() {
        assert!(c == 0.0);
    }
}
