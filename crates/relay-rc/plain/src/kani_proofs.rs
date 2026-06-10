//! Kani harnesses for relay-rc (plain-only sibling).
//!
//! The verification split (the codebase pattern, as in relay-avoid): the
//! COMPARISON/clamp logic is proven here over ALL inputs — a glitching receiver's
//! stick is clamped into [-1,1] (and throttle into [0,1]) BEFORE it is scaled, so
//! it can never command an unbounded angle/rate. The scaled-output bound (the f32
//! multiply by the limit) is proptest-gated — Kani on f32 multiplication is
//! intractable — and follows from the clamp + a non-negative limit.
#![cfg(kani)]

use crate::*;

/// RC-K01 — the stick clamp is the safety floor: `unit(x)` is in [-1, 1] for ANY
/// f32 input (incl. NaN/∞). Centre (0) is the safe default for NaN.
#[kani::proof]
fn verify_unit_clamped() {
    let x: f32 = kani::any();
    let u = unit(x);
    assert!(u >= -1.0 && u <= 1.0);
    if x.is_nan() {
        assert!(u == 0.0);
    }
}

/// RC-K02 — the throttle clamp is in [0, 1] for ANY f32 input; NaN ⇒ 0 (idle).
#[kani::proof]
fn verify_throttle_clamped() {
    let x: f32 = kani::any();
    let t = throttle01(x);
    assert!(t >= 0.0 && t <= 1.0);
    if x.is_nan() {
        assert!(t == 0.0);
    }
}
