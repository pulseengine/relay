//! Kani harnesses for relay-flowrange (plain-only sibling). The GATING logic
//! (comparison) is proven over all inputs; the f32 cos/multiply is concrete/
//! proptest territory.
#![cfg(kani)]

use crate::*;

/// FLOWRANGE-K01 — the rangefinder gate: for any range with concrete sane band
/// and tilt, an out-of-band or NaN range yields None (a bad reading never
/// becomes an altitude); the function is total (no panic).
#[kani::proof]
fn verify_range_gated() {
    let r: f32 = f32::from_bits(kani::any());
    // concrete band + level tilt so cos is concrete (cos in Kani is intractable).
    let out = range_to_altitude(r, 0.0, 0.2, 30.0);
    if !(r.is_finite() && r >= 0.2 && r <= 30.0) {
        assert!(out.is_none());
    }
    let _ = out; // total
}

/// FLOWRANGE-K02 — the optical-flow gate: any non-positive / non-finite height
/// yields None (flow gives no velocity without a height reference); total.
#[kani::proof]
fn verify_flow_gated() {
    let fx: f32 = f32::from_bits(kani::any());
    let fy: f32 = f32::from_bits(kani::any());
    let h: f32 = f32::from_bits(kani::any());
    let out = flow_to_velocity(fx, fy, 0.0, 0.0, h);
    if !(h.is_finite() && h > 0.0) {
        assert!(out.is_none());
    }
}
