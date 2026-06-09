//! Kani harnesses for relay-avoid (plain-only sibling). The COMPARISON/clamp
//! logic is proven here; the f32 √ in braking_speed/precision_velocity is
//! proptest-gated (Kani on libm sqrt is intractable), and its outputs are
//! guarded ≥0/finite by construction so these bounds hold for any √ value.
#![cfg(kani)]

use crate::*;

/// AVOID-K01 — the approach cap is SAFE: for any commanded speed and any allowed
/// speed, the result never exceeds either, and is never negative (a faster
/// command can never push the vehicle past the braking-limited speed).
#[kani::proof]
fn verify_cap_approach_safe() {
    let v_cmd: f32 = kani::any();
    let v_allowed: f32 = kani::any();
    // The allowed speed comes from braking_speed, which is guarded ≥0 finite.
    kani::assume(v_allowed.is_finite() && v_allowed >= 0.0);
    let r = cap_approach(v_cmd, v_allowed);
    assert!(r >= 0.0);
    assert!(r <= v_allowed);
    if v_cmd.is_finite() && v_cmd >= 0.0 {
        assert!(r <= v_cmd);
    }
}

/// AVOID-K02 — `within_radius` is total for any offset/radius (no panic), and
/// the origin is always within any non-negative radius.
#[kani::proof]
fn verify_within_radius_total() {
    let off = [kani::any::<f32>(), kani::any::<f32>()];
    let radius: f32 = kani::any();
    let _ = within_radius(off, radius);
    let r = kani::any::<f32>();
    kani::assume(r.is_finite() && r >= 0.0);
    assert!(within_radius([0.0, 0.0], r));
}
