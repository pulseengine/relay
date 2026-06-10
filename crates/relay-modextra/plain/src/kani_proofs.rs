//! Kani harnesses for relay-modextra (plain-only sibling).
//!
//! Inputs are bounded to the OPERATIONAL ENVELOPE (|coord| ≤ 1e6 m — a thousand
//! km, far beyond any flight) and assumed finite: a position near f32::MAX is not
//! a physical input, and `finite + finite` f32 arithmetic CAN overflow at those
//! magnitudes (Kani caught exactly that). Within the envelope the setpoint
//! generators are total and finite.
#![cfg(kani)]

use crate::*;

const ENV: f32 = 1.0e6;

fn coord() -> f32 {
    let x: f32 = kani::any();
    kani::assume(x.is_finite() && x.abs() <= ENV);
    x
}

fn env3() -> Ned {
    [coord(), coord(), coord()]
}

/// MODEXTRA-K01 — within the operational envelope the follow / land / RTL
/// setpoint generators are total and emit a FINITE setpoint (no NaN/∞ flown into
/// the controller) for any in-envelope pose, any descent rate, any dt.
#[kani::proof]
fn verify_setpoints_finite() {
    let f = follow_setpoint(env3(), env3());
    assert!(f[0].is_finite() && f[1].is_finite() && f[2].is_finite());

    let rate = coord();
    let dt = coord();
    let l = land_in_place(env3(), rate, dt);
    assert!(l[0].is_finite() && l[1].is_finite() && l[2].is_finite());

    let r = rtl_setpoint(env3(), coord());
    assert!(r[0].is_finite() && r[1].is_finite() && r[2].is_finite());
}

/// MODEXTRA-K02 — land_in_place never commands ABOVE the current altitude: the z
/// setpoint is ≥ the current z (down is +z, so only descend/hold) for any
/// in-envelope altitude, descent rate, and dt.
#[kani::proof]
fn verify_land_only_descends() {
    let z = coord();
    let rate = coord();
    let dt = coord();
    let sp = land_in_place([0.0, 0.0, z], rate, dt);
    assert!(sp[2] >= z);
}
