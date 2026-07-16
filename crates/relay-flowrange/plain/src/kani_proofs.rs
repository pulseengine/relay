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

/// FLOWRANGE-K03 (RANGEDRV-P01) — TF02 decode is TOTAL and its quality gate
/// is SOUND over ALL 9-byte patterns: no panic, and an accepted sample is
/// always inside the 0.1–40 m envelope with reliable strength. Integer-only
/// (the f32 is a cm cast), so fully tractable.
#[kani::proof]
fn verify_tf02_decode_total_gate_sound() {
    let frame: [u8; tf02::TF02_FRAME_LEN] = kani::any();
    match tf02::decode_tf02_frame(&frame) {
        Ok(s) => {
            assert!(s.strength >= tf02::TF02_STRENGTH_MIN && s.strength != 0xFFFF);
            assert!(s.distance_m >= 0.1 && s.distance_m <= 40.0);
        }
        Err(_) => {}
    }
}

/// FLOWRANGE-K04 (RANGEDRV-P01) — the streaming scanner is total and its
/// consumed count never exceeds the input for ANY 16-byte stream.
#[kani::proof]
#[kani::unwind(20)]
fn verify_tf02_scan_total() {
    let bytes: [u8; 16] = kani::any();
    let n: usize = kani::any();
    kani::assume(n <= 16);
    let (_, used) = tf02::scan_tf02(&bytes[..n]);
    assert!(used <= n);
}
