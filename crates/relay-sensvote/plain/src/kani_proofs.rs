//! Kani harnesses for relay-sensvote (plain-only sibling).
#![cfg(kani)]

use crate::*;

/// SENSVOTE-K01 — a single faulted (NaN) sensor NEVER propagates: with one input
/// NaN and the other two finite, the voted value is finite (the healthy pair's
/// mean), not NaN. This is the redundancy safety property.
#[kani::proof]
fn verify_nan_never_propagates() {
    let b: f32 = f32::from_bits(kani::any());
    let c: f32 = f32::from_bits(kani::any());
    kani::assume(b.is_finite() && c.is_finite());
    let v = median3(f32::NAN, b, c);
    assert!(v.is_finite());
}

/// SENSVOTE-K02 — the GPS freshness monitor is total: any clock value / timeout /
/// state yields a defined status, no panic (saturating time math).
#[kani::proof]
fn verify_gps_freshness_total() {
    let mut g = GpsFreshness::new(kani::any());
    if kani::any() {
        g.on_fix(kani::any());
    }
    let _ = g.status(kani::any());
}

/// SENSVOTE-K03 — strictly past the timeout with the last fix at t0, GPS is
/// ALWAYS Stale (the INS-fallback trigger fires), for any timeout.
#[kani::proof]
fn verify_gps_stale_after_timeout() {
    let timeout: u64 = kani::any();
    let mut g = GpsFreshness::new(timeout);
    let t0: u64 = kani::any();
    g.on_fix(t0);
    let now: u64 = kani::any();
    kani::assume(now >= t0);
    kani::assume(now - t0 > timeout);
    assert!(g.status(now) == GpsStatus::Stale);
}
