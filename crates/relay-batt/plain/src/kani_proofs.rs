//! Kani harnesses for relay-batt (plain-only sibling).
//!
//! The verification split (the relay-rc pattern): input SANITIZATION and
//! the trip/latch state machine are proven total here over all inputs;
//! the f32 arithmetic paths (coulomb integration, sag compensation, OCV
//! interpolation) are test- and proptest-gated — Kani on nondet f32
//! multiplication is intractable.
#![cfg(kani)]

use crate::{sanitize, TripLatch};

/// BATT-K01 — `sanitize` is total and in-range for ANY f32 input
/// (incl. NaN/±∞), provided the bounds are ordered and the NaN default
/// is inside them: the result is always finite and within [lo, hi].
#[kani::proof]
fn verify_sanitize_total_in_range() {
    let x: f32 = kani::any();
    let lo: f32 = kani::any();
    let hi: f32 = kani::any();
    let nan_default: f32 = kani::any();
    kani::assume(lo.is_finite() && hi.is_finite() && lo <= hi);
    kani::assume(nan_default >= lo && nan_default <= hi);
    let y = sanitize(x, lo, hi, nan_default);
    assert!(y.is_finite());
    assert!(y >= lo && y <= hi);
}

/// BATT-K02 — the latch never clears: for ANY update sequence, once
/// `update` has returned true, every later call returns true (a pack
/// does not un-discharge in flight).
#[kani::proof]
#[kani::unwind(6)]
fn verify_latch_never_clears() {
    let mut latch = TripLatch::default();
    let debounce: f32 = kani::any();
    kani::assume(debounce.is_finite() && debounce >= 0.0);
    let mut tripped = false;
    for _ in 0..4 {
        let dt: f32 = kani::any();
        kani::assume(dt.is_finite() && dt >= 0.0);
        let below: bool = kani::any();
        let out = latch.update(dt, below, debounce);
        if tripped {
            assert!(out, "a latched flag must never clear");
        }
        tripped = tripped || out;
    }
}

/// BATT-K03 — no trip without a sustained excursion: while the excursion
/// condition has never been true, the latch stays clear regardless of
/// dt values (time alone cannot trip a healthy pack).
#[kani::proof]
#[kani::unwind(6)]
fn verify_no_trip_without_excursion() {
    let mut latch = TripLatch::default();
    let debounce: f32 = kani::any();
    kani::assume(debounce.is_finite() && debounce > 0.0);
    for _ in 0..4 {
        let dt: f32 = kani::any();
        kani::assume(dt.is_finite() && dt >= 0.0);
        let out = latch.update(dt, false, debounce);
        assert!(!out, "no excursion, no trip");
    }
}
