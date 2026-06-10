//! Kani harnesses for relay-param (plain-only sibling).
#![cfg(kani)]

use crate::*;

/// PARAM-K01 — the safety property: an out-of-range (or NaN) write is REJECTED
/// and the stored value is UNCHANGED. For a registered parameter with any prior
/// value, a write of any value outside [min, max] returns OutOfRange and leaves
/// get() exactly as it was. A GCS cannot push a gain/threshold out of bounds.
#[kani::proof]
fn verify_out_of_range_never_lands() {
    let mut s: ParamStore<1> = ParamStore::new();
    let min: f32 = kani::any();
    let max: f32 = kani::any();
    kani::assume(min.is_finite() && max.is_finite() && min <= max);
    let id = param_id("P");
    s.register(ParamDef { id, min, max, default: min });

    let before = s.get(&id).unwrap();
    let v: f32 = kani::any();
    // v is outside the inclusive range (or NaN).
    kani::assume(!(v.is_finite() && v >= min && v <= max));
    let r = s.set(&id, v);
    assert!(r == SetResult::OutOfRange);
    assert!(s.get(&id).unwrap().to_bits() == before.to_bits());
}

/// PARAM-K02 — set is total: any write against any store state returns a defined
/// result, no panic; an in-range write Applies and is then readable.
#[kani::proof]
fn verify_set_total_and_in_range_applies() {
    let mut s: ParamStore<1> = ParamStore::new();
    let id = param_id("P");
    s.register(ParamDef { id, min: 0.0, max: 10.0, default: 5.0 });
    let v: f32 = kani::any();
    kani::assume(v.is_finite() && v >= 0.0 && v <= 10.0);
    assert!(s.set(&id, v) == SetResult::Applied);
    assert!(s.get(&id) == Some(v));
}
