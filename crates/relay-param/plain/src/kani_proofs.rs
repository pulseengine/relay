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

// ── PARAM-P03 persistence: WHY there is no direct Kani harness ──────────────
//
// Any nondet data through the persist layer's CRC-32 is CBMC-intractable
// (measured: a fully-nondet 96-byte image >28 CPU-min, even a SINGLE nondet
// byte at a nondet position >20 CPU-min — the relay-traj pattern, #260). A
// proof that does not terminate provides no assurance and would hang the
// required Kani CI gate. The persistence guarantees are instead established
// COMPOSITIONALLY + exhaustively:
//
//   * Schema safety: `persist::load`'s ONLY store mutation is its single
//     `store.set()` call site — and K01/K02 above PROVE `set` never lands an
//     out-of-bounds/non-finite value. NVM content therefore cannot escape the
//     schema regardless of corruption (the composition is a one-call-site
//     inspection, not a conjecture).
//   * Corruption totality: `corruption_sweep_never_yields_out_of_schema`
//     exhaustively flips EVERY byte of a committed image (cargo test, ms),
//     and `persist_proptests` fuzzes random multi-byte corruption + random
//     value roundtrips at proptest scale.
//   * Torn commits: `torn_commit_keeps_previous_image` (the two-slot
//     protocol's guarantee).
//
// UPGRADE PATH (#265): ordeal (certificate-checked QF_BV) is the right tool
// for the CRC step-equivalence + record-codec round-trip as UNBOUNDED
// certificates — re-stating the removed harnesses' properties — once its
// byte-layout (ordeal#64) and equivalence toolkit (ordeal#66) ship.
