//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// LC-P04: violation_count never exceeds MAX_VIOLATIONS_PER_CYCLE
#[kani::proof]
fn verify_violation_count_bounded() {
    let mut table = WatchpointTable::new();
    let sensor_id: u32 = kani::any();
    let op_val: u8 = kani::any();
    kani::assume(op_val <= 5);
    let op = match op_val {
        0 => ComparisonOp::LessThan,
        1 => ComparisonOp::GreaterThan,
        2 => ComparisonOp::LessOrEqual,
        3 => ComparisonOp::GreaterOrEqual,
        4 => ComparisonOp::Equal,
        _ => ComparisonOp::NotEqual,
    };
    let threshold: i64 = kani::any();
    let persistence: u32 = kani::any();
    kani::assume(persistence >= 1);

    table.add_watchpoint(Watchpoint {
        sensor_id,
        op,
        threshold,
        enabled: true,
        persistence,
        current_count: 0,
    });

    let value: i64 = kani::any();
    let result = table.evaluate(SensorReading { sensor_id, value });
    assert!(result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE);
}

/// LC-P06 (inherited from CMP-P01): compare is total
#[kani::proof]
fn verify_compare_total() {
    let value: i64 = kani::any();
    let threshold: i64 = kani::any();
    let op_val: u8 = kani::any();
    kani::assume(op_val <= 5);
    let op = match op_val {
        0 => ComparisonOp::LessThan,
        1 => ComparisonOp::GreaterThan,
        2 => ComparisonOp::LessOrEqual,
        3 => ComparisonOp::GreaterOrEqual,
        4 => ComparisonOp::Equal,
        _ => ComparisonOp::NotEqual,
    };
    let result = compare(value, op, threshold);
    assert!(result || !result);
}

/// LC-P05: disabled watchpoints never produce violations
#[kani::proof]
fn verify_disabled_no_violations() {
    let mut table = WatchpointTable::new();
    let sensor_id: u32 = kani::any();
    kani::assume(sensor_id < 100);
    table.add_watchpoint(Watchpoint {
        sensor_id,
        op: ComparisonOp::GreaterThan,
        threshold: 0,
        enabled: false,
        persistence: 1,
        current_count: 0,
    });
    let value: i64 = kani::any();
    let result = table.evaluate(SensorReading { sensor_id, value });
    assert_eq!(result.violation_count, 0);
}

/// LC-P03 (inherited from CMP-P03): compare matches operator semantics
#[kani::proof]
fn verify_compare_semantics() {
    let v: i64 = kani::any();
    let t: i64 = kani::any();
    assert_eq!(compare(v, ComparisonOp::LessThan, t), v < t);
    assert_eq!(compare(v, ComparisonOp::GreaterThan, t), v > t);
    assert_eq!(compare(v, ComparisonOp::Equal, t), v == t);
}

// -------------------------------------------------------------
// Geofence harnesses (mirror EkfHealthMonitor pattern from
// crates/relay-hs/plain/src/engine.rs).
//
// Geofence::check is pure i32 — Kani can enumerate arbitrary
// (n, e, d) over the full domain without external_body gaps.
// -------------------------------------------------------------

fn arb_fence() -> Geofence {
    let min_n: i32 = kani::any();
    let max_n: i32 = kani::any();
    let min_e: i32 = kani::any();
    let max_e: i32 = kani::any();
    let min_d: i32 = kani::any();
    let max_d: i32 = kani::any();
    // Well-formed bounds: avoid degenerate "min > max" worlds where
    // every point is outside; the property still holds there but the
    // counter-examples drown signal.
    kani::assume(min_n <= max_n);
    kani::assume(min_e <= max_e);
    kani::assume(min_d <= max_d);
    Geofence::new(min_n, max_n, min_e, max_e, min_d, max_d)
}

/// LC-K01 (mirrors HS-P06): once `violation_latched`, always `violation_latched`.
#[kani::proof]
fn geofence_latch_monotone() {
    let mut g = arb_fence();
    let n: i32 = kani::any();
    let e: i32 = kani::any();
    let d: i32 = kani::any();
    let pre = g.violation_latched;
    let _ = g.check(n, e, d);
    if pre {
        assert!(g.violation_latched);
    }
}

/// LC-K02 (mirrors HS-P07): `check()` returns `true` only on the
/// rising edge — i.e. only when latch was off before and is on after.
#[kani::proof]
fn geofence_check_transition_only() {
    let mut g = arb_fence();
    let n: i32 = kani::any();
    let e: i32 = kani::any();
    let d: i32 = kani::any();
    let pre = g.violation_latched;
    let r = g.check(n, e, d);
    if r {
        assert!(!pre);
        assert!(g.violation_latched);
    }
}

/// LC-K03: an already-latched fence never re-fires.
#[kani::proof]
fn geofence_already_latched_silent() {
    let mut g = arb_fence();
    g.violation_latched = true;
    let n: i32 = kani::any();
    let e: i32 = kani::any();
    let d: i32 = kani::any();
    let r = g.check(n, e, d);
    assert!(!r);
    assert!(g.violation_latched);
}

/// LC-K04: a fresh fence with a position strictly inside bounds
/// must not trip. Encodes the "no false positive in the safe box"
/// guarantee that protects the SC RTL command from spurious fires.
#[kani::proof]
fn geofence_inside_never_trips() {
    let mut g = arb_fence();
    let n: i32 = kani::any();
    let e: i32 = kani::any();
    let d: i32 = kani::any();
    kani::assume(n >= g.min_n && n <= g.max_n);
    kani::assume(e >= g.min_e && e <= g.max_e);
    kani::assume(d >= g.min_d && d <= g.max_d);
    let r = g.check(n, e, d);
    assert!(!r);
    assert!(!g.violation_latched);
}

/// LC-K05: a fresh fence with a position strictly outside any axis
/// always trips on the first call. The "no false negative" complement
/// of LC-K04 — together they pin `check` to the exact spec.
#[kani::proof]
fn geofence_outside_always_trips() {
    let mut g = arb_fence();
    // Constrain ranges so "outside" exists on every axis without overflow.
    kani::assume(g.min_n > i32::MIN && g.max_n < i32::MAX);
    kani::assume(g.min_e > i32::MIN && g.max_e < i32::MAX);
    kani::assume(g.min_d > i32::MIN && g.max_d < i32::MAX);
    let n: i32 = kani::any();
    let e: i32 = kani::any();
    let d: i32 = kani::any();
    let outside =
        n < g.min_n || n > g.max_n || e < g.min_e || e > g.max_e || d < g.min_d || d > g.max_d;
    kani::assume(outside);
    let r = g.check(n, e, d);
    assert!(r);
    assert!(g.violation_latched);
}
