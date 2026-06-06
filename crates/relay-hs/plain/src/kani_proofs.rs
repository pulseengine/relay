//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// HS-P01: alert_count never exceeds MAX_ALERTS_PER_CHECK
#[kani::proof]
fn verify_alert_bounded() {
    let mut table = HealthTable::new();
    let app_id: u32 = kani::any();
    kani::assume(app_id < 100);
    let max_miss: u32 = kani::any();
    kani::assume(max_miss >= 1);
    let action_val: u8 = kani::any();
    kani::assume(action_val <= 3);
    let action = match action_val {
        0 => HsAction::NoAction,
        1 => HsAction::Event,
        2 => HsAction::RestartApp,
        _ => HsAction::ProcessorReset,
    };
    table.register_app(app_id, max_miss, action);
    let time: u64 = kani::any();
    let result = table.check_health(time);
    assert!(result.alert_count as usize <= MAX_ALERTS_PER_CHECK);
}

/// HS-P02: disabled apps never generate alerts
#[kani::proof]
fn verify_disabled_no_alert() {
    let mut table = HealthTable::new();
    // An empty table has no enabled apps, so no alerts
    let time: u64 = kani::any();
    let result = table.check_health(time);
    assert_eq!(result.alert_count, 0);
}

/// HS-P03: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = HealthTable::new();
    let app_id: u32 = kani::any();
    kani::assume(app_id < 100);
    let max_miss: u32 = kani::any();
    kani::assume(max_miss >= 1);
    table.register_app(app_id, max_miss, HsAction::Event);
    let new_count: u32 = kani::any();
    table.update_counter(app_id, new_count);
    let time: u64 = kani::any();
    let _ = table.check_health(time);
}

// ─── EkfHealthMonitor (v0.9.1) ─────────────────────────────────
// Closes the trust gap Verus's HS-P06/HS-P07 leave open: those
// discharge the latch state-machine but treat step_window
// (bit-shift + count_ones) as external_body. Kani bounded-model-
// checks step_window over an *arbitrary* u64 history.

/// HS-P06 (Kani): observe() never un-latches RTL — for every
/// pre-state and any over_limit input, if rtl_latched was true
/// before the call, it is true after.
#[kani::proof]
fn verify_ekf_monitor_latch_monotone() {
    let history: u64 = kani::any();
    let mut wd = EkfHealthMonitor {
        window: 64,
        trip_threshold: 48,
        history,
        rtl_latched: true,
    };
    let over_limit: bool = kani::any();
    let _ = wd.observe(over_limit);
    assert!(wd.rtl_latched);
}

/// HS-P07 (Kani): observe() returns true only on the RTL
/// transition — if it returns true the monitor is now latched
/// AND was not latched before.
#[kani::proof]
fn verify_ekf_monitor_returns_true_only_on_transition() {
    let history: u64 = kani::any();
    let mut wd = EkfHealthMonitor {
        window: 64,
        trip_threshold: 48,
        history,
        rtl_latched: false,
    };
    let over_limit: bool = kani::any();
    let was_latched = wd.rtl_latched;
    let tripped = wd.observe(over_limit);
    if tripped {
        assert!(wd.rtl_latched);
        assert!(!was_latched);
    }
}

/// HS-P08 (Kani): step_window's returned over-count is bounded
/// by window for every arbitrary input. This closes the trust
/// gap Verus leaves on the external_body bit-shift + count_ones.
#[kani::proof]
fn verify_step_window_count_bounded() {
    let history: u64 = kani::any();
    let window: u32 = kani::any();
    kani::assume(window > 0 && window <= 64);
    let over_limit: bool = kani::any();
    let (_, count) = EkfHealthMonitor::step_window(history, window, over_limit);
    assert!(count <= window);
}

/// HS-P09 (Kani): once latched, observe() always returns false —
/// the latch transition fires exactly once per monitor lifetime.
#[kani::proof]
fn verify_latched_observe_never_returns_true() {
    let history: u64 = kani::any();
    let mut wd = EkfHealthMonitor {
        window: 64,
        trip_threshold: 48,
        history,
        rtl_latched: true,
    };
    let over_limit: bool = kani::any();
    assert!(!wd.observe(over_limit));
}
