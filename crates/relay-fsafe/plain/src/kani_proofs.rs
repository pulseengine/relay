//! Kani bounded-model-checking harnesses for relay-fsafe (plain-only sibling).
#![cfg(kani)]

use crate::*;

fn any_triggers() -> Triggers {
    Triggers {
        rc_loss: kani::any(),
        datalink_loss: kani::any(),
        geofence_breach: kani::any(),
        position_loss: kani::any(),
        low_battery: kani::any(),
        critical_battery: kani::any(),
        offboard_stale: kani::any(),
    }
}

/// FSAFE-K01 — the arbiter is total: ANY trigger combination against ANY
/// latched state yields a defined action, no panic.
#[kani::proof]
fn verify_evaluate_total() {
    let mut a = FailsafeArbiter::new();
    if kani::any() {
        a.evaluate(any_triggers());
    }
    let _ = a.evaluate(any_triggers());
}

/// FSAFE-K02 — most-severe-wins: the action is at least as severe as the action
/// mapped to EACH active trigger (a quiet trigger never masks a loud one).
#[kani::proof]
fn verify_most_severe_wins() {
    let mut a = FailsafeArbiter::new();
    let t = any_triggers();
    let act = a.evaluate(t);
    if t.offboard_stale {
        assert!(act >= FailsafeAction::Hold);
    }
    if t.rc_loss || t.datalink_loss || t.geofence_breach || t.low_battery {
        assert!(act >= FailsafeAction::Rtl);
    }
    if t.position_loss || t.critical_battery {
        assert!(act >= FailsafeAction::Land);
    }
}

/// FSAFE-K03 — latching: the action is monotone non-decreasing across two
/// arbitrary cycles (a momentary trigger is never silently downgraded).
#[kani::proof]
fn verify_latch_monotone() {
    let mut a = FailsafeArbiter::new();
    let first = a.evaluate(any_triggers());
    let second = a.evaluate(any_triggers());
    assert!(second >= first);
}

/// FSAFE-K04 — NO BLIND RTL: whenever position is lost this cycle, the action is
/// at least `Land` and therefore NEVER `Rtl` (you cannot return to launch with
/// no position fix to navigate home), for any prior latched state.
#[kani::proof]
fn verify_no_blind_rtl() {
    let mut a = FailsafeArbiter::new();
    // Arbitrary prior history, then a cycle with position lost.
    a.evaluate(any_triggers());
    let mut t = any_triggers();
    t.position_loss = true;
    let act = a.evaluate(t);
    assert!(act >= FailsafeAction::Land);
    assert!(act != FailsafeAction::Rtl);
}
