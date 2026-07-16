//! Kani harnesses for relay-preflight (plain-only sibling). The check set is
//! small, so Kani enumerates EVERY combination exhaustively.
#![cfg(kani)]

use crate::*;

fn any_checks() -> PreflightChecks {
    PreflightChecks {
        sensors_healthy: kani::any(),
        estimator_converged: kani::any(),
        calibration_present: kani::any(),
        geofence_loaded: kani::any(),
        battery_ok: kani::any(),
        failsafe_configured: kani::any(),
    }
}

/// PREFLIGHT-K01 — the arming gate is EXACTLY all-pass: arm_check returns Allowed
/// IFF every check passes. Over all 2^6 combinations: a single failing check
/// blocks arming, and arming is permitted only when all pass.
#[kani::proof]
fn verify_arm_iff_all_pass() {
    let c = any_checks();
    let v = arm_check(c);
    if c.all_pass() {
        assert!(v == ArmVerdict::Allowed);
    } else {
        assert!(matches!(v, ArmVerdict::Blocked(_)));
    }
}

/// PREFLIGHT-K02 — when Blocked, the reported check is one that ACTUALLY failed
/// (never a false accusation), and Allowed is never returned with any check
/// failing (no arm-on-fail).
#[kani::proof]
fn verify_blocked_reason_is_real() {
    let c = any_checks();
    match arm_check(c) {
        ArmVerdict::Allowed => assert!(c.all_pass()),
        ArmVerdict::Blocked(reason) => {
            let failed = match reason {
                CheckFail::Sensors => !c.sensors_healthy,
                CheckFail::Estimator => !c.estimator_converged,
                CheckFail::Calibration => !c.calibration_present,
                CheckFail::Geofence => !c.geofence_loaded,
                CheckFail::Battery => !c.battery_ok,
                CheckFail::Failsafe => !c.failsafe_configured,
            };
            assert!(failed);
        }
    }
}

fn any_table() -> CheckTable {
    let mut t = CheckTable::new();
    // exercise every combination of declared/undeclared + pass/fail for
    // the breadth rows, and pass/fail for the always-required six.
    for id in CheckId::ALL {
        if kani::any() {
            t.set(id, kani::any());
        }
    }
    t
}

/// PREFLIGHT-K03 (PREARM-P03) — the table gate is EXACTLY all-required-
/// pass, and a Blocked verdict always names a required, failing row.
#[kani::proof]
fn verify_table_gate_exact() {
    let t = any_table();
    match arm_check_table(&t) {
        TableVerdict::Allowed => {
            for id in CheckId::ALL {
                assert!(!t.is_required(id) || t.passed(id));
            }
        }
        TableVerdict::Blocked(id) => {
            assert!(t.is_required(id) && !t.passed(id));
        }
    }
}

/// PREFLIGHT-K04 (PREARM-P03) — MONOTONE: failing any single row of an
/// Allowed table can never remain Allowed (adding a failing check can
/// never ALLOW arming; declaring + failing a new row always blocks).
#[kani::proof]
fn verify_table_gate_monotone() {
    let mut t = any_table();
    kani::assume(arm_check_table(&t) == TableVerdict::Allowed);
    let which: usize = kani::any();
    kani::assume(which < CHECK_COUNT);
    t.set(CheckId::ALL[which], false);
    assert!(arm_check_table(&t) != TableVerdict::Allowed);
}
