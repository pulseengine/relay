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
