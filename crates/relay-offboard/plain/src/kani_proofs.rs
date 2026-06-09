//! Kani bounded-model-checking harnesses for relay-offboard (plain-only sibling,
//! per the relay convention — never in a verus-stripped file).
#![cfg(kani)]

use crate::*;

fn any_sp() -> OffboardSetpoint {
    OffboardSetpoint {
        position_ned: [kani::any(), kani::any(), kani::any()],
        velocity_ned: [kani::any(), kani::any(), kani::any()],
        yaw: kani::any(),
    }
}

/// OFFB-K01 — `status` is total: no panic / no overflow for ANY clock value,
/// timeout, and receiver state (the saturating-subtraction guarantee).
#[kani::proof]
fn verify_status_total() {
    let timeout: u64 = kani::any();
    let mut r = OffboardReceiver::new(timeout);
    if kani::any() {
        let counter: u64 = kani::any();
        let now: u64 = kani::any();
        r.accept(any_sp(), counter, now);
    }
    let now: u64 = kani::any();
    let _ = r.status(now);
}

/// OFFB-K02 — the FAILSAFE property: strictly past the timeout with no new
/// setpoint, the stream is ALWAYS Stale (a stale stream can never read Active).
#[kani::proof]
fn verify_stale_after_timeout() {
    let timeout: u64 = kani::any();
    let mut r = OffboardReceiver::new(timeout);
    let counter: u64 = kani::any();
    let t0: u64 = kani::any();
    r.accept(any_sp(), counter, t0);
    let now: u64 = kani::any();
    // now is strictly beyond the timeout window (and the subtraction can't wrap).
    kani::assume(now >= t0);
    kani::assume(now - t0 > timeout);
    assert!(r.status(now) == OffboardStatus::Stale);
}

/// OFFB-K03 — a replayed / non-increasing counter is rejected once started, and
/// leaves the last counter unchanged (no replay re-arms the stream).
#[kani::proof]
fn verify_replay_rejected() {
    let mut r = OffboardReceiver::new(kani::any());
    let c0: u64 = kani::any();
    r.accept(any_sp(), c0, kani::any());
    let c1: u64 = kani::any();
    kani::assume(c1 <= c0);
    let accepted = r.accept(any_sp(), c1, kani::any());
    assert!(!accepted);
    assert!(r.last_counter() == c0);
}
