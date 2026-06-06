//! Kani bounded-model-checking harnesses for relay-sec (plain-only).
//!
//! Kept in this sibling module (declared from lib.rs) following the relay
//! convention: BMC harnesses never live in a verus-stripped file, so a regen
//! cannot wipe them. These prove the anti-replay window with NO cryptography —
//! freshness is a state-machine property.
#![cfg(kani)]

use crate::*;

/// SEC-K01 — a counter accepted once is rejected as Replay on immediate
/// re-submit. The fundamental no-double-accept guarantee.
#[kani::proof]
fn verify_no_double_accept() {
    let mut w = ReplayWindow::new();
    let c: u64 = kani::any();
    kani::assume(c > 0);
    // First accept from an empty window is always Fresh.
    assert!(w.accept(c) == ReplayVerdict::Fresh);
    // The very same counter must now be a Replay.
    assert!(w.accept(c) == ReplayVerdict::Replay);
}

/// SEC-K02 — accept never lowers the frontier, for any window built from a
/// real accept and any subsequent counter.
#[kani::proof]
fn verify_frontier_monotone() {
    let mut w = ReplayWindow::new();
    let a: u64 = kani::any();
    kani::assume(a > 0);
    w.accept(a);
    let before = w.highest();
    let b: u64 = kani::any();
    w.accept(b);
    assert!(w.highest() >= before);
}

/// SEC-K03 — accept is total: no panic, no arithmetic overflow, no shift-UB,
/// for ANY counter against a non-trivial window. This is what makes the `<<`
/// in the slide path safe.
#[kani::proof]
fn verify_accept_total() {
    let mut w = ReplayWindow::new();
    let a: u64 = kani::any();
    kani::assume(a > 0);
    w.accept(a);
    let b: u64 = kani::any();
    let _ = w.accept(b); // Kani proves this cannot panic for any b
}

/// SEC-K04 — anything strictly older than the window is TooOld and leaves the
/// window unchanged.
#[kani::proof]
fn verify_too_old_unchanged() {
    let mut w = ReplayWindow::new();
    let h: u64 = kani::any();
    kani::assume(h > WINDOW_BITS); // leave room below the frontier
    w.accept(h);
    let snapshot = w;
    let c: u64 = kani::any();
    kani::assume(c > 0);
    kani::assume(c <= h - WINDOW_BITS); // strictly outside the window
    assert!(w.accept(c) == ReplayVerdict::TooOld);
    assert!(w.highest() == snapshot.highest());
}
