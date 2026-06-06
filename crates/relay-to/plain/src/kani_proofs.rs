//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// TO-P01: subscribe then unsubscribe yields Exclude (not Include)
#[kani::proof]
fn verify_subscribe_unsubscribe() {
    let mut table = SubscriptionTable::new();
    let msg_id: u32 = kani::any();
    let priority: u8 = kani::any();
    let ok = table.subscribe(msg_id, priority);
    assert!(ok);
    assert_eq!(table.evaluate(msg_id), ToDecision::Include);
    let removed = table.unsubscribe(msg_id);
    assert!(removed);
    assert_eq!(table.evaluate(msg_id), ToDecision::Exclude);
}

/// TO-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = SubscriptionTable::new();
    let msg_id: u32 = kani::any();
    let priority: u8 = kani::any();
    let _ = table.subscribe(msg_id, priority);
    let _ = table.unsubscribe(kani::any());
    let _ = table.evaluate(kani::any());
    let _ = table.get_active_count();
}
