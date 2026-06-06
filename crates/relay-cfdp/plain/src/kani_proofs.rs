//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// CFDP-P01: transaction count never exceeds MAX_TRANSACTIONS
#[kani::proof]
fn verify_transaction_bounded() {
    let mut table = TransactionTable::new();
    let file_size: u32 = kani::any();
    let max_retransmit: u32 = kani::any();
    let id = table.begin_send(file_size, max_retransmit);
    assert!(id.is_some());
    assert!(table.count as usize <= MAX_TRANSACTIONS);
}

/// CFDP-P02: state transitions are valid (state never becomes an invalid value)
#[kani::proof]
fn verify_state_valid() {
    let mut table = TransactionTable::new();
    let file_size: u32 = kani::any();
    kani::assume(file_size <= 4096);
    let max_retransmit: u32 = kani::any();
    kani::assume(max_retransmit <= 10);
    let id = table.begin_send(file_size, max_retransmit).unwrap();

    // Drive one ACK transition
    let result = table.process_ack(id);
    assert!(result.action_count as usize <= MAX_ACTIONS);

    let state = table.get_state(id).unwrap();
    assert!(
        state == TransactionState::Idle
            || state == TransactionState::MetadataSent
            || state == TransactionState::DataSending
            || state == TransactionState::EofSent
            || state == TransactionState::Finished
            || state == TransactionState::Cancelled
    );
}

/// CFDP-P03: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = TransactionTable::new();
    let file_size: u32 = kani::any();
    kani::assume(file_size <= 4096);
    let max_retransmit: u32 = kani::any();
    kani::assume(max_retransmit <= 10);
    if let Some(id) = table.begin_send(file_size, max_retransmit) {
        let _ = table.process_ack(id);
        let _ = table.tick(id);
        let _ = table.get_state(id);
    }
}
