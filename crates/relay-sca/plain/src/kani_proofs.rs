//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// SCA-P01: dispatch_count never exceeds MAX_DISPATCH_PER_TICK
#[kani::proof]
fn verify_dispatch_bounded() {
    let mut table = AbsTable::new();
    let execute_at: u64 = kani::any();
    let code: u16 = kani::any();
    let enabled: bool = kani::any();
    table.add_command(AbsCommand {
        execute_at_sec: execute_at,
        command_code: code,
        args: [0u8; 32],
        arg_len: 0,
        dispatched: false,
        enabled,
    });
    let current_time: u64 = kani::any();
    let result = table.process_tick(current_time);
    assert!(result.dispatch_count as usize <= MAX_DISPATCH_PER_TICK);
}

/// SCA-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = AbsTable::new();
    let execute_at: u64 = kani::any();
    let code: u16 = kani::any();
    let enabled: bool = kani::any();
    table.add_command(AbsCommand {
        execute_at_sec: execute_at,
        command_code: code,
        args: [0u8; 32],
        arg_len: 0,
        dispatched: false,
        enabled,
    });
    let _ = table.count();
    let _ = table.process_tick(kani::any());
}
