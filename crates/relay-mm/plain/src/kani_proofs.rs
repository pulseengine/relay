//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// MM-P01: addresses outside [ram_start, ram_end) are rejected
#[kani::proof]
fn verify_out_of_range_rejected() {
    let config = MmConfig {
        ram_start: 0x2000_0000,
        ram_end: 0x2001_0000,
        max_operation_size: 4096,
    };
    let addr: u32 = kani::any();
    kani::assume(addr < config.ram_start || addr >= config.ram_end);
    let req = MmRequest {
        operation: MmOperation::Peek,
        address: addr,
        size: 4,
        value: 0,
    };
    let result = validate_request(&config, &req);
    assert!(result == MmValidation::AddressOutOfRange);
}

/// MM-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let ram_start: u32 = kani::any();
    let ram_end: u32 = kani::any();
    kani::assume(ram_start < ram_end);
    let max_op_size: u32 = kani::any();
    kani::assume(max_op_size >= 1);
    let config = MmConfig {
        ram_start,
        ram_end,
        max_operation_size: max_op_size,
    };
    let op_val: u8 = kani::any();
    kani::assume(op_val <= 4);
    let operation = match op_val {
        0 => MmOperation::Peek,
        1 => MmOperation::Poke,
        2 => MmOperation::LoadFromFile,
        3 => MmOperation::DumpToFile,
        _ => MmOperation::Fill,
    };
    let req = MmRequest {
        operation,
        address: kani::any(),
        size: kani::any(),
        value: kani::any(),
    };
    let _ = validate_request(&config, &req);
}
