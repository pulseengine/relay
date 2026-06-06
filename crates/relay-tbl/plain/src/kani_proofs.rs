//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// TBL-P01: register always succeeds until table is full
#[kani::proof]
fn verify_register_bounded() {
    let mut reg = TableRegistry::new();
    let name_hash: u32 = kani::any();
    let size: u32 = kani::any();
    kani::assume(size <= MAX_TABLE_SIZE as u32);
    let result = reg.register(name_hash, size);
    assert_eq!(result, TblResult::Success);
    assert_eq!(reg.table_count, 1);
}

/// TBL-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut reg = TableRegistry::new();
    let name_hash: u32 = kani::any();
    let size: u32 = kani::any();
    kani::assume(size <= MAX_TABLE_SIZE as u32);
    let _ = reg.register(name_hash, size);
    let _ = reg.get_active(name_hash);
    let _ = reg.activate(name_hash);
    let data: [u8; 4] = [kani::any(), kani::any(), kani::any(), kani::any()];
    let _ = reg.load(name_hash, &data);
}
