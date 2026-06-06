//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// CS-P01: CRC32 is deterministic — same input always yields same output
#[kani::proof]
fn verify_crc_deterministic() {
    let b0: u8 = kani::any();
    let b1: u8 = kani::any();
    let data = [b0, b1];
    let crc1 = crc32_compute(&data);
    let crc2 = crc32_compute(&data);
    assert_eq!(crc1, crc2);
}

/// CS-P02: check_batch result_count never exceeds MAX_CHECK_PER_CYCLE
#[kani::proof]
fn verify_check_bounded() {
    let mut table = ChecksumTable::new();
    let region_id: u32 = kani::any();
    kani::assume(region_id < 100);
    let baseline: u32 = kani::any();
    table.register_region(region_id, baseline);
    let data: [u8; 2] = [kani::any(), kani::any()];
    let time: u64 = kani::any();
    let pairs: [(u32, &[u8]); 1] = [(region_id, &data)];
    let output = table.check_batch(&pairs, time);
    assert!(output.result_count as usize <= MAX_CHECK_PER_CYCLE);
}

/// CS-P03: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let b0: u8 = kani::any();
    let b1: u8 = kani::any();
    let data = [b0, b1];
    let _ = crc32_compute(&data);
    let mut table = ChecksumTable::new();
    let region_id: u32 = kani::any();
    kani::assume(region_id < 100);
    table.register_region(region_id, kani::any());
    let time: u64 = kani::any();
    let _ = table.check_region(region_id, &data, time);
}
