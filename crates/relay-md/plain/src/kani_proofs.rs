//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// MD-P01: request_count never exceeds MAX_SAMPLES_PER_CYCLE
#[kani::proof]
fn verify_sample_bounded() {
    let mut table = DwellTable::new();
    let address: u32 = kani::any();
    let size: u8 = kani::any();
    let rate_divisor: u32 = kani::any();
    kani::assume(rate_divisor >= 1);
    table.add_entry(DwellEntry {
        address,
        size,
        rate_divisor,
        enabled: true,
    });
    let cycle: u32 = kani::any();
    let result = table.get_samples(cycle);
    assert!(result.request_count as usize <= MAX_SAMPLES_PER_CYCLE);
}

/// MD-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = DwellTable::new();
    let address: u32 = kani::any();
    let size: u8 = kani::any();
    let rate_divisor: u32 = kani::any();
    kani::assume(rate_divisor >= 1);
    let enabled: bool = kani::any();
    table.add_entry(DwellEntry {
        address,
        size,
        rate_divisor,
        enabled,
    });
    let cycle: u32 = kani::any();
    let _ = table.get_samples(cycle);
}
