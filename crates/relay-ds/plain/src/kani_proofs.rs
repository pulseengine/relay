//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// DS-P01: decision_count never exceeds MAX_DECISIONS_PER_CHECK
#[kani::proof]
fn verify_decision_bounded() {
    let mut table = FilterTable::new();
    let data_id: u32 = kani::any();
    let dest: u32 = kani::any();
    let ft_val: u8 = kani::any();
    kani::assume(ft_val <= 2);
    let file_type = match ft_val {
        0 => FileType::Sequence,
        1 => FileType::Time,
        _ => FileType::Count,
    };
    table.add_filter(FilterEntry {
        data_id,
        destination: dest,
        enabled: true,
        file_type,
    });
    let query_id: u32 = kani::any();
    let result = table.evaluate(query_id);
    assert!(result.decision_count as usize <= MAX_DECISIONS_PER_CHECK);
}

/// DS-P02: disabled filters never produce decisions
#[kani::proof]
fn verify_disabled_no_decision() {
    let mut table = FilterTable::new();
    let data_id: u32 = kani::any();
    table.add_filter(FilterEntry {
        data_id,
        destination: 0,
        enabled: false,
        file_type: FileType::Sequence,
    });
    let result = table.evaluate(data_id);
    assert_eq!(result.decision_count, 0);
}

/// DS-P03: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = FilterTable::new();
    let data_id: u32 = kani::any();
    let dest: u32 = kani::any();
    let enabled: bool = kani::any();
    table.add_filter(FilterEntry {
        data_id,
        destination: dest,
        enabled,
        file_type: FileType::Sequence,
    });
    let query: u32 = kani::any();
    let _ = table.evaluate(query);
}
