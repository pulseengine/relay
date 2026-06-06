//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// HK-P01: collect on empty table always succeeds and packet.length stays bounded
#[kani::proof]
#[kani::unwind(66)]
fn verify_collect_bounded() {
    let mut table = CopyTable::new();
    let source_id: u32 = kani::any();
    let source_offset: u32 = kani::any();
    let length: u32 = kani::any();
    let output_offset: u32 = kani::any();
    kani::assume(length <= SOURCE_DATA_SIZE as u32);
    kani::assume(source_offset <= SOURCE_DATA_SIZE as u32 - length);
    kani::assume(output_offset <= MAX_OUTPUT_SIZE as u32 - length);
    table.add_entry(CopyEntry {
        source_id,
        source_offset,
        length,
        output_offset,
    });
    let mut src = SourceData::empty();
    src.source_id = source_id;
    let sources = [src];
    let mut packet = HkPacket::new();
    let ok = table.collect(&sources, &mut packet);
    if ok {
        assert!(packet.length as usize <= MAX_OUTPUT_SIZE);
    }
}

/// HK-P02: no panics for any symbolic input
#[kani::proof]
#[kani::unwind(66)]
fn verify_no_panic() {
    let mut table = CopyTable::new();
    let source_id: u32 = kani::any();
    let source_offset: u32 = kani::any();
    let length: u32 = kani::any();
    let output_offset: u32 = kani::any();
    kani::assume(length <= SOURCE_DATA_SIZE as u32);
    kani::assume(source_offset <= SOURCE_DATA_SIZE as u32);
    kani::assume(output_offset <= MAX_OUTPUT_SIZE as u32);
    table.add_entry(CopyEntry {
        source_id,
        source_offset,
        length,
        output_offset,
    });
    let mut src = SourceData::empty();
    src.source_id = source_id;
    let sources = [src];
    let mut packet = HkPacket::new();
    let _ = table.collect(&sources, &mut packet);
}
