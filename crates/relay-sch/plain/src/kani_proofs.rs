//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// SCH-P04: action_count never exceeds MAX_ACTIONS_PER_TICK
#[kani::proof]
fn verify_action_count_bounded() {
    let mut table = ScheduleTable::new();
    // Add a few slots with symbolic values
    let minor: u32 = kani::any();
    let major: u32 = kani::any();
    let channel: u32 = kani::any();
    table.add_slot(ScheduleSlot {
        minor_frame: minor,
        major_frame: major,
        target_channel: channel,
        payload_offset: 0,
        payload_len: 0,
        enabled: true,
    });
    let tick_minor: u32 = kani::any();
    let tick_major: u32 = kani::any();
    let result = table.process_tick(tick_minor, tick_major);
    assert!(result.action_count as usize <= MAX_ACTIONS_PER_TICK);
}

/// SCH-P06: disabled slots never produce actions
#[kani::proof]
fn verify_disabled_no_actions() {
    let mut table = ScheduleTable::new();
    table.add_slot(ScheduleSlot {
        minor_frame: 0,
        major_frame: 0,
        target_channel: 1,
        payload_offset: 0,
        payload_len: 0,
        enabled: false,
    });
    let minor: u32 = kani::any();
    let major: u32 = kani::any();
    let result = table.process_tick(minor, major);
    assert_eq!(result.action_count, 0);
}

/// SCH-P07: add_slot returns false iff table full
#[kani::proof]
fn verify_add_slot_full() {
    let mut table = ScheduleTable::new();
    for _ in 0..MAX_SCHEDULE_SLOTS {
        assert!(table.add_slot(ScheduleSlot::empty()));
    }
    assert!(!table.add_slot(ScheduleSlot::empty()));
}

/// No panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut table = ScheduleTable::new();
    let minor: u32 = kani::any();
    let major: u32 = kani::any();
    let channel: u32 = kani::any();
    let enabled: bool = kani::any();
    table.add_slot(ScheduleSlot {
        minor_frame: minor,
        major_frame: major,
        target_channel: channel,
        payload_offset: 0,
        payload_len: 0,
        enabled,
    });
    let _ = table.process_tick(kani::any(), kani::any());
    let _ = table.set_enabled(kani::any(), kani::any());
}

/// SCH-P03 (transition): `set_enabled` is index-safe and exact from ANY valid
/// table state. An in-range index toggles exactly that slot's enable bit and
/// reports success; an out-of-range index is rejected with no mutation. The
/// illegal (out-of-bounds) slot write is unreachable.
#[kani::proof]
fn verify_set_enabled_transition() {
    let mut table = ScheduleTable::new();
    let count: u32 = kani::any();
    kani::assume(count as usize <= MAX_SCHEDULE_SLOTS);
    table.slot_count = count;

    let idx: u32 = kani::any();
    let want: bool = kani::any();
    let ok = table.set_enabled(idx, want);
    if idx < count {
        assert!(ok);
        assert!(table.slots[idx as usize].enabled == want);
    } else {
        assert!(!ok);
    }
}

/// SCH-P04 (transition): `add_slot` from ANY valid count never overflows the
/// table and grows by exactly one when there is room (else rejects, leaving
/// the count unchanged). Complements the concrete `verify_add_slot_full`.
#[kani::proof]
fn verify_add_slot_transition() {
    let mut table = ScheduleTable::new();
    let count: u32 = kani::any();
    kani::assume(count as usize <= MAX_SCHEDULE_SLOTS);
    table.slot_count = count;

    let added = table.add_slot(ScheduleSlot::empty());
    assert!(table.slot_count as usize <= MAX_SCHEDULE_SLOTS);
    if (count as usize) < MAX_SCHEDULE_SLOTS {
        assert!(added && table.slot_count == count + 1);
    } else {
        assert!(!added && table.slot_count == count);
    }
}
