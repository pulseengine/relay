//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// SC-P01: dispatch_count never exceeds MAX_DISPATCH_PER_TICK
#[kani::proof]
#[kani::unwind(18)]
fn verify_dispatch_bounded() {
    let mut store = CommandStore::new();
    let execute_at: u64 = kani::any();
    let code: u16 = kani::any();
    store.load_ats_command(AtsCommand {
        execute_at_sec: execute_at,
        command_code: code,
        payload_offset: 0,
        payload_len: 0,
        dispatched: false,
    });
    let current_time: u64 = kani::any();
    let result = store.process_tick(current_time);
    assert!(result.dispatch_count as usize <= MAX_DISPATCH_PER_TICK);
}

/// SC-P02: no panics for any symbolic input on load/start/stop
#[kani::proof]
fn verify_no_panic() {
    let mut store = CommandStore::new();
    let execute_at: u64 = kani::any();
    let code: u16 = kani::any();
    let _ = store.load_ats_command(AtsCommand {
        execute_at_sec: execute_at,
        command_code: code,
        payload_offset: 0,
        payload_len: 0,
        dispatched: false,
    });
    let rts_id: u32 = kani::any();
    let _ = store.start_rts(rts_id, kani::any());
    let _ = store.stop_rts(rts_id);
}

/// SC-P03 (transition): the RTS sequence state machine is well-formed across
/// a tick. `current_index` never runs past `command_count` (so
/// `commands[current_index]` is always a valid access), and a sequence stays
/// `running` only while it still has a command left — when it reaches the
/// end it transitions to stopped. Illegal states (index > count, or running
/// past the end) are unreachable.
#[kani::proof]
#[kani::unwind(18)]
fn verify_rts_state_transition() {
    let mut store = CommandStore::new();
    // A small RTS (1..=2 commands), each with a symbolic delay.
    let n: u32 = kani::any();
    kani::assume(n >= 1 && n <= 2);
    let mut i = 0u32;
    while i < n {
        let _ = store.load_rts_command(
            0,
            RtsCommand {
                delay_sec: kani::any(),
                command_code: 0,
                payload_offset: 0,
                payload_len: 0,
            },
        );
        i += 1;
    }
    let t0: u64 = kani::any();
    let _ = store.start_rts(0, t0);
    let t1: u64 = kani::any();
    let _ = store.process_tick(t1);

    let seq = store.rts_sequences[0];
    // index never overruns the command table
    assert!(seq.current_index <= seq.command_count);
    // running implies a command remains (else it must have stopped)
    if seq.running {
        assert!(seq.current_index < seq.command_count);
    }
}
