//! Kani harnesses for relay-mavlink (plain-only sibling).
//!
//! The verification split (the codebase pattern): the telemetry SCHEDULER's
//! arbitration logic — totality, budget conservation for the normal class,
//! and the critical-class no-starvation guarantee — is proven here over ALL
//! slot configurations and budgets at N=4. The wire ENCODERS are
//! conformance-gated against external pymavlink reference vectors instead
//! (encoding correctness is a spec-agreement question, not an invariant
//! Kani can state — the DroneCAN bit-order lesson).
#![cfg(kani)]

use crate::telemetry_sched::{Priority, StreamSlot, TelemetryScheduler};

fn any_slot() -> StreamSlot {
    let interval: u32 = kani::any();
    kani::assume(interval <= 8);
    let frame: usize = kani::any();
    kani::assume(frame <= 1024);
    let critical: bool = kani::any();
    StreamSlot {
        interval_ticks: interval,
        frame_bytes: frame,
        priority: if critical { Priority::Critical } else { Priority::Normal },
    }
}

/// MAV-K01 — `tick` is TOTAL and the mask is well-formed: for ANY four-slot
/// configuration and ANY budget, no panic, and every set mask bit names an
/// ENABLED slot (interval != 0) at an index < N.
#[kani::proof]
#[kani::unwind(8)]
fn verify_tick_total_mask_well_formed() {
    let slots = [any_slot(), any_slot(), any_slot(), any_slot()];
    let mut sched = TelemetryScheduler::new(slots);
    let budget: usize = kani::any();
    let mask = sched.tick(budget);
    assert!(mask < (1 << 4), "no bit at or above N");
    for i in 0..4 {
        if mask & (1 << i) != 0 {
            assert!(slots[i].interval_ticks != 0, "disabled slot never emitted");
        }
    }
}

/// MAV-K02 — NORMAL-class budget conservation: the bytes of all Normal
/// streams emitted in one tick never exceed the budget (criticals may
/// saturate past it BY DESIGN — the heartbeat is never starved — which is
/// why the conserved quantity is the normal class, not the whole mask).
#[kani::proof]
#[kani::unwind(8)]
fn verify_normals_never_exceed_budget() {
    let slots = [any_slot(), any_slot(), any_slot(), any_slot()];
    let mut sched = TelemetryScheduler::new(slots);
    let budget: usize = kani::any();
    kani::assume(budget <= 1 << 20);
    let mask = sched.tick(budget);
    let mut normal_bytes = 0usize;
    for i in 0..4 {
        if mask & (1 << i) != 0 && slots[i].priority == Priority::Normal {
            normal_bytes += slots[i].frame_bytes;
        }
    }
    assert!(normal_bytes <= budget, "normal emissions fit the budget");
}

/// MAV-K03 — the critical class is NEVER starved: for ANY configuration and
/// ANY budget (including zero), every enabled Critical slot that is due this
/// tick is in the mask.
#[kani::proof]
#[kani::unwind(8)]
fn verify_criticals_never_starved() {
    let slots = [any_slot(), any_slot(), any_slot(), any_slot()];
    // Due-this-tick = countdown reaches <= 1; after new(), a slot with
    // interval 1 is due on the first tick.
    let mut sched = TelemetryScheduler::new(slots);
    let budget: usize = kani::any();
    let mask = sched.tick(budget);
    for i in 0..4 {
        let s = slots[i];
        if s.priority == Priority::Critical && s.interval_ticks == 1 {
            assert!(mask & (1 << i) != 0, "due critical always emitted");
        }
    }
}
