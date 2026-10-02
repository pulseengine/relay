//! The published component links an 8 KiB shadow stack, and that budget is a
//! SHIPPED PROPERTY rather than an accident: the no-grow invariant is what lets
//! jess lower the image to bare metal. This test guards it from the only side
//! falcon-core can — the size of the things the component constructs.
//!
//! WHY IT EXISTS, measured. `wasm/cm/cascade` had to stop building `FlightCore`
//! by value: "the by-value form overflowed that stack and trapped as an
//! out-of-bounds access inside `FlightCore::new`". The fix was to construct into
//! the slot and mutate through `&mut`, and the comment explaining it is the only
//! thing standing between that budget and the next person who writes a builder.
//!
//! SUPERVISOR-P01 makes that live again: the component must switch from
//! `FlightCore` to `FlightSupervisor`, which CONTAINS a `FlightCore` plus the
//! mode machine, the failsafe arbiter, the preflight rows and the mission state.
//! Before writing that change I assumed the supervisor would be dramatically
//! larger and the switch structurally blocked. MEASURED, it is not:
//!
//!     FlightCore        2688 B
//!     FlightSupervisor  3208 B   (1.19x — 520 B more)
//!
//! So the same construct-into-slot mitigation extends, with 19% more pressure.
//! That is a manageable risk rather than a blocker, and the number is the reason
//! to believe it. This test is here so the number cannot drift quietly.

use falcon_core::{FlightCore, FlightSupervisor};

/// The component's linked shadow stack, from `wasm/cm/cascade`.
const SHADOW_STACK: usize = 8 * 1024;

/// Half the stack. A construction site may transiently hold the value being
/// returned AND the slot it is moved into, so anything above half the stack
/// cannot be built by value at all, and anything approaching half leaves no room
/// for `new`'s own working set — which is exactly how `FlightCore::new` trapped.
const BUDGET: usize = SHADOW_STACK / 2;

#[test]
fn the_supervisor_still_fits_the_components_shadow_stack() {
    let sup = core::mem::size_of::<FlightSupervisor>();
    assert!(
        sup <= BUDGET,
        "FlightSupervisor is {sup} B, over the {BUDGET} B budget (half of the \
         component's {SHADOW_STACK} B shadow stack). The published component \
         cannot construct this by value — see wasm/cm/cascade/src/lib.rs, where \
         FlightCore::new already trapped as an out-of-bounds access for exactly \
         this reason. Either shrink it, or construct it into its slot and mutate \
         through &mut; do NOT raise the shadow stack, because the 8 KiB no-grow \
         budget is what lets the image lower to bare metal."
    );
}

#[test]
fn the_supervisor_is_not_dramatically_larger_than_the_core() {
    let core_sz = core::mem::size_of::<FlightCore>();
    let sup = core::mem::size_of::<FlightSupervisor>();
    assert!(
        sup >= core_sz,
        "the supervisor contains a core, so it cannot be smaller"
    );
    // 1.5x, against a measured 1.19x. This is the claim SUPERVISOR-P01's
    // feasibility rests on: that swapping FlightCore for FlightSupervisor does
    // not change the ORDER of the memory problem. If this fires, that premise
    // has moved and the switch needs re-planning rather than re-tuning.
    let ratio_num = sup * 100;
    let ratio_den = core_sz * 100;
    assert!(
        ratio_num * 2 <= ratio_den * 3,
        "FlightSupervisor ({sup} B) is more than 1.5x FlightCore ({core_sz} B). \
         SUPERVISOR-P01 assumed the supervisor adds the mode machine, arbiter, \
         preflight rows and mission state WITHOUT changing the order of the \
         component's memory problem. That assumption no longer holds."
    );
}
