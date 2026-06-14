// Relay Scheduler — P3 WASM component (self-contained).
//
// This file contains both:
//   1. The verified core engine (from plain/src/engine.rs)
//   2. The P3 async Guest trait implementation
//
// Built by: bazel build //:relay-sch (rules_wasm_component, wasi_version="p3")
// Verified by: bazel test //:relay_sch_verus_test (src/engine.rs with verus!)

// ═══════════════════════════════════════════════════════════════
// Verified core engine (plain Rust, identical to plain/src/engine.rs)
// ═══════════════════════════════════════════════════════════════

// Run the REAL Verus+Kani-verified engine (#168), not an inline copy.
use relay_sch::engine;

// ═══════════════════════════════════════════════════════════════
// P3 WASM component binding — delegates to verified engine
// ═══════════════════════════════════════════════════════════════

use relay_sch_bindings::exports::pulseengine::relay_scheduler::scheduler::{
    Guest, ScheduleSlot as WitSlot, ScheduledAction as WitAction,
};

struct Component;

static mut TABLE: Option<engine::ScheduleTable> = None;

fn get_table() -> &'static mut engine::ScheduleTable {
    unsafe {
        if TABLE.is_none() {
            TABLE = Some(engine::ScheduleTable::new());
        }
        TABLE.as_mut().unwrap()
    }
}

impl Guest for Component {
    #[cfg(target_arch = "wasm32")]
    async fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::ScheduleTable::new()); }
        Ok(())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::ScheduleTable::new()); }
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    async fn tick(minor_frame: u32, major_frame: u32) -> Vec<WitAction> {
        Self::do_tick(minor_frame, major_frame)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn tick(minor_frame: u32, major_frame: u32) -> Vec<WitAction> {
        Self::do_tick(minor_frame, major_frame)
    }

    #[cfg(target_arch = "wasm32")]
    async fn add_slot(slot: WitSlot) -> bool {
        Self::do_add_slot(slot)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn add_slot(slot: WitSlot) -> bool {
        Self::do_add_slot(slot)
    }

    #[cfg(target_arch = "wasm32")]
    async fn set_enabled(index: u32, enabled: bool) -> bool {
        Self::do_set_enabled(index, enabled)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn set_enabled(index: u32, enabled: bool) -> bool {
        Self::do_set_enabled(index, enabled)
    }

    #[cfg(target_arch = "wasm32")]
    async fn slot_count() -> u32 {
        Self::do_slot_count()
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn slot_count() -> u32 {
        Self::do_slot_count()
    }
}

impl Component {
    fn do_tick(minor_frame: u32, major_frame: u32) -> Vec<WitAction> {
        let table = get_table();
        let result = table.process_tick(minor_frame, major_frame);
        let mut actions = Vec::with_capacity(result.action_count as usize);
        for i in 0..result.action_count as usize {
            actions.push(WitAction {
                target_channel: result.actions[i].target_channel,
                payload_offset: result.actions[i].payload_offset,
                payload_len: result.actions[i].payload_len,
            });
        }
        actions
    }

    fn do_add_slot(slot: WitSlot) -> bool {
        get_table().add_slot(engine::ScheduleSlot {
            minor_frame: slot.minor_frame,
            major_frame: slot.major_frame,
            target_channel: slot.target_channel,
            payload_offset: slot.payload_offset,
            payload_len: slot.payload_len,
            enabled: slot.enabled,
        })
    }

    fn do_set_enabled(index: u32, enabled: bool) -> bool {
        get_table().set_enabled(index, enabled)
    }

    fn do_slot_count() -> u32 {
        get_table().slot_count()
    }
}

relay_sch_bindings::export!(Component with_types_in relay_sch_bindings);
