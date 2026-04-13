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

mod engine {
    pub const MAX_SCHEDULE_SLOTS: usize = 256;
    pub const MAX_ACTIONS_PER_TICK: usize = 16;

    #[derive(Clone, Copy)]
    pub struct ScheduleSlot {
        pub minor_frame: u32,
        pub major_frame: u32,
        pub target_channel: u32,
        pub payload_offset: u32,
        pub payload_len: u32,
        pub enabled: bool,
    }

    #[derive(Clone, Copy)]
    pub struct ScheduledAction {
        pub target_channel: u32,
        pub payload_offset: u32,
        pub payload_len: u32,
    }

    pub struct ScheduleTable {
        slots: [ScheduleSlot; MAX_SCHEDULE_SLOTS],
        slot_count: u32,
    }

    pub struct TickResult {
        pub actions: [ScheduledAction; MAX_ACTIONS_PER_TICK],
        pub action_count: u32,
    }

    impl ScheduleSlot {
        pub const fn empty() -> Self {
            ScheduleSlot { minor_frame: 0, major_frame: 0, target_channel: 0, payload_offset: 0, payload_len: 0, enabled: false }
        }
    }

    impl ScheduledAction {
        pub const fn empty() -> Self {
            ScheduledAction { target_channel: 0, payload_offset: 0, payload_len: 0 }
        }
    }

    impl ScheduleTable {
        pub fn new() -> Self {
            ScheduleTable { slots: [ScheduleSlot::empty(); MAX_SCHEDULE_SLOTS], slot_count: 0 }
        }

        pub fn add_slot(&mut self, slot: ScheduleSlot) -> bool {
            if self.slot_count as usize >= MAX_SCHEDULE_SLOTS { return false; }
            self.slots[self.slot_count as usize] = slot;
            self.slot_count = self.slot_count + 1;
            true
        }

        pub fn set_enabled(&mut self, index: u32, enabled: bool) -> bool {
            if index >= self.slot_count { return false; }
            self.slots[index as usize].enabled = enabled;
            true
        }

        pub fn slot_count(&self) -> u32 { self.slot_count }

        pub fn process_tick(&self, current_minor: u32, current_major: u32) -> TickResult {
            let mut result = TickResult { actions: [ScheduledAction::empty(); MAX_ACTIONS_PER_TICK], action_count: 0 };
            let count = self.slot_count;
            let mut i: u32 = 0;
            while i < count {
                if result.action_count as usize >= MAX_ACTIONS_PER_TICK { break; }
                let slot = self.slots[i as usize];
                if slot.enabled {
                    let minor_match = slot.minor_frame == current_minor;
                    let major_match = slot.major_frame == 0 || slot.major_frame == current_major;
                    if minor_match && major_match {
                        let idx = result.action_count as usize;
                        result.actions[idx] = ScheduledAction {
                            target_channel: slot.target_channel,
                            payload_offset: slot.payload_offset,
                            payload_len: slot.payload_len,
                        };
                        result.action_count = result.action_count + 1;
                    }
                }
                i = i + 1;
            }
            result
        }
    }
}

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
