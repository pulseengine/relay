//! Relay Scheduler — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/core.rs (Verus-annotated). Do not edit manually.

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
                    result.actions[idx] = ScheduledAction { target_channel: slot.target_channel, payload_offset: slot.payload_offset, payload_len: slot.payload_len };
                    result.action_count = result.action_count + 1;
                }
            }
            i = i + 1;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn test_empty() { assert_eq!(ScheduleTable::new().process_tick(0, 0).action_count, 0); }
    #[test] fn test_match() { let mut t = ScheduleTable::new(); t.add_slot(ScheduleSlot { minor_frame: 5, major_frame: 0, target_channel: 42, payload_offset: 0, payload_len: 8, enabled: true }); assert_eq!(t.process_tick(5, 1).action_count, 1); assert_eq!(t.process_tick(6, 1).action_count, 0); }
    #[test] fn test_disabled() { let mut t = ScheduleTable::new(); t.add_slot(ScheduleSlot { minor_frame: 0, major_frame: 0, target_channel: 1, payload_offset: 0, payload_len: 0, enabled: false }); assert_eq!(t.process_tick(0, 0).action_count, 0); }
    #[test] fn test_major() { let mut t = ScheduleTable::new(); t.add_slot(ScheduleSlot { minor_frame: 0, major_frame: 3, target_channel: 10, payload_offset: 0, payload_len: 4, enabled: true }); assert_eq!(t.process_tick(0, 3).action_count, 1); assert_eq!(t.process_tick(0, 2).action_count, 0); }
    #[test] fn test_bounded() { let mut t = ScheduleTable::new(); for ch in 0..(MAX_ACTIONS_PER_TICK as u32 + 10) { t.add_slot(ScheduleSlot { minor_frame: 0, major_frame: 0, target_channel: ch, payload_offset: 0, payload_len: 0, enabled: true }); } assert_eq!(t.process_tick(0, 0).action_count, MAX_ACTIONS_PER_TICK as u32); }
    #[test] fn test_full() { let mut t = ScheduleTable::new(); for _ in 0..MAX_SCHEDULE_SLOTS { assert!(t.add_slot(ScheduleSlot::empty())); } assert!(!t.add_slot(ScheduleSlot::empty())); }
    #[test] fn test_enable() { let mut t = ScheduleTable::new(); t.add_slot(ScheduleSlot { minor_frame: 0, major_frame: 0, target_channel: 1, payload_offset: 0, payload_len: 0, enabled: true }); assert_eq!(t.process_tick(0, 0).action_count, 1); t.set_enabled(0, false); assert_eq!(t.process_tick(0, 0).action_count, 0); assert!(!t.set_enabled(99, true)); }
}
