//! Relay Scheduler — verified core logic.
//!
//! This module contains the schedule table lookup and tick processing
//! logic. It is Verus-annotated and must satisfy all verification tracks:
//!
//!   - Verus (SMT/Z3): invariant preservation, bounds, no overflow
//!   - Rocq (via coq_of_rust on plain/): theorem proofs
//!   - Kani: bounded model checking harnesses
//!   - Lean: scheduling fairness / timing proofs
//!
//! NO async, NO alloc, NO trait objects, NO closures.
//! Write to the intersection of all verification tools.

/// Maximum number of schedule slots in the table.
/// Bounded to prevent unbounded iteration.
pub const MAX_SCHEDULE_SLOTS: usize = 256;

/// Maximum number of actions that can fire in a single tick.
/// Prevents unbounded output.
pub const MAX_ACTIONS_PER_TICK: usize = 16;

/// A single entry in the Schedule Definition Table.
/// Maps a (major_frame, minor_frame) slot to a channel + payload.
#[derive(Clone, Copy)]
pub struct ScheduleSlot {
    /// Which minor frame this slot fires on (0-based).
    pub minor_frame: u32,
    /// Which major frame this slot fires on (0 = every major frame).
    pub major_frame: u32,
    /// Target channel to send the action to.
    pub target_channel: u32,
    /// Payload length (index into payload_store).
    pub payload_offset: u32,
    /// Payload length in bytes.
    pub payload_len: u32,
    /// Whether this slot is enabled.
    pub enabled: bool,
}

/// An action to emit when a schedule slot fires.
#[derive(Clone, Copy)]
pub struct ScheduledAction {
    pub target_channel: u32,
    pub payload_offset: u32,
    pub payload_len: u32,
}

/// The Schedule Definition Table.
/// Fixed-size, no alloc. Slots are statically allocated.
pub struct ScheduleTable {
    slots: [ScheduleSlot; MAX_SCHEDULE_SLOTS],
    slot_count: u32,
}

/// Result of processing a single tick.
pub struct TickResult {
    pub actions: [ScheduledAction; MAX_ACTIONS_PER_TICK],
    pub action_count: u32,
}

impl ScheduleSlot {
    pub const fn empty() -> Self {
        ScheduleSlot {
            minor_frame: 0,
            major_frame: 0,
            target_channel: 0,
            payload_offset: 0,
            payload_len: 0,
            enabled: false,
        }
    }
}

impl ScheduledAction {
    pub const fn empty() -> Self {
        ScheduledAction {
            target_channel: 0,
            payload_offset: 0,
            payload_len: 0,
        }
    }
}

impl ScheduleTable {
    /// Create an empty schedule table.
    pub const fn new() -> Self {
        ScheduleTable {
            slots: [ScheduleSlot::empty(); MAX_SCHEDULE_SLOTS],
            slot_count: 0,
        }
    }

    /// Add a slot to the table. Returns false if table is full.
    pub fn add_slot(&mut self, slot: ScheduleSlot) -> bool {
        if self.slot_count as usize >= MAX_SCHEDULE_SLOTS {
            return false;
        }
        self.slots[self.slot_count as usize] = slot;
        self.slot_count = self.slot_count.wrapping_add(1);
        true
    }

    /// Enable or disable a slot by index. Returns false if index is out of range.
    pub fn set_enabled(&mut self, index: u32, enabled: bool) -> bool {
        if index >= self.slot_count {
            return false;
        }
        self.slots[index as usize].enabled = enabled;
        true
    }

    /// Get current slot count.
    pub fn slot_count(&self) -> u32 {
        self.slot_count
    }

    /// Process a tick: find all matching slots and emit actions.
    ///
    /// This is the core scheduling function. For each enabled slot whose
    /// (minor_frame, major_frame) matches the current tick, emit an action.
    ///
    /// Verified properties:
    ///   - action_count <= MAX_ACTIONS_PER_TICK (bounded output)
    ///   - action_count <= slot_count (can't emit more than slots)
    ///   - all emitted actions reference valid slot data
    ///   - no overflow in index arithmetic
    pub fn process_tick(
        &self,
        current_minor: u32,
        current_major: u32,
    ) -> TickResult {
        let mut result = TickResult {
            actions: [ScheduledAction::empty(); MAX_ACTIONS_PER_TICK],
            action_count: 0,
        };

        let count = self.slot_count as usize;
        let mut i: usize = 0;

        while i < count {
            if result.action_count as usize >= MAX_ACTIONS_PER_TICK {
                break;
            }

            let slot = &self.slots[i];

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
                    result.action_count = result.action_count.wrapping_add(1);
                }
            }

            i = i.wrapping_add(1);
        }

        result
    }
}

// ── Tests (run on plain Rust via verus-strip) ────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table_produces_no_actions() {
        let table = ScheduleTable::new();
        let result = table.process_tick(0, 0);
        assert_eq!(result.action_count, 0);
    }

    #[test]
    fn test_matching_slot_fires() {
        let mut table = ScheduleTable::new();
        table.add_slot(ScheduleSlot {
            minor_frame: 5,
            major_frame: 0, // every major frame
            target_channel: 42,
            payload_offset: 0,
            payload_len: 8,
            enabled: true,
        });

        // Tick at minor=5 should fire
        let result = table.process_tick(5, 1);
        assert_eq!(result.action_count, 1);
        assert_eq!(result.actions[0].target_channel, 42);

        // Tick at minor=6 should not fire
        let result = table.process_tick(6, 1);
        assert_eq!(result.action_count, 0);
    }

    #[test]
    fn test_disabled_slot_does_not_fire() {
        let mut table = ScheduleTable::new();
        table.add_slot(ScheduleSlot {
            minor_frame: 0,
            major_frame: 0,
            target_channel: 1,
            payload_offset: 0,
            payload_len: 0,
            enabled: false,
        });

        let result = table.process_tick(0, 0);
        assert_eq!(result.action_count, 0);
    }

    #[test]
    fn test_major_frame_filtering() {
        let mut table = ScheduleTable::new();
        table.add_slot(ScheduleSlot {
            minor_frame: 0,
            major_frame: 3, // only major frame 3
            target_channel: 10,
            payload_offset: 0,
            payload_len: 4,
            enabled: true,
        });

        // Major frame 3 should fire
        let result = table.process_tick(0, 3);
        assert_eq!(result.action_count, 1);

        // Major frame 2 should not
        let result = table.process_tick(0, 2);
        assert_eq!(result.action_count, 0);
    }

    #[test]
    fn test_action_count_bounded() {
        let mut table = ScheduleTable::new();
        // Add more slots than MAX_ACTIONS_PER_TICK, all matching
        for ch in 0..(MAX_ACTIONS_PER_TICK as u32 + 10) {
            table.add_slot(ScheduleSlot {
                minor_frame: 0,
                major_frame: 0,
                target_channel: ch,
                payload_offset: 0,
                payload_len: 0,
                enabled: true,
            });
        }

        let result = table.process_tick(0, 0);
        assert_eq!(result.action_count, MAX_ACTIONS_PER_TICK as u32);
    }

    #[test]
    fn test_table_full_returns_false() {
        let mut table = ScheduleTable::new();
        for _ in 0..MAX_SCHEDULE_SLOTS {
            assert!(table.add_slot(ScheduleSlot::empty()));
        }
        // Table full
        assert!(!table.add_slot(ScheduleSlot::empty()));
    }

    #[test]
    fn test_enable_disable_slot() {
        let mut table = ScheduleTable::new();
        table.add_slot(ScheduleSlot {
            minor_frame: 0,
            major_frame: 0,
            target_channel: 1,
            payload_offset: 0,
            payload_len: 0,
            enabled: true,
        });

        // Initially fires
        assert_eq!(table.process_tick(0, 0).action_count, 1);

        // Disable it
        assert!(table.set_enabled(0, false));
        assert_eq!(table.process_tick(0, 0).action_count, 0);

        // Re-enable
        assert!(table.set_enabled(0, true));
        assert_eq!(table.process_tick(0, 0).action_count, 1);

        // Out of range
        assert!(!table.set_enabled(99, true));
    }
}
