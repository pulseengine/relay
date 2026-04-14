#![no_main]
use libfuzzer_sys::fuzz_target;
use relay_sch::engine::*;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let mut table = ScheduleTable::new();

    // Parse slots from the input data (each slot needs 9 bytes: 4 minor + 4 major + 1 enabled)
    let slot_region = if data.len() > 8 { &data[..data.len() - 8] } else { &[] as &[u8] };
    let mut offset = 0;
    while offset + 9 <= slot_region.len() && table.slot_count() < MAX_SCHEDULE_SLOTS as u32 {
        let minor = u32::from_le_bytes([
            slot_region[offset],
            slot_region[offset + 1],
            slot_region[offset + 2],
            slot_region[offset + 3],
        ]);
        let major = u32::from_le_bytes([
            slot_region[offset + 4],
            slot_region[offset + 5],
            slot_region[offset + 6],
            slot_region[offset + 7],
        ]);
        let enabled = slot_region[offset + 8] & 1 == 1;
        table.add_slot(ScheduleSlot {
            minor_frame: minor,
            major_frame: major,
            target_channel: offset as u32,
            payload_offset: 0,
            payload_len: 0,
            enabled,
        });
        offset += 9;
    }

    // Last 8 bytes: tick parameters (4 minor + 4 major)
    let tail = &data[data.len() - 8..];
    let tick_minor = u32::from_le_bytes([tail[0], tail[1], tail[2], tail[3]]);
    let tick_major = u32::from_le_bytes([tail[4], tail[5], tail[6], tail[7]]);

    let result = table.process_tick(tick_minor, tick_major);
    assert!(result.action_count as usize <= MAX_ACTIONS_PER_TICK);
});
