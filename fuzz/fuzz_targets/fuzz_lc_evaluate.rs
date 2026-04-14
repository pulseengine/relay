#![no_main]
use libfuzzer_sys::fuzz_target;
use relay_lc::engine::*;

fuzz_target!(|data: &[u8]| {
    if data.len() < 13 {
        return;
    }
    let mut table = WatchpointTable::new();
    // Parse first bytes as watchpoint config
    let sensor_id = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let op_byte = data[4] % 6;
    let op = match op_byte {
        0 => ComparisonOp::LessThan,
        1 => ComparisonOp::GreaterThan,
        2 => ComparisonOp::LessOrEqual,
        3 => ComparisonOp::GreaterOrEqual,
        4 => ComparisonOp::Equal,
        _ => ComparisonOp::NotEqual,
    };
    let threshold = i64::from_le_bytes([
        data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12],
    ]);
    table.add_watchpoint(Watchpoint {
        sensor_id,
        op,
        threshold,
        enabled: true,
        persistence: 1,
        current_count: 0,
    });
    // Evaluate with remaining bytes as value
    if data.len() >= 21 {
        let value = i64::from_le_bytes([
            data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20],
        ]);
        let result = table.evaluate(SensorReading { sensor_id, value });
        assert!(result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE);
    }
});
