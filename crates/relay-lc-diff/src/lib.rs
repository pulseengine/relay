//! Differential testing for relay-lc: compare the optimized engine against a
//! naive reference implementation that is intentionally simple and obviously correct.

pub use relay_lc::engine::{ComparisonOp, SensorReading, Watchpoint, WatchpointTable};

/// Reference compare — intentionally simple, no optimization.
pub fn reference_compare(value: i64, op: ComparisonOp, threshold: i64) -> bool {
    match op {
        ComparisonOp::LessThan => value < threshold,
        ComparisonOp::GreaterThan => value > threshold,
        ComparisonOp::LessOrEqual => value <= threshold,
        ComparisonOp::GreaterOrEqual => value >= threshold,
        ComparisonOp::Equal => value == threshold,
        ComparisonOp::NotEqual => value != threshold,
    }
}

/// Simplified watchpoint descriptor for the reference implementation.
/// (sensor_id, op, threshold, enabled, persistence)
#[derive(Clone, Copy)]
pub struct RefWatchpoint {
    pub sensor_id: u32,
    pub op: ComparisonOp,
    pub threshold: i64,
    pub enabled: bool,
    pub persistence: u32,
}

/// Reference evaluate — linear scan, no table structure, no bounded output array.
/// Returns the set of watchpoint indices that fired (after persistence tracking).
pub fn reference_evaluate(
    watchpoints: &[RefWatchpoint],
    persistence_counts: &mut [u32],
    sensor_id: u32,
    value: i64,
) -> Vec<usize> {
    let mut violations = vec![];
    for (i, wp) in watchpoints.iter().enumerate() {
        if !wp.enabled || wp.sensor_id != sensor_id {
            continue;
        }
        let violated = reference_compare(value, wp.op, wp.threshold);
        if violated {
            persistence_counts[i] = persistence_counts[i].saturating_add(1);
            if persistence_counts[i] >= wp.persistence {
                violations.push(i);
            }
        } else {
            persistence_counts[i] = 0;
        }
    }
    violations
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use relay_lc::engine::{compare, MAX_VIOLATIONS_PER_CYCLE};

    fn op_from_u8(v: u8) -> ComparisonOp {
        match v % 6 {
            0 => ComparisonOp::LessThan,
            1 => ComparisonOp::GreaterThan,
            2 => ComparisonOp::LessOrEqual,
            3 => ComparisonOp::GreaterOrEqual,
            4 => ComparisonOp::Equal,
            _ => ComparisonOp::NotEqual,
        }
    }

    proptest! {
        /// For every (value, op, threshold) triple, relay_lc::compare and reference_compare agree.
        #[test]
        fn compare_agrees(
            value in prop::num::i64::ANY,
            threshold in prop::num::i64::ANY,
            op_byte in 0u8..6,
        ) {
            let op = op_from_u8(op_byte);
            let engine_result = compare(value, op, threshold);
            let ref_result = reference_compare(value, op, threshold);
            prop_assert_eq!(engine_result, ref_result,
                "compare disagreement: value={}, op={:?}, threshold={}", value, op_byte, threshold);
        }

        /// For a single watchpoint with persistence=1, the set of violations from
        /// the engine matches the reference (modulo the MAX_VIOLATIONS_PER_CYCLE cap).
        #[test]
        fn single_eval_agrees(
            sensor_id in 0u32..50,
            num_wps in 1usize..20,
            ops in proptest::collection::vec(0u8..6, 1..20),
            thresholds in proptest::collection::vec(-500i64..500, 1..20),
            value in -1000i64..1000,
        ) {
            let n = num_wps.min(ops.len()).min(thresholds.len());
            let ref_wps: Vec<RefWatchpoint> = (0..n).map(|i| RefWatchpoint {
                sensor_id,
                op: op_from_u8(ops[i]),
                threshold: thresholds[i],
                enabled: true,
                persistence: 1,
            }).collect();

            let mut persistence_counts = vec![0u32; n];
            let ref_violations = reference_evaluate(&ref_wps, &mut persistence_counts, sensor_id, value);

            let mut table = WatchpointTable::new();
            for rw in &ref_wps {
                table.add_watchpoint(Watchpoint {
                    sensor_id: rw.sensor_id,
                    op: rw.op,
                    threshold: rw.threshold,
                    enabled: rw.enabled,
                    persistence: rw.persistence,
                    current_count: 0,
                });
            }
            let engine_result = table.evaluate(SensorReading { sensor_id, value });

            // Engine caps at MAX_VIOLATIONS_PER_CYCLE; reference does not.
            let expected_count = ref_violations.len().min(MAX_VIOLATIONS_PER_CYCLE);
            prop_assert_eq!(engine_result.violation_count as usize, expected_count,
                "violation count mismatch: engine={}, ref={} (capped={})",
                engine_result.violation_count, ref_violations.len(), expected_count);
        }

        /// Multi-step persistence differential: run a sequence of readings and
        /// verify the engine and reference agree at each step.
        #[test]
        fn persistence_sequence_agrees(
            threshold in -100i64..100,
            persistence in 1u32..6,
            values in proptest::collection::vec(-200i64..200, 1..20),
        ) {
            let sensor_id = 1u32;
            let op = ComparisonOp::GreaterThan;

            let ref_wps = vec![RefWatchpoint {
                sensor_id, op, threshold, enabled: true, persistence,
            }];
            let mut persistence_counts = vec![0u32; 1];

            let mut table = WatchpointTable::new();
            table.add_watchpoint(Watchpoint {
                sensor_id, op, threshold, enabled: true, persistence, current_count: 0,
            });

            for &v in &values {
                let ref_violations = reference_evaluate(&ref_wps, &mut persistence_counts, sensor_id, v);
                let engine_result = table.evaluate(SensorReading { sensor_id, value: v });

                let expected = if ref_violations.is_empty() { 0u32 } else { 1 };
                prop_assert_eq!(engine_result.violation_count, expected,
                    "persistence mismatch at value={}: engine={}, ref={}",
                    v, engine_result.violation_count, expected);
            }
        }
    }
}
