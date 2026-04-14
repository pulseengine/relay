//! Relay Limit Checker — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/core.rs (Verus-annotated). Do not edit manually.

pub const MAX_WATCHPOINTS: usize = 128;
pub const MAX_VIOLATIONS_PER_CYCLE: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ComparisonOp { LessThan = 0, GreaterThan = 1, LessOrEqual = 2, GreaterOrEqual = 3, Equal = 4, NotEqual = 5 }

#[derive(Clone, Copy, Debug)]
pub struct Watchpoint { pub sensor_id: u32, pub op: ComparisonOp, pub threshold: i64, pub enabled: bool, pub persistence: u32, pub current_count: u32 }

#[derive(Clone, Copy, Debug)]
pub struct Violation { pub watchpoint_id: u32, pub measured: i64, pub threshold: i64, pub op: ComparisonOp }

#[derive(Clone, Copy, Debug)]
pub struct SensorReading { pub sensor_id: u32, pub value: i64 }

pub struct EvalResult { pub violations: [Violation; MAX_VIOLATIONS_PER_CYCLE], pub violation_count: u32 }

pub struct WatchpointTable { entries: [Watchpoint; MAX_WATCHPOINTS], entry_count: u32 }

pub fn compare(value: i64, op: ComparisonOp, threshold: i64) -> bool {
    match op {
        ComparisonOp::LessThan => value < threshold,
        ComparisonOp::GreaterThan => value > threshold,
        ComparisonOp::LessOrEqual => value <= threshold,
        ComparisonOp::GreaterOrEqual => value >= threshold,
        ComparisonOp::Equal => value == threshold,
        ComparisonOp::NotEqual => value != threshold,
    }
}

impl Watchpoint { pub const fn empty() -> Self { Watchpoint { sensor_id: 0, op: ComparisonOp::LessThan, threshold: 0, enabled: false, persistence: 1, current_count: 0 } } }
impl Violation { pub const fn empty() -> Self { Violation { watchpoint_id: 0, measured: 0, threshold: 0, op: ComparisonOp::LessThan } } }

impl WatchpointTable {
    pub const NEW: Self = WatchpointTable { entries: [Watchpoint::empty(); MAX_WATCHPOINTS], entry_count: 0 };
    pub fn new() -> Self { Self::NEW }

    pub fn add_watchpoint(&mut self, wp: Watchpoint) -> bool {
        if self.entry_count as usize >= MAX_WATCHPOINTS { return false; }
        self.entries[self.entry_count as usize] = wp;
        self.entry_count = self.entry_count + 1;
        true
    }

    pub fn count(&self) -> u32 { self.entry_count }

    pub fn evaluate(&mut self, reading: SensorReading) -> EvalResult {
        let mut result = EvalResult { violations: [Violation::empty(); MAX_VIOLATIONS_PER_CYCLE], violation_count: 0 };
        let count = self.entry_count;
        let mut i: u32 = 0;
        while i < count {
            if result.violation_count as usize >= MAX_VIOLATIONS_PER_CYCLE { break; }
            let idx = i as usize;
            let enabled = self.entries[idx].enabled;
            let sid = self.entries[idx].sensor_id;
            let op = self.entries[idx].op;
            let threshold = self.entries[idx].threshold;
            let persistence = self.entries[idx].persistence;
            if enabled && sid == reading.sensor_id {
                let violated = compare(reading.value, op, threshold);
                if violated {
                    self.entries[idx].current_count = if self.entries[idx].current_count < u32::MAX { self.entries[idx].current_count + 1 } else { u32::MAX };
                    if self.entries[idx].current_count >= persistence {
                        let vidx = result.violation_count as usize;
                        result.violations[vidx] = Violation { watchpoint_id: i, measured: reading.value, threshold, op };
                        result.violation_count = result.violation_count + 1;
                    }
                } else {
                    self.entries[idx].current_count = 0;
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

    #[test] fn test_empty() { let mut t = WatchpointTable::new(); assert_eq!(t.evaluate(SensorReading { sensor_id: 1, value: 100 }).violation_count, 0); }
    #[test] fn test_gt_violation() { let mut t = WatchpointTable::new(); t.add_watchpoint(Watchpoint { sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 50, enabled: true, persistence: 1, current_count: 0 }); assert_eq!(t.evaluate(SensorReading { sensor_id: 1, value: 100 }).violation_count, 1); assert_eq!(t.evaluate(SensorReading { sensor_id: 1, value: 30 }).violation_count, 0); }
    #[test] fn test_persistence() { let mut t = WatchpointTable::new(); t.add_watchpoint(Watchpoint { sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 50, enabled: true, persistence: 3, current_count: 0 }); let r = SensorReading { sensor_id: 1, value: 100 }; assert_eq!(t.evaluate(r).violation_count, 0); assert_eq!(t.evaluate(r).violation_count, 0); assert_eq!(t.evaluate(r).violation_count, 1); }
    #[test] fn test_persistence_reset() { let mut t = WatchpointTable::new(); t.add_watchpoint(Watchpoint { sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 50, enabled: true, persistence: 3, current_count: 0 }); let bad = SensorReading { sensor_id: 1, value: 100 }; let good = SensorReading { sensor_id: 1, value: 10 }; t.evaluate(bad); t.evaluate(bad); t.evaluate(good); assert_eq!(t.evaluate(bad).violation_count, 0); assert_eq!(t.evaluate(bad).violation_count, 0); assert_eq!(t.evaluate(bad).violation_count, 1); }
    #[test] fn test_sensor_filter() { let mut t = WatchpointTable::new(); t.add_watchpoint(Watchpoint { sensor_id: 42, op: ComparisonOp::LessThan, threshold: 10, enabled: true, persistence: 1, current_count: 0 }); assert_eq!(t.evaluate(SensorReading { sensor_id: 99, value: 0 }).violation_count, 0); assert_eq!(t.evaluate(SensorReading { sensor_id: 42, value: 5 }).violation_count, 1); }
    #[test] fn test_disabled() { let mut t = WatchpointTable::new(); t.add_watchpoint(Watchpoint { sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 0, enabled: false, persistence: 1, current_count: 0 }); assert_eq!(t.evaluate(SensorReading { sensor_id: 1, value: 999 }).violation_count, 0); }
    #[test] fn test_ops() { assert!(compare(5, ComparisonOp::LessThan, 10)); assert!(compare(10, ComparisonOp::GreaterThan, 5)); assert!(compare(5, ComparisonOp::Equal, 5)); assert!(compare(5, ComparisonOp::NotEqual, 6)); }
    #[test] fn test_bounded() { let mut t = WatchpointTable::new(); for _ in 0..(MAX_VIOLATIONS_PER_CYCLE + 10) { t.add_watchpoint(Watchpoint { sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 0, enabled: true, persistence: 1, current_count: 0 }); } assert_eq!(t.evaluate(SensorReading { sensor_id: 1, value: 100 }).violation_count, MAX_VIOLATIONS_PER_CYCLE as u32); }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    prop_compose! {
        fn arb_watchpoint()(
            sensor_id in 0u32..100,
            op in 0u8..6,
            threshold in -1000i64..1000,
            persistence in 1u32..10,
        ) -> Watchpoint {
            let op = match op {
                0 => ComparisonOp::LessThan, 1 => ComparisonOp::GreaterThan,
                2 => ComparisonOp::LessOrEqual, 3 => ComparisonOp::GreaterOrEqual,
                4 => ComparisonOp::Equal, _ => ComparisonOp::NotEqual,
            };
            Watchpoint { sensor_id, op, threshold, enabled: true, persistence, current_count: 0 }
        }
    }

    proptest! {
        #[test]
        fn output_always_bounded(
            wps in proptest::collection::vec(arb_watchpoint(), 1..20),
            sensor_id in 0u32..100,
            value in -2000i64..2000,
        ) {
            let mut table = WatchpointTable::new();
            for wp in &wps { table.add_watchpoint(*wp); }
            let result = table.evaluate(SensorReading { sensor_id, value });
            prop_assert!(result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE);
            prop_assert!(result.violation_count <= table.count());
        }

        #[test]
        fn disabled_never_fires(
            sensor_id in 0u32..100,
            threshold in -1000i64..1000,
            value in -2000i64..2000,
        ) {
            let mut table = WatchpointTable::new();
            table.add_watchpoint(Watchpoint {
                sensor_id, op: ComparisonOp::GreaterThan, threshold,
                enabled: false, persistence: 1, current_count: 0,
            });
            let result = table.evaluate(SensorReading { sensor_id, value });
            prop_assert_eq!(result.violation_count, 0);
        }

        #[test]
        fn compare_matches_rust(
            value in i64::MIN..i64::MAX,
            threshold in i64::MIN..i64::MAX,
        ) {
            prop_assert_eq!(compare(value, ComparisonOp::LessThan, threshold), value < threshold);
            prop_assert_eq!(compare(value, ComparisonOp::GreaterThan, threshold), value > threshold);
            prop_assert_eq!(compare(value, ComparisonOp::Equal, threshold), value == threshold);
        }

        #[test]
        fn persistence_requires_consecutive(
            value in 1i64..1000,
            persistence in 2u32..10,
        ) {
            let mut table = WatchpointTable::new();
            table.add_watchpoint(Watchpoint {
                sensor_id: 1, op: ComparisonOp::GreaterThan, threshold: 0,
                enabled: true, persistence, current_count: 0,
            });
            // First (persistence-1) evaluations should NOT fire
            for _ in 0..persistence-1 {
                let r = table.evaluate(SensorReading { sensor_id: 1, value });
                prop_assert_eq!(r.violation_count, 0);
            }
            // The persistence-th evaluation SHOULD fire
            let r = table.evaluate(SensorReading { sensor_id: 1, value });
            prop_assert_eq!(r.violation_count, 1);
        }
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// LC-P04: violation_count never exceeds MAX_VIOLATIONS_PER_CYCLE
    #[kani::proof]
    fn verify_violation_count_bounded() {
        let mut table = WatchpointTable::new();
        let sensor_id: u32 = kani::any();
        let op_val: u8 = kani::any();
        kani::assume(op_val <= 5);
        let op = match op_val {
            0 => ComparisonOp::LessThan, 1 => ComparisonOp::GreaterThan,
            2 => ComparisonOp::LessOrEqual, 3 => ComparisonOp::GreaterOrEqual,
            4 => ComparisonOp::Equal, _ => ComparisonOp::NotEqual,
        };
        let threshold: i64 = kani::any();
        let persistence: u32 = kani::any();
        kani::assume(persistence >= 1);

        table.add_watchpoint(Watchpoint {
            sensor_id, op, threshold, enabled: true, persistence, current_count: 0,
        });

        let value: i64 = kani::any();
        let result = table.evaluate(SensorReading { sensor_id, value });
        assert!(result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE);
    }

    /// LC-P06: compare is total — never panics for any input
    #[kani::proof]
    fn verify_compare_total() {
        let value: i64 = kani::any();
        let threshold: i64 = kani::any();
        let op_val: u8 = kani::any();
        kani::assume(op_val <= 5);
        let op = match op_val {
            0 => ComparisonOp::LessThan, 1 => ComparisonOp::GreaterThan,
            2 => ComparisonOp::LessOrEqual, 3 => ComparisonOp::GreaterOrEqual,
            4 => ComparisonOp::Equal, _ => ComparisonOp::NotEqual,
        };
        let result = compare(value, op, threshold);
        // Just verifying it doesn't panic and returns a bool
        assert!(result || !result);
    }

    /// LC-P05: disabled watchpoints never produce violations
    #[kani::proof]
    fn verify_disabled_no_violations() {
        let mut table = WatchpointTable::new();
        let sensor_id: u32 = kani::any();
        kani::assume(sensor_id < 100);
        table.add_watchpoint(Watchpoint {
            sensor_id, op: ComparisonOp::GreaterThan, threshold: 0,
            enabled: false, persistence: 1, current_count: 0,
        });
        let value: i64 = kani::any();
        let result = table.evaluate(SensorReading { sensor_id, value });
        assert_eq!(result.violation_count, 0);
    }

    /// LC-P03: compare matches the operator semantics
    #[kani::proof]
    fn verify_compare_semantics() {
        let v: i64 = kani::any();
        let t: i64 = kani::any();
        assert_eq!(compare(v, ComparisonOp::LessThan, t), v < t);
        assert_eq!(compare(v, ComparisonOp::GreaterThan, t), v > t);
        assert_eq!(compare(v, ComparisonOp::Equal, t), v == t);
    }
}
