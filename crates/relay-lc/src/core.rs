//! Relay Limit Checker — verified core logic.
//!
//! Monitors sensor readings against configurable thresholds.
//! Two-table design:
//!   - Watchpoint Definition Table (WDT): individual data monitors
//!   - Actionpoint Definition Table (ADT): logical combinations triggering responses
//!
//! NO async, NO alloc, NO trait objects, NO closures.
//! Write to the intersection of all verification tools.

/// Maximum number of watchpoints.
pub const MAX_WATCHPOINTS: usize = 128;

/// Maximum number of actionpoints.
pub const MAX_ACTIONPOINTS: usize = 64;

/// Maximum number of violations per evaluation cycle.
pub const MAX_VIOLATIONS_PER_CYCLE: usize = 32;

/// Maximum number of watchpoint inputs to an actionpoint.
pub const MAX_ACTIONPOINT_INPUTS: usize = 8;

/// Comparison operator for threshold evaluation.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ComparisonOp {
    LessThan = 0,
    GreaterThan = 1,
    LessOrEqual = 2,
    GreaterOrEqual = 3,
    Equal = 4,
    NotEqual = 5,
}

/// Logical operator for combining watchpoint results in actionpoints.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LogicOp {
    And = 0,
    Or = 1,
}

/// A single watchpoint: monitors one sensor against one threshold.
#[derive(Clone, Copy)]
pub struct Watchpoint {
    /// Sensor ID to monitor.
    pub sensor_id: u32,
    /// Comparison operator.
    pub op: ComparisonOp,
    /// Threshold value.
    pub threshold: i64,
    /// Whether this watchpoint is enabled.
    pub enabled: bool,
    /// Consecutive violation count before triggering.
    pub persistence: u32,
    /// Current consecutive violation count.
    pub current_count: u32,
}

/// A single actionpoint: logical combination of watchpoint results.
#[derive(Clone, Copy)]
pub struct Actionpoint {
    /// Watchpoint indices that feed this actionpoint.
    pub inputs: [u32; MAX_ACTIONPOINT_INPUTS],
    /// Number of active inputs.
    pub input_count: u32,
    /// How to combine inputs.
    pub logic: LogicOp,
    /// Whether this actionpoint is enabled.
    pub enabled: bool,
    /// Channel to notify when actionpoint triggers.
    pub target_channel: u32,
}

/// A limit violation detected during evaluation.
#[derive(Clone, Copy)]
pub struct Violation {
    /// Which watchpoint triggered.
    pub watchpoint_id: u32,
    /// The sensor value that exceeded the threshold.
    pub measured: i64,
    /// The threshold that was exceeded.
    pub threshold: i64,
    /// The comparison that triggered.
    pub op: ComparisonOp,
}

/// A sensor reading to evaluate.
#[derive(Clone, Copy)]
pub struct SensorReading {
    pub sensor_id: u32,
    /// Value scaled to i64 fixed-point (avoids f64 in verified code).
    /// Scale factor is application-defined (e.g., 1000 = milliunit).
    pub value: i64,
}

/// Result of evaluating one cycle of sensor readings.
pub struct EvalResult {
    pub violations: [Violation; MAX_VIOLATIONS_PER_CYCLE],
    pub violation_count: u32,
}

/// The Watchpoint Definition Table.
pub struct WatchpointTable {
    entries: [Watchpoint; MAX_WATCHPOINTS],
    entry_count: u32,
}

/// Compare a value against a threshold using the given operator.
///
/// Verified property: this function is total and deterministic.
/// No overflow possible: comparison of two i64 values.
fn compare(value: i64, op: ComparisonOp, threshold: i64) -> bool {
    match op {
        ComparisonOp::LessThan => value < threshold,
        ComparisonOp::GreaterThan => value > threshold,
        ComparisonOp::LessOrEqual => value <= threshold,
        ComparisonOp::GreaterOrEqual => value >= threshold,
        ComparisonOp::Equal => value == threshold,
        ComparisonOp::NotEqual => value != threshold,
    }
}

impl Watchpoint {
    pub const fn empty() -> Self {
        Watchpoint {
            sensor_id: 0,
            op: ComparisonOp::LessThan,
            threshold: 0,
            enabled: false,
            persistence: 1,
            current_count: 0,
        }
    }
}

impl Violation {
    pub const fn empty() -> Self {
        Violation {
            watchpoint_id: 0,
            measured: 0,
            threshold: 0,
            op: ComparisonOp::LessThan,
        }
    }
}

impl WatchpointTable {
    pub const fn new() -> Self {
        WatchpointTable {
            entries: [Watchpoint::empty(); MAX_WATCHPOINTS],
            entry_count: 0,
        }
    }

    /// Add a watchpoint. Returns false if table is full.
    pub fn add_watchpoint(&mut self, wp: Watchpoint) -> bool {
        if self.entry_count as usize >= MAX_WATCHPOINTS {
            return false;
        }
        self.entries[self.entry_count as usize] = wp;
        self.entry_count = self.entry_count.wrapping_add(1);
        true
    }

    /// Get the watchpoint count.
    pub fn count(&self) -> u32 {
        self.entry_count
    }

    /// Evaluate a single sensor reading against all watchpoints.
    ///
    /// Verified properties:
    ///   - violation_count <= MAX_VIOLATIONS_PER_CYCLE (bounded output)
    ///   - violation_count <= entry_count (can't exceed watchpoints)
    ///   - all violations reference valid watchpoint data
    ///   - persistence counter increments correctly
    ///   - persistence counter resets on non-violation
    pub fn evaluate(
        &mut self,
        reading: SensorReading,
    ) -> EvalResult {
        let mut result = EvalResult {
            violations: [Violation::empty(); MAX_VIOLATIONS_PER_CYCLE],
            violation_count: 0,
        };

        let count = self.entry_count as usize;
        let mut i: usize = 0;

        while i < count {
            if result.violation_count as usize >= MAX_VIOLATIONS_PER_CYCLE {
                break;
            }

            let wp = &mut self.entries[i];

            if wp.enabled && wp.sensor_id == reading.sensor_id {
                let violated = compare(reading.value, wp.op, wp.threshold);

                if violated {
                    wp.current_count = wp.current_count.saturating_add(1);

                    if wp.current_count >= wp.persistence {
                        let idx = result.violation_count as usize;
                        result.violations[idx] = Violation {
                            watchpoint_id: i as u32,
                            measured: reading.value,
                            threshold: wp.threshold,
                            op: wp.op,
                        };
                        result.violation_count = result.violation_count.wrapping_add(1);
                    }
                } else {
                    wp.current_count = 0;
                }
            }

            i = i.wrapping_add(1);
        }

        result
    }
}

// ── Tests ────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table_no_violations() {
        let mut table = WatchpointTable::new();
        let reading = SensorReading { sensor_id: 1, value: 100 };
        let result = table.evaluate(reading);
        assert_eq!(result.violation_count, 0);
    }

    #[test]
    fn test_greater_than_violation() {
        let mut table = WatchpointTable::new();
        table.add_watchpoint(Watchpoint {
            sensor_id: 1,
            op: ComparisonOp::GreaterThan,
            threshold: 50,
            enabled: true,
            persistence: 1,
            current_count: 0,
        });

        // Value 100 > 50 → violation
        let result = table.evaluate(SensorReading { sensor_id: 1, value: 100 });
        assert_eq!(result.violation_count, 1);
        assert_eq!(result.violations[0].measured, 100);
        assert_eq!(result.violations[0].threshold, 50);

        // Value 30 < 50 → no violation
        let result = table.evaluate(SensorReading { sensor_id: 1, value: 30 });
        assert_eq!(result.violation_count, 0);
    }

    #[test]
    fn test_persistence_delays_violation() {
        let mut table = WatchpointTable::new();
        table.add_watchpoint(Watchpoint {
            sensor_id: 1,
            op: ComparisonOp::GreaterThan,
            threshold: 50,
            enabled: true,
            persistence: 3, // must exceed 3 consecutive times
            current_count: 0,
        });

        let reading = SensorReading { sensor_id: 1, value: 100 };

        // First two: count building, no violation yet
        assert_eq!(table.evaluate(reading).violation_count, 0);
        assert_eq!(table.evaluate(reading).violation_count, 0);

        // Third: persistence met, violation fires
        assert_eq!(table.evaluate(reading).violation_count, 1);
    }

    #[test]
    fn test_persistence_resets_on_normal() {
        let mut table = WatchpointTable::new();
        table.add_watchpoint(Watchpoint {
            sensor_id: 1,
            op: ComparisonOp::GreaterThan,
            threshold: 50,
            enabled: true,
            persistence: 3,
            current_count: 0,
        });

        let bad = SensorReading { sensor_id: 1, value: 100 };
        let good = SensorReading { sensor_id: 1, value: 10 };

        // Two violations, then a normal reading resets counter
        table.evaluate(bad);
        table.evaluate(bad);
        table.evaluate(good); // resets

        // Need 3 more consecutive violations now
        assert_eq!(table.evaluate(bad).violation_count, 0);
        assert_eq!(table.evaluate(bad).violation_count, 0);
        assert_eq!(table.evaluate(bad).violation_count, 1);
    }

    #[test]
    fn test_sensor_id_filtering() {
        let mut table = WatchpointTable::new();
        table.add_watchpoint(Watchpoint {
            sensor_id: 42,
            op: ComparisonOp::LessThan,
            threshold: 10,
            enabled: true,
            persistence: 1,
            current_count: 0,
        });

        // Different sensor → no match
        let result = table.evaluate(SensorReading { sensor_id: 99, value: 0 });
        assert_eq!(result.violation_count, 0);

        // Correct sensor → violation
        let result = table.evaluate(SensorReading { sensor_id: 42, value: 5 });
        assert_eq!(result.violation_count, 1);
    }

    #[test]
    fn test_disabled_watchpoint_ignored() {
        let mut table = WatchpointTable::new();
        table.add_watchpoint(Watchpoint {
            sensor_id: 1,
            op: ComparisonOp::GreaterThan,
            threshold: 0,
            enabled: false,
            persistence: 1,
            current_count: 0,
        });

        let result = table.evaluate(SensorReading { sensor_id: 1, value: 999 });
        assert_eq!(result.violation_count, 0);
    }

    #[test]
    fn test_all_comparison_ops() {
        assert!(compare(5, ComparisonOp::LessThan, 10));
        assert!(!compare(10, ComparisonOp::LessThan, 5));
        assert!(compare(10, ComparisonOp::GreaterThan, 5));
        assert!(!compare(5, ComparisonOp::GreaterThan, 10));
        assert!(compare(5, ComparisonOp::LessOrEqual, 5));
        assert!(compare(5, ComparisonOp::GreaterOrEqual, 5));
        assert!(compare(5, ComparisonOp::Equal, 5));
        assert!(!compare(5, ComparisonOp::NotEqual, 5));
        assert!(compare(5, ComparisonOp::NotEqual, 6));
    }

    #[test]
    fn test_violation_count_bounded() {
        let mut table = WatchpointTable::new();
        // Add more watchpoints than MAX_VIOLATIONS_PER_CYCLE, all matching
        for _ in 0..(MAX_VIOLATIONS_PER_CYCLE + 10) {
            table.add_watchpoint(Watchpoint {
                sensor_id: 1,
                op: ComparisonOp::GreaterThan,
                threshold: 0,
                enabled: true,
                persistence: 1,
                current_count: 0,
            });
        }

        let result = table.evaluate(SensorReading { sensor_id: 1, value: 100 });
        assert_eq!(result.violation_count, MAX_VIOLATIONS_PER_CYCLE as u32);
    }
}
