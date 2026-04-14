//! C API for Relay Limit Checker — cFS-compatible drop-in replacement.
//!
//! This module exposes the verified WatchpointTable engine behind C function
//! signatures compatible with NASA cFS LC (lc_watch.c). A cFS build can link
//! this instead of the original C implementation.
//!
//! Rich Landau's pattern (Goddard): "comment something out in C, rewrite it
//! in Rust, link it with the existing C and have it all work together."
//!
//! Scaling contract for float↔i64 conversion:
//!   cFS LC uses double for watchpoint thresholds and telemetry values.
//!   This API receives f64 from C, converts to i64 fixed-point (×1000),
//!   calls the verified engine, converts back.
//!   Precision: 0.001 units. Range: ±9.2×10¹⁵ (i64/1000).
//!
//! The verified engine (8 Verus properties, 8 cargo tests, 4 Kani harnesses)
//! is untouched. This file is ONLY glue.

use crate::engine::{
    ComparisonOp, EvalResult, SensorReading, Violation, Watchpoint,
    WatchpointTable, MAX_VIOLATIONS_PER_CYCLE, MAX_WATCHPOINTS,
};

/// Fixed-point scaling factor: multiply f64 by this to get i64.
const SCALE: f64 = 1000.0;

/// Convert f64 to i64 fixed-point. Saturates at i64 bounds.
fn f64_to_fixed(v: f64) -> i64 {
    let scaled = v * SCALE;
    if scaled >= i64::MAX as f64 {
        i64::MAX
    } else if scaled <= i64::MIN as f64 {
        i64::MIN
    } else {
        scaled as i64
    }
}

/// Convert i64 fixed-point back to f64.
fn fixed_to_f64(v: i64) -> f64 {
    v as f64 / SCALE
}

// ═══════════════════════════════════════════════════════════════
// cFS-compatible C types
// ═══════════════════════════════════════════════════════════════

/// cFS LC comparison operators — matches LC_OPER_* defines in lc_tbldefs.h
#[repr(C)]
pub enum RelayLcOper {
    LessThan = 1,
    LessOrEqual = 2,
    NotEqual = 3,
    GreaterThan = 4,
    GreaterOrEqual = 5,
    Equal = 6,
}

/// cFS LC watchpoint result — matches LC_WATCH_* defines
#[repr(C)]
pub enum RelayLcWatchResult {
    False = 0,
    True = 1,
    Error = 2,
    Stale = 3,
}

/// Result of evaluating one sensor reading.
#[repr(C)]
pub struct RelayLcEvalResult {
    /// Number of watchpoints that triggered
    pub violation_count: u32,
    /// Watchpoint IDs that triggered (up to MAX_VIOLATIONS_PER_CYCLE)
    pub violated_ids: [u32; MAX_VIOLATIONS_PER_CYCLE],
    /// Measured values for each violation (f64, converted back from fixed-point)
    pub measured_values: [f64; MAX_VIOLATIONS_PER_CYCLE],
    /// Thresholds for each violation
    pub thresholds: [f64; MAX_VIOLATIONS_PER_CYCLE],
}

// ═══════════════════════════════════════════════════════════════
// Global state — single WatchpointTable instance
// ═══════════════════════════════════════════════════════════════

use core::cell::UnsafeCell;

struct TableHolder(UnsafeCell<WatchpointTable>);
unsafe impl Sync for TableHolder {}

static TABLE: TableHolder = TableHolder(UnsafeCell::new(WatchpointTable::NEW));

fn get_table() -> &'static mut WatchpointTable {
    unsafe { &mut *TABLE.0.get() }
}

// ═══════════════════════════════════════════════════════════════
// C API — drop-in replacement for lc_watch.c functions
// ═══════════════════════════════════════════════════════════════

fn cfs_oper_to_comparison(oper: u32) -> Option<ComparisonOp> {
    match oper {
        1 => Some(ComparisonOp::LessThan),
        2 => Some(ComparisonOp::LessOrEqual),
        3 => Some(ComparisonOp::NotEqual),
        4 => Some(ComparisonOp::GreaterThan),
        5 => Some(ComparisonOp::GreaterOrEqual),
        6 => Some(ComparisonOp::Equal),
        _ => None,
    }
}

/// Initialize the verified limit checker. Call once at app startup.
/// Replaces: LC_AppInit() watchpoint table initialization.
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_init() -> i32 {
    *get_table() = WatchpointTable::new();
    0 // CFE_SUCCESS
}

/// Add a watchpoint. Replaces: loading a watchpoint from LC_WDT.
///
/// # Parameters
/// - `sensor_id`: Message ID + byte offset identifier for this data point
/// - `oper`: Comparison operator (1=LT, 2=LE, 3=NE, 4=GT, 5=GE, 6=EQ)
/// - `threshold`: Threshold value as f64 (converted to i64 fixed-point internally)
/// - `persistence`: Number of consecutive violations before triggering
///
/// # Returns
/// 0 on success, -1 if table full, -2 if invalid operator
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_add_watchpoint(
    sensor_id: u32,
    oper: u32,
    threshold: f64,
    persistence: u32,
) -> i32 {
    let op = match cfs_oper_to_comparison(oper) {
        Some(op) => op,
        None => return -2,
    };

    let wp = Watchpoint {
        sensor_id,
        op,
        threshold: f64_to_fixed(threshold),
        enabled: true,
        persistence: if persistence == 0 { 1 } else { persistence },
        current_count: 0,
    };

    if get_table().add_watchpoint(wp) {
        0
    } else {
        -1 // table full
    }
}

/// Evaluate a single sensor reading against all watchpoints.
/// Replaces: LC_WPOffsetValid + LC_GetSizedWPData + LC_OperatorCompare
///
/// This is the hot path. The verified engine guarantees:
///   - violation_count <= MAX_VIOLATIONS_PER_CYCLE (bounded output)
///   - violation_count <= watchpoint_count (can't exceed table)
///   - compare() is total for all operator values (no UB)
///   - persistence counter increments/resets correctly
///
/// # Parameters
/// - `sensor_id`: Which sensor this reading came from
/// - `value`: The measured value as f64
/// - `result`: Output struct filled with violation details
///
/// # Returns
/// Number of violations (0 = all watchpoints passed)
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_evaluate(
    sensor_id: u32,
    value: f64,
    result: *mut RelayLcEvalResult,
) -> u32 {
    let reading = SensorReading {
        sensor_id,
        value: f64_to_fixed(value),
    };

    let eval = get_table().evaluate(reading);

    if !result.is_null() {
        let out = unsafe { &mut *result };
        out.violation_count = eval.violation_count;
        for i in 0..eval.violation_count as usize {
            if i >= MAX_VIOLATIONS_PER_CYCLE {
                break;
            }
            out.violated_ids[i] = eval.violations[i].watchpoint_id;
            out.measured_values[i] = fixed_to_f64(eval.violations[i].measured);
            out.thresholds[i] = fixed_to_f64(eval.violations[i].threshold);
        }
    }

    eval.violation_count
}

/// Get current watchpoint count.
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_watchpoint_count() -> u32 {
    get_table().count()
}

/// Reset all watchpoints (clear table).
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_reset() -> i32 {
    *get_table() = WatchpointTable::new();
    0
}

/// Maximum watchpoints supported.
#[unsafe(no_mangle)]
pub extern "C" fn relay_lc_max_watchpoints() -> u32 {
    MAX_WATCHPOINTS as u32
}

// ═══════════════════════════════════════════════════════════════
// C header generation hint
// ═══════════════════════════════════════════════════════════════

// The corresponding C header (relay_lc.h) would be:
//
// #ifndef RELAY_LC_H
// #define RELAY_LC_H
//
// #include <stdint.h>
//
// #define RELAY_LC_MAX_VIOLATIONS 32
// #define RELAY_LC_MAX_WATCHPOINTS 128
//
// /* Operators — matches LC_OPER_* */
// #define RELAY_LC_LT 1
// #define RELAY_LC_LE 2
// #define RELAY_LC_NE 3
// #define RELAY_LC_GT 4
// #define RELAY_LC_GE 5
// #define RELAY_LC_EQ 6
//
// typedef struct {
//     uint32_t violation_count;
//     uint32_t violated_ids[RELAY_LC_MAX_VIOLATIONS];
//     double   measured_values[RELAY_LC_MAX_VIOLATIONS];
//     double   thresholds[RELAY_LC_MAX_VIOLATIONS];
// } relay_lc_eval_result_t;
//
// int32_t  relay_lc_init(void);
// int32_t  relay_lc_add_watchpoint(uint32_t sensor_id, uint32_t oper,
//                                   double threshold, uint32_t persistence);
// uint32_t relay_lc_evaluate(uint32_t sensor_id, double value,
//                            relay_lc_eval_result_t *result);
// uint32_t relay_lc_watchpoint_count(void);
// int32_t  relay_lc_reset(void);
// uint32_t relay_lc_max_watchpoints(void);
//
// #endif /* RELAY_LC_H */

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_c_api_init_and_count() {
        relay_lc_init();
        assert_eq!(relay_lc_watchpoint_count(), 0);
    }

    #[test]
    fn test_c_api_add_and_evaluate() {
        relay_lc_init();

        // Add watchpoint: sensor 1, value > 50.0
        assert_eq!(relay_lc_add_watchpoint(1, 4, 50.0, 1), 0); // GT

        // Evaluate: 100.0 > 50.0 → violation
        let mut result = RelayLcEvalResult {
            violation_count: 0,
            violated_ids: [0; MAX_VIOLATIONS_PER_CYCLE],
            measured_values: [0.0; MAX_VIOLATIONS_PER_CYCLE],
            thresholds: [0.0; MAX_VIOLATIONS_PER_CYCLE],
        };
        let count = relay_lc_evaluate(1, 100.0, &mut result);
        assert_eq!(count, 1);
        assert_eq!(result.violation_count, 1);
        assert!((result.measured_values[0] - 100.0).abs() < 0.01);
        assert!((result.thresholds[0] - 50.0).abs() < 0.01);

        // Evaluate: 30.0 < 50.0 → no violation
        let count = relay_lc_evaluate(1, 30.0, &mut result);
        assert_eq!(count, 0);
    }

    #[test]
    fn test_c_api_invalid_operator() {
        relay_lc_init();
        assert_eq!(relay_lc_add_watchpoint(1, 99, 50.0, 1), -2);
    }

    #[test]
    fn test_c_api_table_full() {
        relay_lc_init();
        for i in 0..MAX_WATCHPOINTS {
            assert_eq!(relay_lc_add_watchpoint(i as u32, 4, 50.0, 1), 0);
        }
        assert_eq!(relay_lc_add_watchpoint(999, 4, 50.0, 1), -1);
    }

    #[test]
    fn test_c_api_persistence() {
        relay_lc_init();
        // Persistence = 3: must exceed threshold 3 consecutive times
        relay_lc_add_watchpoint(1, 4, 50.0, 3);

        let mut result = RelayLcEvalResult {
            violation_count: 0,
            violated_ids: [0; MAX_VIOLATIONS_PER_CYCLE],
            measured_values: [0.0; MAX_VIOLATIONS_PER_CYCLE],
            thresholds: [0.0; MAX_VIOLATIONS_PER_CYCLE],
        };

        assert_eq!(relay_lc_evaluate(1, 100.0, &mut result), 0); // 1st
        assert_eq!(relay_lc_evaluate(1, 100.0, &mut result), 0); // 2nd
        assert_eq!(relay_lc_evaluate(1, 100.0, &mut result), 1); // 3rd → fires
    }

    #[test]
    fn test_c_api_f64_conversion_precision() {
        relay_lc_init();
        // Threshold: 3.14159
        relay_lc_add_watchpoint(1, 4, 3.14159, 1);

        let mut result = RelayLcEvalResult {
            violation_count: 0,
            violated_ids: [0; MAX_VIOLATIONS_PER_CYCLE],
            measured_values: [0.0; MAX_VIOLATIONS_PER_CYCLE],
            thresholds: [0.0; MAX_VIOLATIONS_PER_CYCLE],
        };

        // 3.15 > 3.14159 → violation
        assert_eq!(relay_lc_evaluate(1, 3.15, &mut result), 1);
        // Threshold should round-trip close to 3.141 (×1000 truncation)
        assert!((result.thresholds[0] - 3.141).abs() < 0.01);
    }

    #[test]
    fn test_c_api_reset() {
        relay_lc_init();
        relay_lc_add_watchpoint(1, 4, 50.0, 1);
        assert_eq!(relay_lc_watchpoint_count(), 1);
        relay_lc_reset();
        assert_eq!(relay_lc_watchpoint_count(), 0);
    }
}
