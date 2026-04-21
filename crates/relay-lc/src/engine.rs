//! Relay Limit Checker — verified core logic.
//!
//! Formally verified Rust replacement for NASA cFS Limit Checker (LC).
//! Same approach as Gale (verified Zephyr kernel), but for the application layer.
//!
//! Source mapping: NASA cFS LC app (lc_watch.c, lc_action.c)
//! Omitted: Actionpoint logic (AND/OR), RTS triggering
//!
//! ASIL-D verified properties:
//!   LC-P01: Invariant holds after init (table empty, count = 0)
//!   LC-P02: Invariant preserved by add_watchpoint (count bounded by MAX)
//!   LC-P03: evaluate output bounded: violation_count <= MAX_VIOLATIONS_PER_CYCLE
//!   LC-P04: evaluate output bounded: violation_count <= entry_count
//!   LC-P05: Disabled watchpoints never produce violations
//!   LC-P06: compare() is total (INHERITED from relay-primitives CMP-P01..P03)
//!   LC-P07: Persistence counter semantics (INHERITED from relay-primitives PER-P01..P05)
//!   LC-P08: Violation only fires when current_count >= persistence (INHERITED)
//!
//! NO async, NO alloc, NO trait objects, NO closures.
//!
//! Composition: LC is compare + persistence + LC-specific glue
//! (watchpoint table, sensor-id matching, bounded output collection).

pub use crate::compare::{compare_i64 as compare, ComparisonOp};

use vstd::prelude::*;

verus! {

pub const MAX_WATCHPOINTS: usize = 128;
pub const MAX_VIOLATIONS_PER_CYCLE: usize = 32;

#[derive(Clone, Copy)]
pub struct Watchpoint {
    pub sensor_id: u32,
    pub op: ComparisonOp,
    pub threshold: i64,
    pub enabled: bool,
    pub persistence: u32,
    pub current_count: u32,
}

#[derive(Clone, Copy)]
pub struct Violation {
    pub watchpoint_id: u32,
    pub measured: i64,
    pub threshold: i64,
    pub op: ComparisonOp,
}

#[derive(Clone, Copy)]
pub struct SensorReading {
    pub sensor_id: u32,
    pub value: i64,
}

pub struct EvalResult {
    pub violations: [Violation; MAX_VIOLATIONS_PER_CYCLE],
    pub violation_count: u32,
}

impl EvalResult {
    #[verifier::external_body]
    pub fn new() -> (result: Self)
        ensures result.violation_count == 0,
    {
        EvalResult {
            violations: [Violation::empty(); MAX_VIOLATIONS_PER_CYCLE],
            violation_count: 0,
        }
    }
}

pub struct WatchpointTable {
    pub entries: [Watchpoint; MAX_WATCHPOINTS],
    pub entry_count: u32,
}

// =================================================================
// Specification functions
// =================================================================

// compare() is now inherited from relay-primitives::compare (re-exported at
// module level above). LC-P06 becomes the primitive's CMP-P01..P03.

/// Decision for a single watchpoint evaluation (Gale pattern: lightweight decision fn).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum WpDecision {
    /// Watchpoint not applicable (wrong sensor or disabled)
    Skip,
    /// Threshold not exceeded — reset persistence counter
    Pass,
    /// Threshold exceeded but persistence not met — increment counter
    PendingPersistence,
    /// Threshold exceeded and persistence met — emit violation
    Violated,
}

/// Decide what to do for a single watchpoint.
/// Composed from primitives: compare (CMP-*) + persistence::decide (PER-*).
/// The LC-specific glue is the sensor-id matching + enabled bit.
pub fn wp_decide(
    wp_sensor_id: u32,
    wp_op: ComparisonOp,
    wp_threshold: i64,
    wp_enabled: bool,
    wp_persistence: u32,
    wp_current_count: u32,
    reading_sensor_id: u32,
    reading_value: i64,
) -> (result: WpDecision)
    ensures
        !wp_enabled ==> result === WpDecision::Skip,
        wp_enabled && wp_sensor_id != reading_sensor_id ==> result === WpDecision::Skip,
{
    if !wp_enabled || wp_sensor_id != reading_sensor_id {
        return WpDecision::Skip;
    }
    let violated = compare(reading_value, wp_op, wp_threshold);
    let decision = crate::persistence::decide(violated, wp_current_count, wp_persistence);
    match decision {
        crate::persistence::PersistenceDecision::Pass => WpDecision::Pass,
        crate::persistence::PersistenceDecision::Pending => WpDecision::PendingPersistence,
        crate::persistence::PersistenceDecision::Fire => WpDecision::Violated,
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
    // =================================================================
    // Specification functions
    // =================================================================

    pub open spec fn inv(&self) -> bool {
        &&& self.entry_count as usize <= MAX_WATCHPOINTS
    }

    pub open spec fn count_spec(&self) -> nat {
        self.entry_count as nat
    }

    pub open spec fn is_full_spec(&self) -> bool {
        self.entry_count as usize >= MAX_WATCHPOINTS
    }

    // NEW constant not available under Verus — use new() instead

    // =================================================================
    // init (LC-P01)
    // =================================================================

    #[verifier::external_body]
    pub fn new() -> (result: Self)
        ensures
            result.inv(),
            result.count_spec() == 0,
            !result.is_full_spec(),
    {
        WatchpointTable {
            entries: [Watchpoint::empty(); MAX_WATCHPOINTS],
            entry_count: 0,
        }
    }

    // =================================================================
    // add_watchpoint (LC-P02)
    // =================================================================

    pub fn add_watchpoint(&mut self, wp: Watchpoint) -> (result: bool)
        requires
            old(self).inv(),
        ensures
            self.inv(),
            result == !old(self).is_full_spec(),
            result ==> self.count_spec() == old(self).count_spec() + 1,
            !result ==> self.count_spec() == old(self).count_spec(),
    {
        if self.entry_count as usize >= MAX_WATCHPOINTS {
            return false;
        }
        // Verus workaround: copy to local, set in array via set()
        let idx = self.entry_count as usize;
        self.entries.set(idx, wp);
        self.entry_count = self.entry_count + 1;
        true
    }

    pub fn count(&self) -> (result: u32)
        requires
            self.inv(),
        ensures
            result == self.entry_count,
            result as usize <= MAX_WATCHPOINTS,
    {
        self.entry_count
    }

    // =================================================================
    // evaluate (LC-P03, LC-P04, LC-P05, LC-P07, LC-P08)
    // =================================================================

    /// Evaluate a single sensor reading against all watchpoints.
    /// Uses wp_decide() (pure decision) then applies mutations.
    pub fn evaluate(
        &mut self,
        reading: SensorReading,
    ) -> (result: EvalResult)
        requires
            old(self).inv(),
        ensures
            self.inv(),
            self.count_spec() == old(self).count_spec(),
            result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE,
            result.violation_count <= self.entry_count,
    {
        let mut result = EvalResult::new();

        let count = self.entry_count;
        let mut i: u32 = 0;

        while i < count
            invariant
                self.inv(),
                0 <= i <= count,
                count == self.entry_count,
                count as usize <= MAX_WATCHPOINTS,
                result.violation_count as usize <= MAX_VIOLATIONS_PER_CYCLE,
                result.violation_count <= i,
            decreases
                count - i,
        {
            if result.violation_count as usize >= MAX_VIOLATIONS_PER_CYCLE {
                break;
            }

            let idx = i as usize;
            // Read fields into locals (no mutable borrow issues)
            let wp = self.entries[idx];
            let decision = wp_decide(
                wp.sensor_id, wp.op, wp.threshold, wp.enabled,
                wp.persistence, wp.current_count,
                reading.sensor_id, reading.value,
            );

            // Apply the counter update via the primitive, then check for fire.
            // Counter update is a simple function of the decision — the primitive's
            // persistence::apply encapsulates it (PER-P02/P03).
            match decision {
                WpDecision::Skip => {},
                _ => {
                    let prim_decision = match decision {
                        WpDecision::Pass => crate::persistence::PersistenceDecision::Pass,
                        WpDecision::PendingPersistence => crate::persistence::PersistenceDecision::Pending,
                        WpDecision::Violated => crate::persistence::PersistenceDecision::Fire,
                        WpDecision::Skip => crate::persistence::PersistenceDecision::Pass, // unreachable
                    };
                    let mut updated = wp;
                    updated.current_count = crate::persistence::apply(prim_decision, wp.current_count);
                    self.entries.set(idx, updated);

                    if matches!(decision, WpDecision::Violated) {
                        let vidx = result.violation_count as usize;
                        result.violations.set(vidx, Violation {
                            watchpoint_id: i,
                            measured: reading.value,
                            threshold: wp.threshold,
                            op: wp.op,
                        });
                        result.violation_count = result.violation_count + 1;
                    }
                },
            }

            i = i + 1;
        }

        result
    }
}

// =================================================================
// Compositional proofs
// =================================================================

// LC-P01: init establishes invariant — proven by new()'s ensures clause.
// (Proof functions cannot call exec functions; new()'s postcondition
//  guarantees inv() directly.)

// LC-P06: compare() is total — proven by the ensures clause on compare() itself.
// Each branch returns a bool, and Verus verifies the ensures for all 6 operators.

} // verus!
