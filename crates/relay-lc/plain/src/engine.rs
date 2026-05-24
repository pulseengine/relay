//! Relay Limit Checker — plain Rust (Verus-stripped mirror of ../src/engine.rs).
//!
//! LC = compare (from relay-primitives) + persistence (from relay-primitives)
//!      + LC-specific glue (watchpoint table, sensor-id match, bounded output).
//! Source of truth: ../src/engine.rs.

pub use crate::compare::{compare_i64 as compare, ComparisonOp};

pub const MAX_WATCHPOINTS: usize = 128;
pub const MAX_VIOLATIONS_PER_CYCLE: usize = 32;

#[derive(Clone, Copy, Debug)]
pub struct Watchpoint { pub sensor_id: u32, pub op: ComparisonOp, pub threshold: i64, pub enabled: bool, pub persistence: u32, pub current_count: u32 }

#[derive(Clone, Copy, Debug)]
pub struct Violation { pub watchpoint_id: u32, pub measured: i64, pub threshold: i64, pub op: ComparisonOp }

#[derive(Clone, Copy, Debug)]
pub struct SensorReading { pub sensor_id: u32, pub value: i64 }

pub struct EvalResult { pub violations: [Violation; MAX_VIOLATIONS_PER_CYCLE], pub violation_count: u32 }

pub struct WatchpointTable { entries: [Watchpoint; MAX_WATCHPOINTS], entry_count: u32 }

impl Watchpoint { pub const fn empty() -> Self { Watchpoint { sensor_id: 0, op: ComparisonOp::LessThan, threshold: 0, enabled: false, persistence: 1, current_count: 0 } } }
impl Violation { pub const fn empty() -> Self { Violation { watchpoint_id: 0, measured: 0, threshold: 0, op: ComparisonOp::LessThan } } }

impl WatchpointTable {
    pub const NEW: Self = WatchpointTable { entries: [Watchpoint::empty(); MAX_WATCHPOINTS], entry_count: 0 };
    pub fn new() -> Self { Self::NEW }

    pub fn add_watchpoint(&mut self, wp: Watchpoint) -> bool {
        if self.entry_count as usize >= MAX_WATCHPOINTS { return false; }
        self.entries[self.entry_count as usize] = wp;
        self.entry_count += 1;
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
            let wp = self.entries[idx];
            if wp.enabled && wp.sensor_id == reading.sensor_id {
                // Composition of verified primitives: compare → persistence::decide → persistence::apply.
                let violated = compare(reading.value, wp.op, wp.threshold);
                let decision = crate::persistence::decide(violated, wp.current_count, wp.persistence);
                self.entries[idx].current_count = crate::persistence::apply(decision, wp.current_count);
                if decision == crate::persistence::PersistenceDecision::Fire {
                    let vidx = result.violation_count as usize;
                    result.violations[vidx] = Violation { watchpoint_id: i, measured: reading.value, threshold: wp.threshold, op: wp.op };
                    result.violation_count += 1;
                }
            }
            i += 1;
        }
        result
    }
}

// =================================================================
// Geofence (LC-P09, LC-P10): position-bounds violation latch (v0.10)
// =================================================================
//
// Verus-stripped from ../src/engine.rs.
//   LC-P09: violation_latched is monotone (once true, always true).
//   LC-P10: check() returns true only on the violation transition.
//
// Position is in centimetres (i32) so the engine stays pure integer
// — the SITL converts from f32 metres at the boundary.

pub struct Geofence {
    pub min_n: i32,
    pub max_n: i32,
    pub min_e: i32,
    pub max_e: i32,
    pub min_d: i32,
    pub max_d: i32,
    pub violation_latched: bool,
}

impl Geofence {
    pub fn new(
        min_n: i32,
        max_n: i32,
        min_e: i32,
        max_e: i32,
        min_d: i32,
        max_d: i32,
    ) -> Self {
        Geofence {
            min_n,
            max_n,
            min_e,
            max_e,
            min_d,
            max_d,
            violation_latched: false,
        }
    }

    /// Feed one *true* position sample (cm, NED). Returns `true`
    /// only on the tick the latch trips.
    pub fn check(&mut self, n: i32, e: i32, d: i32) -> bool {
        if self.violation_latched {
            return false;
        }
        let inside = n >= self.min_n
            && n <= self.max_n
            && e >= self.min_e
            && e <= self.max_e
            && d >= self.min_d
            && d <= self.max_d;
        if !inside {
            self.violation_latched = true;
            return true;
        }
        false
    }

    pub fn violation_active(&self) -> bool {
        self.violation_latched
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

    // --- Geofence unit tests (v0.12) — give miri something concrete
    // to interpret. The exhaustive arbitrary-input coverage lives in
    // the kani_proofs module below; these are the deterministic spot
    // checks miri walks.
    fn fence() -> Geofence {
        Geofence::new(-1_000, 1_000, -1_000, 1_000, -1_000, 1_000)
    }

    #[test] fn geofence_inside_does_not_trip() {
        let mut g = fence();
        assert!(!g.check(0, 0, 0));
        assert!(!g.violation_active());
    }

    #[test] fn geofence_outside_n_trips_once() {
        let mut g = fence();
        assert!(g.check(2_000, 0, 0));   // rising edge
        assert!(g.violation_active());
        assert!(!g.check(3_000, 0, 0));  // already latched — silent
        assert!(!g.check(0, 0, 0));      // even returning inside — still silent
    }

    #[test] fn geofence_outside_e_trips() {
        let mut g = fence();
        assert!(g.check(0, -2_000, 0));
        assert!(g.violation_active());
    }

    #[test] fn geofence_outside_d_trips() {
        let mut g = fence();
        assert!(g.check(0, 0, 2_000));
        assert!(g.violation_active());
    }

    #[test] fn geofence_boundary_inclusive() {
        // Exact boundary values are inside per >= / <= in check().
        let mut g = fence();
        assert!(!g.check(1_000, 1_000, 1_000));
        assert!(!g.check(-1_000, -1_000, -1_000));
    }
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
            for _ in 0..persistence-1 {
                let r = table.evaluate(SensorReading { sensor_id: 1, value });
                prop_assert_eq!(r.violation_count, 0);
            }
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

    /// LC-P06 (inherited from CMP-P01): compare is total
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

    /// LC-P03 (inherited from CMP-P03): compare matches operator semantics
    #[kani::proof]
    fn verify_compare_semantics() {
        let v: i64 = kani::any();
        let t: i64 = kani::any();
        assert_eq!(compare(v, ComparisonOp::LessThan, t), v < t);
        assert_eq!(compare(v, ComparisonOp::GreaterThan, t), v > t);
        assert_eq!(compare(v, ComparisonOp::Equal, t), v == t);
    }

    // -------------------------------------------------------------
    // Geofence harnesses (mirror EkfHealthMonitor pattern from
    // crates/relay-hs/plain/src/engine.rs).
    //
    // Geofence::check is pure i32 — Kani can enumerate arbitrary
    // (n, e, d) over the full domain without external_body gaps.
    // -------------------------------------------------------------

    fn arb_fence() -> Geofence {
        let min_n: i32 = kani::any();
        let max_n: i32 = kani::any();
        let min_e: i32 = kani::any();
        let max_e: i32 = kani::any();
        let min_d: i32 = kani::any();
        let max_d: i32 = kani::any();
        // Well-formed bounds: avoid degenerate "min > max" worlds where
        // every point is outside; the property still holds there but the
        // counter-examples drown signal.
        kani::assume(min_n <= max_n);
        kani::assume(min_e <= max_e);
        kani::assume(min_d <= max_d);
        Geofence::new(min_n, max_n, min_e, max_e, min_d, max_d)
    }

    /// LC-K01 (mirrors HS-P06): once `violation_latched`, always `violation_latched`.
    #[kani::proof]
    fn geofence_latch_monotone() {
        let mut g = arb_fence();
        let n: i32 = kani::any();
        let e: i32 = kani::any();
        let d: i32 = kani::any();
        let pre = g.violation_latched;
        let _ = g.check(n, e, d);
        if pre {
            assert!(g.violation_latched);
        }
    }

    /// LC-K02 (mirrors HS-P07): `check()` returns `true` only on the
    /// rising edge — i.e. only when latch was off before and is on after.
    #[kani::proof]
    fn geofence_check_transition_only() {
        let mut g = arb_fence();
        let n: i32 = kani::any();
        let e: i32 = kani::any();
        let d: i32 = kani::any();
        let pre = g.violation_latched;
        let r = g.check(n, e, d);
        if r {
            assert!(!pre);
            assert!(g.violation_latched);
        }
    }

    /// LC-K03: an already-latched fence never re-fires.
    #[kani::proof]
    fn geofence_already_latched_silent() {
        let mut g = arb_fence();
        g.violation_latched = true;
        let n: i32 = kani::any();
        let e: i32 = kani::any();
        let d: i32 = kani::any();
        let r = g.check(n, e, d);
        assert!(!r);
        assert!(g.violation_latched);
    }

    /// LC-K04: a fresh fence with a position strictly inside bounds
    /// must not trip. Encodes the "no false positive in the safe box"
    /// guarantee that protects the SC RTL command from spurious fires.
    #[kani::proof]
    fn geofence_inside_never_trips() {
        let mut g = arb_fence();
        let n: i32 = kani::any();
        let e: i32 = kani::any();
        let d: i32 = kani::any();
        kani::assume(n >= g.min_n && n <= g.max_n);
        kani::assume(e >= g.min_e && e <= g.max_e);
        kani::assume(d >= g.min_d && d <= g.max_d);
        let r = g.check(n, e, d);
        assert!(!r);
        assert!(!g.violation_latched);
    }

    /// LC-K05: a fresh fence with a position strictly outside any axis
    /// always trips on the first call. The "no false negative" complement
    /// of LC-K04 — together they pin `check` to the exact spec.
    #[kani::proof]
    fn geofence_outside_always_trips() {
        let mut g = arb_fence();
        // Constrain ranges so "outside" exists on every axis without overflow.
        kani::assume(g.min_n > i32::MIN && g.max_n < i32::MAX);
        kani::assume(g.min_e > i32::MIN && g.max_e < i32::MAX);
        kani::assume(g.min_d > i32::MIN && g.max_d < i32::MAX);
        let n: i32 = kani::any();
        let e: i32 = kani::any();
        let d: i32 = kani::any();
        let outside = n < g.min_n || n > g.max_n
                   || e < g.min_e || e > g.max_e
                   || d < g.min_d || d > g.max_d;
        kani::assume(outside);
        let r = g.check(n, e, d);
        assert!(r);
        assert!(g.violation_latched);
    }
}
