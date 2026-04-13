// Relay Limit Checker — P3 WASM component (self-contained).
// Verified core engine + P3 async Guest trait.

mod engine {
    pub const MAX_WATCHPOINTS: usize = 128;
    pub const MAX_VIOLATIONS_PER_CYCLE: usize = 32;

    #[derive(Clone, Copy, PartialEq, Eq)]
    #[repr(u8)]
    pub enum ComparisonOp {
        LessThan = 0, GreaterThan = 1, LessOrEqual = 2,
        GreaterOrEqual = 3, Equal = 4, NotEqual = 5,
    }

    #[derive(Clone, Copy)]
    pub struct Watchpoint {
        pub sensor_id: u32, pub op: ComparisonOp, pub threshold: i64,
        pub enabled: bool, pub persistence: u32, pub current_count: u32,
    }

    #[derive(Clone, Copy)]
    pub struct Violation {
        pub watchpoint_id: u32, pub measured: i64, pub threshold: i64, pub op: ComparisonOp,
    }

    #[derive(Clone, Copy)]
    pub struct SensorReading { pub sensor_id: u32, pub value: i64 }

    pub struct CheckResult {
        pub violations: [Violation; MAX_VIOLATIONS_PER_CYCLE],
        pub violation_count: u32,
    }

    pub struct WatchpointTable {
        entries: [Watchpoint; MAX_WATCHPOINTS],
        entry_count: u32,
    }

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

    impl Watchpoint {
        pub const fn empty() -> Self {
            Watchpoint {
                sensor_id: 0, op: ComparisonOp::LessThan, threshold: 0,
                enabled: false, persistence: 1, current_count: 0,
            }
        }
    }

    impl Violation {
        pub const fn empty() -> Self {
            Violation { watchpoint_id: 0, measured: 0, threshold: 0, op: ComparisonOp::LessThan }
        }
    }

    impl WatchpointTable {
        pub fn new() -> Self {
            WatchpointTable { entries: [Watchpoint::empty(); MAX_WATCHPOINTS], entry_count: 0 }
        }

        pub fn add_watchpoint(&mut self, wp: Watchpoint) -> bool {
            if self.entry_count as usize >= MAX_WATCHPOINTS { return false; }
            self.entries[self.entry_count as usize] = wp;
            self.entry_count = self.entry_count + 1;
            true
        }

        pub fn count(&self) -> u32 { self.entry_count }

        pub fn check(&mut self, reading: SensorReading) -> CheckResult {
            let mut result = CheckResult {
                violations: [Violation::empty(); MAX_VIOLATIONS_PER_CYCLE],
                violation_count: 0,
            };
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
                        let cc = self.entries[idx].current_count;
                        self.entries[idx].current_count = if cc < u32::MAX { cc + 1 } else { u32::MAX };
                        if self.entries[idx].current_count >= persistence {
                            let vidx = result.violation_count as usize;
                            result.violations[vidx] = Violation {
                                watchpoint_id: i, measured: reading.value, threshold, op,
                            };
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
}

// P3 WASM component binding

use relay_lc_bindings::exports::pulseengine::relay_limit_checker::limit_checker::{
    Guest, ComparisonOp as WitOp, SensorReading as WitReading,
    Violation as WitViolation, Watchpoint as WitWp,
};

struct Component;
static mut TABLE: Option<engine::WatchpointTable> = None;

fn get_table() -> &'static mut engine::WatchpointTable {
    unsafe {
        if TABLE.is_none() {
            TABLE = Some(engine::WatchpointTable::new());
        }
        TABLE.as_mut().unwrap()
    }
}

fn wit_to_op(op: WitOp) -> engine::ComparisonOp {
    match op {
        WitOp::LessThan => engine::ComparisonOp::LessThan,
        WitOp::GreaterThan => engine::ComparisonOp::GreaterThan,
        WitOp::LessOrEqual => engine::ComparisonOp::LessOrEqual,
        WitOp::GreaterOrEqual => engine::ComparisonOp::GreaterOrEqual,
        WitOp::Equal => engine::ComparisonOp::Equal,
        WitOp::NotEqual => engine::ComparisonOp::NotEqual,
    }
}

impl Guest for Component {
    #[cfg(target_arch = "wasm32")]
    async fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::WatchpointTable::new()); }
        Ok(())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::WatchpointTable::new()); }
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    async fn add_watchpoint(wp: WitWp) -> bool { Self::do_add(wp) }
    #[cfg(not(target_arch = "wasm32"))]
    fn add_watchpoint(wp: WitWp) -> bool { Self::do_add(wp) }

    #[cfg(target_arch = "wasm32")]
    async fn evaluate(reading: WitReading) -> Vec<WitViolation> { Self::do_check(reading) }
    #[cfg(not(target_arch = "wasm32"))]
    fn evaluate(reading: WitReading) -> Vec<WitViolation> { Self::do_check(reading) }

    #[cfg(target_arch = "wasm32")]
    async fn count() -> u32 { get_table().count() }
    #[cfg(not(target_arch = "wasm32"))]
    fn count() -> u32 { get_table().count() }
}

impl Component {
    fn do_add(wp: WitWp) -> bool {
        get_table().add_watchpoint(engine::Watchpoint {
            sensor_id: wp.sensor_id,
            op: wit_to_op(wp.op),
            threshold: wp.threshold,
            enabled: wp.enabled,
            persistence: wp.persistence,
            current_count: 0,
        })
    }

    fn do_check(reading: WitReading) -> Vec<WitViolation> {
        let result = get_table().check(engine::SensorReading {
            sensor_id: reading.sensor_id,
            value: reading.value,
        });
        let mut v = Vec::with_capacity(result.violation_count as usize);
        for i in 0..result.violation_count as usize {
            v.push(WitViolation {
                watchpoint_id: result.violations[i].watchpoint_id,
                measured: result.violations[i].measured,
                threshold: result.violations[i].threshold,
            });
        }
        v
    }
}

relay_lc_bindings::export!(Component with_types_in relay_lc_bindings);
