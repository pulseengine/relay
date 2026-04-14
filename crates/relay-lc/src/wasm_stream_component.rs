// Relay Limit Checker — P3 Stream Transformer WASM component.
//
// Takes stream<sensor-reading>, emits stream<violation>.
// The verified engine (8 Verus properties, Z3 proven) processes each reading.
//
// This is the stream-native P3 interface — the future of Relay.

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
        pub entries: [Watchpoint; MAX_WATCHPOINTS],
        pub entry_count: u32,
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
            Watchpoint { sensor_id: 0, op: ComparisonOp::LessThan, threshold: 0, enabled: false, persistence: 1, current_count: 0 }
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
            self.entry_count += 1;
            true
        }
        pub fn evaluate(&mut self, reading: SensorReading) -> CheckResult {
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
                            result.violations[vidx] = Violation { watchpoint_id: i, measured: reading.value, threshold, op };
                            result.violation_count += 1;
                        }
                    } else {
                        self.entries[idx].current_count = 0;
                    }
                }
                i += 1;
            }
            result
        }
    }
}

// P3 Stream binding

use relay_lc_stream_bindings::exports::pulseengine::relay_limit_checker_stream::limit_checker_stream::{
    Guest, ComparisonOp as WitOp, SensorReading as WitReading,
    Violation as WitViolation, Watchpoint as WitWp,
};

struct Component;

static mut TABLE: Option<engine::WatchpointTable> = None;

fn get_table() -> &'static mut engine::WatchpointTable {
    unsafe {
        if TABLE.is_none() { TABLE = Some(engine::WatchpointTable::new()); }
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
    async fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::WatchpointTable::new()); }
        Ok(())
    }

    async fn add_watchpoint(wp: WitWp) -> bool {
        get_table().add_watchpoint(engine::Watchpoint {
            sensor_id: wp.sensor_id, op: wit_to_op(wp.op), threshold: wp.threshold,
            enabled: wp.enabled, persistence: wp.persistence, current_count: 0,
        })
    }

    /// STREAM TRANSFORMER: reads from input stream, evaluates each reading
    /// against watchpoints, writes violations to output stream.
    async fn monitor(
        mut readings: wit_bindgen::rt::async_support::StreamReader<WitReading>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitViolation> {
        let (mut writer, reader) = relay_lc_stream_bindings::wit_stream::new::<WitViolation>();

        // Process readings inline — monitor is already async
        while let Some(reading) = readings.next().await {
            let result = get_table().evaluate(engine::SensorReading {
                sensor_id: reading.sensor_id,
                value: reading.value,
            });
            let mut violations = Vec::new();
            for i in 0..result.violation_count as usize {
                let v = &result.violations[i];
                violations.push(WitViolation {
                    watchpoint_id: v.watchpoint_id,
                    measured: v.measured,
                    threshold: v.threshold,
                });
            }
            if !violations.is_empty() {
                let _ = writer.write(violations).await;
            }
        }

        reader
    }
}

relay_lc_stream_bindings::export!(Component with_types_in relay_lc_stream_bindings);
