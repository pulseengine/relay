// Relay Limit Checker — P3 Stream Transformer WASM component.
//
// Takes stream<sensor-reading>, emits stream<violation>.
// The verified engine (8 Verus properties, Z3 proven) processes each reading.
//
// This is the stream-native P3 interface — the future of Relay.

// Run the REAL Verus+Kani-verified engine (#168) — not an inline copy.
use relay_lc::engine;

// P3 Stream binding

use relay_lc_stream_bindings::exports::pulseengine::relay_limit_checker::limit_checker_stream::{
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
