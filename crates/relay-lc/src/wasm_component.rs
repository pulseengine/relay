// Relay Limit Checker — P3 WASM component (self-contained).
// Verified core engine + P3 async Guest trait.

// Run the REAL Verus+Kani-verified engine (#168), not an inline copy.
use relay_lc::engine;

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
