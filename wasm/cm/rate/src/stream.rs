// Falcon Rate Controller — P3 Stream Transformer WASM component.
//
// Takes stream<rate-input>, emits stream<torque-setpoint>.
// Wraps the verified relay-rate body-rate PID (RatePid) — the first
// control-cascade engine promoted to a P3 async stream (#168 cascade arc).
//
// Built as a rust_wasm_component_bindgen (wasi_version="p3"), the same proven
// pattern as the cFS stream components — deps on the verified :relay-rate
// Bazel library, NOT an inline copy of the controller.

use relay_rate::{RatePid, Timestamp};

use falcon_rate_stream_bindings::exports::falcon::rate_stream::rate_stream::{
    Guest, RateInput as WitInput, TorqueSetpoint as WitTorque,
};

struct Component;

// Stateful across the stream: the PID integrator/derivative state persists,
// and the timestamp is a synthesised 1 kHz counter (as in the sync component).
static mut PID: Option<RatePid> = None;
static mut TICK_MS: u64 = 0;

fn pid() -> &'static mut RatePid {
    unsafe {
        if PID.is_none() { PID = Some(RatePid::new()); }
        PID.as_mut().unwrap()
    }
}

fn next_timestamp() -> Timestamp {
    unsafe {
        let ms = TICK_MS;
        TICK_MS += 1;
        Timestamp {
            seconds: ms / 1000,
            fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32,
        }
    }
}

impl Guest for Component {
    /// STREAM TRANSFORMER: reads rate-loop inputs, steps the verified PID on
    /// each, writes the torque setpoint to the output stream.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitTorque> {
        let (mut writer, reader) = falcon_rate_stream_bindings::wit_stream::new::<WitTorque>();

        while let Some(inp) = inputs.next().await {
            let torque = pid().tick(
                next_timestamp(),
                [inp.wx, inp.wy, inp.wz],
                [inp.rx, inp.ry, inp.rz],
            );
            let out = WitTorque {
                tx: torque[0],
                ty: torque[1],
                tz: torque[2],
                thrust: inp.thrust, // thrust passes straight through the rate loop
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_rate_stream_bindings::export!(Component with_types_in falcon_rate_stream_bindings);
