// Falcon Attitude Controller — P3 Stream Transformer WASM component.
//
// Takes stream<att-input>, emits stream<rate-setpoint>.
// Wraps the verified relay-att AttController — the third control-cascade
// engine promoted to a P3 async stream (#168 cascade arc).
//
// Built as a rust_wasm_component_bindgen (wasi_version="p3"), the proven
// cFS-stream pattern, over the verified :relay-att Bazel library.

use relay_att::{AttController, Timestamp};

use falcon_attitude_stream_bindings::exports::falcon::attitude_stream::attitude_stream::{
    AttInput as WitInput, Guest, RateSetpoint as WitRate,
};

struct Component;

// Stateful across the stream: the controller state persists, and the timestamp
// is a synthesised 1 kHz counter (as in the sync component).
static mut ATT: Option<AttController> = None;
static mut TICK_MS: u64 = 0;

fn att() -> &'static mut AttController {
    unsafe {
        if ATT.is_none() { ATT = Some(AttController::new()); }
        ATT.as_mut().unwrap()
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
    /// STREAM TRANSFORMER: reads attitude-loop inputs, steps the verified
    /// attitude controller on each, writes the body-rate setpoint to the output.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitRate> {
        let (mut writer, reader) =
            falcon_attitude_stream_bindings::wit_stream::new::<WitRate>();

        while let Some(inp) = inputs.next().await {
            let rate = att().tick(
                next_timestamp(),
                [inp.qw, inp.qx, inp.qy, inp.qz],
                [inp.sp_qw, inp.sp_qx, inp.sp_qy, inp.sp_qz],
            );
            let out = WitRate {
                rx: rate[0],
                ry: rate[1],
                rz: rate[2],
                thrust: inp.thrust, // thrust passes straight through the attitude loop
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_attitude_stream_bindings::export!(Component with_types_in falcon_attitude_stream_bindings);
