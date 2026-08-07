// Falcon Attitude Controller — P3 Stream Transformer (shared cascade-stream types).
//
// stream<att-input> -> stream<rate-setpoint>, where att-input is
// { state: vehicle-state, sp: attitude-setpoint } — shared types so the cascade
// composes (STREAM-P10). Drives the verified relay-att AttController.

use relay_att::{AttController, Timestamp};

use falcon_attitude_stream_bindings::exports::pulseengine::falcon_cascade_stream::attitude_stream::{
    AttInput as WitInput, Guest, RateSetpoint as WitRate,
};

struct Component;

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
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitRate> {
        let (mut writer, reader) =
            falcon_attitude_stream_bindings::wit_stream::new::<WitRate>();

        while let Some(inp) = inputs.next().await {
            // Attitude estimate from the state; setpoint quaternion from upstream.
            let rate = att().tick(
                next_timestamp(),
                [inp.state.qw, inp.state.qx, inp.state.qy, inp.state.qz],
                [inp.sp.qw, inp.sp.qx, inp.sp.qy, inp.sp.qz],
            );
            let out = WitRate {
                rx: rate[0],
                ry: rate[1],
                rz: rate[2],
                thrust: inp.sp.thrust,
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_attitude_stream_bindings::export!(Component with_types_in falcon_attitude_stream_bindings);
