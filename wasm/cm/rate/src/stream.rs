// Falcon Rate Controller — P3 Stream Transformer (shared cascade-stream types).
//
// stream<rate-input> -> stream<torque-setpoint>, where rate-input is
// { state: vehicle-state, sp: rate-setpoint } — shared types so the cascade
// composes (STREAM-P10). Drives the verified relay-rate body-rate PID.

use relay_rate::{RatePid, Timestamp};

use falcon_rate_stream_bindings::exports::falcon::cascade_stream::rate_stream::{
    Guest, RateInput as WitInput, TorqueSetpoint as WitTorque,
};

struct Component;

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
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitTorque> {
        let (mut writer, reader) =
            falcon_rate_stream_bindings::wit_stream::new::<WitTorque>();

        while let Some(inp) = inputs.next().await {
            // Body rates from the estimated state; setpoint from the rate loop.
            let torque = pid().tick(
                next_timestamp(),
                [inp.state.wx, inp.state.wy, inp.state.wz],
                [inp.sp.rx, inp.sp.ry, inp.sp.rz],
            );
            let out = WitTorque {
                tx: torque[0],
                ty: torque[1],
                tz: torque[2],
                thrust: inp.sp.thrust,
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_rate_stream_bindings::export!(Component with_types_in falcon_rate_stream_bindings);
