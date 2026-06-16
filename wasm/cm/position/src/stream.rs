// Falcon Position Controller — P3 Stream Transformer WASM component.
//
// Takes stream<pos-input>, emits stream<attitude-setpoint>.
// Wraps the verified relay-pos PosController — the second control-cascade
// engine promoted to a P3 async stream (#168 cascade arc).
//
// Built as a rust_wasm_component_bindgen (wasi_version="p3"), the proven
// cFS-stream pattern, over the verified :relay-pos Bazel library.

use relay_pos::{PosController, PositionSetpoint, Timestamp};

use falcon_position_stream_bindings::exports::falcon::position_stream::position_stream::{
    AttitudeSetpoint as WitAtt, Guest, PosInput as WitInput,
};

struct Component;

// Stateful across the stream: the controller integrator state persists, and the
// timestamp is a synthesised 1 kHz counter (as in the sync component).
static mut POS: Option<PosController> = None;
static mut TICK_MS: u64 = 0;

fn pos() -> &'static mut PosController {
    unsafe {
        if POS.is_none() { POS = Some(PosController::new()); }
        POS.as_mut().unwrap()
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
    /// STREAM TRANSFORMER: reads position-loop inputs, steps the verified
    /// position controller on each, writes the attitude setpoint to the output.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitAtt> {
        let (mut writer, reader) =
            falcon_position_stream_bindings::wit_stream::new::<WitAtt>();

        while let Some(inp) = inputs.next().await {
            let setpoint = PositionSetpoint {
                position_ned: [inp.target_north, inp.target_east, inp.target_down],
                velocity_ned: [0.0, 0.0, 0.0],
                yaw_setpoint: inp.target_yaw,
            };
            let att = pos().tick(
                next_timestamp(),
                [inp.pos_n, inp.pos_e, inp.pos_d],
                [inp.vel_n, inp.vel_e, inp.vel_d],
                [inp.qw, inp.qx, inp.qy, inp.qz],
                setpoint,
            );
            let out = WitAtt {
                qw: att.quaternion[0],
                qx: att.quaternion[1],
                qy: att.quaternion[2],
                qz: att.quaternion[3],
                thrust: att.thrust,
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_position_stream_bindings::export!(Component with_types_in falcon_position_stream_bindings);
