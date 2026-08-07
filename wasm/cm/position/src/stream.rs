// Falcon Position Controller — P3 Stream Transformer (shared cascade-stream types).
//
// stream<pos-input> -> stream<attitude-setpoint>, where pos-input is
// { state: vehicle-state, target: waypoint } — shared types so the cascade
// composes (STREAM-P10). Drives the verified relay-pos PosController.

use relay_pos::{PosController, PositionSetpoint, Timestamp};

use falcon_position_stream_bindings::exports::pulseengine::falcon_cascade_stream::position_stream::{
    AttitudeSetpoint as WitAtt, Guest, PosInput as WitInput,
};

struct Component;

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
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitAtt> {
        let (mut writer, reader) =
            falcon_position_stream_bindings::wit_stream::new::<WitAtt>();

        while let Some(inp) = inputs.next().await {
            let s = &inp.state;
            let t = &inp.target;
            let setpoint = PositionSetpoint {
                position_ned: [t.north, t.east, t.down],
                velocity_ned: [0.0, 0.0, 0.0],
                yaw_setpoint: t.yaw,
            };
            let att = pos().tick(
                next_timestamp(),
                [s.pos_n, s.pos_e, s.pos_d],
                [s.vel_n, s.vel_e, s.vel_d],
                [s.qw, s.qx, s.qy, s.qz],
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
