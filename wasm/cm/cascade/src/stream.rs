// Falcon Control Cascade — COMPOSED P3 Stream Transformer component.
//
// stream<cascade-input{imu, target}> -> stream<motor-pwm>, running the full
// flight-control pipeline (iekf -> pos -> att -> rate -> mixer) per tick. The
// streamed counterpart of the sync falcon-cascade socket: one orchestrating
// component driving all FIVE verified engines directly (STREAM-P11, v1.73).
//
// wac composes function-call interfaces, not stream data-flow, so a streamed
// pipeline is one orchestrating component rather than a wac_plug of the five
// stream components. Single input stream of a combined record — no multi-stream
// zip. Each stage runs its actual verified engine; no inline copies.

use relay_att::{AttController, Timestamp as AttTs};
use relay_iekf::{Iekf, Imu as RImu};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PositionSetpoint, Timestamp as PosTs};
use relay_rate::{RatePid, Timestamp as RateTs};

use falcon_cascade_stream_composed_bindings::exports::falcon::cascade_stream::cascade_stream::{
    CascadeInput as WitInput, Guest, MotorPwm as WitPwm,
};

struct Component;

static mut EKF: Option<Iekf> = None;
static mut POS: Option<PosController> = None;
static mut ATT: Option<AttController> = None;
static mut RATE: Option<RatePid> = None;
static mut TICK_MS: u64 = 0;

fn ekf() -> &'static mut Iekf {
    unsafe {
        if EKF.is_none() { EKF = Some(Iekf::level()); }
        EKF.as_mut().unwrap()
    }
}
fn pos() -> &'static mut PosController {
    unsafe {
        if POS.is_none() { POS = Some(PosController::new()); }
        POS.as_mut().unwrap()
    }
}
fn att() -> &'static mut AttController {
    unsafe {
        if ATT.is_none() { ATT = Some(AttController::new()); }
        ATT.as_mut().unwrap()
    }
}
fn rate() -> &'static mut RatePid {
    unsafe {
        if RATE.is_none() { RATE = Some(RatePid::new()); }
        RATE.as_mut().unwrap()
    }
}

impl Guest for Component {
    /// One tick of the full cascade per input. Mirrors the sync cascade socket's
    /// step(): estimate -> position -> attitude -> rate -> mix.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitPwm> {
        let (mut writer, reader) =
            falcon_cascade_stream_composed_bindings::wit_stream::new::<WitPwm>();

        while let Some(inp) = inputs.next().await {
            let ms = unsafe {
                let m = TICK_MS;
                TICK_MS += 1;
                m
            };
            let (s, fr) = (ms / 1000, ((ms % 1000) * (1u64 << 32) / 1000) as u32);

            // 1. State estimation (SE2(3) propagate + gravity update).
            let accel = [inp.imu.ax, inp.imu.ay, inp.imu.az];
            let gyro = [inp.imu.gx, inp.imu.gy, inp.imu.gz];
            let e = ekf();
            e.propagate(RImu { gyro, accel }, 0.001);
            e.update_gravity(accel, 0.5);
            let st = e.state();
            let quat = [st.q[0], st.q[1], st.q[2], st.q[3]];

            // 2. Position loop -> attitude setpoint.
            let att_out = pos().tick(
                PosTs { seconds: s, fraction: fr },
                [st.p[0], st.p[1], st.p[2]],
                [st.v[0], st.v[1], st.v[2]],
                quat,
                PositionSetpoint {
                    position_ned: [inp.target.north, inp.target.east, inp.target.down],
                    velocity_ned: [0.0, 0.0, 0.0],
                    yaw_setpoint: inp.target.yaw,
                },
            );
            let att_sp = [
                att_out.quaternion[0],
                att_out.quaternion[1],
                att_out.quaternion[2],
                att_out.quaternion[3],
            ];
            let thrust = att_out.thrust;

            // 3. Attitude loop -> body-rate setpoint (estimate quat vs setpoint quat).
            let rate_sp = att().tick(AttTs { seconds: s, fraction: fr }, quat, att_sp);

            // 4. Rate loop -> torque (measured body rate = gyro passthrough).
            let torque = rate().tick(RateTs { seconds: s, fraction: fr }, gyro, rate_sp);

            // 5. Control allocation -> per-motor PWM.
            let mut mixer = QuadMixer::new();
            let m = mixer.mix(torque, thrust);
            let out = WitPwm { m1: m[0], m2: m[1], m3: m[2], m4: m[3] };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_cascade_stream_composed_bindings::export!(Component with_types_in falcon_cascade_stream_composed_bindings);
