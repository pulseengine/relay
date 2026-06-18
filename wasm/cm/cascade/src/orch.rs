//! Shared cascade orchestration — the per-tick flight-control pipeline
//! (iekf -> pos -> att -> rate -> mixer) over the five VERIFIED engines.
//!
//! Single source of the wiring, used by BOTH the async P3 stream component
//! (`stream.rs`, exported as the `monitor` stream transformer) and the sync
//! coverage sibling (`step.rs`, exported as a call-return `step` so a witness
//! MC/DC harness can drive the SAME engine branches — #202; the async-lift
//! `monitor` stream export is not call-return invokable).
//!
//! No engine logic lives here: this is pure orchestration glue over the engine
//! crates, so neither component carries an inline copy of the cascade wiring.

use relay_att::{AttController, Timestamp as AttTs};
use relay_iekf::{Iekf, Imu as RImu};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PositionSetpoint, Timestamp as PosTs};
use relay_rate::{RatePid, Timestamp as RateTs};

/// One raw cascade input tick: body accel + gyro and the NED waypoint + yaw.
pub struct TickIn {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub north: f32,
    pub east: f32,
    pub down: f32,
    pub yaw: f32,
}

/// The four motor PWMs of a quad.
pub struct TickOut {
    pub m1: f32,
    pub m2: f32,
    pub m3: f32,
    pub m4: f32,
}

/// Per-component cascade state. The engines hold integrators / filters that must
/// persist across ticks, so a caller keeps one `Cascade` for the component's
/// life and feeds it one `TickIn` per control cycle.
pub struct Cascade {
    ekf: Iekf,
    pos: PosController,
    att: AttController,
    rate: RatePid,
    tick_ms: u64,
}

impl Default for Cascade {
    fn default() -> Self {
        Self::new()
    }
}

impl Cascade {
    pub fn new() -> Self {
        Self {
            ekf: Iekf::level(),
            pos: PosController::new(),
            att: AttController::new(),
            rate: RatePid::new(),
            tick_ms: 0,
        }
    }

    /// One tick of the full cascade: estimate -> position -> attitude -> rate -> mix.
    pub fn tick(&mut self, inp: TickIn) -> TickOut {
        let ms = self.tick_ms;
        self.tick_ms += 1;
        let (s, fr) = (ms / 1000, ((ms % 1000) * (1u64 << 32) / 1000) as u32);

        // 1. State estimation (SE2(3) propagate + gravity update).
        let accel = [inp.ax, inp.ay, inp.az];
        let gyro = [inp.gx, inp.gy, inp.gz];
        self.ekf.propagate(RImu { gyro, accel }, 0.001);
        self.ekf.update_gravity(accel, 0.5);
        let st = self.ekf.state();
        let quat = [st.q[0], st.q[1], st.q[2], st.q[3]];

        // 2. Position loop -> attitude setpoint.
        let att_out = self.pos.tick(
            PosTs { seconds: s, fraction: fr },
            [st.p[0], st.p[1], st.p[2]],
            [st.v[0], st.v[1], st.v[2]],
            quat,
            PositionSetpoint {
                position_ned: [inp.north, inp.east, inp.down],
                velocity_ned: [0.0, 0.0, 0.0],
                yaw_setpoint: inp.yaw,
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
        let rate_sp = self.att.tick(AttTs { seconds: s, fraction: fr }, quat, att_sp);

        // 4. Rate loop -> torque (measured body rate = gyro passthrough).
        let torque = self.rate.tick(RateTs { seconds: s, fraction: fr }, gyro, rate_sp);

        // 5. Control allocation -> per-motor PWM.
        let mut mixer = QuadMixer::new();
        let m = mixer.mix(torque, thrust);
        TickOut {
            m1: m[0],
            m2: m[1],
            m3: m[2],
            m4: m[3],
        }
    }
}
