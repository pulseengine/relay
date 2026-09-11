//! The composed cascade, run NATIVELY — the same crates the wasm stage
//! components wrap, wired the same way `wasm/cm/cascade` wires them.
//!
//! This exists for one claim: **you develop in wasm and deploy that same wasm,
//! unchanged**. That is only true if the Component Model is transparent — if
//! the identical Rust, reached through a wasm component boundary, computes the
//! identical answer. This module is the control side of that experiment.
//!
//! Fidelity matters more than elegance here. Every constant below is copied
//! from the corresponding component rather than chosen, INCLUDING the ones that
//! are wrong: each stage fabricates its own clock at a different hardcoded rate
//! (position 20 ms, attitude 4 ms, rate 1 ms) even though the cascade calls all
//! five once per step. Mirroring the bug is the point — a differential test
//! that silently "fixed" it on one side would compare two different programs
//! and report a difference that means nothing.

use relay_att::AttController;
use relay_iekf::{Iekf, Imu as IekfImu};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PositionSetpoint};
use relay_rate::RatePid;

/// Each crate declares its OWN `Timestamp` — same shape, distinct types, no
/// shared definition. So the conversion is written once per crate rather than
/// once, which is worth noticing: five components each re-deriving the same
/// two fields is how they ended up with five different hardcoded tick rates.
fn ts_pos(ms: u64) -> relay_pos::Timestamp {
    relay_pos::Timestamp { seconds: ms / 1000, fraction: frac(ms) }
}
fn ts_att(ms: u64) -> relay_att::Timestamp {
    relay_att::Timestamp { seconds: ms / 1000, fraction: frac(ms) }
}
fn ts_rate(ms: u64) -> relay_rate::Timestamp {
    relay_rate::Timestamp { seconds: ms / 1000, fraction: frac(ms) }
}
fn frac(ms: u64) -> u32 {
    ((ms % 1000) * (1u64 << 32) / 1000) as u32
}

pub struct NativeCascade {
    iekf: Iekf,
    pos: PosController,
    att: AttController,
    rate: RatePid,
    /// Per-stage tick counters, kept SEPARATE because the components keep them
    /// separate and advance them at different rates.
    n_pos: u64,
    n_att: u64,
    n_rate: u64,
}

impl Default for NativeCascade {
    fn default() -> Self {
        Self::new()
    }
}

impl NativeCascade {
    pub fn new() -> Self {
        Self {
            iekf: Iekf::level(),
            pos: PosController::new(),
            att: AttController::new(),
            rate: RatePid::new(),
            n_pos: 0,
            n_att: 0,
            n_rate: 0,
        }
    }

    /// One full cascade pass: estimate -> position -> attitude -> rate -> mixer.
    /// Mirrors `wasm/cm/cascade/src/lib.rs::step` call for call.
    pub fn step(&mut self, accel: [f32; 3], gyro: [f32; 3], target: [f32; 4]) -> [f32; 4] {
        // ── ekf (wasm/cm/iekf): 1 kHz hardcoded, gravity update at var 0.5 ──
        self.iekf.propagate(IekfImu { gyro, accel }, 0.001);
        self.iekf.update_gravity(accel, 0.5);
        let st = self.iekf.state();

        // ── position (wasm/cm/position): 50 Hz -> 20 ms per tick ────────────
        let sp = PositionSetpoint {
            position_ned: [target[0], target[1], target[2]],
            velocity_ned: [0.0, 0.0, 0.0],
            yaw_setpoint: target[3],
        };
        let att_sp = self.pos.tick(ts_pos(self.n_pos * 20), st.p, st.v, st.q, sp);
        self.n_pos += 1;

        // ── attitude (wasm/cm/attitude): 250 Hz -> 4 ms per tick ────────────
        let rate_sp = self.att.tick(ts_att(self.n_att * 4), st.q, att_sp.quaternion);
        self.n_att += 1;

        // ── rate (wasm/cm/rate): 1 kHz -> 1 ms per tick. Body rates come from
        //    the GYRO, not the estimator: the components pass state.wx/wy/wz,
        //    which the iekf component fills from imu.gx/gy/gz rather than from
        //    the filtered state. Mirroring that exactly.
        let torque = self.rate.tick(ts_rate(self.n_rate), gyro, rate_sp);
        self.n_rate += 1;

        // ── mixer (wasm/cm/falcon-mixer): constructed fresh per call ────────
        let mut mixer = QuadMixer::new();
        mixer.mix([torque[0], torque[1], torque[2]], att_sp.thrust)
    }
}
