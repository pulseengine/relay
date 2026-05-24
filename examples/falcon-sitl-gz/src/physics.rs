//! `Physics` trait — the abstraction a real Gazebo bridge implements.
//!
//! Same architectural pattern as the HITL harness's `HitlBench` trait
//! and `FrameSource`: one tiny interface decouples the verified
//! cascade from the IO that gives it sensor data + consumes its
//! motor commands. Verified path (`relay-ekf` → `relay-pos` →
//! `relay-att` → `relay-rate` → `relay-mix-quad`) is unchanged.
//!
//! `MockPhysics` is the in-process reference implementation —
//! a copy of `examples/falcon-sitl-hover`'s `Plant` struct, kept
//! small so the scaffold exercises end-to-end. `GazeboPhysics` is
//! the stub for the real bridge.

use libm::sqrtf;
use relay_ekf::{quat_mul, ImuSample};

// Same physical constants the falcon-sitl-hover SITL uses.
pub const INERTIA: f32 = 0.0125; // kg·m²
pub const FRICTION: f32 = 0.005;
pub const THRUST_SCALE: f32 = 20.0; // N at full PWM
pub const GRAVITY: f32 = 9.81;
pub const DRAG_COEFFICIENT: f32 = 0.05;

/// What the verified cascade needs from "the world".
///
/// `step` advances the physics by `dt` under the given motor PWMs;
/// `measure` reads back an IMU sample + true position for the
/// estimator + safety chain. `position_ned_m` is what the
/// `relay-lc::Geofence::check` sees post-conversion.
pub trait Physics {
    /// Backend name for the verdict log.
    fn name(&self) -> &'static str;

    /// Advance physics by `dt` seconds under the given motor PWMs
    /// (4 values, each in [0, 1]). For the falcon-quad airframe.
    fn step(&mut self, motor_pwm: [f32; 4], dt: f32);

    /// Read IMU body-frame samples + true NED position (m).
    /// `noise_std` lets the impl add Gaussian noise; the
    /// `MockPhysics` impl uses a tiny xorshift; real gz-sim would
    /// already have noise baked into its IMU model so the parameter
    /// becomes a no-op there.
    fn measure(&mut self, noise_std: f32) -> (ImuSample, [f32; 3]);
}

/// In-process reference impl — same toy integrator as
/// `examples/falcon-sitl-hover`'s `Plant`. Kept here so the
/// scaffold is runnable without external dependencies.
pub struct MockPhysics {
    /// Body-frame angular velocity (rad/s).
    pub omega: [f32; 3],
    /// Body-to-NED unit quaternion.
    pub q: [f32; 4],
    /// Position in NED frame (m).
    pub p_ned: [f32; 3],
    /// Velocity in NED frame (m/s).
    pub v_ned: [f32; 3],
    /// xorshift state for the IMU noise generator.
    pub rng: u64,
}

impl MockPhysics {
    pub fn at_rest() -> Self {
        Self {
            omega: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            p_ned: [0.0; 3],
            v_ned: [0.0; 3],
            rng: 0xCAFE_BABE_DEAD_BEEF,
        }
    }

    fn next_unit_normal(&mut self) -> f32 {
        // Box-Muller from two uniforms in (0, 1].
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u1 = ((self.rng >> 11) as f32 / (1u64 << 53) as f32).max(1e-9);
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u2 = (self.rng >> 11) as f32 / (1u64 << 53) as f32;
        let r = libm::sqrtf(-2.0 * libm::logf(u1));
        let theta = 2.0 * std::f32::consts::PI * u2;
        r * libm::cosf(theta)
    }
}

impl Physics for MockPhysics {
    fn name(&self) -> &'static str { "mock" }

    fn step(&mut self, motor_pwm: [f32; 4], dt: f32) {
        // Sum the four motor PWMs into a normalised collective thrust;
        // mixer's torque is approximated as zero in this scaffold (the
        // full mixer-to-physics torque mapping lives in falcon-sitl-
        // hover; this stub keeps the cascade running so the scaffold
        // ends with a complete loop).
        let thrust_normalised =
            ((motor_pwm[0] + motor_pwm[1] + motor_pwm[2] + motor_pwm[3]) / 4.0).clamp(0.0, 1.0);

        // Rotational dynamics under (zero) torque + friction.
        for i in 0..3 {
            self.omega[i] += ((-FRICTION * self.omega[i]) / INERTIA) * dt;
        }
        // Integrate quaternion from angular velocity.
        let qdot = quat_mul(self.q, [0.0, self.omega[0], self.omega[1], self.omega[2]]);
        let mut q_new = [
            self.q[0] + 0.5 * qdot[0] * dt,
            self.q[1] + 0.5 * qdot[1] * dt,
            self.q[2] + 0.5 * qdot[2] * dt,
            self.q[3] + 0.5 * qdot[3] * dt,
        ];
        let n = sqrtf(
            q_new[0] * q_new[0] + q_new[1] * q_new[1]
                + q_new[2] * q_new[2] + q_new[3] * q_new[3],
        );
        if n > 1.0e-12 {
            q_new = [q_new[0] / n, q_new[1] / n, q_new[2] / n, q_new[3] / n];
            self.q = q_new;
        }

        // Translational — thrust body up rotated into NED, plus gravity, minus drag.
        let t = thrust_normalised * THRUST_SCALE;
        let thrust_body = [0.0, 0.0, -t];
        let qv = [0.0, thrust_body[0], thrust_body[1], thrust_body[2]];
        let qc = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let t1 = quat_mul(self.q, quat_mul(qv, qc));
        let thrust_ned = [t1[1], t1[2], t1[3]];
        for i in 0..3 {
            let g = if i == 2 { GRAVITY } else { 0.0 };
            let drag = DRAG_COEFFICIENT * self.v_ned[i];
            let a = thrust_ned[i] + g - drag;
            self.v_ned[i] += a * dt;
            self.p_ned[i] += self.v_ned[i] * dt;
        }
    }

    fn measure(&mut self, noise_std: f32) -> (ImuSample, [f32; 3]) {
        let gyro_body = [
            self.omega[0] + noise_std * self.next_unit_normal(),
            self.omega[1] + noise_std * self.next_unit_normal(),
            self.omega[2] + noise_std * self.next_unit_normal(),
        ];
        // Body-frame accel: gravity rotated into body via q. Simplified
        // — at small attitude angles the accel reads [0, 0, -g] body
        // plus thrust contribution; we approximate as the body-frame
        // thrust the controller would feel for closed-loop testing.
        let accel_body = [
            noise_std * self.next_unit_normal(),
            noise_std * self.next_unit_normal(),
            -GRAVITY + noise_std * self.next_unit_normal(),
        ];
        let sample = ImuSample {
            time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
            accel_body,
            gyro_body,
        };
        (sample, self.p_ned)
    }
}

/// Stub for the Gazebo Sim bridge. The bench-side wiring lives in
/// `gz-transport` (Protobuf + ZeroMQ); this stub documents what the
/// final impl would call but doesn't drag in the heavy dep tree
/// (which is C++ + bindgen) for the v0.16.1 deliverable.
///
/// To make this real on a bench:
///
///   1. Spin up `gz sim falcon-quad-world.sdf` (an SDF world with
///      a quadcopter model carrying IMU + GPS sensors and four
///      motor plugins).
///   2. Replace the body of `step` here with a publish to
///      `/world/falcon/model/quad/joint/<rotor_n>/cmd_vel` for
///      each rotor (Gazebo's standard MulticopterMotorModel
///      plugin).
///   3. Replace `measure` with a subscribe to
///      `/world/falcon/model/quad/link/imu_link/sensor/imu_sensor/imu`
///      and `/.../navsat`. Convert Gazebo's IMU frame to falcon's
///      NED + body convention (the conversion is the same as the
///      MavlinkBench's Home::project_ned_cm equirectangular step).
///
/// `gz-transport` Rust bindings: track
/// https://github.com/gazebosim/gz-msgs and `gz-transport-rs` (or
/// generate Protobuf bindings from `gz/msgs/*.proto` directly via
/// `prost-build`).
pub struct GazeboPhysics {
    pub world_name: String,
    pub model_name: String,
}

impl GazeboPhysics {
    pub fn new(world: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            world_name: world.into(),
            model_name: model.into(),
        }
    }
}

impl Physics for GazeboPhysics {
    fn name(&self) -> &'static str { "gazebo (stub)" }

    fn step(&mut self, _motor_pwm: [f32; 4], _dt: f32) {
        // TODO(bench): publish per-rotor cmd_vel messages to
        //   /world/{world_name}/model/{model_name}/joint/<rotor_n>/cmd_vel
        // and await a physics step from gz-sim.
        eprintln!(
            "GazeboPhysics::step is a stub — world={} model={}; see physics.rs for the bench-wire-up recipe",
            self.world_name, self.model_name,
        );
    }

    fn measure(&mut self, _noise_std: f32) -> (ImuSample, [f32; 3]) {
        // TODO(bench): subscribe to imu_sensor + navsat topics and
        // return the latest sample. Convert Gazebo's body-frame +
        // ENU to falcon's body-frame + NED.
        (
            ImuSample {
                time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                accel_body: [0.0; 3],
                gyro_body: [0.0; 3],
            },
            [0.0, 0.0, 0.0],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_physics_at_rest_stays_quiet() {
        let mut p = MockPhysics::at_rest();
        // No motor input, no gravity-only fall (we step with zero PWM
        // → zero thrust → falls under gravity; check the fall is
        // physically reasonable).
        p.step([0.0; 4], 0.01);
        assert!(p.v_ned[2] > 0.0, "no thrust → should accelerate down (+z NED)");
        assert!(p.v_ned[2] < 1.0, "1 step at dt=0.01 → v ≈ g*dt = 0.098 m/s");
    }

    #[test]
    fn mock_physics_hover_with_full_thrust_climbs() {
        let mut p = MockPhysics::at_rest();
        // Full PWM on all 4 motors: thrust = THRUST_SCALE = 20 N >> gravity.
        // After 1 step at dt=0.01 we expect upward (-z NED) velocity.
        p.step([1.0; 4], 0.01);
        assert!(p.v_ned[2] < 0.0, "max thrust → upward velocity (-z NED)");
    }

    #[test]
    fn mock_physics_measure_returns_sensible_imu() {
        let mut p = MockPhysics::at_rest();
        let (s, pos) = p.measure(0.0);
        assert_eq!(pos, [0.0; 3]);
        // No noise → gyro reads angular velocity (zero at rest).
        assert_eq!(s.gyro_body[0], 0.0);
        // Accel z at rest reads -gravity (NED z-axis points down).
        assert!((s.accel_body[2] - (-GRAVITY)).abs() < 1e-3);
    }

    #[test]
    fn gazebo_stub_compiles_and_returns_zeros() {
        let mut g = GazeboPhysics::new("falcon", "quad");
        g.step([0.5; 4], 0.01);
        let (s, pos) = g.measure(0.0);
        assert_eq!(pos, [0.0; 3]);
        assert_eq!(s.accel_body[0], 0.0);
    }
}
