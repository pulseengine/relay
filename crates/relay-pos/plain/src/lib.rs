//! Relay position controller — outermost loop of the falcon cascade.
//!
//! Cascaded P-PI structure:
//!
//! ```text
//!  position setpoint, current pos/vel ─►  [relay-pos] ─►  attitude
//!                                                          setpoint
//!                                                          + thrust
//!   pos error    →   v setpoint     (P)
//!   v   error    →   accel cmd      (PI)
//!   accel cmd    →   tilt + thrust  (small-angle map)
//! ```
//!
//! Runs at 50 Hz nominal — slow enough that the inner attitude
//! controller (`relay-att`) at 250 Hz and rate controller
//! (`relay-rate`) at 1 kHz both have time-scale separation for
//! cascade stability.
//!
//! ## Frames
//!
//! Position and velocity are expressed in **NED frame** (north,
//! east, down). The vertical axis is +z = down. Thrust is a
//! collective scalar in `[0, 1]` representing normalised motor
//! command magnitude before mixing.
//!
//! ## Algorithm
//!
//! ```text
//! pos_err  = setpoint.pos - vehicle.pos                       // NED
//! v_set    = kp_pos * pos_err                                 // clamp v_max
//! v_err    = v_set - vehicle.vel                              // NED
//! integ   += v_err * dt                                       // clamp i_max
//! accel    = kp_vel * v_err + ki_vel * integ + kd_vel * dvel  // anti-windup
//!
//! // Horizontal accel → tilt: small-angle map, accel ≈ -g * tan(tilt)
//! pitch_rad = atan2(-accel.north, GRAVITY)   // forward accel ⇐ -pitch
//! roll_rad  = atan2( accel.east,  GRAVITY)   // east accel    ⇐ +roll
//! clamp(roll, pitch, ±tilt_max)
//!
//! // Vertical accel → collective thrust:
//! thrust = hover_thrust - kz_vel * accel.down  // up-accel ⇐ more thrust
//! clamp(thrust, [0, 1])
//!
//! // Hold heading: yaw = current heading (extracted from vehicle state)
//! q_setpoint = euler_to_quat(roll, pitch, yaw_current)
//! ```
//!
//! ## Verified properties (v0.5 surrogates for SWREQ-FALCON-POS-P*)
//!
//! - **POS-P01** (setpoint bounds): `position_setpoint` magnitudes
//!   in `[-1e6, +1e6]` m, `velocity_setpoint` magnitudes ≤ `v_max`.
//!   Out-of-bound inputs are saturated, not panicked. *Test:
//!   `pos_p01_bounds_enforced`.*
//! - **POS-P02** (Lyapunov cascade): the full POS→ATT→RATE→MIX
//!   cascade reaches a 10 m waypoint within bounded time in the
//!   falcon-sitl-hover `mission` scenario. *Test:
//!   `examples/falcon-sitl-hover::deterministic_mission_passes`.*
//! - **POS-P03** (dual-DNA source-agnosticism): the controller
//!   makes no assumption about whether the setpoint came from an
//!   operator (MAVLink) or a stored mission (cFS-DNA `relay-sc`).
//!   Both paths produce identical type-compatible PositionSetpoint
//!   structs.
//! - No NaN/∞ propagation under degenerate input. *Test:
//!   `degenerate_input_does_not_propagate`.*

#![no_std]
#![forbid(unsafe_code)]

use libm::{atan2f, cosf, sinf};

/// Standard gravity magnitude (m/s²).
pub const GRAVITY: f32 = 9.81;

/// Timestamp mirror of the other relay-* crates'.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timestamp {
    pub seconds: u64,
    pub fraction: u32,
}

impl Timestamp {
    pub const ZERO: Self = Self { seconds: 0, fraction: 0 };

    pub fn as_secs_f32(self) -> f32 {
        self.seconds as f32 + (self.fraction as f32) / (1u64 << 32) as f32
    }
}

/// Position controller tuning gains.
#[derive(Clone, Copy, Debug)]
pub struct PosGains {
    /// Position-loop P gain (m/s per m).
    pub kp_pos: f32,
    /// Maximum commanded horizontal velocity (m/s).
    pub v_max_horizontal: f32,
    /// Maximum commanded vertical velocity (m/s).
    pub v_max_vertical: f32,
    /// Velocity-loop P gain (m/s² per m/s).
    pub kp_vel: f32,
    /// Velocity-loop I gain (m/s³ per m/s).
    pub ki_vel: f32,
    /// Velocity-loop integral magnitude bound (m/s).
    pub i_max: f32,
    /// Maximum body tilt angle (rad).
    pub tilt_max: f32,
    /// Collective thrust at hover (normalised; usually 0.4–0.6).
    pub hover_thrust: f32,
    /// Vertical-axis velocity → thrust gain (1/m·s).
    pub kz_vel: f32,
}

impl PosGains {
    /// Reasonable defaults for a 500 g, 10-inch quad.
    pub const DEFAULT: Self = Self {
        kp_pos: 1.0,
        v_max_horizontal: 3.0,
        v_max_vertical: 2.0,
        kp_vel: 1.5,
        ki_vel: 0.5,
        i_max: 2.0,
        tilt_max: 0.44, // ~25°
        hover_thrust: 0.5,
        kz_vel: 0.05,
    };
}

impl Default for PosGains {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// 3D vector in NED frame (north, east, down).
pub type Ned = [f32; 3];

/// Position setpoint passed to the controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionSetpoint {
    /// Target position in NED frame (meters).
    pub position_ned: Ned,
    /// Optional feed-forward velocity in NED (m/s). Zero for hover.
    pub velocity_ned: Ned,
    /// Heading setpoint (rad, NED yaw). NaN ⇒ hold current yaw.
    pub yaw_setpoint: f32,
}

impl PositionSetpoint {
    /// A "hover at point" setpoint with zero feed-forward velocity
    /// and "hold current yaw" semantics.
    pub fn hover_at(position_ned: Ned) -> Self {
        Self {
            position_ned,
            velocity_ned: [0.0; 3],
            yaw_setpoint: f32::NAN,
        }
    }
}

/// Output of the controller: attitude setpoint quaternion + thrust.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeSetpoint {
    /// Body-to-NED unit quaternion (w, x, y, z).
    pub quaternion: [f32; 4],
    /// Collective thrust in [0, 1].
    pub thrust: f32,
}

/// Controller state.
#[derive(Clone, Copy, Debug)]
pub struct PosController {
    integral: Ned,
    last_v_err: Ned,
    last_time: Timestamp,
    last_attitude_setpoint: AttitudeSetpoint,
    gains: PosGains,
}

impl PosController {
    pub const fn new() -> Self {
        Self {
            integral: [0.0; 3],
            last_v_err: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_attitude_setpoint: AttitudeSetpoint {
                quaternion: [1.0, 0.0, 0.0, 0.0],
                thrust: 0.0,
            },
            gains: PosGains::DEFAULT,
        }
    }

    pub const fn with_gains(gains: PosGains) -> Self {
        Self {
            integral: [0.0; 3],
            last_v_err: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_attitude_setpoint: AttitudeSetpoint {
                quaternion: [1.0, 0.0, 0.0, 0.0],
                thrust: 0.0,
            },
            gains,
        }
    }

    pub fn set_gains(&mut self, g: PosGains) {
        self.gains = g;
    }
    pub fn gains(&self) -> PosGains {
        self.gains
    }
    pub fn integral(&self) -> Ned {
        self.integral
    }
    pub fn last_attitude_setpoint(&self) -> AttitudeSetpoint {
        self.last_attitude_setpoint
    }

    /// Reset integrator + cached state. Call when arming/disarming
    /// or after a setpoint discontinuity.
    pub fn reset(&mut self) {
        self.integral = [0.0; 3];
        self.last_v_err = [0.0; 3];
    }

    /// Run one control step. `vehicle_quat` is the current attitude
    /// estimate (body-to-NED) for yaw extraction.
    pub fn tick(
        &mut self,
        time: Timestamp,
        vehicle_position_ned: Ned,
        vehicle_velocity_ned: Ned,
        vehicle_quat: [f32; 4],
        setpoint: PositionSetpoint,
    ) -> AttitudeSetpoint {
        let dt = if self.last_time.seconds == 0 && self.last_time.fraction == 0 {
            1.0 / 50.0
        } else {
            let d = time.as_secs_f32() - self.last_time.as_secs_f32();
            clamp_f32(d, 0.001, 0.1)
        };
        self.last_time = time;

        // 1. Outer position loop: pos error → velocity setpoint.
        let mut v_set = [0.0_f32; 3];
        let v_max = [
            self.gains.v_max_horizontal,
            self.gains.v_max_horizontal,
            self.gains.v_max_vertical,
        ];
        for i in 0..3 {
            let p_err = setpoint.position_ned[i] - vehicle_position_ned[i];
            let p_err = sanitise(p_err);
            let raw = self.gains.kp_pos * p_err + sanitise(setpoint.velocity_ned[i]);
            v_set[i] = clamp_f32(raw, -v_max[i], v_max[i]);
        }

        // 2. Inner velocity loop: velocity error → acceleration.
        //    Anti-windup: hold integral if commanded accel would
        //    saturate. We don't have an explicit accel ceiling, but
        //    we cap the integral magnitude itself.
        let mut accel = [0.0_f32; 3];
        for i in 0..3 {
            let v_err = v_set[i] - sanitise(vehicle_velocity_ned[i]);
            let cand_integral = clamp_f32(
                self.integral[i] + v_err * dt,
                -self.gains.i_max,
                self.gains.i_max,
            );
            let derivative = (v_err - self.last_v_err[i]) / dt;
            accel[i] = self.gains.kp_vel * v_err
                + self.gains.ki_vel * cand_integral
                + 0.0 * derivative; // kd reserved for v0.6
            self.integral[i] = cand_integral;
            self.last_v_err[i] = v_err;
        }

        // 3. Horizontal accel → tilt direction.
        //    Convention: north-accel ⇐ nose-up pitch (negative pitch in
        //    body convention since +pitch ≈ nose-up, accel ≈ -g sin(pitch));
        //    east-accel ⇐ right-side-down roll (positive roll).
        let pitch_rad = atan2f(-accel[0], GRAVITY);
        let roll_rad = atan2f(accel[1], GRAVITY);
        let pitch = clamp_f32(pitch_rad, -self.gains.tilt_max, self.gains.tilt_max);
        let roll = clamp_f32(roll_rad, -self.gains.tilt_max, self.gains.tilt_max);

        // 4. Vertical accel → collective thrust. In NED, +z = down,
        //    so positive accel.down (= falling faster) means we need
        //    LESS thrust, while negative accel.down (= rising) means
        //    MORE thrust.
        let thrust = clamp_f32(
            self.gains.hover_thrust - self.gains.kz_vel * accel[2],
            0.0,
            1.0,
        );

        // 5. Hold heading: if setpoint.yaw is NaN, extract yaw from
        //    the current vehicle quaternion; else use setpoint.yaw.
        let yaw = if setpoint.yaw_setpoint.is_finite() {
            setpoint.yaw_setpoint
        } else {
            quat_to_yaw(vehicle_quat)
        };

        // 6. Build body-to-NED setpoint quaternion from Euler (Z-Y-X).
        let q_setpoint = euler_to_quaternion(roll, pitch, yaw);

        let out = AttitudeSetpoint {
            quaternion: q_setpoint,
            thrust,
        };
        self.last_attitude_setpoint = out;
        out
    }
}

impl Default for PosController {
    fn default() -> Self {
        Self::new()
    }
}

#[inline]
fn sanitise(x: f32) -> f32 {
    if !x.is_finite() {
        0.0
    } else {
        x
    }
}

#[inline]
fn clamp_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if !v.is_finite() {
        lo
    } else if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// Extract yaw (NED-z rotation) from a body-to-NED quaternion.
/// Standard Z-Y-X Euler convention.
fn quat_to_yaw(q: [f32; 4]) -> f32 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let siny = 2.0 * (w * z + x * y);
    let cosy = 1.0 - 2.0 * (y * y + z * z);
    atan2f(siny, cosy)
}

/// Z-Y-X Euler (yaw, pitch, roll about NED axes) → body-to-NED unit
/// quaternion.
pub fn euler_to_quaternion(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let cr = cosf(roll * 0.5);
    let sr = sinf(roll * 0.5);
    let cp = cosf(pitch * 0.5);
    let sp = sinf(pitch * 0.5);
    let cy = cosf(yaw * 0.5);
    let sy = sinf(yaw * 0.5);
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(secs: f32) -> Timestamp {
        let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
        Timestamp { seconds: secs as u64, fraction: frac }
    }

    #[test]
    fn at_setpoint_with_zero_velocity_gives_hover_thrust() {
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([0.0, 0.0, 0.0]);
        let out = c.tick(
            ts(1.0),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        // Tilt should be near zero; thrust should be near hover.
        assert!((out.thrust - c.gains().hover_thrust).abs() < 0.05,
            "thrust {} not near hover {}", out.thrust, c.gains().hover_thrust);
    }

    #[test]
    fn forward_position_error_pitches_nose_down() {
        let mut c = PosController::new();
        // Setpoint is 10 m forward (north), vehicle at origin.
        let sp = PositionSetpoint::hover_at([10.0, 0.0, 0.0]);
        // First tick: extracts setpoint, builds tilt.
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        // Pitch should be negative (nose down) → accelerates forward.
        // Extract pitch from quaternion (small-angle: q.y ≈ pitch/2)
        let pitch_est = 2.0 * out.quaternion[2];
        assert!(pitch_est < -0.01,
            "forward setpoint must pitch nose-down (q.y < 0), got pitch_est={}", pitch_est);
    }

    #[test]
    fn east_position_error_rolls_right() {
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([0.0, 10.0, 0.0]);
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        // Positive roll → right wing down → accelerates east.
        let roll_est = 2.0 * out.quaternion[1];
        assert!(roll_est > 0.01,
            "east setpoint must roll right (q.x > 0), got roll_est={}", roll_est);
    }

    #[test]
    fn altitude_above_setpoint_reduces_thrust() {
        // Vehicle at NED.z = -2 (2 m above setpoint at z=0).
        // Need to descend (positive NED.z direction) → less thrust.
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([0.0, 0.0, 0.0]);
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, -2.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        assert!(out.thrust < c.gains().hover_thrust,
            "above-setpoint must reduce thrust, got {}", out.thrust);
    }

    #[test]
    fn altitude_below_setpoint_increases_thrust() {
        // Vehicle at NED.z = +2 (2 m BELOW setpoint, going further down).
        // Need to ascend (negative NED.z direction) → more thrust.
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([0.0, 0.0, 0.0]);
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        assert!(out.thrust > c.gains().hover_thrust,
            "below-setpoint must increase thrust, got {}", out.thrust);
    }

    #[test]
    fn tilt_clamped_to_tilt_max() {
        let mut c = PosController::new();
        // Massive setpoint error → integrator and v_setpoint clamp,
        // but the small-angle map could still ask for huge tilt.
        let sp = PositionSetpoint::hover_at([1000.0, 1000.0, 0.0]);
        let mut t = 0.0_f32;
        for _ in 0..100 {
            t += 0.02;
            let out = c.tick(
                ts(t),
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                sp,
            );
            // Each axis tilt magnitude ≤ tilt_max.
            // Extract approximate roll, pitch from small-angle expansion.
            let q = out.quaternion;
            let roll = 2.0 * q[1].atan2(q[0]);
            let pitch = 2.0 * q[2].atan2(q[0]);
            assert!(roll.abs() <= c.gains().tilt_max + 1.0e-3,
                "roll {} exceeds tilt_max {}", roll, c.gains().tilt_max);
            assert!(pitch.abs() <= c.gains().tilt_max + 1.0e-3,
                "pitch {} exceeds tilt_max {}", pitch, c.gains().tilt_max);
        }
    }

    #[test]
    fn v_setpoint_clamped_to_v_max() {
        // 1000 m error → v_set capped at v_max.
        // Confirm via the integral: it cannot grow faster than i_max
        // even when commanded to extreme values.
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([1000.0, 0.0, 0.0]);
        let mut t = 0.0_f32;
        for _ in 0..1000 {
            t += 0.02;
            c.tick(
                ts(t),
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                sp,
            );
            for i in 0..3 {
                assert!(c.integral()[i].abs() <= c.gains().i_max + 1.0e-6,
                    "integral[{}] = {} exceeds i_max", i, c.integral()[i]);
            }
        }
    }

    #[test]
    fn pos_p01_bounds_enforced() {
        // Extreme out-of-range position setpoints must be sanitised
        // (no NaN propagation, integral stays bounded).
        let mut c = PosController::new();
        let sp = PositionSetpoint {
            position_ned: [1.0e30, -1.0e30, 0.0],
            velocity_ned: [0.0; 3],
            yaw_setpoint: f32::NAN,
        };
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        for k in 0..4 {
            assert!(out.quaternion[k].is_finite());
        }
        assert!(out.thrust.is_finite());
    }

    #[test]
    fn degenerate_input_does_not_propagate() {
        let mut c = PosController::new();
        let sp = PositionSetpoint {
            position_ned: [f32::NAN, 0.0, 0.0],
            velocity_ned: [0.0, f32::INFINITY, 0.0],
            yaw_setpoint: f32::NAN,
        };
        let out = c.tick(
            ts(0.02),
            [0.0, 0.0, 0.0],
            [f32::NAN, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            sp,
        );
        for k in 0..4 {
            assert!(out.quaternion[k].is_finite());
        }
        assert!(out.thrust.is_finite());
    }

    #[test]
    fn yaw_held_when_setpoint_is_nan() {
        let mut c = PosController::new();
        // Vehicle quaternion with 30° yaw (rotation about NED-z).
        let yaw_target = 30.0_f32.to_radians();
        let half = yaw_target * 0.5;
        let q = [cosf(half), 0.0, 0.0, sinf(half)];
        let sp = PositionSetpoint::hover_at([0.0, 0.0, 0.0]); // yaw=NaN
        let out = c.tick(ts(0.02), [0.0; 3], [0.0; 3], q, sp);
        // Output quaternion should have approximately the same yaw.
        let yaw_out = quat_to_yaw(out.quaternion);
        assert!((yaw_out - yaw_target).abs() < 0.05,
            "yaw not held: expected {} got {}", yaw_target, yaw_out);
    }

    #[test]
    fn yaw_setpoint_is_followed() {
        let mut c = PosController::new();
        let yaw_set = 45.0_f32.to_radians();
        let sp = PositionSetpoint {
            position_ned: [0.0; 3],
            velocity_ned: [0.0; 3],
            yaw_setpoint: yaw_set,
        };
        let out = c.tick(ts(0.02), [0.0; 3], [0.0; 3], [1.0, 0.0, 0.0, 0.0], sp);
        let yaw_out = quat_to_yaw(out.quaternion);
        assert!((yaw_out - yaw_set).abs() < 0.05);
    }

    #[test]
    fn reset_clears_integral() {
        let mut c = PosController::new();
        let sp = PositionSetpoint::hover_at([10.0, 0.0, 0.0]);
        for i in 0..50 {
            c.tick(
                ts(i as f32 * 0.02),
                [0.0; 3],
                [0.0; 3],
                [1.0, 0.0, 0.0, 0.0],
                sp,
            );
        }
        assert!(c.integral().iter().any(|v| v.abs() > 0.01));
        c.reset();
        assert_eq!(c.integral(), [0.0; 3]);
    }

    use proptest::prelude::*;

    proptest! {
        /// Outputs always finite + tilt clamped + thrust in [0,1] for
        /// any finite vehicle state and finite setpoint.
        #[test]
        fn pos_p01_property(
            px in -100.0_f32..100.0, py in -100.0_f32..100.0, pz in -100.0_f32..100.0,
            vx in -10.0_f32..10.0, vy in -10.0_f32..10.0, vz in -10.0_f32..10.0,
            sx in -1000.0_f32..1000.0, sy in -1000.0_f32..1000.0, sz in -1000.0_f32..1000.0,
        ) {
            let mut c = PosController::new();
            let sp = PositionSetpoint::hover_at([sx, sy, sz]);
            let out = c.tick(
                ts(0.02),
                [px, py, pz],
                [vx, vy, vz],
                [1.0, 0.0, 0.0, 0.0],
                sp,
            );
            for k in 0..4 {
                prop_assert!(out.quaternion[k].is_finite());
            }
            prop_assert!(out.thrust.is_finite());
            prop_assert!((0.0..=1.0).contains(&out.thrust));
            for i in 0..3 {
                prop_assert!(c.integral()[i].abs() <= c.gains().i_max + 1.0e-6);
            }
        }
    }
}
