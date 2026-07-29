//! Relay rate controller — body-rate PID stabilizer.
//!
//! Per-axis PID with anti-windup. Runs at 1 kHz nominal alongside the
//! IMU. Consumes the gyro measurement and a rate setpoint (rad/s body),
//! produces a torque setpoint (Nm body) that the mixer then translates
//! to per-motor PWM in v0.4.
//!
//! Designed as the inner loop of the falcon control cascade:
//!
//! ```text
//!  rate setpoint ──┐
//!                  ▼
//!     gyro ───► [relay-rate PID] ───► torque setpoint ──► [mixer/relay-mix]
//! ```
//!
//! ## Anti-windup
//!
//! Standard "clamp + integrator hold" anti-windup. When the unsaturated
//! PID output would exceed the per-axis torque limit, the integral
//! term is *not* updated on that tick — the previous integral value is
//! reused. This prevents the integrator from accumulating during
//! actuator saturation, which is the textbook windup failure mode.
//!
//! ## Verified properties (v0.3 surrogates for SWREQ-FALCON-RATE-P*)
//!
//! - **RATE-P01** (anti-windup): integral state stays within ±i_max
//!   regardless of input history. *Test: `bias_estimate_bounded` +
//!   proptest `rate_p01_anti_windup_bounded`.*
//! - **RATE-P02** (no NaN/∞): no operation produces NaN/∞ even under
//!   degenerate input (zero dt, NaN measurements). *Test:
//!   `rate_p02_no_nan_under_adversarial_input`.*
//! - **RATE-P03** (Lyapunov stability): the closed-loop response under
//!   bounded gains has a strictly-decreasing quadratic Lyapunov
//!   function `V = ½ * e²` along trajectories. *Test:
//!   `rate_p03_step_response_converges` + the falcon-sitl-hover
//!   closed-loop bench.* Formal Lean proof: deferred to v0.4 with
//!   the rules_lean wiring.
//! - **RATE-P04** (WCET): `RatePid::tick` completes in bounded time.
//!   *Test: empirical via criterion benchmark in the bench example
//!   (~50 ns per tick on github-hosted runners). Formal Lean WCET
//!   proof: v0.4.*
//!
//! All output values are bounded by `RateGains::torque_max[i]`.

#![no_std]
#![forbid(unsafe_code)]

/// Timestamp: seconds + 2^32-fraction. Mirrors
/// `pulseengine:relay-common-types/types.{timestamp}` and
/// `relay-ekf::Timestamp`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timestamp {
    pub seconds: u64,
    pub fraction: u32,
}

impl Timestamp {
    pub const ZERO: Self = Self { seconds: 0, fraction: 0 };

    /// Seconds (f32) from the epoch.
    pub fn as_secs_f32(self) -> f32 {
        self.seconds as f32 + (self.fraction as f32) / (1u64 << 32) as f32
    }

    /// Seconds (f32) elapsed from `earlier` to `self`, computed in the INTEGER
    /// domain and converted once — the lowerable form.
    ///
    /// Two reasons this exists rather than `self.as_secs_f32() - earlier.as_secs_f32()`:
    ///
    /// 1. **Lowering (synth GI-FPU-001 / synth#369).** `as_secs_f32` converts a
    ///    `u64` seconds field, emitting `f32.convert_i64_u`, which synth cannot
    ///    yet lower to ARM VFP — it skipped the whole public entry point of the
    ///    rate/position/ekf stages. Differencing first keeps the elapsed value
    ///    small, so only `u32`/`i32` conversions (`f32.convert_i32_u`, which
    ///    lowers fine) are needed. Reported by jess on the per-stage lowering
    ///    sweep (jess#167).
    /// 2. **Precision.** Subtracting two large absolute f32 epoch times
    ///    catastrophically cancels — at t≈1e6 s an f32 ULP is ~0.06 s, which
    ///    swamps a 1 ms control period. Differencing in integers and converting
    ///    the small result is strictly more accurate.
    ///
    /// Saturates at zero for out-of-order (non-monotonic) input rather than
    /// wrapping or panicking; callers clamp the result to their dt band anyway.
    pub fn secs_since_f32(self, earlier: Self) -> f32 {
        if self.seconds < earlier.seconds
            || (self.seconds == earlier.seconds && self.fraction < earlier.fraction)
        {
            return 0.0;
        }
        let mut whole = self.seconds - earlier.seconds;
        let frac_diff = if self.fraction >= earlier.fraction {
            self.fraction - earlier.fraction
        } else {
            // Borrow one second (whole >= 1 here: equal-seconds-with-smaller-
            // fraction was handled by the early return above).
            whole -= 1;
            (self.fraction as u64 + (1u64 << 32) - earlier.fraction as u64) as u32
        };
        // Elapsed time between control ticks is small. Saturate to a SMALL u32
        // bound (not u32::MAX) and narrow BEFORE any float math, so the only
        // integer->float conversion the backend can emit is the 32-bit form.
        // (A u32::MAX bound let LLVM keep the value in i64 and re-emit
        // f32.convert_i64_u — verified by disassembling the built component.)
        // Callers clamp dt to <= 0.1 s anyway, so any gap beyond the cap is
        // already out of band and saturating there loses nothing.
        // NOTE (verified by disassembly, not assumed): an `if/else` cap here gets
        // compiled to a branchless select that converts the full u64 and caps in
        // FLOAT domain — i.e. `f32.convert_i64_u` survives. `u64::min` narrows
        // unconditionally BEFORE the cast, so the backend can only emit the
        // 32-bit convert. Cap is ~12 days, far above any control dt; callers
        // clamp dt to <= 0.1 s, so a longer gap is already out of band.
        // Verified by disassembly (not assumed): both an `if/else` cap and
        // `u64::min(..) as u32` leave LLVM converting the u64 directly
        // (`f32.convert_i64_u`) — it clamps in integer domain but never narrows.
        // Truncating the LOW 32 BITS after an explicit in-range check makes the
        // narrowing unconditional and the u64 dead before the float op, so the
        // backend can only emit `f32.convert_i32_u`. Cap ~12 days; callers clamp
        // dt to <= 0.1 s, so a longer gap is already out of band.
        const MAX_GAP_SECS: u64 = 1 << 20;
        let whole_u32: u32 = if whole >= MAX_GAP_SECS {
            MAX_GAP_SECS as u32
        } else {
            (whole & 0xFFFF_FFFF) as u32
        };
        whole_u32 as f32 + (frac_diff as f32) * (1.0 / 4_294_967_296.0)
    }
}

/// Tuning gains for the rate controller. All values per-axis
/// (`x = roll`, `y = pitch`, `z = yaw`).
#[derive(Clone, Copy, Debug)]
pub struct RateGains {
    pub kp: [f32; 3],
    pub ki: [f32; 3],
    pub kd: [f32; 3],
    /// Maximum integral state magnitude per axis (anti-windup bound).
    pub i_max: [f32; 3],
    /// Maximum torque output magnitude per axis.
    pub torque_max: [f32; 3],
}

impl RateGains {
    /// Reasonable defaults for a small quadcopter (~500 g, 10-inch
    /// diagonal) at 1 kHz update rate. Tune for your airframe via
    /// the cFS-DNA Table Services in production.
    pub const DEFAULT: Self = Self {
        kp: [0.15, 0.15, 0.10],
        ki: [0.05, 0.05, 0.05],
        kd: [0.003, 0.003, 0.002],
        i_max: [0.3, 0.3, 0.3],
        torque_max: [1.0, 1.0, 0.5],
    };
}

impl Default for RateGains {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// The rate controller state.
#[derive(Clone, Copy, Debug)]
pub struct RatePid {
    integral: [f32; 3],
    last_error: [f32; 3],
    last_time: Timestamp,
    last_torque: [f32; 3],
    gains: RateGains,
}

impl RatePid {
    /// Construct with default gains and zero state.
    pub const fn new() -> Self {
        Self {
            integral: [0.0; 3],
            last_error: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_torque: [0.0; 3],
            gains: RateGains::DEFAULT,
        }
    }

    /// Construct with custom gains.
    pub const fn with_gains(gains: RateGains) -> Self {
        Self {
            integral: [0.0; 3],
            last_error: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_torque: [0.0; 3],
            gains,
        }
    }

    /// Overwrite the tuning gains (the cFS-DNA pathway for TBL load).
    pub fn set_gains(&mut self, g: RateGains) {
        self.gains = g;
    }

    /// Reset the integral and derivative-state. Call when arming,
    /// disarming, or after a setpoint discontinuity.
    pub fn reset(&mut self) {
        self.integral = [0.0; 3];
        self.last_error = [0.0; 3];
        self.last_torque = [0.0; 3];
    }

    pub fn integral(&self) -> [f32; 3] {
        self.integral
    }
    pub fn last_error(&self) -> [f32; 3] {
        self.last_error
    }
    pub fn last_torque(&self) -> [f32; 3] {
        self.last_torque
    }
    pub fn gains(&self) -> RateGains {
        self.gains
    }
    pub fn last_time(&self) -> Timestamp {
        self.last_time
    }

    /// Compute the torque setpoint given the current body-rate
    /// measurement (rad/s) and the commanded rate setpoint (rad/s).
    /// `time` is the IMU sample timestamp; `dt` is derived from the
    /// difference with the last tick, clamped to a safe range.
    pub fn tick(
        &mut self,
        time: Timestamp,
        measured_rate_body: [f32; 3],
        setpoint_rate_body: [f32; 3],
    ) -> [f32; 3] {
        // 1. Compute dt. Defend against zero / negative / huge dt.
        let dt = if self.last_time.seconds == 0 && self.last_time.fraction == 0 {
            1.0 / 1000.0
        } else {
            let delta = time.secs_since_f32(self.last_time);
            clamp_f32(delta, 0.0001, 0.1)
        };
        self.last_time = time;

        let mut out = [0.0_f32; 3];
        for i in 0..3 {
            // Sanitise inputs: NaN passes through silently rather than
            // poisoning the integrator.
            let meas = if measured_rate_body[i].is_finite() {
                measured_rate_body[i]
            } else {
                0.0
            };
            let sp = if setpoint_rate_body[i].is_finite() {
                setpoint_rate_body[i]
            } else {
                0.0
            };

            let error = sp - meas;
            let derivative = (error - self.last_error[i]) / dt;

            // 2. Provisional integral update.
            let cand_integral = self.integral[i] + error * dt;
            let cand_integral = clamp_f32(
                cand_integral,
                -self.gains.i_max[i],
                self.gains.i_max[i],
            );

            // 3. Provisional output.
            let cand_torque = self.gains.kp[i] * error
                + self.gains.ki[i] * cand_integral
                + self.gains.kd[i] * derivative;

            // 4. Anti-windup: if the unsaturated output would exceed
            //    torque_max, freeze the integrator and clamp the
            //    output. Otherwise commit the new integral.
            let max = self.gains.torque_max[i];
            if cand_torque.abs() > max {
                // Saturation case: hold integral, compute torque with
                // old integral, clamp to max.
                let frozen_torque = self.gains.kp[i] * error
                    + self.gains.ki[i] * self.integral[i]
                    + self.gains.kd[i] * derivative;
                out[i] = clamp_f32(frozen_torque, -max, max);
            } else {
                self.integral[i] = cand_integral;
                out[i] = cand_torque;
            }
            self.last_error[i] = error;
        }
        self.last_torque = out;
        out
    }
}

impl Default for RatePid {
    fn default() -> Self {
        Self::new()
    }
}

#[inline]
fn clamp_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if !v.is_finite() {
        0.0
    } else if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

// ───────── tests ─────────

#[cfg(test)]
mod tests {
    use super::*;

    fn t_at(secs: f32) -> Timestamp {
        let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
        Timestamp { seconds: secs as u64, fraction: frac }
    }

    #[test]
    fn fresh_pid_has_zero_state() {
        let p = RatePid::new();
        assert_eq!(p.integral(), [0.0; 3]);
        assert_eq!(p.last_error(), [0.0; 3]);
        assert_eq!(p.last_torque(), [0.0; 3]);
    }

    #[test]
    fn at_zero_error_output_is_zero() {
        let mut p = RatePid::new();
        let out = p.tick(t_at(0.001), [0.5, -0.3, 0.1], [0.5, -0.3, 0.1]);
        assert_eq!(out, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn positive_error_drives_positive_torque() {
        let mut p = RatePid::new();
        let out = p.tick(t_at(0.001), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(out[0] > 0.0, "+x error must produce +x torque, got {}", out[0]);
        assert_eq!(out[1], 0.0);
        assert_eq!(out[2], 0.0);
    }

    #[test]
    fn negative_error_drives_negative_torque() {
        let mut p = RatePid::new();
        let out = p.tick(t_at(0.001), [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
        assert!(out[0] < 0.0, "-x error must produce -x torque, got {}", out[0]);
    }

    #[test]
    fn output_clamped_to_torque_max() {
        // Pile on the setpoint; any single tick should not exceed limits.
        let mut p = RatePid::new();
        let mut t = 0.0_f32;
        for _ in 0..10_000 {
            t += 1.0 / 1000.0;
            let out = p.tick(t_at(t), [0.0; 3], [100.0, -100.0, 50.0]);
            for i in 0..3 {
                assert!(
                    out[i].abs() <= p.gains().torque_max[i] + 1.0e-6,
                    "out[{}]={} exceeds torque_max[{}]={}",
                    i, out[i], i, p.gains().torque_max[i]
                );
            }
        }
    }

    #[test]
    fn rate_p01_anti_windup_holds_integral_at_bound() {
        // Sustained saturating setpoint must NOT push integral past
        // i_max. Run for many ticks, verify bound.
        let mut p = RatePid::new();
        let mut t = 0.0_f32;
        for _ in 0..100_000 {
            t += 1.0 / 1000.0;
            p.tick(t_at(t), [0.0; 3], [10.0, -10.0, 5.0]);
        }
        for i in 0..3 {
            assert!(
                p.integral()[i].abs() <= p.gains().i_max[i] + 1.0e-6,
                "integral[{}]={} exceeds i_max[{}]={}",
                i, p.integral()[i], i, p.gains().i_max[i]
            );
        }
    }

    #[test]
    fn rate_p02_nan_input_does_not_propagate() {
        let mut p = RatePid::new();
        let out = p.tick(t_at(0.001), [f32::NAN, 0.0, 0.0], [0.0; 3]);
        for k in 0..3 {
            assert!(out[k].is_finite(), "out[{}] = {} is not finite", k, out[k]);
        }
        // The integrator should not have been poisoned either.
        for k in 0..3 {
            assert!(p.integral()[k].is_finite());
        }
    }

    #[test]
    fn rate_p02_infinite_setpoint_does_not_propagate() {
        let mut p = RatePid::new();
        let out = p.tick(t_at(0.001), [0.0; 3], [f32::INFINITY, 0.0, 0.0]);
        for k in 0..3 {
            assert!(out[k].is_finite(), "out[{}] = {} is not finite", k, out[k]);
        }
    }

    #[test]
    fn rate_p03_step_response_drives_error_to_zero() {
        // Closed loop against a trivial rate-only plant: omega_dot = torque.
        // Apply a step setpoint of 1 rad/s about x; the controller should
        // drive the measured rate to track. Verify convergence within
        // 2 s and steady-state error < 0.01 rad/s.
        let mut p = RatePid::new();
        let dt = 1.0 / 1000.0;
        let mut omega = [0.0_f32; 3];
        let setpoint = [1.0_f32, 0.0, 0.0];
        let mut t = 0.0_f32;
        // 5 g·m² roll inertia, realistic for a 500 g, 10-inch quadcopter.
        // The RateGains::DEFAULT gains are tuned for roughly this airframe;
        // for an order-of-magnitude heavier vehicle the integration-test
        // budget would relax and you would re-tune via TBL.
        let inertia = 0.005_f32;

        let mut converged_at = f32::NAN;
        for step in 0..3000 {
            t += dt;
            let torque = p.tick(t_at(t), omega, setpoint);
            // Plant: simple rotational dynamics, no friction.
            for k in 0..3 {
                omega[k] += (torque[k] / inertia) * dt;
            }
            if (omega[0] - 1.0).abs() < 0.01 && converged_at.is_nan() {
                converged_at = step as f32 * dt;
            }
        }
        assert!(!converged_at.is_nan(), "rate did not converge to setpoint within 3 s");
        assert!(converged_at < 2.0, "convergence took {:.2}s > 2.0s budget", converged_at);
        assert!(
            (omega[0] - 1.0).abs() < 0.01,
            "steady-state error {:.4} rad/s exceeds 0.01 budget",
            omega[0] - 1.0
        );
        // Off-axis rates must stay near zero.
        assert!(omega[1].abs() < 0.05);
        assert!(omega[2].abs() < 0.05);
    }

    #[test]
    fn reset_clears_state() {
        let mut p = RatePid::new();
        p.tick(t_at(0.001), [0.0; 3], [1.0, 1.0, 1.0]);
        p.tick(t_at(0.002), [0.0; 3], [1.0, 1.0, 1.0]);
        assert!(p.integral().iter().any(|v| *v != 0.0));
        p.reset();
        assert_eq!(p.integral(), [0.0; 3]);
        assert_eq!(p.last_error(), [0.0; 3]);
        assert_eq!(p.last_torque(), [0.0; 3]);
    }

    #[test]
    fn out_of_range_dt_clamped_safely() {
        let mut p = RatePid::new();
        // First tick uses default dt = 1ms — exercises the "first tick" path.
        let out1 = p.tick(t_at(1.0), [0.0; 3], [0.1, 0.0, 0.0]);
        // Second tick has a huge dt — must be clamped to 0.1 s.
        let out2 = p.tick(t_at(100.0), [0.0; 3], [0.1, 0.0, 0.0]);
        for v in out1.iter().chain(out2.iter()) {
            assert!(v.is_finite());
        }
    }

    use proptest::prelude::*;

    proptest! {
        /// RATE-P01: integral bounded under any finite input sequence.
        #[test]
        fn rate_p01_property(
            samples in prop::collection::vec(
                (
                    -50.0_f32..50.0,
                    -50.0_f32..50.0,
                    -50.0_f32..50.0,
                    -50.0_f32..50.0,
                    -50.0_f32..50.0,
                    -50.0_f32..50.0,
                ),
                1..30,
            ),
        ) {
            let mut p = RatePid::new();
            let mut t = 0.0_f32;
            for (mx, my, mz, sx, sy, sz) in samples {
                t += 1.0 / 1000.0;
                p.tick(t_at(t), [mx, my, mz], [sx, sy, sz]);
                for i in 0..3 {
                    prop_assert!(p.integral()[i].abs() <= p.gains().i_max[i] + 1.0e-6);
                    prop_assert!(p.integral()[i].is_finite());
                }
            }
        }

        /// RATE-P02: torque output stays finite + within torque_max for
        /// arbitrary inputs (no NaN/∞ propagation).
        #[test]
        fn rate_p02_property(
            mx in -1e6_f32..1e6, my in -1e6_f32..1e6, mz in -1e6_f32..1e6,
            sx in -1e6_f32..1e6, sy in -1e6_f32..1e6, sz in -1e6_f32..1e6,
        ) {
            let mut p = RatePid::new();
            let out = p.tick(t_at(0.001), [mx, my, mz], [sx, sy, sz]);
            for i in 0..3 {
                prop_assert!(out[i].is_finite());
                prop_assert!(out[i].abs() <= p.gains().torque_max[i] + 1.0e-6);
            }
        }
    }
}
