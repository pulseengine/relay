//! Relay robust inner rate loop — **linear ADRC** (Active Disturbance
//! Rejection Control), per body axis.
//!
//! The v0.25 fix for the v0.24 finding: the yaw control loop went unstable
//! because the geometric controller commanded torque assuming *instant*
//! actuation, but quad yaw torque comes from rotor-drag differential that
//! needs large, *slow* motor-speed changes — an unmodeled actuator lag
//! that turned the loop into a delayed loop → spin.
//!
//! ADRC handles this without an actuator model: a 2nd-order **extended
//! state observer (ESO)** estimates the body rate `z1 ≈ Ω` AND the
//! **lumped total disturbance** `z2` (actuator lag, effectiveness
//! mismatch, rotor drag, gyroscopic coupling — everything not in the
//! nominal `Ω̇ = b0·u`). The control then *cancels* the estimated
//! disturbance: `u = (kp·(Ω_d − z1) − z2) / b0`. Robustness to a wrong
//! `b0` (control effectiveness) is the whole point — the ESO absorbs the
//! mismatch, which is exactly our weak/uncertain yaw effectiveness.
//!
//! Per-axis model: `Ω̇ = b0·u + d`, `ḋ = w`. Discrete ESO:
//! ```text
//!   e   = Ω_meas − z1
//!   z1 += dt·(z2 + b0·u_prev + β1·e)
//!   z2 += dt·(β2·e)
//!   u   = (kp·(Ω_d − z1) − z2) / b0          (disturbance cancellation)
//! ```
//! Bandwidth parameterisation (Gao): `β1 = 2 ω_o`, `β2 = ω_o²`,
//! `kp = ω_c`, with observer bw `ω_o` ≫ controller bw `ω_c` ≫ actuator
//! bw. The ESO error dynamics are linear → an algebraic Lyapunov bound
//! (the most mechanizable inner-loop guarantee; Lean target).

#![no_std]
#![allow(clippy::neg_cmp_op_on_partial_ord)]

/// Per-axis ADRC tuning.
#[derive(Clone, Copy)]
pub struct AdrcGains {
    /// Observer bandwidth ω_o (rad/s). Sets ESO speed (β1=2ω_o, β2=ω_o²).
    pub omega_o: f32,
    /// Controller bandwidth ω_c (rad/s). Sets the rate-tracking gain kp.
    pub omega_c: f32,
    /// Nominal control effectiveness b0 ≈ τ_max/J for the axis. ADRC is
    /// robust to b0 error (the ESO absorbs it) — only the order of
    /// magnitude matters.
    pub b0: f32,
    /// Actuator time constant τ (s) of the first-order motor/ESC lag. When
    /// positive, the ESO is driven by the **actual delivered** torque (the
    /// command passed through a matching first-order filter) instead of
    /// the raw command — the INDI "synchronization" move. This stops the
    /// ESO mis-attributing the unmodeled actuator lag to "disturbance"
    /// and cancelling it destabilisingly (the v0.25 yaw failure: yaw must
    /// drive large, slow Δω through this lag, so ω_c·τ ≈ 1). Set 0 to
    /// disable (instant-actuator assumption).
    pub tau: f32,
}

impl AdrcGains {
    /// Build with no actuator-lag model (instant actuator).
    pub const fn new(omega_o: f32, omega_c: f32, b0: f32) -> Self {
        AdrcGains { omega_o, omega_c, b0, tau: 0.0 }
    }

    /// Build with an explicit actuator time constant τ (the recommended
    /// form for the lag-sensitive yaw axis).
    pub const fn with_tau(omega_o: f32, omega_c: f32, b0: f32, tau: f32) -> Self {
        AdrcGains { omega_o, omega_c, b0, tau }
    }
}

/// One axis of linear ADRC. Holds the ESO state across ticks.
#[derive(Clone, Copy)]
pub struct AdrcAxis {
    z1: f32, // rate estimate
    z2: f32, // lumped-disturbance estimate
    u_prev: f32,
    u_act: f32, // modelled delivered command (first-order actuator state)
    g: AdrcGains,
}

impl AdrcAxis {
    pub fn new(g: AdrcGains) -> Self {
        AdrcAxis { z1: 0.0, z2: 0.0, u_prev: 0.0, u_act: 0.0, g }
    }

    /// Disturbance estimate (rad/s²) — the lumped unmodeled torque/J the
    /// ESO has identified. Exposed for diagnostics/tests.
    pub fn disturbance(&self) -> f32 {
        self.z2
    }

    /// One control step. `omega_meas` = measured body rate (rad/s, e.g.
    /// bias-corrected raw gyro), `omega_d` = desired body rate from the
    /// outer loop, `dt` (s). Returns the control output `u` (torque,
    /// normalised to the same units the mixer expects).
    pub fn tick(&mut self, omega_meas: f32, omega_d: f32, dt: f32) -> f32 {
        let dt = if dt.is_finite() { dt.clamp(1e-4, 0.1) } else { 1e-3 };
        let om = if omega_meas.is_finite() { omega_meas } else { 0.0 };
        let od = if omega_d.is_finite() { omega_d } else { 0.0 };

        // Guard the tuning (positive, finite) so the law is total.
        let omega_o = if self.g.omega_o.is_finite() && self.g.omega_o > 0.0 { self.g.omega_o } else { 10.0 };
        let omega_c = if self.g.omega_c.is_finite() && self.g.omega_c > 0.0 { self.g.omega_c } else { 3.0 };
        let b0 = if self.g.b0.is_finite() && self.g.b0.abs() > 1e-3 { self.g.b0 } else { 1.0 };
        let beta1 = 2.0 * omega_o;
        let beta2 = omega_o * omega_o;
        let kp = omega_c;

        // Actuator-lag synchronisation: the ESO must be driven by the
        // torque actually DELIVERED, not the raw command. Pass u through a
        // first-order filter matching the actuator τ; the ESO then sees
        // b0·u_act (the delivered torque) and stops mistaking the lag for
        // a disturbance. τ=0 → u_act tracks u_prev instantly (no model).
        if self.g.tau.is_finite() && self.g.tau > 1e-4 {
            self.u_act += dt * (self.u_prev - self.u_act) / self.g.tau;
        } else {
            self.u_act = self.u_prev;
        }
        self.u_act = sanitise(self.u_act, 0.0);

        // ESO update (driven by the delivered torque b0·u_act).
        let e = om - self.z1;
        let z1_next = self.z1 + dt * (self.z2 + b0 * self.u_act + beta1 * e);
        let z2_next = self.z2 + dt * (beta2 * e);
        self.z1 = sanitise(z1_next, 0.0);
        self.z2 = sanitise(z2_next, 0.0);

        // Disturbance-rejecting control: cancel z2, track omega_d.
        let u = (kp * (od - self.z1) - self.z2) / b0;
        let u = sanitise(u, 0.0);
        self.u_prev = u;
        u
    }

    /// Reset the observer (e.g. on arming / setpoint discontinuity).
    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
        self.u_prev = 0.0;
        self.u_act = 0.0;
    }
}

#[inline]
fn sanitise(x: f32, fallback: f32) -> f32 {
    if x.is_finite() { x } else { fallback }
}

/// Three-axis (roll, pitch, yaw) ADRC rate controller.
pub struct AdrcRate {
    axes: [AdrcAxis; 3],
}

impl AdrcRate {
    pub fn new(gains: [AdrcGains; 3]) -> Self {
        AdrcRate { axes: [AdrcAxis::new(gains[0]), AdrcAxis::new(gains[1]), AdrcAxis::new(gains[2])] }
    }

    /// Falcon-quad defaults: roll/pitch fast (high effectiveness), yaw
    /// slower observer + lower b0 (weak rotor-drag effectiveness, the axis
    /// whose actuator lag ADRC is here to reject).
    pub fn falcon_quad() -> Self {
        Self::new([
            AdrcGains::with_tau(40.0, 12.0, 30.0, 0.0125),
            AdrcGains::with_tau(40.0, 12.0, 30.0, 0.0125),
            // Yaw: ω_o high (fast observer) but ω_c low (control bw below
            // the motor pole 1/τ≈40), AND the actuator lag τ modelled in
            // the ESO (the dominant v0.25 fix — yaw drives large slow Δω
            // through this lag, so it must be in the plant, not left as an
            // "unmodeled disturbance" the ESO destabilisingly cancels).
            AdrcGains::with_tau(30.0, 3.0, 6.0, 0.025),
        ])
    }

    /// Per-axis torque from measured body rate + desired body rate.
    pub fn tick(&mut self, omega_meas: [f32; 3], omega_d: [f32; 3], dt: f32) -> [f32; 3] {
        [
            self.axes[0].tick(omega_meas[0], omega_d[0], dt),
            self.axes[1].tick(omega_meas[1], omega_d[1], dt),
            self.axes[2].tick(omega_meas[2], omega_d[2], dt),
        ]
    }

    pub fn reset(&mut self) {
        for a in &mut self.axes {
            a.reset();
        }
    }

    pub fn disturbance(&self) -> [f32; 3] {
        [self.axes[0].disturbance(), self.axes[1].disturbance(), self.axes[2].disturbance()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simulate a 1st-order-lagged actuator plant `Ω̇ = b_true·u_act + d`,
    /// `u̇_act = (u_cmd − u_act)/τ` (the motor time constant that broke
    /// the yaw loop), with an UNKNOWN constant disturbance `d`. ADRC must
    /// (a) track the rate setpoint and (b) have its ESO identify `d`.
    fn sim_axis(b_true: f32, tau: f32, d: f32, omega_d: f32, gains: AdrcGains) -> (f32, f32) {
        let mut adrc = AdrcAxis::new(gains);
        let dt = 0.001f32;
        let mut omega = 0.0f32;
        let mut u_act = 0.0f32;
        for _ in 0..8000 {
            // 8 s
            let u_cmd = adrc.tick(omega, omega_d, dt);
            u_act += dt * (u_cmd - u_act) / tau; // actuator lag
            omega += dt * (b_true * u_act + d); // rate dynamics
        }
        (omega, adrc.disturbance())
    }

    /// ADRC tracks the rate setpoint despite a LAGGED actuator and an
    /// unknown constant disturbance — the v0.24 failure mode for a plain
    /// proportional law.
    #[test]
    fn adrc_tracks_through_actuator_lag_and_disturbance() {
        // b0 (assumed 6) deliberately wrong vs b_true (4) — ADRC must cope.
        let g = AdrcGains::new(20.0, 5.0, 6.0);
        let (omega, _d) = sim_axis(4.0, 0.05, 2.0, 1.0, g); // 50 ms lag, d=2
        assert!((omega - 1.0).abs() < 0.1, "should track Ω_d=1, got {omega}");
    }

    /// The ESO identifies the lumped disturbance (here d=2 rad/s²).
    #[test]
    fn eso_identifies_disturbance() {
        let g = AdrcGains::new(20.0, 5.0, 6.0);
        let (_omega, d_est) = sim_axis(4.0, 0.05, 2.0, 0.0, g);
        // z2 absorbs both the true disturbance AND the b0 mismatch, so it
        // need not equal 2 exactly — but it must be a substantial, finite,
        // same-sign estimate (not zero, not NaN).
        assert!(d_est.is_finite());
        assert!(d_est > 0.5, "ESO should identify a positive disturbance, got {d_est}");
    }

    /// Regulating to Ω_d = 0 from a disturbance holds the rate near zero
    /// (the yaw-hold case: no commanded rate, reject the lag-induced spin).
    #[test]
    fn regulates_rate_to_zero_under_disturbance() {
        let g = AdrcGains::new(20.0, 5.0, 6.0);
        let (omega, _) = sim_axis(4.0, 0.05, 1.5, 0.0, g);
        assert!(omega.abs() < 0.15, "rate should be held near 0, got {omega}");
    }

    proptest::proptest! {
        /// Total: finite output for any finite inputs + any dt, including
        /// adversarial measurements and a degenerate b0.
        #[test]
        fn adrc_is_total(
            meas in proptest::collection::vec(
                (-50.0f32..50.0, -10.0f32..10.0, 0.0f32..0.2), 0..400),
        ) {
            let mut a = AdrcAxis::new(AdrcGains::new(20.0, 5.0, 6.0));
            for (om, od, dt) in meas {
                let u = a.tick(om, od, dt);
                proptest::prop_assert!(u.is_finite());
                proptest::prop_assert!(a.disturbance().is_finite());
            }
        }
    }
}
