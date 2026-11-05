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
        AdrcGains {
            omega_o,
            omega_c,
            b0,
            tau: 0.0,
        }
    }

    /// Build with an explicit actuator time constant τ (the recommended
    /// form for the lag-sensitive yaw axis).
    pub const fn with_tau(omega_o: f32, omega_c: f32, b0: f32, tau: f32) -> Self {
        AdrcGains {
            omega_o,
            omega_c,
            b0,
            tau,
        }
    }

    /// Bandwidth-parameterized construction (Gao 2003): pick the controller
    /// bandwidth ω_c and derive the observer bandwidth from the **separation
    /// ratio** ω_o = separation·ω_c. The standard ADRC rule is
    /// separation ∈ [3, 10]: the ESO must be several times faster than the
    /// controller so its estimate has settled before the controller acts,
    /// yet not so fast it sits at the delay-limited crossover and starves
    /// phase margin. This constructor makes [`well_separated`] true by
    /// construction for any `min_ratio ≤ separation`.
    pub fn from_bandwidth(omega_c: f32, separation: f32, b0: f32, tau: f32) -> Self {
        AdrcGains {
            omega_o: separation * omega_c,
            omega_c,
            b0,
            tau,
        }
    }

    /// Observer/controller timescale-separation invariant: ω_o ≥ ratio·ω_c.
    /// Division-free (Kani-friendly). The v0.33 robustness certificate —
    /// an under-separated ESO (ω_o ≈ ω_c) is the classic marginal-stability
    /// tuning; this predicate gates against it.
    #[inline]
    pub fn well_separated(&self, min_ratio: f32) -> bool {
        self.omega_o.is_finite()
            && self.omega_c.is_finite()
            && self.omega_c > 0.0
            && self.omega_o >= min_ratio * self.omega_c
    }

    /// Forward-Euler ESO discrete-stability bound. The 2nd-order observer has
    /// a continuous double pole at −ω_o; forward Euler maps it to a discrete
    /// double pole at 1 − ω_o·dt, inside the unit circle iff 0 < ω_o·dt < 2.
    /// Division-free. At ω_o = 40, dt = 1 ms this holds with a ~50× margin —
    /// i.e. the inner ESO is NOT the cadence-fragile element (that is the
    /// outer position→attitude cascade). A `margin` < 1 leaves headroom.
    #[inline]
    pub fn eso_dt_stable(&self, dt: f32, margin: f32) -> bool {
        self.omega_o.is_finite() && dt.is_finite() && dt > 0.0 && self.omega_o * dt < 2.0 * margin
    }
}

/// One axis of linear ADRC. Holds the ESO state across ticks.
#[derive(Clone, Copy)]
pub struct AdrcAxis {
    z1: f32, // rate estimate
    z2: f32, // lumped-disturbance estimate
    u_prev: f32,
    u_act: f32, // modelled delivered command (first-order actuator state)
    /// Fraction of `u_prev` the ACTUATOR ALLOCATION actually delivered, in
    /// [0, 1]; 1.0 until told otherwise, so an allocator that never reports
    /// leaves behaviour bit-identical to before this field existed.
    ///
    /// Distinct from `u_act`, which models the actuator's LAG. Lag says the
    /// torque arrives late; this says part of it never arrives at all,
    /// because a saturated mixer scaled it away (#270).
    u_delivered_frac: f32,
    g: AdrcGains,
}

impl AdrcAxis {
    pub fn new(g: AdrcGains) -> Self {
        AdrcAxis {
            z1: 0.0,
            z2: 0.0,
            u_prev: 0.0,
            u_act: 0.0,
            u_delivered_frac: 1.0,
            g,
        }
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
        let dt = if dt.is_finite() {
            dt.clamp(1e-4, 0.1)
        } else {
            1e-3
        };
        let om = if omega_meas.is_finite() {
            omega_meas
        } else {
            0.0
        };
        let od = if omega_d.is_finite() { omega_d } else { 0.0 };

        // Guard the tuning (positive, finite) so the law is total.
        let omega_o = if self.g.omega_o.is_finite() && self.g.omega_o > 0.0 {
            self.g.omega_o
        } else {
            10.0
        };
        let omega_c = if self.g.omega_c.is_finite() && self.g.omega_c > 0.0 {
            self.g.omega_c
        } else {
            3.0
        };
        let b0 = if self.g.b0.is_finite() && self.g.b0.abs() > 1e-3 {
            self.g.b0
        } else {
            1.0
        };
        let beta1 = 2.0 * omega_o;
        let beta2 = omega_o * omega_o;
        let kp = omega_c;

        // Actuator-lag synchronisation: the ESO must be driven by the
        // torque actually DELIVERED, not the raw command. Pass u through a
        // first-order filter matching the actuator τ; the ESO then sees
        // b0·u_act (the delivered torque) and stops mistaking the lag for
        // a disturbance. τ=0 → u_act tracks u_prev instantly (no model).
        //
        // ANTI-WINDUP (#270). `u_prev` is what we COMMANDED; the allocator may
        // have delivered only `frac * u_prev`. Driving the ESO with the
        // command under saturation makes it integrate a torque that was never
        // applied: the residual grows, z2 winds up at beta2 = omega_o^2, the
        // control answers with MORE torque, and the loop sustains its own
        // saturation. Measured on the gz plant as a rail-to-rail 3.5 Hz limit
        // cycle established within 1 s of takeoff, never recovering.
        //
        // `frac` is set by `set_delivered_fraction` AFTER the allocator has
        // run, so tick N uses tick N-1's fraction. That is correct and not an
        // off-by-one: the ESO is one tick behind by construction (it is told
        // what was applied, which is only knowable after application).
        let u_delivered = self.u_prev * self.u_delivered_frac;
        if self.g.tau.is_finite() && self.g.tau > 1e-4 {
            self.u_act += dt * (u_delivered - self.u_act) / self.g.tau;
        } else {
            self.u_act = u_delivered;
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
        self.u_delivered_frac = 1.0;
    }

    /// Tell the observer what fraction of the last commanded control the
    /// actuator allocation actually delivered (see `u_delivered_frac`).
    /// Out-of-range or non-finite input is clamped to [0, 1]; 1.0 restores
    /// the pre-#270 behaviour exactly.
    pub fn set_delivered_fraction(&mut self, frac: f32) {
        self.u_delivered_frac = if frac.is_finite() {
            frac.clamp(0.0, 1.0)
        } else {
            1.0
        };
    }
}

#[inline]
fn sanitise(x: f32, fallback: f32) -> f32 {
    if x.is_finite() { x } else { fallback }
}

/// 2nd-order (Butterworth) low-pass biquad, Direct Form I (the state is the
/// last two inputs and the last two outputs, `x1 x2 y1 y2`).
/// PX4 puts exactly this ahead of its rate loop (`IMU_GYRO_CUTOFF`): a
/// high-gain ESO that differentiates RAW gyro chases sensor/motor noise
/// straight into the actuators, so the inner loop needs a band-limited
/// input. The filter adds ~one-period latency, which the 1 kHz loop
/// affords (the actuator-lag analysis: ω_c must stay below 1/τ anyway).
#[derive(Clone, Copy)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
    /// A re-design into or out of this design REBUILDS with zeroed state
    /// instead of keeping it: set for pass-through and for the near-Nyquist
    /// band `fc >= LPF_KEEP_STATE_MAX_FC_FRAC * fs` (see that constant).
    rebuild_on_retune: bool,
}

/// A re-design KEEPS the filter state only when both designs have their
/// cutoff below this fraction of the sample rate; in the near-Nyquist band
/// `0.45 fs <= fc < 0.5 fs` (and for pass-through) it REBUILDS with zeroed
/// state, exactly as before the review of v1.140 (reviews of PR #644).
///
/// The band is still DESIGNED, as it always was: its Butterworth has a zero
/// at Nyquist, and at a 127.5-133.3 Hz loop that zero is the only thing that
/// attenuates the hover rotor line (~66.7 Hz) aliased to just below Nyquist
/// (the notch cannot engage there). Making the band pass-through (aca46b8)
/// removed it and a hover with rotor vibration diverged at 130 Hz. But its
/// poles sit near z = -1, so keeping a history recorded under another design
/// rang at Nyquist (36550af: Disarmed / Terminated at 120-124 Hz after a
/// pre-arm bump); rebuilding with zeroed state is what the parent flew.
pub const LPF_KEEP_STATE_MAX_FC_FRAC: f32 = 0.45;

/// The largest pole radius a designed low-pass may have. The RBJ design in
/// f32 put a pole ON or OUTSIDE the unit circle for fs in (120.00001,
/// 120.0114) Hz (|pole| up to 1.000244, f32 rounding of sin/cos near pi);
/// such a design is moved to a slightly lower cutoff until its poles are
/// inside this radius (`Biquad::lowpass`). The Nyquist zero is kept.
pub const LPF_MAX_POLE_RADIUS: f32 = 0.9999;

/// The RBJ Butterworth low-pass coefficients `(b0, b1, b2, a1, a2)`, f32,
/// exactly as designed before the review of PR #644.
fn rbj_lowpass(fc: f32, fs: f32) -> (f32, f32, f32, f32, f32) {
    let w0 = 2.0 * core::f32::consts::PI * fc / fs;
    let (sw, cw) = (relay_math::sinf(w0), relay_math::cosf(w0));
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let alpha = sw / (2.0 * q);
    let a0 = 1.0 + alpha;
    (
        ((1.0 - cw) * 0.5) / a0,
        (1.0 - cw) / a0,
        ((1.0 - cw) * 0.5) / a0,
        (-2.0 * cw) / a0,
        (1.0 - alpha) / a0,
    )
}

/// Both poles of `z^2 + a1 z + a2` strictly inside radius `r` (Jury's test
/// on the polynomial scaled to radius `r`; sqrt-free, evaluated in f64).
fn poles_inside(a1: f32, a2: f32, r: f32) -> bool {
    let r = r as f64;
    let (a1, a2) = (a1 as f64 / r, a2 as f64 / (r * r));
    a2.abs() < 1.0 && a1.abs() < 1.0 + a2
}

impl Biquad {
    /// Butterworth LPF at cutoff `fc` (Hz), sample rate `fs` (Hz). RBJ
    /// cookbook coefficients with Q = 1/√2. Degenerate args, or `fc >= fs/2`,
    /// give pass-through. The coefficients are bit-identical to the design
    /// before the review of PR #644 except where that design had a pole
    /// outside [`LPF_MAX_POLE_RADIUS`]: there the cutoff is lowered in steps
    /// of 1e-4 fs until it has not (b = k [1, 2, 1] in every design, so the
    /// zero at Nyquist is kept).
    pub fn lowpass(fc: f32, fs: f32) -> Self {
        if !(fc > 0.0) || !(fs > 0.0) || fc >= 0.5 * fs {
            return Self::passthrough();
        }
        let mut design_fc = fc;
        let mut c = rbj_lowpass(design_fc, fs);
        let mut k = 0;
        while !poles_inside(c.3, c.4, LPF_MAX_POLE_RADIUS) && k < 1000 {
            design_fc -= 1.0e-4 * fs;
            c = rbj_lowpass(design_fc, fs);
            k += 1;
        }
        Biquad {
            b0: c.0,
            b1: c.1,
            b2: c.2,
            a1: c.3,
            a2: c.4,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
            rebuild_on_retune: fc >= LPF_KEEP_STATE_MAX_FC_FRAC * fs,
        }
    }

    /// Re-design for cutoff `fc` at sample rate `fs` (the coefficients
    /// [`Biquad::lowpass`] would give, degenerate args included), KEEPING the
    /// Direct Form I history `x1 x2 y1 y2`. A re-design is a change of
    /// coefficients, not a restart: the last two inputs and outputs are
    /// physical signal values whatever the rate, so the next output is
    /// continuous with the signal instead of starting again from zero
    /// (review of v1.140, defect A: rebuilding with zeroed state on every
    /// jittered frame lost rate control).
    ///
    /// EXCEPT into or out of the near-Nyquist band or pass-through
    /// ([`LPF_KEEP_STATE_MAX_FC_FRAC`]): then it REBUILDS with zeroed state,
    /// exactly as `lowpass` builds it and as every re-design did before the
    /// review of v1.140 (reviews of PR #644).
    pub fn retune_lowpass(&mut self, fc: f32, fs: f32) {
        let d = Self::lowpass(fc, fs);
        if d.rebuild_on_retune || self.rebuild_on_retune {
            *self = d;
            return;
        }
        self.b0 = d.b0;
        self.b1 = d.b1;
        self.b2 = d.b2;
        self.a1 = d.a1;
        self.a2 = d.a2;
    }

    /// The design is the identity (no filtering).
    pub fn is_passthrough(&self) -> bool {
        self.b0 == 1.0 && self.b1 == 0.0 && self.b2 == 0.0 && self.a1 == 0.0 && self.a2 == 0.0
    }

    fn passthrough() -> Self {
        Biquad {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
            rebuild_on_retune: true,
        }
    }

    pub fn filter(&mut self, x: f32) -> f32 {
        let x = if x.is_finite() { x } else { 0.0 };
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        let y = if y.is_finite() { y } else { 0.0 };
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

/// Three independent LPF biquads — one per gyro axis.
#[derive(Clone, Copy)]
pub struct GyroLpf {
    axes: [Biquad; 3],
}

impl GyroLpf {
    /// Butterworth LPF at `fc` Hz on each axis; `fc <= 0` disables (pass-through).
    pub fn new(fc: f32, fs: f32) -> Self {
        let b = Biquad::lowpass(fc, fs);
        GyroLpf { axes: [b, b, b] }
    }

    /// Re-design every axis for `fc` at `fs`, keeping each axis's filter
    /// history ([`Biquad::retune_lowpass`]). `GyroLpf::new(fc, fs)` gives the
    /// same coefficients with the history zeroed.
    pub fn retune(&mut self, fc: f32, fs: f32) {
        for b in self.axes.iter_mut() {
            b.retune_lowpass(fc, fs);
        }
    }

    pub fn filter(&mut self, gyro: [f32; 3]) -> [f32; 3] {
        [
            self.axes[0].filter(gyro[0]),
            self.axes[1].filter(gyro[1]),
            self.axes[2].filter(gyro[2]),
        ]
    }
}

/// Three-axis (roll, pitch, yaw) ADRC rate controller.
pub struct AdrcRate {
    axes: [AdrcAxis; 3],
}

impl AdrcRate {
    pub fn new(gains: [AdrcGains; 3]) -> Self {
        AdrcRate {
            axes: [
                AdrcAxis::new(gains[0]),
                AdrcAxis::new(gains[1]),
                AdrcAxis::new(gains[2]),
            ],
        }
    }

    /// Falcon-quad defaults: roll/pitch fast (high effectiveness), yaw
    /// slower observer + lower b0 (weak rotor-drag effectiveness, the axis
    /// whose actuator lag ADRC is here to reject).
    pub fn falcon_quad() -> Self {
        Self::new([
            // b0 (control effectiveness ≈ τ_max/J) raised 30→60 for the gz
            // plant: the real gz falcon-quad is ~2× more responsive than the
            // analytic SimBackend the ADRC was matched to (motorConstant 8.55e-6
            // × arm 0.088 / J 0.0217 → higher α per unit torque), so b0=30 left
            // the rate-loop gain ~2× too high → tumble. The ESO absorbs the
            // residual b0 error for the analytic plant (46) too. v1.113.
            AdrcGains::with_tau(40.0, 12.0, 60.0, 0.0125),
            AdrcGains::with_tau(40.0, 12.0, 60.0, 0.0125),
            // Yaw: ω_o high (fast observer) but ω_c low (control bw below
            // the motor pole 1/τ≈40), AND the actuator lag τ modelled in
            // the ESO (the dominant v0.25 fix — yaw drives large slow Δω
            // through this lag, so it must be in the plant, not left as an
            // "unmodeled disturbance" the ESO destabilisingly cancels).
            AdrcGains::with_tau(30.0, 3.0, 18.0, 0.025),
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

    /// Report the fraction of the last commanded torque the ALLOCATOR actually
    /// delivered, per axis, so each observer integrates what was applied rather
    /// than what was asked for (#270).
    ///
    /// A saturating mixer scales the torque triple by one scalar `s`, so the
    /// same `s` is normally passed for all three axes — but the signature is
    /// per-axis because the axes are independent observers with different
    /// tuning (yaw runs `omega_c=3.0, tau=0.025` against roll/pitch's `12.0`
    /// and `0.0125`), and an allocator that desaturates in PRIORITY order
    /// (MIX-P06: thrust ≻ roll/pitch ≻ yaw) gives up yaw before roll/pitch.
    /// Passing one scalar to all three would then be a lie about the two axes
    /// that kept their authority.
    pub fn set_delivered_fraction(&mut self, frac: [f32; 3]) {
        self.axes[0].set_delivered_fraction(frac[0]);
        self.axes[1].set_delivered_fraction(frac[1]);
        self.axes[2].set_delivered_fraction(frac[2]);
    }

    pub fn disturbance(&self) -> [f32; 3] {
        [
            self.axes[0].disturbance(),
            self.axes[1].disturbance(),
            self.axes[2].disturbance(),
        ]
    }
}

/// Symmetric clamp that is total over all f32 (NaN/±∞ → fallback to the
/// bound). Split out so the saturation invariant is a pure comparison the
/// model checker can discharge WITHOUT reasoning about the filter's f32
/// arithmetic (the same trick the simplex shield uses).
#[inline]
fn clamp_sym(x: f32, m: f32) -> f32 {
    let x = if x.is_finite() { x } else { 0.0 };
    // m = +∞ disables the limit (pass-through); NaN / negative is degenerate
    // and pins to 0 (fail-safe). A finite m ≥ 0 clamps to [−m, m].
    if m.is_nan() || m < 0.0 {
        return 0.0;
    }
    if x > m {
        m
    } else if x < -m {
        -m
    } else {
        x
    }
}

/// 2nd-order critically-damped **command filter** (Farrell/Polycarpou
/// command-filtered backstepping) for the outer→inner interface.
///
/// The marginal-stability "control-margin wall" is a *cascade* timescale
/// violation: the outer position loop can step the desired body-rate ω_d
/// faster than the inner loop settles, so the inner loop chases a moving
/// target through its actuator lag and the closed cascade sits on the edge.
/// This filter bounds the *bandwidth* (and optionally the magnitude/rate) of
/// the command entering the inner loop, restoring separation. Critically
/// damped (ζ = 1) ⇒ no overshoot, so it never adds command energy.
///
/// State (y, ẏ); per step (continuous double pole at −ω_n):
///   ẏ += dt·(ω_n²·(u − y) − 2ω_n·ẏ);  y += dt·ẏ
/// with ẏ saturated to `max_rate` and y to `max_mag`. The saturation is the
/// mechanically-verified property (`clamp_sym`); BIBO/no-overshoot are
/// proptest'd.
#[derive(Clone, Copy)]
pub struct CommandFilter {
    omega_n: f32,
    max_mag: f32,
    max_rate: f32,
    y: f32,
    yd: f32,
}

impl CommandFilter {
    /// `omega_n` = filter bandwidth (rad/s) — set BELOW the inner controller
    /// bandwidth ω_c for separation. `max_mag`/`max_rate` saturate the
    /// command and its slew (use f32::INFINITY to disable a limit).
    pub fn new(omega_n: f32, max_mag: f32, max_rate: f32) -> Self {
        let omega_n = if omega_n.is_finite() && omega_n > 0.0 {
            omega_n
        } else {
            1.0
        };
        CommandFilter {
            omega_n,
            max_mag,
            max_rate,
            y: 0.0,
            yd: 0.0,
        }
    }

    /// Filter one sample of the raw command `u` over `dt` seconds; returns
    /// the smoothed, separation-bounded command.
    pub fn step(&mut self, u: f32, dt: f32) -> f32 {
        let dt = if dt.is_finite() {
            dt.clamp(1e-4, 0.1)
        } else {
            1e-3
        };
        let u = if u.is_finite() { u } else { 0.0 };
        // Sanitise state so the law is total even from a non-finite state.
        let y0 = if self.y.is_finite() { self.y } else { 0.0 };
        let yd0 = if self.yd.is_finite() { self.yd } else { 0.0 };
        let wn = self.omega_n;
        // 2nd-order critically-damped update (2ζω_n = 2ω_n).
        let yd_next = yd0 + dt * (wn * wn * (u - y0) - 2.0 * wn * yd0);
        self.yd = clamp_sym(yd_next, self.max_rate);
        let y_next = y0 + dt * self.yd;
        self.y = clamp_sym(y_next, self.max_mag);
        self.y
    }

    /// Filtered command value.
    pub fn value(&self) -> f32 {
        self.y
    }

    /// Filtered command rate (ẏ) — the command-filtered derivative the
    /// backstepping compensator would use; also feeds feedforward.
    pub fn rate(&self) -> f32 {
        self.yd
    }

    pub fn reset(&mut self) {
        self.y = 0.0;
        self.yd = 0.0;
    }
}

/// Three-axis command filter (roll/pitch/yaw desired-rate shaping).
#[derive(Clone, Copy)]
pub struct CommandFilter3 {
    axes: [CommandFilter; 3],
}

impl CommandFilter3 {
    pub fn new(omega_n: f32, max_mag: f32, max_rate: f32) -> Self {
        let f = CommandFilter::new(omega_n, max_mag, max_rate);
        CommandFilter3 { axes: [f, f, f] }
    }

    pub fn step(&mut self, u: [f32; 3], dt: f32) -> [f32; 3] {
        [
            self.axes[0].step(u[0], dt),
            self.axes[1].step(u[1], dt),
            self.axes[2].step(u[2], dt),
        ]
    }

    pub fn reset(&mut self) {
        for a in &mut self.axes {
            a.reset();
        }
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
        assert!(
            d_est > 0.5,
            "ESO should identify a positive disturbance, got {d_est}"
        );
    }

    /// The larger pole magnitude of `z^2 + a1 z + a2` (f64 on the f32
    /// coefficients actually designed).
    fn max_pole(b: &Biquad) -> f64 {
        let (a1, a2) = (b.a1 as f64, b.a2 as f64);
        let d = a1 * a1 - 4.0 * a2;
        if d < 0.0 {
            a2.abs().sqrt()
        } else {
            let r = d.sqrt();
            ((-a1 + r) / 2.0).abs().max(((-a1 - r) / 2.0).abs())
        }
    }

    /// **Every designed (non-pass-through) low-pass is stable** (review of
    /// PR #644). In f32 the 60 Hz Butterworth at fs in (120.00001, 120.0114)
    /// Hz put a pole at |z| up to 1.000244: an UNSTABLE filter, reachable by
    /// frame periods of 8.3325-8.3333 ms. Swept over fs 10-10000 Hz (0.01 Hz
    /// steps to 1 kHz, 0.5 Hz above) plus EVERY f32 value in [119.99,
    /// 120.05] and [133.0, 134.0] Hz (the state-keeping boundary): each
    /// design is either the exact pass-through, exactly where `fc >= fs/2`
    /// (as it always was), or has every pole strictly inside the unit
    /// circle; every design whose pre-review RBJ coefficients were already
    /// inside [`LPF_MAX_POLE_RADIUS`] is BIT-IDENTICAL to them, and every
    /// design keeps the Nyquist zero (b0 - b1 + b2 = 0, b1 = 2 b0).
    #[test]
    fn every_designed_lowpass_has_its_poles_inside_the_unit_circle() {
        extern crate std;
        use std::vec::Vec;
        let mut rates: Vec<f32> = Vec::new();
        let mut k = 0u32;
        loop {
            let fs = 10.0 + 0.01 * k as f32;
            if fs > 1000.0 {
                break;
            }
            rates.push(fs);
            k += 1;
        }
        let mut fs = 1000.0f32;
        while fs <= 10000.0 {
            rates.push(fs);
            fs += 0.5;
        }
        for (lo, hi) in [(119.99f32, 120.05f32), (133.0, 134.0)] {
            let mut x = lo;
            while x <= hi {
                rates.push(x);
                x = f32::from_bits(x.to_bits() + 1);
            }
        }
        let (mut designed, mut pass, mut worst) = (0u32, 0u32, 0.0f64);
        let (mut moved, mut rbj_worst) = (0u32, 0.0f64);
        for &fs in &rates {
            let b = Biquad::lowpass(60.0, fs);
            let is_pass = (b.b0, b.b1, b.b2, b.a1, b.a2) == (1.0, 0.0, 0.0, 0.0, 0.0);
            assert_eq!(
                is_pass,
                60.0 >= 0.5 * fs,
                "fs {fs}: pass-through must be exactly the band fc >= fs/2"
            );
            if is_pass {
                pass += 1;
                continue;
            }
            designed += 1;
            assert_eq!(b.b1, 2.0 * b.b0, "fs {fs}: the Nyquist zero must be kept");
            assert_eq!(b.b2, b.b0, "fs {fs}: the Nyquist zero must be kept");
            let rbj = rbj_lowpass(60.0, fs);
            if poles_inside(rbj.3, rbj.4, LPF_MAX_POLE_RADIUS) {
                assert_eq!(
                    (b.b0, b.b1, b.b2, b.a1, b.a2),
                    rbj,
                    "fs {fs}: must equal the pre-review design"
                );
            } else {
                moved += 1;
                rbj_worst = rbj_worst.max(max_pole(&Biquad {
                    a1: rbj.3,
                    a2: rbj.4,
                    ..b
                }));
            }
            let p = max_pole(&b);
            worst = worst.max(p);
            assert!(
                p < 1.0,
                "fs {fs} Hz: designed 60 Hz low-pass has |pole| {p}"
            );
        }
        std::eprintln!(
            "{designed} designed (worst |pole| {worst:.6}), {pass} pass-through, of {} rates; \
             {moved} moved to a lower cutoff (their pre-review worst |pole| {rbj_worst:.6})",
            rates.len()
        );
        assert!(
            rbj_worst >= 1.0,
            "control: the pre-review design WAS unstable in the band"
        );
    }

    /// **A retuned Direct Form I biquad keeps its state** (review of v1.140,
    /// defect A). Settled on DC 1.0 at 1 kHz, then re-designed for 250 Hz:
    /// the history words are unchanged, so the very next output is still the
    /// DC value (both designs have unity DC gain), where a freshly built
    /// filter restarts from zero.
    #[test]
    fn a_retuned_biquad_keeps_its_state() {
        let mut b = Biquad::lowpass(60.0, 1000.0);
        for _ in 0..1000 {
            b.filter(1.0);
        }
        let (x1, x2, y1, y2) = (b.x1, b.x2, b.y1, b.y2);
        b.retune_lowpass(60.0, 250.0);
        assert_eq!((b.x1, b.x2, b.y1, b.y2), (x1, x2, y1, y2));
        let fresh_design = Biquad::lowpass(60.0, 250.0);
        assert_eq!(
            (b.b0, b.b1, b.b2, b.a1, b.a2),
            (
                fresh_design.b0,
                fresh_design.b1,
                fresh_design.b2,
                fresh_design.a1,
                fresh_design.a2
            ),
            "retune must give exactly the coefficients lowpass() designs"
        );
        let kept = b.filter(1.0);
        let mut fresh = Biquad::lowpass(60.0, 250.0);
        let restarted = fresh.filter(1.0);
        assert!(
            (kept - 1.0).abs() < 1e-3,
            "retuned output must continue the settled DC value, got {kept}"
        );
        assert!(
            restarted < 0.5,
            "control: a fresh filter restarts from zero ({restarted})"
        );
        // The three-axis wrapper keeps every axis's state the same way.
        let mut g = GyroLpf::new(60.0, 1000.0);
        for _ in 0..1000 {
            g.filter([1.0, -2.0, 0.5]);
        }
        g.retune(60.0, 250.0);
        let y = g.filter([1.0, -2.0, 0.5]);
        assert!(
            (y[0] - 1.0).abs() < 1e-3 && (y[1] + 2.0).abs() < 2e-3 && (y[2] - 0.5).abs() < 1e-3
        );
    }

    /// **The transient of a coefficient swap under kept state**, measured
    /// (review of v1.140, defect A). A body-rate signal (offset `dc` plus a
    /// `f` Hz, 1 rad/s sinusoid) is filtered at 1 kHz; at `t_switch` the
    /// sample rate drops to 250 Hz and the filter is re-designed mid-signal.
    /// Reference: a filter designed for 250 Hz that sampled the same signal at
    /// 250 Hz all along. Returns the worst deviation from the reference over
    /// the next 0.2 s, for the kept-state retune and for a freshly built
    /// (zeroed) filter.
    fn swap_transient(dc: f32, f: f32, t_switch: f32, from: f32, to: f32) -> (f32, f32) {
        let sig = |t: f32| dc + relay_math::sinf(2.0 * core::f32::consts::PI * f * t);
        let mut kept = Biquad::lowpass(60.0, from);
        let n_from = (t_switch * from + 0.5) as u32;
        for k in 0..n_from {
            kept.filter(sig(k as f32 / from));
        }
        let mut reference = Biquad::lowpass(60.0, to);
        let n_to = (t_switch * to + 0.5) as u32;
        for k in 0..n_to {
            reference.filter(sig(k as f32 / to));
        }
        kept.retune_lowpass(60.0, to);
        let mut fresh = Biquad::lowpass(60.0, to);
        let (mut e_kept, mut e_fresh) = (0.0f32, 0.0f32);
        for k in 0..((0.2 * to) as u32) {
            let x = sig(t_switch + k as f32 / to);
            let r = reference.filter(x);
            e_kept = e_kept.max((kept.filter(x) - r).abs());
            e_fresh = e_fresh.max((fresh.filter(x) - r).abs());
        }
        (e_kept, e_fresh)
    }

    #[test]
    fn a_coefficient_swap_under_kept_state_has_a_small_transient() {
        extern crate std;
        for (from, to) in [(1000.0f32, 250.0f32), (250.0, 1000.0)] {
            for (dc, f) in [(0.5f32, 0.0f32), (0.5, 2.0), (0.0, 10.0), (0.0, 30.0)] {
                let (mut wk, mut wf) = (0.0f32, 0.0f32);
                for ph in 0..8 {
                    let t = 0.5 + ph as f32 / (8.0 * f.max(1.0));
                    let (k, fr) = swap_transient(dc, f, t, from, to);
                    wk = wk.max(k);
                    wf = wf.max(fr);
                }
                std::eprintln!(
                    "{from}->{to} Hz swap, {dc} + sin({f} Hz): worst deviation kept {wk:.4} fresh {wf:.4}"
                );
                // Measured (1000->250 / 250->1000): DC 0.0000/0.0000 vs
                // 0.36/0.49 fresh; 2 Hz 0.034/0.029 vs 1.09/1.46; 10 Hz
                // 0.17/0.13 vs 0.72/0.94; 30 Hz 0.64/0.55 vs 0.77/0.95. The
                // kept history is samples at the OLD spacing, so the swap's
                // transient grows with the signal's slope; a restart's is the
                // whole signal. Never worse than a restart, and at the body
                // rates a hover carries (near DC) it all but vanishes.
                assert!(wk < wf, "kept {wk} must be below a restart {wf}");
                if f <= 2.0 {
                    assert!(
                        wk < 0.05 * wf,
                        "low-frequency swap: kept {wk} vs fresh {wf}"
                    );
                }
            }
        }
    }

    /// The gyro LPF passes DC unchanged and strongly attenuates a
    /// high-frequency tone (above the cutoff) — the noise the high-gain
    /// ESO would otherwise pump into the motors.
    #[test]
    fn gyro_lpf_passes_dc_attenuates_hf() {
        let fs = 1000.0;
        let mut lpf = GyroLpf::new(60.0, fs);
        // DC: settle, then output ≈ input.
        let mut dc = 0.0;
        for _ in 0..500 {
            dc = lpf.filter([1.0, 0.0, 0.0])[0];
        }
        assert!((dc - 1.0).abs() < 0.02, "DC should pass, got {dc}");
        // 300 Hz tone (≫60 Hz cutoff): output amplitude ≪ 1.
        let mut lpf2 = GyroLpf::new(60.0, fs);
        let mut peak = 0.0f32;
        for k in 0..2000 {
            let x = relay_math::sinf(2.0 * core::f32::consts::PI * 300.0 * (k as f32) / fs);
            let y = lpf2.filter([x, 0.0, 0.0])[0].abs();
            if k > 200 && y > peak {
                peak = y;
            }
        }
        assert!(peak < 0.15, "300 Hz should be attenuated, peak {peak}");
    }

    /// Regulating to Ω_d = 0 from a disturbance holds the rate near zero
    /// (the yaw-hold case: no commanded rate, reject the lag-induced spin).
    #[test]
    fn regulates_rate_to_zero_under_disturbance() {
        let g = AdrcGains::new(20.0, 5.0, 6.0);
        let (omega, _) = sim_axis(4.0, 0.05, 1.5, 0.0, g);
        assert!(
            omega.abs() < 0.15,
            "rate should be held near 0, got {omega}"
        );
    }

    proptest::proptest! {
        /// Total: finite output for any finite inputs + any dt, including
        /// adversarial measurements and a degenerate b0.
        #[test]
        fn adrc_is_total(
            // Integer ranges scaled to floats: proptest 1.10's float-range
            // sampler has a flaky internal assertion (float_samplers.rs:462).
            // om∈[-50,50], od∈[-10,10], dt∈[0,0.2] — same ranges, integer-sampled.
            meas in proptest::collection::vec(
                (-50_000i32..=50_000, -10_000i32..=10_000, 0u32..=200_000), 0..400),
        ) {
            let mut a = AdrcAxis::new(AdrcGains::new(20.0, 5.0, 6.0));
            for (om_i, od_i, dt_i) in meas {
                let (om, od, dt) = (om_i as f32 / 1000.0, od_i as f32 / 1000.0, dt_i as f32 / 1_000_000.0);
                let u = a.tick(om, od, dt);
                proptest::prop_assert!(u.is_finite());
                proptest::prop_assert!(a.disturbance().is_finite());
            }
        }
    }

    // ── v0.33 inner-loop robustness ──────────────────────────────────────

    /// The bandwidth constructor enforces the separation invariant, and the
    /// shipped falcon-quad tuning satisfies it (roll/pitch 40/12≈3.33×, yaw
    /// 30/3=10×) — both ≥ 3× (the standard ADRC lower bound).
    #[test]
    fn falcon_tuning_is_well_separated() {
        let rp = AdrcGains::with_tau(40.0, 12.0, 30.0, 0.0125);
        let yaw = AdrcGains::with_tau(30.0, 3.0, 6.0, 0.025);
        assert!(rp.well_separated(3.0), "roll/pitch must be ≥3× separated");
        assert!(yaw.well_separated(3.0), "yaw must be ≥3× separated");
        // Under-separated tuning (ω_o ≈ ω_c) is correctly rejected.
        assert!(!AdrcGains::new(13.0, 12.0, 30.0).well_separated(3.0));
        // from_bandwidth makes it true by construction.
        assert!(AdrcGains::from_bandwidth(12.0, 5.0, 30.0, 0.0).well_separated(5.0));
    }

    /// The inner ESO is NOT the cadence-fragile element: the forward-Euler
    /// stability bound ω_o·dt < 2 holds at 1 kHz with a ~50× margin, and
    /// stays true across a ±15% dt sweep. (The fragility is the outer
    /// position→attitude cascade — see v0.33 notes.)
    #[test]
    fn eso_cadence_margin_is_large() {
        let g = AdrcGains::with_tau(40.0, 12.0, 30.0, 0.0125);
        for &dt in &[0.00085, 0.001, 0.00115] {
            assert!(g.eso_dt_stable(dt, 1.0), "ESO must be dt-stable at {dt}");
        }
        // Margin: stable all the way out to ~50× the nominal period.
        assert!(g.eso_dt_stable(0.045, 1.0));
        assert!(!g.eso_dt_stable(0.06, 1.0)); // ω_o·dt = 2.4 > 2 → unstable
    }

    /// Critically-damped command filter: a unit step rises monotonically to
    /// the target with NO overshoot (never adds command energy) and settles.
    #[test]
    fn command_filter_no_overshoot_on_step() {
        let mut cf = CommandFilter::new(20.0, f32::INFINITY, f32::INFINITY);
        let dt = 0.001;
        let mut peak = 0.0f32;
        let mut last = 0.0f32;
        for _ in 0..2000 {
            let y = cf.step(1.0, dt);
            if y > peak {
                peak = y;
            }
            last = y;
        }
        // Critically damped ⇒ the step response never overshoots the target
        // (peak ≤ 1 to within float settling), and it settles to it.
        assert!(peak <= 1.0 + 1e-3, "no overshoot, peak {peak}");
        assert!((last - 1.0).abs() < 0.02, "should settle to 1, got {last}");
    }

    /// Saturation: the magnitude and rate clamps hold regardless of input.
    #[test]
    fn command_filter_saturates() {
        let mut cf = CommandFilter::new(50.0, 0.5, 3.0);
        for _ in 0..1000 {
            let y = cf.step(10.0, 0.001); // huge command
            assert!(y.abs() <= 0.5 + 1e-6, "mag clamp, {y}");
            assert!(cf.rate().abs() <= 3.0 + 1e-6, "rate clamp, {}", cf.rate());
        }
    }

    proptest::proptest! {
        /// Command filter is BIBO + saturating under a ±15% dt sweep and
        /// bounded command: y stays within max_mag, ẏ within max_rate.
        #[test]
        fn command_filter_bibo_under_dt_jitter(
            // integer-sampled to dodge the proptest 1.10 float-range sampler
            // flake; u∈[-5,5], dt∈[0.00085,0.00115] (±15% of 1 kHz).
            seq in proptest::collection::vec(
                (-5_000i32..=5_000, 850u32..=1150), 0..500),
        ) {
            let mut cf = CommandFilter::new(25.0, 2.0, 40.0);
            for (u_i, dt_i) in seq {
                let (u, dt) = (u_i as f32 / 1000.0, dt_i as f32 / 1_000_000.0);
                let y = cf.step(u, dt);
                proptest::prop_assert!(y.abs() <= 2.0 + 1e-4);
                proptest::prop_assert!(cf.rate().abs() <= 40.0 + 1e-4);
                proptest::prop_assert!(y.is_finite());
            }
        }

        /// Cadence robustness of the inner ESO: under a ±15% dt jitter and a
        /// bounded measured-rate sequence, the control + disturbance estimate
        /// stay BOUNDED (not merely finite) — the empirical face of the
        /// ω_o·dt < 2 margin.
        #[test]
        fn eso_bounded_under_dt_jitter(
            // integer-sampled to dodge the proptest 1.10 float-range sampler
            // flake; om∈[-3,3], dt∈[0.00085,0.00115] (±15% of 1 kHz).
            seq in proptest::collection::vec(
                (-3_000i32..=3_000, 850u32..=1150), 0..1000),
        ) {
            let mut a = AdrcAxis::new(AdrcGains::with_tau(40.0, 12.0, 30.0, 0.0125));
            for (om_i, dt_i) in seq {
                let (om, dt) = (om_i as f32 / 1000.0, dt_i as f32 / 1_000_000.0);
                let u = a.tick(om, 0.0, dt); // regulate to 0
                proptest::prop_assert!(u.abs() < 1.0e3, "control bounded, {u}");
                proptest::prop_assert!(a.disturbance().abs() < 1.0e4, "z2 bounded");
            }
        }
    }
}

#[cfg(kani)]
mod kani_harness {
    use super::*;

    /// from_bandwidth makes the separation invariant true by construction:
    /// for any ω_c > 0 and separation ≥ 3, well_separated(3) holds.
    #[kani::proof]
    fn verify_from_bandwidth_separation() {
        let omega_c: f32 = kani::any();
        let sep: f32 = kani::any();
        let b0: f32 = kani::any();
        kani::assume(omega_c.is_finite() && omega_c > 0.1 && omega_c < 1.0e3);
        kani::assume(sep.is_finite() && sep >= 3.0 && sep < 1.0e3);
        kani::assume(b0.is_finite());
        let g = AdrcGains::from_bandwidth(omega_c, sep, b0, 0.0);
        assert!(g.well_separated(3.0));
    }

    /// The command-filter saturation contract: after a step, the value is
    /// within ±max_mag and the rate within ±max_rate, for ANY state/input
    /// (incl. non-finite), given finite non-negative bounds. The clamp is
    /// split from the f32 arithmetic so this is a comparison-only proof.
    #[kani::proof]
    fn verify_command_filter_saturation() {
        let omega_n: f32 = kani::any();
        let max_mag: f32 = kani::any();
        let max_rate: f32 = kani::any();
        let y: f32 = kani::any();
        let yd: f32 = kani::any();
        let u: f32 = kani::any();
        let dt: f32 = kani::any();
        kani::assume(omega_n.is_finite() && omega_n > 0.0 && omega_n < 1.0e3);
        kani::assume(max_mag.is_finite() && max_mag >= 0.0 && max_mag <= 1.0e3);
        kani::assume(max_rate.is_finite() && max_rate >= 0.0 && max_rate <= 1.0e4);
        kani::assume(u.is_finite() && u.abs() <= 1.0e3);
        // Inductive invariant: the state entering a step is itself within the
        // saturation bounds (maintained by the previous step's clamp). Proving
        // the step preserves it gives the running guarantee for all time.
        kani::assume(y.is_finite() && y.abs() <= max_mag);
        kani::assume(yd.is_finite() && yd.abs() <= max_rate);
        let mut cf = CommandFilter {
            omega_n,
            max_mag,
            max_rate,
            y,
            yd,
        };
        let out = cf.step(u, dt);
        assert!(out <= max_mag && out >= -max_mag);
        assert!(cf.rate() <= max_rate && cf.rate() >= -max_rate);
    }
}
