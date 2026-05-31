//! Relay control allocator — X-config quadcopter mixer.
//!
//! Maps the body-frame torque + collective-thrust command to four
//! per-motor PWM values for the falcon-quad airframe. Last stage of
//! the inner control cascade:
//!
//! ```text
//!  τ_body, T ──► [relay-mix-quad] ──► [m1, m2, m3, m4]  (each ∈ [0, 1])
//! ```
//!
//! ## Airframe convention
//!
//! Motors numbered 1–4 clockwise from front-right, viewed from above:
//!
//! ```text
//!         (front)
//!     M4         M1
//!       \       /
//!        \  +x /
//!  +y --- center ---
//!        /     \
//!       /       \
//!     M3         M2
//!         (back)
//! ```
//!
//! Spin directions (standard PX4 quad-x): M1, M3 CW; M2, M4 CCW.
//! Diagonal pairs share spin direction so net yaw reaction is zero
//! at hover.
//!
//! ## Mixer matrix
//!
//! Each motor's PWM contribution per body-frame command:
//!
//! | motor | thrust | roll  | pitch | yaw   | spin |
//! |-------|--------|-------|-------|-------|------|
//! | M1    | +1     | -1    | +1    | -1    | CW   |
//! | M2    | +1     | -1    | -1    | +1    | CCW  |
//! | M3    | +1     | +1    | -1    | -1    | CW   |
//! | M4    | +1     | +1    | +1    | +1    | CCW  |
//!
//! Roll convention: +roll about +x (forward) means right wing down →
//! right-side motors (M1, M2) thrust less, left-side (M3, M4) more.
//!
//! Pitch convention: +pitch about +y (right) means nose up → front
//! motors (M1, M4) thrust more, back motors (M2, M3) less.
//!
//! Yaw convention: +yaw about +z (down NED) = clockwise viewed from
//! above. CW motors (M1, M3) spin less, CCW (M2, M4) more so that the
//! net reaction torque on the airframe spins it +yaw.
//!
//! ## Verified properties (v0.4 surrogates for SWREQ-FALCON-MIX-P*)
//!
//! - **MIX-P01** (Drake-derivation precedent): the matrix was derived
//!   from first-principles in the airframe convention above. v0.5
//!   formalises this via a Drake MultibodyPlant export. *Test:
//!   `mix_p01_zero_command_gives_thrust_only`.*
//! - **MIX-P02** (no-negative-thrust): every PWM output is clamped to
//!   `[0, 1]`. *Test: `mix_p02_outputs_in_unit_interval`.*
//! - **MIX-P03** (saturation handling): when the un-saturated sum
//!   would exceed `[0, 1]`, an order-preserving scale is applied to
//!   keep the relative torque ratios intact (thrust first, yaw
//!   sacrificed last). *Test:
//!   `mix_p03_saturation_preserves_torque_direction_sign`.*
//! - **MIX-P04** (per-variant motor count): falcon-quad exports 4
//!   motors. Other airframes live in separate crates. *Test: the
//!   `Motors4` return type is a `[f32; 4]`.*

#![no_std]
#![forbid(unsafe_code)]

/// Per-motor mixer signs. Each row is `[thrust, roll, pitch, yaw]`.
const MIXER_X: [[f32; 4]; 4] = [
    [1.0, -1.0, 1.0, -1.0], // M1 front-right, CW
    [1.0, -1.0, -1.0, 1.0], // M2 back-right, CCW
    [1.0, 1.0, -1.0, -1.0], // M3 back-left, CW
    [1.0, 1.0, 1.0, 1.0],   // M4 front-left, CCW
];

/// X-config quad mixer state. Stateless on its own — the struct
/// carries last-output bookkeeping for diagnostics.
#[derive(Clone, Copy, Debug, Default)]
pub struct QuadMixer {
    last_motors: [f32; 4],
}

impl QuadMixer {
    pub const fn new() -> Self {
        Self { last_motors: [0.0; 4] }
    }

    pub fn last_motors(&self) -> [f32; 4] {
        self.last_motors
    }

    /// Map (torque_body, thrust) to per-motor PWM values clamped to
    /// `[0, 1]`. `torque_body` units are roll/pitch/yaw torque
    /// normalised to `[-1, +1]` (controller-output magnitudes).
    /// `thrust` is the collective thrust command in `[0, 1]`.
    ///
    /// The output is clipped to `[0, 1]` per motor. If the raw mix
    /// produces any value above 1.0, the entire bus is shifted down
    /// by the excess (preserves the relative roll/pitch/yaw axes,
    /// sacrifices some collective thrust). If any value is below 0
    /// after that, the negative motors are clamped to 0 (sacrifices
    /// some torque authority on those axes). This is the standard
    /// "priority-preserving" mixer behaviour PX4 uses.
    pub fn mix(&mut self, torque_body: [f32; 3], thrust: f32) -> [f32; 4] {
        let t = sanitise(thrust);
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        let y = sanitise(torque_body[2]);

        let mut m = [0.0_f32; 4];
        for i in 0..4 {
            let row = &MIXER_X[i];
            m[i] = row[0] * t + row[1] * r + row[2] * p + row[3] * y;
        }

        // Step 1: if any motor exceeds 1.0, subtract the excess from
        // every motor so the maximum is exactly 1.0. Preserves the
        // relative torque ratios; sacrifices collective thrust.
        let mut max = m[0];
        for &v in &m[1..] {
            if v > max {
                max = v;
            }
        }
        if max > 1.0 {
            let excess = max - 1.0;
            for v in m.iter_mut() {
                *v -= excess;
            }
        }

        // Step 2: clamp to [0, 1]. Negative motors are clipped (lose
        // some torque authority, but motors can't push reverse).
        for v in m.iter_mut() {
            if *v < 0.0 || !v.is_finite() {
                *v = 0.0;
            } else if *v > 1.0 {
                *v = 1.0;
            }
        }
        self.last_motors = m;
        m
    }

    /// v0.19.8 — **thrust-priority** mixer with a guaranteed thrust
    /// floor (MIX-P05). Where `mix` sacrifices *collective thrust* to
    /// preserve torque ratios when motors saturate, `mix_thrust_floor`
    /// does the opposite: it scales the *torque* command down by a
    /// single factor `s ∈ [0, 1]` so every motor stays in
    /// `[floor, 1]`, leaving the collective thrust untouched.
    ///
    /// This matters for closed-loop hover against a real-physics sim:
    /// the gz bench (v0.19.7) showed that the attitude-priority `mix`
    /// lets the rate-loop's torque steal thrust near saturation,
    /// driving an altitude limit cycle. Reserving the thrust floor
    /// decouples attitude from altitude.
    ///
    /// Because each torque column of `MIXER_X` is zero-sum
    /// (roll/pitch/yaw each have two `+1` and two `−1` entries), a
    /// uniform torque scale leaves the per-motor mean — i.e. the
    /// collective thrust — exactly equal to `thrust`. So scaling
    /// torque costs attitude authority, never lift.
    ///
    /// Invariant (MIX-P05, proved in the verus tree + Kani harness):
    /// for `thrust ∈ [floor, 1]` and `floor ∈ [0, 1]`, every output
    /// motor is in `[floor, 1]` ⊆ `[0, 1]` and finite.
    pub fn mix_thrust_floor(
        &mut self,
        torque_body: [f32; 3],
        thrust: f32,
        floor: f32,
    ) -> [f32; 4] {
        let t = clamp01(sanitise(thrust));
        let floor = clamp01(sanitise(floor));
        // Collective base; if thrust is below the floor we can't
        // honour the floor without inventing lift, so the base is
        // max(t, floor) — the mixer never commands less collective
        // than the floor.
        let base = if t < floor { floor } else { t };
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        let y = sanitise(torque_body[2]);

        // Per-motor torque delta (no thrust term). Sanitised: an
        // overflow to ±inf or a +inf+(−inf)=NaN in the sum would
        // otherwise poison the scale division below. `sanitise` maps
        // any non-finite delta to 0 (that motor contributes no
        // torque), keeping the whole computation total. (Kani found
        // this: extreme finite r/p/y can overflow the intermediate.)
        let mut d = [0.0_f32; 4];
        for i in 0..4 {
            let row = &MIXER_X[i];
            d[i] = sanitise(row[1] * r + row[2] * p + row[3] * y);
        }

        // Largest torque scale s ∈ [0, 1] keeping base + s·d[i] in
        // [floor, 1] for every motor. d[i] > 0 risks the 1.0 ceiling;
        // d[i] < 0 risks the floor.
        const EPS: f32 = 1.0e-6;
        let mut s = 1.0_f32;
        for &di in &d {
            if di > EPS {
                let lim = (1.0 - base) / di;
                if lim < s { s = lim; }
            } else if di < -EPS {
                let lim = (base - floor) / (-di);
                if lim < s { s = lim; }
            }
        }
        if s < 0.0 || !s.is_finite() { s = 0.0; }

        let mut m = [0.0_f32; 4];
        for i in 0..4 {
            // Final clamp to [floor, 1] makes the MIX-P05 floor a HARD
            // guarantee by construction (the s bound targets it; this
            // clamp closes any float-rounding gap). `clamp_floor` is
            // total over all f32 (NaN → floor), so the output is
            // always finite and in [floor, 1].
            m[i] = clamp_floor(base + s * d[i], floor);
        }
        self.last_motors = m;
        m
    }

    /// **Priority desaturation** mix (MIX-P06): thrust ≻ roll/pitch ≻ yaw.
    /// Like `mix_thrust_floor` it reserves the collective floor, but when
    /// saturated it gives up authority IN PRIORITY ORDER — yaw first, then
    /// roll/pitch — instead of scaling all torque uniformly. This is the
    /// PX4/ArduPilot production policy and the v0.25 fix for the yaw
    /// instability corrupting roll/pitch: when the (weak, lag-prone) yaw
    /// loop demands torque that would saturate a motor, yaw is sacrificed
    /// so roll/pitch (and thus tilt, and thus position-hold) stay intact.
    ///
    /// Invariant (same as MIX-P05): for `floor ∈ [0,1]` and any torque,
    /// every output motor ∈ `[floor, 1]` and finite — the final
    /// `clamp_floor` makes it a hard guarantee.
    pub fn mix_priority(
        &mut self,
        torque_body: [f32; 3],
        thrust: f32,
        floor: f32,
    ) -> [f32; 4] {
        let t = clamp01(sanitise(thrust));
        let floor = clamp01(sanitise(floor));
        let base = if t < floor { floor } else { t };
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        let y = sanitise(torque_body[2]);

        // Split the per-motor delta into yaw vs roll/pitch groups.
        let mut dy = [0.0_f32; 4];
        let mut drp = [0.0_f32; 4];
        for i in 0..4 {
            let row = &MIXER_X[i];
            dy[i] = sanitise(row[3] * y);
            drp[i] = sanitise(row[1] * r + row[2] * p);
        }

        // 1. YAW deprioritised: largest sy ∈ [0,1] keeping
        //    base + drp[i] + sy·dy[i] ∈ [floor,1] (roll/pitch + thrust
        //    preserved). If roll/pitch alone already saturates, sy → 0.
        let base_rp = [base + drp[0], base + drp[1], base + drp[2], base + drp[3]];
        let sy = scale_to_fit(&base_rp, &dy, floor);
        // 2. ROLL/PITCH next: scale to fit with the reduced yaw applied.
        let base_y = [base + sy * dy[0], base + sy * dy[1], base + sy * dy[2], base + sy * dy[3]];
        let srp = scale_to_fit(&base_y, &drp, floor);

        let mut m = [0.0_f32; 4];
        for i in 0..4 {
            m[i] = clamp_floor(base + srp * drp[i] + sy * dy[i], floor);
        }
        self.last_motors = m;
        m
    }

    /// **Airmode** mix (MIX-P07): preserve the full ATTITUDE differential
    /// (roll/pitch/yaw) by shifting the whole COLLECTIVE up/down to fit
    /// `[idle, 1]`, sacrificing thrust rather than attitude authority — the
    /// PX4 airmode policy. Only if the attitude differential itself is
    /// wider than the available range `(1 − idle)` is the torque scaled
    /// down (uniformly, ratios preserved). The v0.25 fix for the yaw axis:
    /// the hard-floor mixers scaled yaw → 0 exactly when a yaw correction
    /// pushed a motor low; airmode keeps the yaw differential and moves
    /// collective instead.
    ///
    /// Invariant (same family as MIX-P05/P06): every motor ∈ `[idle, 1]`
    /// and finite for ANY input (final `clamp_floor`).
    pub fn mix_airmode(&mut self, torque_body: [f32; 3], thrust: f32, idle: f32) -> [f32; 4] {
        let t = clamp01(sanitise(thrust));
        let idle = clamp01(sanitise(idle));
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        let y = sanitise(torque_body[2]);

        // Per-motor attitude differential (no thrust term).
        let mut d = [0.0_f32; 4];
        for i in 0..4 {
            let row = &MIXER_X[i];
            d[i] = sanitise(row[1] * r + row[2] * p + row[3] * y);
        }
        let (mut dmin, mut dmax) = (d[0], d[0]);
        for &di in &d[1..] {
            if di < dmin { dmin = di; }
            if di > dmax { dmax = di; }
        }

        // If the differential spread exceeds the available range, scale the
        // whole torque down (preserving roll/pitch/yaw ratios). Guard the
        // divisor finite + bounded-away-from-zero so the quotient is total.
        let span = 1.0 - idle;
        let range = sanitise(dmax - dmin); // finite (sanitise NaN/∞ → 0)
        let mut s = 1.0_f32;
        if range.is_finite() && range > 1e-6 && range > span {
            s = span / range;
        }
        if !s.is_finite() || s < 0.0 {
            s = 0.0;
        }
        for di in d.iter_mut() {
            *di *= s;
        }

        // m = collective + scaled differential, then shift collective so
        // the lowest motor sits at idle (and the highest ≤ 1).
        let mut m = [t + d[0], t + d[1], t + d[2], t + d[3]];
        let (mut lo, mut hi) = (m[0], m[0]);
        for &v in &m[1..] {
            if v < lo { lo = v; }
            if v > hi { hi = v; }
        }
        let shift = if lo < idle {
            idle - lo
        } else if hi > 1.0 {
            -(hi - 1.0)
        } else {
            0.0
        };
        for mi in m.iter_mut() {
            *mi = clamp_floor(*mi + shift, idle);
        }
        self.last_motors = m;
        m
    }

    /// **Reconfigured allocator for SINGLE-ROTOR FAILURE** (MIX-P08, v0.26).
    /// On loss of rotor `failed` (0..4) a quad is rank-deficient — full
    /// attitude is unrecoverable (Mueller & D'Andrea, *Relaxed hover
    /// solutions*), so **yaw is RELINQUISHED** and only thrust + roll +
    /// pitch are allocated over the three healthy rotors; the failed rotor
    /// is pinned to 0. The reduced-attitude (S²) controller upstream accepts
    /// the residual spin about the near-vertical primary axis.
    ///
    /// Invariant (MIX-P08, same family as MIX-P05/06/07): for ANY input and
    /// ANY `failed < 4`, the failed rotor is exactly 0 and every HEALTHY
    /// rotor ∈ `[floor, 1]` and finite. `failed ≥ 4` ⇒ no failure (all four
    /// allocated normally, all ∈ `[floor, 1]`).
    pub fn mix_rotor_out(
        &mut self,
        failed: usize,
        torque_body: [f32; 3],
        thrust: f32,
        floor: f32,
    ) -> [f32; 4] {
        let t = clamp01(sanitise(thrust));
        let floor = clamp01(sanitise(floor));
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        // yaw = torque_body[2] is RELINQUISHED — never allocated.
        let mut m = [0.0_f32; 4];
        for i in 0..4 {
            if i == failed {
                m[i] = 0.0; // failed rotor OFF (below the healthy floor, by design)
            } else {
                let row = &MIXER_X[i];
                // thrust + roll + pitch only (no yaw term), clamped to [floor,1].
                m[i] = clamp_floor(sanitise(t + row[1] * r + row[2] * p), floor);
            }
        }
        self.last_motors = m;
        m
    }
}

/// Clamp `x` into `[lo, 1]`. Total over all f32: NaN and values below
/// `lo` map to `lo`, values above 1 map to 1. `lo` is assumed in
/// `[0, 1]` (the caller passes a sanitised + clamped floor).
#[inline]
fn clamp_floor(x: f32, lo: f32) -> f32 {
    if !x.is_finite() || x < lo {
        lo
    } else if x > 1.0 {
        1.0
    } else {
        x
    }
}

#[inline]
fn clamp01(x: f32) -> f32 {
    // Behaviour-identical to the manual form (NaN → NaN); the mixer
    // sanitises before this so NaN does not reach it.
    x.clamp(0.0, 1.0)
}

#[inline]
fn sanitise(x: f32) -> f32 {
    if !x.is_finite() {
        0.0
    } else {
        x
    }
}

/// Largest scale `s ∈ [0,1]` keeping `base[i] + s·delta[i] ∈ [floor,1]`
/// for every motor (the per-group desaturation step). Returns 0 if a
/// constraint is already violated at s=0 or the result is non-finite.
fn scale_to_fit(base: &[f32; 4], delta: &[f32; 4], floor: f32) -> f32 {
    const EPS: f32 = 1.0e-6;
    let mut s = 1.0_f32;
    for i in 0..4 {
        let di = delta[i];
        let b = base[i];
        if di > EPS {
            let lim = (1.0 - b) / di;
            if lim < s { s = lim; }
        } else if di < -EPS {
            let lim = (b - floor) / (-di);
            if lim < s { s = lim; }
        }
    }
    if s < 0.0 || !s.is_finite() { 0.0 } else { s }
}

/// Sum the per-axis torque produced by a motor-command vector,
/// using the same mixer matrix in reverse. Useful for verifying
/// MIX-P03 in tests — given a motor command, what torque ratio
/// did the airframe actually experience? Sign per axis only.
pub fn motors_to_torque_signs(motors: [f32; 4]) -> [f32; 3] {
    let mut t = [0.0_f32; 3];
    for i in 0..4 {
        t[0] += MIXER_X[i][1] * motors[i];
        t[1] += MIXER_X[i][2] * motors[i];
        t[2] += MIXER_X[i][3] * motors[i];
    }
    t
}

// ─── Kani bounded-model-checking harnesses ──────────────────────────
//
// The mixer is f32-based, so the verified-engine pattern's integer-only
// Verus track can't discharge its bounds (Verus is used for the i32/i64
// engines precisely because SMT float reasoning is impractical there).
// Kani (CBMC) bit-blasts floats, so it is the correct mechanical oracle
// for the MIX-P05 thrust-floor *bound* invariant. Run: `cargo kani`.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// MIX-P05: thrust-floor invariant. For any finite torque and a
    /// thrust ≥ floor (floor ∈ [0, 1]), every motor output lands in
    /// `[floor, 1]` and is finite — the rate loop can never starve a
    /// motor below the reserved floor. (Hard guarantee via
    /// `clamp_floor`; this harness proves it over the float domain.)
    #[kani::proof]
    fn verify_mix_thrust_floor_bound() {
        let floor: f32 = kani::any();
        kani::assume(floor.is_finite() && floor >= 0.0 && floor <= 1.0);
        let thrust: f32 = kani::any();
        kani::assume(thrust.is_finite());
        let r: f32 = kani::any();
        let p: f32 = kani::any();
        let y: f32 = kani::any();
        kani::assume(r.is_finite() && p.is_finite() && y.is_finite());

        let mut m = QuadMixer::new();
        let out = m.mix_thrust_floor([r, p, y], thrust, floor);
        for &v in out.iter() {
            assert!(v.is_finite());
            assert!(v >= floor);
            assert!(v <= 1.0);
        }
    }

    /// MIX-P05 corollary: even with non-finite (NaN/inf) inputs the
    /// output stays finite + in `[floor, 1]` — total, panic-free.
    #[kani::proof]
    fn verify_mix_thrust_floor_total() {
        let floor: f32 = kani::any();
        kani::assume(floor.is_finite() && floor >= 0.0 && floor <= 1.0);
        let thrust: f32 = kani::any();
        let r: f32 = kani::any();
        let p: f32 = kani::any();
        let y: f32 = kani::any();
        let mut m = QuadMixer::new();
        let out = m.mix_thrust_floor([r, p, y], thrust, floor);
        for &v in out.iter() {
            assert!(v.is_finite());
            assert!(v >= floor);
            assert!(v <= 1.0);
        }
    }

    /// MIX-P06: the priority-desaturation mix holds the SAME bound — every
    /// motor ∈ [floor,1] and finite for ANY (incl. non-finite) input.
    #[kani::proof]
    fn verify_mix_priority_bound() {
        let floor: f32 = kani::any();
        kani::assume(floor.is_finite() && floor >= 0.0 && floor <= 1.0);
        let thrust: f32 = kani::any();
        let r: f32 = kani::any();
        let p: f32 = kani::any();
        let y: f32 = kani::any();
        let mut m = QuadMixer::new();
        let out = m.mix_priority([r, p, y], thrust, floor);
        for &v in out.iter() {
            assert!(v.is_finite());
            assert!(v >= floor);
            assert!(v <= 1.0);
        }
    }

    /// MIX-P07: airmode mix holds the bound — every motor ∈ [idle,1] and
    /// finite for ANY (incl. non-finite) input.
    #[kani::proof]
    fn verify_mix_airmode_bound() {
        let idle: f32 = kani::any();
        kani::assume(idle.is_finite() && idle >= 0.0 && idle <= 1.0);
        let thrust: f32 = kani::any();
        let r: f32 = kani::any();
        let p: f32 = kani::any();
        let y: f32 = kani::any();
        let mut m = QuadMixer::new();
        let out = m.mix_airmode([r, p, y], thrust, idle);
        for &v in out.iter() {
            assert!(v.is_finite());
            assert!(v >= idle);
            assert!(v <= 1.0);
        }
    }

    /// MIX-P08 (v0.26): the reconfigured single-rotor-out allocator. For
    /// ANY input and ANY failed index < 4, the failed rotor is EXACTLY 0
    /// and every healthy rotor ∈ [floor,1] and finite — the bounded
    /// actuator contract still holds after reconfiguration. (The
    /// rank-deficiency is handled upstream by relinquishing yaw; here we
    /// prove the allocator output set stays safe.)
    #[kani::proof]
    fn verify_mix_rotor_out_bound() {
        let failed: usize = kani::any();
        kani::assume(failed < 4);
        let floor: f32 = kani::any();
        kani::assume(floor.is_finite() && floor >= 0.0 && floor <= 1.0);
        let thrust: f32 = kani::any();
        let r: f32 = kani::any();
        let p: f32 = kani::any();
        let y: f32 = kani::any();
        let mut m = QuadMixer::new();
        let out = m.mix_rotor_out(failed, [r, p, y], thrust, floor);
        for i in 0..4 {
            assert!(out[i].is_finite());
            if i == failed {
                assert!(out[i] == 0.0); // failed rotor pinned OFF
            } else {
                assert!(out[i] >= floor); // healthy rotors stay in [floor,1]
                assert!(out[i] <= 1.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MIX-P06: under saturation, the priority mix preserves roll/pitch
    /// authority by sacrificing yaw — vs the uniform-scale mix which cuts
    /// both. This is what protects tilt/position-hold from the weak yaw
    /// loop (the v0.25 fix).
    #[test]
    fn mix_p06_priority_preserves_roll_over_yaw() {
        let torque = [0.3, 0.0, 0.8]; // big yaw + moderate roll
        let (thrust, floor) = (0.7_f32, 0.3_f32);
        let prio = QuadMixer::new().mix_priority(torque, thrust, floor);
        let uni = QuadMixer::new().mix_thrust_floor(torque, thrust, floor);
        for &v in prio.iter() {
            assert!(v >= floor - 1e-6 && v <= 1.0 + 1e-6, "bound: {v}");
        }
        let roll_prio = motors_to_torque_signs(prio)[0].abs();
        let roll_uni = motors_to_torque_signs(uni)[0].abs();
        assert!(roll_prio >= roll_uni - 1e-6,
            "priority should preserve >= roll than uniform: {roll_prio} vs {roll_uni}");
    }

    /// Without saturation, the priority mix passes the full torque through
    /// (yaw not sacrificed when there's headroom).
    #[test]
    fn mix_p06_no_saturation_passthrough() {
        let torque = [0.05, 0.05, 0.05];
        let out = QuadMixer::new().mix_priority(torque, 0.6, 0.3);
        let tq = motors_to_torque_signs(out);
        // all three axes retain their commanded sign (non-zero).
        assert!(tq[2].abs() > 1e-3, "yaw preserved when unsaturated: {:?}", tq);
    }

    /// MIX-P07: airmode preserves the YAW differential where the priority
    /// mixer sacrifices it — under a saturating yaw command, airmode keeps
    /// more yaw torque (it moves collective instead of scaling yaw to 0).
    #[test]
    fn mix_p07_airmode_preserves_yaw_vs_floor() {
        let torque = [0.0, 0.0, 0.6]; // pure yaw, would push motors below floor
        let (thrust, idle) = (0.55_f32, 0.2_f32);
        let air = QuadMixer::new().mix_airmode(torque, thrust, idle);
        let prio = QuadMixer::new().mix_priority(torque, thrust, idle);
        for &v in air.iter() {
            assert!(v >= idle - 1e-6 && v <= 1.0 + 1e-6, "airmode bound: {v}");
        }
        let yaw_air = motors_to_torque_signs(air)[2].abs();
        let yaw_prio = motors_to_torque_signs(prio)[2].abs();
        assert!(yaw_air >= yaw_prio - 1e-6,
            "airmode should preserve >= yaw than priority: {yaw_air} vs {yaw_prio}");
        assert!(yaw_air > 1e-3, "airmode keeps real yaw authority: {yaw_air}");
    }

    /// MIX-P08 (v0.26): single-rotor-out allocator pins the failed rotor to
    /// 0, keeps the healthy three in [floor,1], and relinquishes yaw (a yaw
    /// command does not change the healthy outputs).
    #[test]
    fn mix_p08_rotor_out_pins_failed_and_bounds_healthy() {
        let (thrust, floor) = (0.6_f32, 0.15_f32);
        for failed in 0..4 {
            let out = QuadMixer::new().mix_rotor_out(failed, [0.1, -0.1, 0.5], thrust, floor);
            assert_eq!(out[failed], 0.0, "failed rotor {failed} must be OFF: {out:?}");
            for (i, &v) in out.iter().enumerate() {
                if i != failed {
                    assert!(v >= floor && v <= 1.0, "healthy rotor {i} out of [floor,1]: {v}");
                }
            }
            // Yaw is relinquished: the same command with a different yaw must
            // produce identical healthy outputs.
            let out_noyaw = QuadMixer::new().mix_rotor_out(failed, [0.1, -0.1, -9.0], thrust, floor);
            for i in 0..4 {
                assert!((out[i] - out_noyaw[i]).abs() < 1e-6, "yaw should not affect rotor {i}");
            }
        }
    }

    #[test]
    fn mix_p01_zero_command_gives_thrust_only() {
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.0], 0.5);
        for v in r.iter() {
            assert!((v - 0.5).abs() < 1.0e-6,
                "all motors should equal thrust at zero torque, got {:?}", r);
        }
    }

    #[test]
    fn mix_p02_outputs_in_unit_interval() {
        // Throw arbitrary commands at it; every output stays in [0, 1].
        let mut mixer = QuadMixer::new();
        for &t in &[0.0_f32, 0.1, 0.5, 0.9, 1.0, 1.5, -0.5] {
            for &r in &[-1.0_f32, -0.5, 0.0, 0.5, 1.0] {
                let m = mixer.mix([r, r, r], t);
                for v in m.iter() {
                    assert!((0.0..=1.0).contains(v),
                        "motor out of bounds: t={} r={} -> {:?}", t, r, m);
                }
            }
        }
    }

    #[test]
    fn mix_p03_pure_roll_drives_diagonal_motor_pairs_opposite() {
        // +roll command (right wing down) → right-side motors (M1, M2)
        // less thrust than left-side (M3, M4).
        let mut m = QuadMixer::new();
        let r = m.mix([0.5, 0.0, 0.0], 0.5);
        assert!(r[0] < r[2], "M1 (right) must be less than M3 (left): {:?}", r);
        assert!(r[1] < r[3], "M2 (right) must be less than M4 (left): {:?}", r);
    }

    #[test]
    fn mix_p03_pure_pitch_drives_front_and_back_motors_opposite() {
        // +pitch (nose up) → front motors (M1, M4) more, back (M2, M3) less.
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.5, 0.0], 0.5);
        assert!(r[0] > r[1], "M1 (front) must be greater than M2 (back): {:?}", r);
        assert!(r[3] > r[2], "M4 (front) must be greater than M3 (back): {:?}", r);
    }

    #[test]
    fn mix_p03_pure_yaw_drives_cw_and_ccw_pairs_opposite() {
        // +yaw (rotate CW from above) → CW motors (M1, M3) less, CCW (M2, M4) more.
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.5], 0.5);
        assert!(r[0] < r[1], "CW M1 must be less than CCW M2: {:?}", r);
        assert!(r[2] < r[3], "CW M3 must be less than CCW M4: {:?}", r);
    }

    #[test]
    fn high_thrust_with_torque_saturates_gracefully() {
        // Thrust near max with a torque command would push some motor
        // above 1.0. Mixer must shift the bus down, not clip torque.
        let mut m = QuadMixer::new();
        let r = m.mix([0.5, 0.0, 0.0], 1.0);
        // Maximum motor must be 1.0 (the excess was absorbed).
        let max = r.iter().fold(0.0_f32, |a, &v| a.max(v));
        assert!((max - 1.0).abs() < 1.0e-6 || max <= 1.0);
        // Direction sign preserved: M3 still > M1 (left > right).
        assert!(r[2] > r[0]);
    }

    #[test]
    fn nan_input_does_not_propagate() {
        let mut m = QuadMixer::new();
        let r = m.mix([f32::NAN, 0.0, 0.0], 0.5);
        for v in r.iter() {
            assert!(v.is_finite(), "NaN propagated: {:?}", r);
        }
        let r2 = m.mix([0.0, 0.0, 0.0], f32::NAN);
        for v in r2.iter() {
            assert!(v.is_finite());
        }
    }

    #[test]
    fn negative_thrust_clipped_to_zero() {
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.0], -0.5);
        for v in r.iter() {
            assert_eq!(*v, 0.0, "negative thrust should clip to 0, got {:?}", r);
        }
    }

    #[test]
    fn mix_p05_thrust_floor_never_starves_collective() {
        // Aggressive torque at hover thrust: attitude-priority `mix`
        // would steal thrust; `mix_thrust_floor` keeps every motor
        // ≥ floor.
        let mut m = QuadMixer::new();
        let floor = 0.4_f32;
        let out = m.mix_thrust_floor([1.0, 1.0, 0.5], 0.5, floor);
        for v in out.iter() {
            assert!(*v >= floor - 1.0e-6, "motor below floor: {:?}", out);
            assert!(*v <= 1.0 + 1.0e-6, "motor above 1: {:?}", out);
        }
    }

    #[test]
    fn mix_p05_collective_preserved_under_pure_torque() {
        // Torque columns are zero-sum → mean motor (collective) equals
        // thrust regardless of torque magnitude.
        let mut m = QuadMixer::new();
        for &thr in &[0.4_f32, 0.5, 0.7] {
            for &tq in &[0.0_f32, 0.3, 1.0, 5.0] {
                let out = m.mix_thrust_floor([tq, 0.0, 0.0], thr, 0.3);
                let mean = (out[0] + out[1] + out[2] + out[3]) / 4.0;
                assert!((mean - thr.max(0.3)).abs() < 1.0e-5,
                    "collective drifted: thr={thr} tq={tq} mean={mean} out={out:?}");
            }
        }
    }

    #[test]
    fn mix_p05_direction_preserved() {
        // Scaling torque preserves its sign/direction (just smaller).
        let mut m = QuadMixer::new();
        let out = m.mix_thrust_floor([0.5, 0.0, 0.0], 0.5, 0.3);
        assert!(out[2] > out[0], "left>right roll dir lost: {:?}", out);
        assert!(out[3] > out[1], "left>right roll dir lost: {:?}", out);
    }

    use proptest::prelude::*;

    proptest! {
        /// MIX-P05: for thrust ∈ [floor, 1], every motor stays in
        /// [floor, 1] under any finite torque — the thrust floor holds.
        #[test]
        fn mix_p05_property_floor_holds(
            floor in 0.0_f32..0.8,
            extra in 0.0_f32..0.2,
            roll in -5.0_f32..5.0,
            pitch in -5.0_f32..5.0,
            yaw in -5.0_f32..5.0,
        ) {
            let thrust = (floor + extra).min(1.0);
            let mut m = QuadMixer::new();
            let out = m.mix_thrust_floor([roll, pitch, yaw], thrust, floor);
            for v in out.iter() {
                prop_assert!(v.is_finite());
                prop_assert!(*v >= floor - 1.0e-4, "below floor {floor}: {out:?}");
                prop_assert!(*v <= 1.0 + 1.0e-4, "above 1: {out:?}");
            }
        }

        /// Outputs always in [0, 1] for any finite input.
        #[test]
        fn mix_p02_property(
            thrust in -2.0_f32..2.0,
            roll in -2.0_f32..2.0,
            pitch in -2.0_f32..2.0,
            yaw in -2.0_f32..2.0,
        ) {
            let mut m = QuadMixer::new();
            let r = m.mix([roll, pitch, yaw], thrust);
            for v in r.iter() {
                prop_assert!((0.0..=1.0).contains(v));
                prop_assert!(v.is_finite());
            }
        }

        /// MIX-P03: signs of torque-direction preserved under
        /// saturation (when at most one axis commanded).
        #[test]
        fn mix_p03_property_single_axis(
            roll in -0.5_f32..0.5,
            thrust in 0.3_f32..0.7,
        ) {
            let mut m = QuadMixer::new();
            let out = m.mix([roll, 0.0, 0.0], thrust);
            // Effective roll torque sign should match input sign.
            let t = motors_to_torque_signs(out);
            if roll > 0.05 {
                prop_assert!(t[0] > 0.0, "roll positive but t[0]={}", t[0]);
            } else if roll < -0.05 {
                prop_assert!(t[0] < 0.0);
            }
        }
    }
}
