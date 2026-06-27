//! Relay arming sequencer — verified startup state machine.
//!
//! Kills falcon root-cause #1 ("startup tip-over"): at spawn the body
//! is subject to rotor-imbalance + IMU transients. If the rate-PID is
//! allowed to demand attitude torque immediately, the mixer converts
//! those transients into bang-bang motor commands and the body tumbles
//! off-axis before steady state can establish.
//!
//! The pre-v0.19.9 bench worked around this with an ad-hoc `t < 0.5 s`
//! time gate that zeroed torque. That gate is *time-only*: it engages
//! the rate loop at t=0.5 s regardless of whether the body is actually
//! level. If the quad spawned tilted, or was bumped during spin-up, the
//! time gate engages torque into a bad attitude anyway.
//!
//! This sequencer replaces it with a state machine that gates torque
//! authority on **time AND confirmed-level**:
//!
//! ```text
//!   Disarmed ──arm()──▶ SpinUp ──spinup_ticks──▶ LevelHold ──┐
//!   (thrust 0,         (thrust ramp 0→1,        (thrust 1,    │ level_count
//!    torque off)        torque off)              torque off)  │ ≥ required
//!                                                             ▼
//!                                                          Armed
//!                                                       (torque ON)
//! ```
//!
//! ## Verified property (the RC#1 killer)
//!
//! **ARM-P01 (torque gating):** attitude-torque authority is enabled
//! *only* in `Armed`, and `Armed` is reachable *only* through `LevelHold`
//! after `level_count` reaches `level_ticks_required` consecutive
//! below-threshold-tilt ticks. Equivalently: `torque_authority` can never
//! be true unless the body was confirmed level. This is an **inductive**
//! invariant over a single `tick()`, so Kani's bounded check covers all
//! (unbounded-length) input sequences — see `kani_proofs`.
//!
//! Why Kani, not Verus: the only float in the machine is the tilt
//! threshold compare; the rest is integer phase/counter logic. Like
//! `relay-mix-quad`, this is plain Rust + Kani (CBMC bit-blasts the f32
//! compare, NaN included) + proptest, not the integer-only Verus track.

#![no_std]

/// Phase encoded as a `u8` so the Kani invariant can reason about it as
/// a bounded integer (`phase <= ARMED`). Values are stable wire codes.
pub const DISARMED: u8 = 0;
pub const SPINUP: u8 = 1;
pub const LEVEL_HOLD: u8 = 2;
pub const ARMED: u8 = 3;

/// Static configuration. The bench derives tick counts from its loop dt
/// (e.g. `spinup_ticks = round(0.3 s / dt)`).
#[derive(Clone, Copy)]
pub struct ArmingConfig {
    /// Ticks spent ramping thrust 0→1 with torque OFF before the level
    /// check begins. Lets the props reach idle without a jerk.
    pub spinup_ticks: u32,
    /// Consecutive below-threshold-tilt ticks required to confirm level
    /// and transition LevelHold → Armed.
    pub level_ticks_required: u32,
    /// |tilt| (rad) must be strictly below this to count as a level tick.
    pub tilt_thresh_rad: f32,
}

impl ArmingConfig {
    /// Defaults tuned for the falcon-quad bench at 100 Hz:
    /// 0.3 s spin-up, 0.1 s (10 ticks) of confirmed level, 5° threshold.
    pub const fn falcon_quad_100hz() -> Self {
        ArmingConfig { spinup_ticks: 30, level_ticks_required: 10, tilt_thresh_rad: 0.087 }
    }
}

/// Per-tick output handed to the bench / cascade glue.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmingOutput {
    pub phase: u8,
    /// `true` iff the rate loop's attitude torque may be applied. False
    /// in every phase except `Armed`.
    pub torque_authority: bool,
    /// Collective-thrust scale in [0, 1]. Ramps during SpinUp, 1.0 after.
    pub thrust_scale: f32,
}

/// The arming state machine. `tick()` advances it one control step.
pub struct ArmingSequencer {
    phase: u8,
    /// Ticks elapsed in the current phase (drives the SpinUp ramp).
    tick_in_phase: u32,
    /// Consecutive below-threshold-tilt ticks seen in LevelHold.
    level_count: u32,
    cfg: ArmingConfig,
}

impl ArmingSequencer {
    pub fn new(cfg: ArmingConfig) -> Self {
        ArmingSequencer { phase: DISARMED, tick_in_phase: 0, level_count: 0, cfg }
    }

    pub fn phase(&self) -> u8 { self.phase }
    pub fn level_count(&self) -> u32 { self.level_count }

    /// Advance one control tick.
    ///
    /// * `tilt_rad` — magnitude of the body tilt from vertical (rad). A
    ///   NaN/∞ tilt never counts as level (the `< thresh` compare is
    ///   false), so a degenerate estimate can never arm the vehicle.
    /// * `arm_request` — operator/bench arm command. Moves Disarmed →
    ///   SpinUp; ignored in later phases (the machine latches forward).
    pub fn tick(&mut self, tilt_rad: f32, arm_request: bool) -> ArmingOutput {
        match self.phase {
            DISARMED if arm_request => {
                self.phase = SPINUP;
                self.tick_in_phase = 0;
            }
            SPINUP => {
                self.tick_in_phase = self.tick_in_phase.saturating_add(1);
                if self.tick_in_phase >= self.cfg.spinup_ticks {
                    self.phase = LEVEL_HOLD;
                    self.tick_in_phase = 0;
                    self.level_count = 0;
                }
            }
            LEVEL_HOLD => {
                // NaN-safe: `< thresh` is false for NaN/∞ → resets count.
                if tilt_rad.abs() < self.cfg.tilt_thresh_rad {
                    self.level_count = self.level_count.saturating_add(1);
                } else {
                    self.level_count = 0;
                }
                if self.level_count >= self.cfg.level_ticks_required {
                    // The ONLY edge into Armed — guarded by the level gate.
                    self.phase = ARMED;
                }
            }
            _ => { /* ARMED (and any out-of-range code) latches. */ }
        }

        ArmingOutput {
            phase: self.phase,
            torque_authority: self.phase == ARMED,
            thrust_scale: self.thrust_scale(),
        }
    }

    /// Collective-thrust scale for the current phase, clamped to [0, 1]
    /// and total (never NaN/∞, even if `spinup_ticks == 0`).
    fn thrust_scale(&self) -> f32 {
        match self.phase {
            DISARMED => 0.0,
            SPINUP => {
                if self.cfg.spinup_ticks == 0 {
                    1.0
                } else {
                    let r = self.tick_in_phase as f32 / self.cfg.spinup_ticks as f32;
                    // clamp also maps a non-finite r (impossible here, but
                    // keeps the function total for the Kani total harness).
                    if r.is_nan() { 0.0 } else { r.clamp(0.0, 1.0) }
                }
            }
            _ => 1.0, // LevelHold, Armed (and latched codes): full thrust.
        }
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// Build an arbitrary *reachable-shaped* sequencer state and config.
    /// We bound the config to sane ranges (Kani needs finite domains) and
    /// constrain the state to satisfy the inductive invariant on entry —
    /// then prove `tick()` preserves it. Induction over ticks then covers
    /// all input sequences of any length.
    fn arbitrary() -> (ArmingSequencer, f32, bool) {
        let phase: u8 = kani::any();
        kani::assume(phase <= ARMED);
        let spinup_ticks: u32 = kani::any();
        kani::assume(spinup_ticks <= 1000);
        let level_ticks_required: u32 = kani::any();
        kani::assume(level_ticks_required >= 1 && level_ticks_required <= 1000);
        let tilt_thresh_rad: f32 = kani::any();
        kani::assume(tilt_thresh_rad.is_finite() && tilt_thresh_rad > 0.0);

        let tick_in_phase: u32 = kani::any();
        kani::assume(tick_in_phase <= 2000);
        let level_count: u32 = kani::any();
        // Inductive precondition: not already past the gate while in
        // LevelHold (else we'd already be Armed).
        kani::assume(level_count < level_ticks_required);

        let cfg = ArmingConfig { spinup_ticks, level_ticks_required, tilt_thresh_rad };
        let seq = ArmingSequencer { phase, tick_in_phase, level_count, cfg };
        (seq, kani::any(), kani::any())
    }

    /// ARM-P01 — torque gating + phase bounds + thrust totality, proved
    /// inductive over one tick.
    #[kani::proof]
    fn verify_arming_torque_gated() {
        let (mut seq, tilt, arm_req) = arbitrary();
        let prev_phase = seq.phase;
        let prev_level = seq.level_count;
        let req = seq.cfg.level_ticks_required;

        let out = seq.tick(tilt, arm_req);

        // Output torque authority is exactly "we are Armed".
        assert!(out.torque_authority == (seq.phase == ARMED));
        assert!(out.phase == seq.phase);
        // Phase stays in range.
        assert!(seq.phase <= ARMED);
        // RC#1 killer: a *new* entry into Armed can come ONLY from
        // LevelHold, and only once the level gate is satisfied.
        if prev_phase != ARMED && seq.phase == ARMED {
            assert!(prev_phase == LEVEL_HOLD);
            assert!(prev_level + 1 >= req);
        }
        // Thrust scale is always a valid, finite collective in [0,1].
        assert!(out.thrust_scale.is_finite());
        assert!(out.thrust_scale >= 0.0 && out.thrust_scale <= 1.0);
    }

    /// Totality — `tick()` is panic-free and thrust stays bounded even
    /// for NaN/∞ tilt and a zero spin-up duration.
    #[kani::proof]
    fn verify_arming_total() {
        let phase: u8 = kani::any();
        let spinup_ticks: u32 = kani::any();
        kani::assume(spinup_ticks <= 1000);
        let level_ticks_required: u32 = kani::any();
        kani::assume(level_ticks_required <= 1000);
        let tilt_thresh_rad: f32 = kani::any();
        let cfg = ArmingConfig { spinup_ticks, level_ticks_required, tilt_thresh_rad };
        let tick_in_phase: u32 = kani::any();
        let level_count: u32 = kani::any();
        let mut seq = ArmingSequencer { phase, tick_in_phase, level_count, cfg };

        let out = seq.tick(kani::any(), kani::any()); // tilt may be NaN/∞
        assert!(out.thrust_scale.is_finite());
        assert!(out.thrust_scale >= 0.0 && out.thrust_scale <= 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> ArmingConfig {
        ArmingConfig { spinup_ticks: 5, level_ticks_required: 3, tilt_thresh_rad: 0.087 }
    }

    /// ARM-P01: torque authority is never granted before the level gate.
    #[test]
    fn arm_p01_torque_gated_until_level_confirmed() {
        let mut s = ArmingSequencer::new(cfg());
        // Disarmed: torque off no matter what.
        let o = s.tick(0.0, false);
        assert_eq!(o.phase, DISARMED);
        assert!(!o.torque_authority);

        // Arm → SpinUp; torque stays off through the whole ramp.
        for _ in 0..5 {
            let o = s.tick(0.0, true);
            assert!(!o.torque_authority, "torque must stay off during spin-up");
            assert_eq!(o.phase, SPINUP);
        }
        // Next tick enters LevelHold (still no torque).
        let o = s.tick(0.0, true);
        assert_eq!(o.phase, LEVEL_HOLD);
        assert!(!o.torque_authority);

        // Three consecutive level ticks confirm → Armed on the 3rd.
        s.tick(0.0, true); // level_count 1
        s.tick(0.0, true); // level_count 2
        let o = s.tick(0.0, true); // level_count 3 == required → Armed
        assert_eq!(o.phase, ARMED);
        assert!(o.torque_authority, "torque engages only after level confirmed");
    }

    /// A tilt above threshold during LevelHold resets the counter, so the
    /// vehicle cannot arm while it is not level — the property the
    /// time-only `t < 0.5` hack could not provide.
    #[test]
    fn arm_p01_tilt_resets_level_count_prevents_arming() {
        let mut s = ArmingSequencer::new(cfg());
        s.tick(0.0, true);
        for _ in 0..5 { s.tick(0.0, true); } // reach LevelHold
        assert_eq!(s.phase(), LEVEL_HOLD);

        s.tick(0.0, true); // count 1
        s.tick(0.0, true); // count 2
        // A tilt spike (10°, above the 5° threshold) wipes the progress.
        let o = s.tick(0.2, true);
        assert_eq!(s.level_count(), 0, "tilt must reset the level counter");
        assert!(!o.torque_authority);
        assert_eq!(o.phase, LEVEL_HOLD);
    }

    /// NaN tilt can never confirm level — a degenerate estimate must not
    /// arm the vehicle.
    #[test]
    fn arm_p01_nan_tilt_never_arms() {
        let mut s = ArmingSequencer::new(cfg());
        s.tick(0.0, true);
        for _ in 0..5 { s.tick(0.0, true); }
        for _ in 0..100 {
            let o = s.tick(f32::NAN, true);
            assert!(!o.torque_authority, "NaN tilt must never arm");
        }
    }

    /// Thrust ramps monotonically 0→1 during spin-up and holds at 1.
    #[test]
    fn spinup_thrust_ramps_monotonic() {
        let mut s = ArmingSequencer::new(cfg());
        let mut last = -1.0_f32;
        s.tick(0.0, true); // enter SpinUp (tick_in_phase becomes 1)
        for _ in 0..5 {
            let o = s.tick(0.0, true);
            assert!(o.thrust_scale >= last, "thrust scale must not decrease in spin-up");
            assert!((0.0..=1.0).contains(&o.thrust_scale));
            last = o.thrust_scale;
        }
    }

    proptest::proptest! {
        /// Across arbitrary tick sequences, torque authority implies the
        /// machine is in Armed, and thrust_scale stays a finite [0,1].
        #[test]
        fn arm_p01_property_torque_implies_armed(
            tilts in proptest::collection::vec(-1.0_f32..1.0, 0..400),
            arm_at in 0usize..50,
        ) {
            let mut s = ArmingSequencer::new(cfg());
            for (i, &tilt) in tilts.iter().enumerate() {
                let o = s.tick(tilt, i >= arm_at);
                proptest::prop_assert!(o.thrust_scale.is_finite());
                proptest::prop_assert!((0.0..=1.0).contains(&o.thrust_scale));
                proptest::prop_assert_eq!(o.torque_authority, o.phase == ARMED);
            }
        }
    }
}
