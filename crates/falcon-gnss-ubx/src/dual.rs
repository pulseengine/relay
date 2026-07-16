//! Dual-receiver GNSS selection/blending (GNSS-P02, v1.121).
//!
//! The first vehicle carries 2× u-blox M9N; the estimator input was a
//! single lane. This selector turns two (possibly absent, possibly lying)
//! fixes into ONE estimator input plus a divergence flag:
//!
//! - **Health per receiver**: presence, 3D fix, satellite count, reported
//!   accuracy, and an innovation gate against the estimator's position (a
//!   jump the filter would reject disqualifies the receiver THIS cycle).
//! - **Inverse-variance blending** when both are healthy — the output sits
//!   between the fixes, weighted by reported accuracy.
//! - **Failover within one update** when the active side degrades: the
//!   output continues from the healthy receiver; because blending always
//!   lies between the two fixes, the step on failover is bounded by the
//!   healthy receiver's distance to the blend — within its own accuracy
//!   when the receivers agreed.
//! - **Divergence flag** (latched): sustained disagreement beyond the gate
//!   raises `diverged` for the spoof monitor and pre-arm checks; the
//!   selector then trusts the receiver consistent with the estimator.
//!
//! Pure, `no_std`, total (GNSS-K05): any pair of inputs — including
//! NaN/∞-poisoned fixes — yields a finite selection or an explicit
//! no-fix, never a NaN.

/// One receiver's fix for one selector cycle, in local NED metres
/// (projection from LLH happens at the driver/integration layer).
#[derive(Clone, Copy, Debug)]
pub struct NedFix {
    pub pos: [f32; 3],
    /// Reported horizontal accuracy estimate, metres (M9N hAcc).
    pub acc_m: f32,
    pub sats: u8,
    /// 3D fix (or better) reported.
    pub fix_ok: bool,
}

/// Which source produced this cycle's output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GnssSource {
    A,
    B,
    Blend,
    None,
}

/// The selector's per-cycle decision.
#[derive(Clone, Copy, Debug)]
pub struct GnssDecision {
    /// The estimator input: a finite NED position, or `None` (no usable fix).
    pub pos: Option<[f32; 3]>,
    /// Accuracy accompanying `pos` (the blend's, or the selected receiver's).
    pub acc_m: f32,
    pub source: GnssSource,
    /// Latched: the two receivers disagreed beyond the divergence gate for
    /// `DIVERGE_DEBOUNCE` consecutive dual-healthy cycles — spoof-monitor
    /// and pre-arm input. Cleared only by [`DualGnss::reset`].
    pub diverged: bool,
}

/// Satellite floor for a receiver to count as healthy.
pub const MIN_SATS: u8 = 6;
/// Accuracy ceiling (m) — a receiver reporting worse is unhealthy.
pub const MAX_ACC_M: f32 = 10.0;
/// Accuracy floor (m) — reports below are clamped (an M9N never reports
/// sub-centimetre; a denormal acc would overflow the 1/acc² blend weight
/// into ∞−∞ = NaN — caught by GNSS-K05).
pub const MIN_ACC_M: f32 = 0.01;
/// Position sanity bound (m): a |NED| beyond this is a lying receiver,
/// and bounding it keeps every blend product finite (no f32 overflow).
pub const MAX_POS_M: f32 = 1.0e6;
/// Innovation gate (m) against the estimator position: a fix jumping
/// further than this from the estimate is disqualified this cycle.
pub const INNOVATION_GATE_M: f32 = 15.0;
/// Divergence gate between the two receivers, in multiples of their
/// combined reported accuracy (floored at 5 m absolute).
pub const DIVERGE_ACC_MULT: f32 = 3.0;
pub const DIVERGE_FLOOR_M: f32 = 5.0;
/// Consecutive dual-healthy divergent cycles before the flag latches.
pub const DIVERGE_DEBOUNCE: u32 = 10;

fn sane3(p: &[f32; 3]) -> bool {
    p.iter().all(|v| v.is_finite() && *v >= -MAX_POS_M && *v <= MAX_POS_M)
}

/// Squared 2D distance — every gate compares SQUARED quantities (order-
/// preserving for non-negatives), keeping sqrt out of the decision path
/// entirely (Kani tractability + no libm on the hot path).
fn dist2d_sq(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}

/// Health-gate one receiver's fix: present, finite, 3D, enough sats,
/// sane accuracy, and (when an estimate exists) inside the innovation gate.
fn healthy(fix: &Option<NedFix>, est: Option<[f32; 3]>) -> Option<NedFix> {
    let f = (*fix)?;
    if !f.fix_ok || f.sats < MIN_SATS {
        return None;
    }
    if !sane3(&f.pos) || !f.acc_m.is_finite() || f.acc_m <= 0.0 || f.acc_m > MAX_ACC_M {
        return None;
    }
    let f = NedFix { acc_m: f.acc_m.max(MIN_ACC_M), ..f };
    if let Some(e) = est {
        if sane3(&e) && dist2d_sq(&f.pos, &e) > INNOVATION_GATE_M * INNOVATION_GATE_M {
            return None;
        }
    }
    Some(f)
}

/// The dual-receiver selector. One instance per vehicle; call
/// [`update`](Self::update) once per fix interval.
pub struct DualGnss {
    diverge_count: u32,
    diverged: bool,
}

impl Default for DualGnss {
    fn default() -> Self {
        Self::new()
    }
}

impl DualGnss {
    pub fn new() -> Self {
        DualGnss { diverge_count: 0, diverged: false }
    }

    /// Clear the latched divergence flag (ground reset only).
    pub fn reset(&mut self) {
        self.diverge_count = 0;
        self.diverged = false;
    }

    /// One selection cycle. `est` is the estimator's current NED position
    /// (the innovation-consistency reference), if converged.
    pub fn update(
        &mut self,
        a: Option<NedFix>,
        b: Option<NedFix>,
        est: Option<[f32; 3]>,
    ) -> GnssDecision {
        let ha = healthy(&a, est);
        let hb = healthy(&b, est);
        match (ha, hb) {
            (Some(fa), Some(fb)) => {
                // Divergence watch (only meaningful with both healthy).
                let gate = DIVERGE_FLOOR_M.max(DIVERGE_ACC_MULT * (fa.acc_m + fb.acc_m));
                if dist2d_sq(&fa.pos, &fb.pos) > gate * gate {
                    self.diverge_count = self.diverge_count.saturating_add(1);
                    if self.diverge_count >= DIVERGE_DEBOUNCE {
                        self.diverged = true;
                    }
                } else {
                    self.diverge_count = 0;
                }
                if self.diverged {
                    // Trust the receiver consistent with the estimator when
                    // one exists; else the better-accuracy one — but never
                    // blend two stories that contradict each other.
                    let pick_a = match est {
                        Some(e) if sane3(&e) => dist2d_sq(&fa.pos, &e) <= dist2d_sq(&fb.pos, &e),
                        _ => fa.acc_m <= fb.acc_m,
                    };
                    let f = if pick_a { fa } else { fb };
                    return GnssDecision {
                        pos: Some(f.pos),
                        acc_m: f.acc_m,
                        source: if pick_a { GnssSource::A } else { GnssSource::B },
                        diverged: true,
                    };
                }
                // Inverse-variance blend: w ∝ 1/acc².
                let wa = 1.0 / (fa.acc_m * fa.acc_m);
                let wb = 1.0 / (fb.acc_m * fb.acc_m);
                let wsum = wa + wb;
                let mut pos = [0.0f32; 3];
                for (i, p) in pos.iter_mut().enumerate() {
                    *p = (fa.pos[i] * wa + fb.pos[i] * wb) / wsum;
                }
                // Blended accuracy: conservatively the better receiver's
                // (the true inverse-variance value is smaller; reporting
                // min keeps the field sqrt-free and never over-claims).
                let acc = if fa.acc_m <= fb.acc_m { fa.acc_m } else { fb.acc_m };
                GnssDecision { pos: Some(pos), acc_m: acc, source: GnssSource::Blend, diverged: false }
            }
            (Some(f), None) => GnssDecision {
                pos: Some(f.pos),
                acc_m: f.acc_m,
                source: GnssSource::A,
                diverged: self.diverged,
            },
            (None, Some(f)) => GnssDecision {
                pos: Some(f.pos),
                acc_m: f.acc_m,
                source: GnssSource::B,
                diverged: self.diverged,
            },
            (None, None) => GnssDecision {
                pos: None,
                acc_m: f32::MAX,
                source: GnssSource::None,
                diverged: self.diverged,
            },
        }
    }

    pub fn diverged(&self) -> bool {
        self.diverged
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(x: f32, y: f32, acc: f32) -> Option<NedFix> {
        Some(NedFix { pos: [x, y, -10.0], acc_m: acc, sats: 12, fix_ok: true })
    }

    /// Both healthy: the blend lies between the fixes, weighted toward the
    /// more accurate receiver, and the blended accuracy beats both.
    #[test]
    fn blends_toward_the_better_receiver() {
        let mut d = DualGnss::new();
        let dec = d.update(fix(0.0, 0.0, 1.0), fix(1.0, 0.0, 2.0), None);
        let p = dec.pos.unwrap();
        assert_eq!(dec.source, GnssSource::Blend);
        assert!(p[0] > 0.0 && p[0] < 0.5, "weighted toward A (acc 1 vs 2): {}", p[0]);
        assert!(dec.acc_m <= 1.0, "blend accuracy never worse than the best receiver");
        assert!(!dec.diverged);
    }

    /// Failover within ONE update: receiver A dies; the output continues
    /// from B with a step bounded by the healthy receiver's accuracy.
    #[test]
    fn failover_within_one_fix_interval() {
        let mut d = DualGnss::new();
        let before = d.update(fix(0.0, 0.0, 1.0), fix(0.8, 0.0, 1.0), None);
        let p0 = before.pos.unwrap();
        // A drops out entirely:
        let after = d.update(None, fix(0.8, 0.0, 1.0), None);
        assert_eq!(after.source, GnssSource::B);
        let p1 = after.pos.unwrap();
        let step = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2)).sqrt();
        assert!(step <= 1.0, "failover step {} must stay within B's accuracy", step);
    }

    /// Degradation failovers: accuracy collapse and a jump the innovation
    /// gate rejects both disqualify a receiver the SAME cycle.
    #[test]
    fn accuracy_collapse_and_jump_disqualify() {
        let mut d = DualGnss::new();
        let dec = d.update(
            Some(NedFix { pos: [0.0; 3], acc_m: 50.0, sats: 12, fix_ok: true }),
            fix(0.1, 0.0, 1.0),
            None,
        );
        assert_eq!(dec.source, GnssSource::B, "acc 50 m > ceiling disqualifies A");
        // jump: estimator at origin, A reports 40 m away.
        let dec = d.update(fix(40.0, 0.0, 1.0), fix(0.1, 0.0, 1.0), Some([0.0; 3]));
        assert_eq!(dec.source, GnssSource::B, "innovation gate rejects the jump");
    }

    /// Divergence: sustained disagreement latches the flag; the selector
    /// stops blending and picks the estimator-consistent receiver.
    #[test]
    fn divergence_latches_and_picks_consistent() {
        // A split inside the 15 m innovation gate but beyond the divergence
        // gate (5 m floor, 3·(1+1)=6 m): 10 m apart, estimator nearer A.
        let mut d = DualGnss::new();
        let mut last = None;
        for _ in 0..DIVERGE_DEBOUNCE {
            last = Some(d.update(fix(0.0, 0.0, 1.0), fix(10.0, 0.0, 1.0), Some([2.0, 0.0, -10.0])));
        }
        let dec = last.unwrap();
        assert!(dec.diverged, "sustained 10 m split must latch divergence");
        assert_eq!(dec.source, GnssSource::A, "picks the estimator-consistent side");
        // and it stays latched through re-agreement:
        let dec = d.update(fix(0.0, 0.0, 1.0), fix(0.1, 0.0, 1.0), None);
        assert!(dec.diverged, "divergence is latched");
    }

    /// A brief disagreement below the debounce does NOT latch.
    #[test]
    fn transient_divergence_does_not_latch() {
        let mut d = DualGnss::new();
        for _ in 0..(DIVERGE_DEBOUNCE - 1) {
            d.update(fix(0.0, 0.0, 1.0), fix(10.0, 0.0, 1.0), None);
        }
        let dec = d.update(fix(0.0, 0.0, 1.0), fix(0.1, 0.0, 1.0), None);
        assert!(!dec.diverged);
        assert_eq!(dec.source, GnssSource::Blend);
    }

    /// No usable fix on either side: explicit no-fix, never a made-up pos.
    #[test]
    fn dual_loss_is_no_fix() {
        let mut d = DualGnss::new();
        let dec = d.update(None, None, None);
        assert!(dec.pos.is_none());
        assert_eq!(dec.source, GnssSource::None);
        let dec = d.update(
            Some(NedFix { pos: [f32::NAN; 3], acc_m: 1.0, sats: 12, fix_ok: true }),
            Some(NedFix { pos: [0.0; 3], acc_m: 1.0, sats: 3, fix_ok: true }),
            None,
        );
        assert!(dec.pos.is_none(), "NaN pos and 3 sats both unhealthy");
    }
}
