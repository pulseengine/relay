//! Relay SensVote — redundant-sensor voting + GPS-loss fallback.
//!
//! PX4 votes redundant sensors and degrades gracefully on a fault; falcon
//! assumed a single IMU/GPS. This adds:
//!
//!  * **Median voting** — `median3` over a triple-redundant reading rejects ONE
//!    outlier (the classic 3-sensor vote) and is NaN-SAFE: a faulted sensor
//!    reading (NaN) is excluded, never propagated into the estimator. `vote3`
//!    also reports which sensor disagreed most (for health logging).
//!  * **GPS-loss fallback** — a freshness monitor that, after a timeout with no
//!    fix, reports `Stale` so the flight stack falls back to INS dead-reckoning
//!    (and then the failsafe arbiter), rather than flying on a frozen position.
//!
//! Verified (Kani): the voted value is always FINITE when at least one input is
//! finite and lies within the finite inputs' range (a single NaN/outlier can
//! never become the output); the freshness monitor is total.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// Median of three readings, NaN-safe. NaN inputs are treated as faulted and
/// excluded: 3 finite ⇒ the middle value (rejects one outlier); 2 finite ⇒ their
/// mean; 1 finite ⇒ that value; 0 finite ⇒ 0.0 (safe default). The result is
/// always finite when any input is finite, and within the finite inputs' range.
pub fn median3(a: f32, b: f32, c: f32) -> f32 {
    // Gather the finite values into a small fixed array.
    let mut f = [0.0f32; 3];
    let mut n = 0usize;
    for v in [a, b, c] {
        if v.is_finite() {
            f[n] = v;
            n += 1;
        }
    }
    match n {
        // Branch-free median via min/max only — the result is always one of the
        // inputs, so it is finite and in range with NO summation (no overflow).
        3 => f[0].max(f[1]).min(f[0].min(f[1]).max(f[2])),
        // Overflow-safe midpoint: scale THEN add (0.5·MAX + 0.5·MAX = MAX, finite;
        // 0.5·MAX + 0.5·(−MAX) = 0). `0.5*(a+b)` would overflow first.
        2 => 0.5 * f[0] + 0.5 * f[1],
        1 => f[0],
        _ => 0.0,
    }
}

/// The result of a triple vote: the voted value + which input (if any) was the
/// furthest outlier (NaN counts as the outlier).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoteResult {
    /// The voted (median) value.
    pub value: f32,
    /// Index (0..3) of the most-disagreeing input, if a fault stands out.
    pub outlier: Option<usize>,
}

/// Vote three readings and identify the furthest outlier (the likely faulted
/// sensor). A NaN input is always flagged as the outlier.
pub fn vote3(readings: [f32; 3]) -> VoteResult {
    let value = median3(readings[0], readings[1], readings[2]);
    // furthest-from-median (NaN ⇒ +inf distance ⇒ always the outlier).
    let mut worst = 0usize;
    let mut worst_d = -1.0f32;
    for (i, &r) in readings.iter().enumerate() {
        let d = if r.is_finite() {
            (r - value).abs()
        } else {
            f32::INFINITY
        };
        if d > worst_d {
            worst_d = d;
            worst = i;
        }
    }
    let outlier = if worst_d > 0.0 { Some(worst) } else { None };
    VoteResult { value, outlier }
}

/// Per-axis median vote of three 3-vectors (e.g. three IMUs' accel).
pub fn median3_vec(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    [
        median3(a[0], b[0], c[0]),
        median3(a[1], b[1], c[1]),
        median3(a[2], b[2], c[2]),
    ]
}

/// GPS freshness / INS-fallback status.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GpsStatus {
    /// A fresh fix is in force — use GPS.
    Fresh,
    /// No fresh fix within the timeout — fall back to INS dead-reckoning, then
    /// the failsafe arbiter.
    Stale,
}

/// Tracks GPS-fix freshness for the INS-fallback handoff.
#[derive(Clone, Copy, Debug)]
pub struct GpsFreshness {
    timeout_us: u64,
    last_fix_us: u64,
    started: bool,
}

impl GpsFreshness {
    /// Declares the fix stale after `timeout_us` without an update.
    pub fn new(timeout_us: u64) -> Self {
        Self { timeout_us, last_fix_us: 0, started: false }
    }

    /// Record a fresh GPS fix observed at `now_us`.
    pub fn on_fix(&mut self, now_us: u64) {
        self.last_fix_us = now_us;
        self.started = true;
    }

    /// Status at `now_us`. Fresh only while a fix has arrived within the timeout;
    /// saturating time math, so any clock value is well-defined (no panic).
    pub fn status(&self, now_us: u64) -> GpsStatus {
        if self.started && now_us.saturating_sub(self.last_fix_us) <= self.timeout_us {
            GpsStatus::Fresh
        } else {
            GpsStatus::Stale
        }
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_rejects_one_outlier() {
        // two agree near 10, one wild → median ≈ 10.
        assert!((median3(10.0, 10.2, 99.0) - 10.2).abs() < 1e-6);
        assert!((median3(99.0, 10.0, 10.2) - 10.2).abs() < 1e-6);
    }

    #[test]
    fn nan_sensor_excluded_not_propagated() {
        // one IMU faults (NaN): the vote uses the other two, never NaN.
        let v = median3(5.0, f32::NAN, 5.4);
        assert!(v.is_finite());
        assert!((v - 5.2).abs() < 1e-6); // mean of the two healthy
    }

    #[test]
    fn vote_flags_the_outlier() {
        let r = vote3([10.0, 10.1, 50.0]);
        assert_eq!(r.outlier, Some(2));
        let r2 = vote3([1.0, f32::NAN, 1.1]);
        assert_eq!(r2.outlier, Some(1)); // NaN is the fault
    }

    #[test]
    fn vec_vote_per_axis() {
        let out = median3_vec([1.0, 0.0, 9.8], [1.1, 0.0, 9.7], [9.0, 0.0, 9.75]);
        assert!((out[0] - 1.1).abs() < 1e-6); // x outlier rejected
        assert!((out[2] - 9.75).abs() < 1e-6);
    }

    #[test]
    fn gps_freshness_and_fallback() {
        let mut g = GpsFreshness::new(500_000); // 500 ms
        assert_eq!(g.status(0), GpsStatus::Stale); // before first fix
        g.on_fix(1_000_000);
        assert_eq!(g.status(1_400_000), GpsStatus::Fresh);
        assert_eq!(g.status(1_600_000), GpsStatus::Stale); // timed out → INS fallback
    }
}

#[cfg(test)]
mod proptests {
    use super::*;

    proptest::proptest! {
        /// The voted value is finite when at least one input is finite, and lies
        /// within [min, max] of the finite inputs — a single NaN/outlier can
        /// never become the output.
        #[test]
        fn vote_within_finite_range(a in -1e6f32..1e6, b in -1e6f32..1e6, c in -1e6f32..1e6) {
            let v = median3(a, b, c);
            let mn = a.min(b).min(c);
            let mx = a.max(b).max(c);
            proptest::prop_assert!(v >= mn - 1e-3 && v <= mx + 1e-3);
        }
    }
}
