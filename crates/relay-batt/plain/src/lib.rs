//! relay-batt — verified battery state estimation (BATTERY-P02, v1.120).
//!
//! Raw pack voltage is a LIAR in both directions: under a throttle punch the
//! I·R sag reads a half-full pack as empty (false failsafe), and at low
//! throttle a genuinely spent pack rebounds above the threshold (hidden
//! emergency). This engine estimates the state the failsafe should actually
//! act on:
//!
//! - **Coulomb counting** — consumed charge integrated from the current
//!   sense (PM02D), drift-bounded to `[0, capacity]`.
//! - **Sag compensation** — resting voltage `v + i·R` with the pack's
//!   internal resistance estimated online from correlated V/I steps
//!   (PX4 parity: `BAT1_R_INTERNAL`, but estimated rather than configured).
//! - **Conservative SoC fusion** — `min(coulomb SoC, OCV SoC)`: EITHER a
//!   high integrated consumption OR a low resting voltage pulls the estimate
//!   down; both must be healthy for the pack to read healthy.
//! - **Flagged voltage-only fallback** — with no current sense there is no
//!   sag compensation and no coulomb count: raw-voltage thresholds apply
//!   with a WIDER margin and the state carries a loud `degraded` flag
//!   (surfaced in SYS_STATUS via [`sys_status_fields`]).
//! - **Debounced, latched failsafe flags** — `low`/`critical` trip only
//!   after a sustained excursion (a pack does not un-discharge in flight;
//!   once tripped they latch until [`BatteryEstimator::reset`]).
//!
//! Verification split (the relay-rc pattern): input sanitization and the
//! trip/latch logic are Kani-proven total in `kani_proofs.rs` (BATT-K01..
//! K03); the f32 arithmetic paths (integration accuracy, sag scenarios,
//! OCV interpolation) are test- and proptest-gated — Kani on nondet f32
//! multiplication is intractable.
//!
//! no_std / no_alloc / forbid(unsafe). Pure: no clock, no I/O — the caller
//! feeds `(dt, volts, amps)` per cycle.

#![no_std]
#![forbid(unsafe_code)]

/// Pack + threshold configuration. Defaults model the first vehicle's
/// 4S 5000 mAh Gens Ace behind a PM02D.
#[derive(Clone, Copy, Debug)]
pub struct BattConfig {
    /// Usable capacity, mAh.
    pub capacity_mah: f32,
    /// Series cell count.
    pub cells: u32,
    /// Initial internal-resistance estimate, whole-pack ohms. Refined
    /// online; a 4S pack with connectors is typically 15–40 mΩ.
    pub r_internal: f32,
    /// State-of-charge low threshold (failsafe: RTL), 0..1.
    pub low_soc: f32,
    /// State-of-charge critical threshold (failsafe: LAND), 0..1.
    pub crit_soc: f32,
    /// Voltage-only fallback LOW threshold, volts per cell. WIDER margin
    /// than the SoC path — without current sense, sag is indistinguishable
    /// from discharge, so the fallback must trip earlier.
    pub low_v_cell_fallback: f32,
    /// Voltage-only fallback CRITICAL threshold, volts per cell.
    pub crit_v_cell_fallback: f32,
    /// Sustained-excursion time before a flag trips, seconds.
    pub debounce_s: f32,
}

impl Default for BattConfig {
    fn default() -> Self {
        BattConfig {
            capacity_mah: 5000.0,
            cells: 4,
            r_internal: 0.024,
            low_soc: 0.25,
            crit_soc: 0.10,
            low_v_cell_fallback: 3.70,
            crit_v_cell_fallback: 3.55,
            debounce_s: 2.0,
        }
    }
}

/// The estimated battery state, returned every update. All fields are
/// finite for ANY input (sanitization is Kani-proven).
#[derive(Clone, Copy, Debug, Default)]
pub struct BattState {
    /// Raw terminal voltage after sanitization, V.
    pub volts: f32,
    /// Sag-compensated resting voltage, V (== `volts` in fallback mode).
    pub rest_volts: f32,
    /// Sanitized discharge current, A (0 in fallback mode).
    pub current_a: f32,
    /// Coulomb-counted consumption, mAh, in [0, capacity].
    pub consumed_mah: f32,
    /// State of charge, 0..1. Conservative min-fusion (coulomb, OCV) with
    /// current sense; OCV of the RAW voltage in fallback mode.
    pub soc: f32,
    /// Online internal-resistance estimate, ohms.
    pub r_est: f32,
    /// Low-battery failsafe flag (latched, debounced).
    pub low: bool,
    /// Critical-battery failsafe flag (latched, debounced).
    pub critical: bool,
    /// True when running the flagged voltage-only fallback (no current
    /// sense). Surfaced in SYS_STATUS — a degraded estimate the operator
    /// must know about.
    pub degraded: bool,
}

/// Clamp with an explicit NaN policy: a non-finite sample becomes
/// `nan_default` (then clamped). Kani-proven total (BATT-K01).
#[inline]
pub fn sanitize(x: f32, lo: f32, hi: f32, nan_default: f32) -> f32 {
    let x = if x.is_finite() { x } else { nan_default };
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// LiPo open-circuit-voltage → state-of-charge, per cell, piecewise linear.
/// Rest voltages (not under load): 3.50 V ⇒ 0, 4.20 V ⇒ 1. Total for any
/// f32 (non-finite ⇒ 0.0: an unreadable voltage reads as EMPTY — the
/// conservative direction for a safety flag).
pub fn ocv_soc_cell(v_cell: f32) -> f32 {
    const CURVE: [(f32, f32); 8] = [
        (3.50, 0.00),
        (3.65, 0.10),
        (3.72, 0.20),
        (3.79, 0.40),
        (3.85, 0.55),
        (3.95, 0.75),
        (4.10, 0.95),
        (4.20, 1.00),
    ];
    let v = sanitize(v_cell, 0.0, 5.0, 0.0);
    if v <= CURVE[0].0 {
        return 0.0;
    }
    let mut i = 1;
    while i < CURVE.len() {
        let (v1, s1) = CURVE[i];
        if v <= v1 {
            let (v0, s0) = CURVE[i - 1];
            return s0 + (s1 - s0) * (v - v0) / (v1 - v0);
        }
        i += 1;
    }
    1.0
}

/// Debounced latch: the flag trips after `debounce_s` of SUSTAINED
/// excursion and stays tripped. Kani-proven (BATT-K02/K03): once latched
/// never clears, and a shorter-than-debounce excursion never trips.
#[derive(Clone, Copy, Debug, Default)]
pub struct TripLatch {
    below_s: f32,
    latched: bool,
}

impl TripLatch {
    /// Advance by `dt` seconds with the excursion condition `below`.
    pub fn update(&mut self, dt: f32, below: bool, debounce_s: f32) -> bool {
        if self.latched {
            return true;
        }
        if below {
            self.below_s += dt;
            if self.below_s >= debounce_s {
                self.latched = true;
            }
        } else {
            self.below_s = 0.0;
        }
        self.latched
    }

    pub fn is_latched(&self) -> bool {
        self.latched
    }
}

/// The estimator. One instance per pack; call [`update`](Self::update)
/// every supervisor cycle.
pub struct BatteryEstimator {
    cfg: BattConfig,
    consumed_mah: f32,
    r_est: f32,
    prev_v: f32,
    prev_i: f32,
    have_prev: bool,
    low: TripLatch,
    critical: TripLatch,
}

/// Sanitization bounds: a 12S pack tops out near 51 V; PM02D senses to
/// ~120 A and the X500 never draws 200. Anything outside is a sensor lie.
const V_MAX: f32 = 60.0;
const I_MAX: f32 = 500.0;
const DT_MAX: f32 = 1.0;
/// Internal-resistance estimate bounds, whole-pack ohms.
const R_MIN: f32 = 0.001;
const R_MAX: f32 = 0.2;
/// Only V/I steps this large update the R estimate (below it the
/// quotient is noise-dominated).
const R_STEP_MIN_A: f32 = 5.0;
/// R-estimate low-pass blend per accepted sample.
const R_ALPHA: f32 = 0.05;

impl BatteryEstimator {
    pub fn new(cfg: BattConfig) -> Self {
        let r0 = sanitize(cfg.r_internal, R_MIN, R_MAX, 0.024);
        BatteryEstimator {
            cfg,
            consumed_mah: 0.0,
            r_est: r0,
            prev_v: 0.0,
            prev_i: 0.0,
            have_prev: false,
            low: TripLatch::default(),
            critical: TripLatch::default(),
        }
    }

    /// Clear latches and the coulomb count (new pack / bench reset). The
    /// R estimate is kept — it is a property of the pack + harness.
    pub fn reset(&mut self) {
        self.consumed_mah = 0.0;
        self.low = TripLatch::default();
        self.critical = TripLatch::default();
        self.have_prev = false;
    }

    /// Advance one cycle: `dt_s` seconds, terminal `volts`, and the
    /// current sense (`None` = no PM02D data ⇒ flagged voltage-only
    /// fallback). Total: any input yields a finite, bounded state.
    pub fn update(&mut self, dt_s: f32, volts: f32, amps: Option<f32>) -> BattState {
        let dt = sanitize(dt_s, 0.0, DT_MAX, 0.0);
        let v = sanitize(volts, 0.0, V_MAX, 0.0);
        let cells = if self.cfg.cells == 0 { 1 } else { self.cfg.cells } as f32;

        match amps {
            Some(a) => {
                let i = sanitize(a, 0.0, I_MAX, 0.0);
                // Coulomb count: A·s → mAh, drift-bounded.
                let cap = sanitize(self.cfg.capacity_mah, 1.0, 1.0e6, 5000.0);
                self.consumed_mah =
                    sanitize(self.consumed_mah + i * dt * (1000.0 / 3600.0), 0.0, cap, cap);

                // Online R: accept only decorrelation-safe big current steps.
                if self.have_prev {
                    let di = i - self.prev_i;
                    let dv = v - self.prev_v;
                    if !(-R_STEP_MIN_A..=R_STEP_MIN_A).contains(&di) {
                        let r_sample = sanitize(-dv / di, R_MIN, R_MAX, self.r_est);
                        self.r_est = sanitize(
                            self.r_est + R_ALPHA * (r_sample - self.r_est),
                            R_MIN,
                            R_MAX,
                            0.024,
                        );
                    }
                }
                self.prev_v = v;
                self.prev_i = i;
                self.have_prev = true;

                let rest = sanitize(v + i * self.r_est, 0.0, V_MAX, 0.0);
                let soc_coulomb = sanitize(1.0 - self.consumed_mah / cap, 0.0, 1.0, 0.0);
                let soc_ocv = ocv_soc_cell(rest / cells);
                // Conservative fusion: either signal low pulls the SoC down.
                let soc = if soc_coulomb < soc_ocv { soc_coulomb } else { soc_ocv };

                let low = self.low.update(dt, soc < self.cfg.low_soc, self.cfg.debounce_s);
                let critical =
                    self.critical.update(dt, soc < self.cfg.crit_soc, self.cfg.debounce_s);
                BattState {
                    volts: v,
                    rest_volts: rest,
                    current_a: i,
                    consumed_mah: self.consumed_mah,
                    soc,
                    r_est: self.r_est,
                    low,
                    critical,
                    degraded: false,
                }
            }
            None => {
                // Voltage-only fallback: no sag compensation possible —
                // WIDER per-cell margins, loud degraded flag. No coulomb
                // progress (consumption unknown, held — not zeroed).
                self.have_prev = false;
                let v_cell = v / cells;
                let low = self.low.update(
                    dt,
                    v_cell < self.cfg.low_v_cell_fallback,
                    self.cfg.debounce_s,
                );
                let critical = self.critical.update(
                    dt,
                    v_cell < self.cfg.crit_v_cell_fallback,
                    self.cfg.debounce_s,
                );
                BattState {
                    volts: v,
                    rest_volts: v,
                    current_a: 0.0,
                    consumed_mah: self.consumed_mah,
                    soc: ocv_soc_cell(v_cell),
                    r_est: self.r_est,
                    low,
                    critical,
                    degraded: true,
                }
            }
        }
    }
}

/// MAV_SYS_STATUS_SENSOR_BATTERY — the SYS_STATUS `onboard_control_sensors_
/// health` bit that goes UNHEALTHY when the estimate is degraded
/// (voltage-only fallback). MAVLink common: bit 33 is battery... the
/// classic 32-bit field uses `MAV_SYS_STATUS_SENSOR_BATTERY = 0x4000000`.
pub const SYS_STATUS_SENSOR_BATTERY: u32 = 0x400_0000;

/// Map a [`BattState`] onto the SYS_STATUS battery fields:
/// `(voltage_battery mV, current_battery cA, battery_remaining %, healthy)`.
/// `healthy == false` ⇔ degraded fallback — clear [`SYS_STATUS_SENSOR_
/// BATTERY`] in `onboard_control_sensors_health` so the GCS shows the
/// battery sensor unhealthy. Saturating, total.
pub fn sys_status_fields(s: &BattState) -> (u16, i16, i8, bool) {
    let mv = sanitize(s.volts * 1000.0, 0.0, 65535.0, 65535.0) as u16;
    let ca = sanitize(s.current_a * 100.0, 0.0, 32767.0, -1.0) as i16;
    let pct = sanitize(s.soc * 100.0, 0.0, 100.0, -1.0) as i8;
    (mv, ca, pct, !s.degraded)
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> BattConfig {
        BattConfig::default()
    }

    /// Half-full pack per coulomb count. A 60C punch (300 A — far beyond
    /// what the X500 can draw; the worst case) sags the terminal voltage
    /// through the raw threshold, but the compensated state does NOT trip
    /// the failsafe.
    #[test]
    fn sag_punch_does_not_false_trigger() {
        let mut est = BatteryEstimator::new(cfg());
        let r = 0.024f32;
        // Cruise at half charge: rest 3.85 V/cell (55% OCV), hover 20 A.
        est.consumed_mah = 2500.0; // half consumed
        let v_rest = 3.85 * 4.0;
        for _ in 0..500 {
            let v = v_rest - 20.0 * r;
            let s = est.update(0.02, v, Some(20.0));
            assert!(!s.low && !s.critical, "cruise at half pack must not trip");
        }
        // 3-second 300 A punch: terminal sags to 3.85*4 - 300*0.024 = 8.2 V
        // (2.05 V/cell — WAY below any raw threshold).
        for _ in 0..150 {
            let v = v_rest - 300.0 * r;
            let s = est.update(0.02, v, Some(300.0));
            assert!(
                !s.low && !s.critical,
                "sag-compensated failsafe must not trip on a punch (soc {})",
                s.soc
            );
        }
    }

    /// The inverse case the sag fix must not break: the SAME healthy
    /// resting voltage with HIGH integrated consumption trips — the
    /// coulomb side of the conservative fusion.
    #[test]
    fn high_consumption_trips_at_healthy_voltage() {
        let mut est = BatteryEstimator::new(cfg());
        est.consumed_mah = 4600.0; // 92% consumed ⇒ soc 8% < crit 10%
        let mut tripped = false;
        for _ in 0..300 {
            // Rebound voltage looks healthy (3.85 V/cell) at low current.
            let s = est.update(0.02, 3.85 * 4.0, Some(2.0));
            tripped = s.critical;
        }
        assert!(tripped, "spent pack must trip critical despite healthy volts");
    }

    /// mAh integration error < 2% over a simulated 20-minute flight with
    /// a NOISY current sense (deterministic LCG noise, zero-mean).
    #[test]
    fn coulomb_error_under_two_percent_over_20min() {
        let mut est = BatteryEstimator::new(BattConfig {
            capacity_mah: 20000.0, // headroom so the count is not clamp-saturated
            ..cfg()
        });
        let dt = 0.02f32;
        let steps = (20.0 * 60.0 / dt) as u32; // 60k steps
        let mut lcg: u32 = 0x1234_5678;
        let mut true_mah = 0.0f64;
        let mut s = BattState::default();
        for k in 0..steps {
            // Duty profile: hover 18 A with 40 A climbs every 2 min.
            let t = k as f32 * dt;
            let i_true = if (t / 120.0).fract() < 0.1 { 40.0 } else { 18.0 };
            lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
            // Zero-mean ±2 A uniform sensor noise.
            let noise = ((lcg >> 8) as f32 / 16777216.0 - 0.5) * 4.0;
            true_mah += (i_true as f64) * (dt as f64) * (1000.0 / 3600.0);
            s = est.update(dt, 15.2, Some(i_true + noise));
        }
        let err = ((s.consumed_mah as f64 - true_mah) / true_mah).abs();
        assert!(
            err < 0.02,
            "integration error {:.3}% (true {:.0} mAh, est {:.0} mAh)",
            err * 100.0,
            true_mah,
            s.consumed_mah
        );
    }

    /// No current sense ⇒ the WIDER voltage-only margins apply and the
    /// state is flagged degraded; the same voltage that is fine under the
    /// compensated path trips the fallback.
    #[test]
    fn fallback_is_wider_margin_and_flagged() {
        // The same 3.68 V/cell terminal voltage, both ways: the fallback
        // trips its wider 3.70 V/cell threshold and flags degraded; the
        // compensated path credits the 15 A sag (rest ≈ 3.77 V/cell,
        // soc ≈ 0.34) and stays clear — the margin the fallback gives up.
        let mut fb = BatteryEstimator::new(cfg());
        let mut comp = BatteryEstimator::new(cfg());
        let v = 3.68 * 4.0;
        let mut s_fb = BattState::default();
        let mut s_comp = BattState::default();
        for _ in 0..200 {
            s_fb = fb.update(0.02, v, None);
            s_comp = comp.update(0.02, v, Some(15.0));
        }
        assert!(s_fb.degraded, "fallback must be flagged");
        assert!(!s_comp.degraded);
        assert!(s_fb.low, "wider fallback margin trips at 3.68 < 3.70 V/cell");
        assert!(
            s_comp.rest_volts > s_fb.rest_volts,
            "compensation credits the sag the fallback cannot"
        );
        let (_, _, _, healthy) = sys_status_fields(&s_fb);
        assert!(!healthy, "degraded fallback reads unhealthy in SYS_STATUS");
        let (_, _, _, healthy) = sys_status_fields(&s_comp);
        assert!(healthy);
    }

    /// Latching: once low trips it stays tripped through recovery-looking
    /// samples (a pack does not un-discharge in flight).
    #[test]
    fn flags_latch() {
        let mut est = BatteryEstimator::new(cfg());
        est.consumed_mah = 4000.0; // soc 20% < low 25%
        for _ in 0..300 {
            est.update(0.02, 3.9 * 4.0, Some(5.0));
        }
        // "Recovery": rebound voltage + zeroed consumption cannot unlatch.
        let s = est.update(0.02, 4.2 * 4.0, Some(0.0));
        assert!(s.low, "low latch must hold");
    }

    /// Debounce: a sub-debounce transient does not trip.
    #[test]
    fn transient_does_not_trip() {
        let mut est = BatteryEstimator::new(cfg());
        est.consumed_mah = 4000.0; // soc 20% < low
        // 1 s below (debounce is 2 s), then healthy again.
        for _ in 0..50 {
            est.update(0.02, 3.9 * 4.0, Some(5.0));
        }
        est.consumed_mah = 1000.0;
        let mut s = BattState::default();
        for _ in 0..100 {
            s = est.update(0.02, 3.9 * 4.0, Some(5.0));
        }
        assert!(!s.low, "1 s excursion under a 2 s debounce must not latch");
    }

    /// R estimation: with a synthetic pack of known R, current steps pull
    /// the online estimate toward truth.
    #[test]
    fn r_estimate_converges() {
        let r_true = 0.040f32;
        let mut est = BatteryEstimator::new(BattConfig {
            r_internal: 0.010, // start 4x off
            ..cfg()
        });
        let v_rest = 15.4f32;
        let mut s = BattState::default();
        for k in 0..2000 {
            let i = if k % 2 == 0 { 10.0 } else { 30.0 }; // 20 A steps
            s = est.update(0.02, v_rest - i * r_true, Some(i));
        }
        assert!(
            (s.r_est - r_true).abs() < 0.005,
            "R estimate {:.4} should approach true {:.4}",
            s.r_est,
            r_true
        );
    }

    /// OCV curve sanity: endpoints, monotonicity on the grid, midpoint.
    #[test]
    fn ocv_curve_shape() {
        assert_eq!(ocv_soc_cell(3.50), 0.0);
        assert_eq!(ocv_soc_cell(4.20), 1.0);
        assert_eq!(ocv_soc_cell(2.0), 0.0);
        assert_eq!(ocv_soc_cell(5.0), 1.0);
        assert_eq!(ocv_soc_cell(f32::NAN), 0.0, "unreadable reads empty");
        let mut prev = -1.0f32;
        let mut v = 3.4f32;
        while v < 4.25 {
            let s = ocv_soc_cell(v);
            assert!(s >= prev, "OCV curve must be monotone");
            prev = s;
            v += 0.01;
        }
    }

    mod proptests {
        use super::super::*;
        use proptest::prelude::*;

        proptest! {
            /// Totality over arbitrary f32 bit patterns: every state field
            /// finite and in range, regardless of input garbage.
            #[test]
            fn update_total(dt in any::<f32>(), v in any::<f32>(),
                            i in any::<f32>(), has_i in any::<bool>(),
                            n in 1usize..50) {
                let mut est = BatteryEstimator::new(BattConfig::default());
                for _ in 0..n {
                    let s = est.update(dt, v, has_i.then_some(i));
                    prop_assert!(s.volts.is_finite() && (0.0..=60.0).contains(&s.volts));
                    prop_assert!(s.rest_volts.is_finite() && (0.0..=60.0).contains(&s.rest_volts));
                    prop_assert!(s.consumed_mah.is_finite() && s.consumed_mah >= 0.0);
                    prop_assert!(s.soc.is_finite() && (0.0..=1.0).contains(&s.soc));
                    prop_assert!(s.r_est.is_finite() && (0.001..=0.2).contains(&s.r_est));
                    let (_, _, pct, _) = sys_status_fields(&s);
                    prop_assert!((0..=100).contains(&pct));
                }
            }
        }
    }
}
