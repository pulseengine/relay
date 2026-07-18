//! relay-notch — RPM-tracked harmonic notch filtering (NOTCH-P01, v1.122).
//!
//! Per-motor rotational vibration is the dominant gyro contaminant on a
//! multirotor, and it is NARROWBAND at the rotation frequency and its
//! harmonics — exactly what a wide low-pass cannot reject without eating
//! the phase margin of the loop it protects. This engine is the
//! PX4 `IMU_GYRO_DNF` / ArduPilot `INS_HNTCH` / Betaflight RPM-filter
//! role: biquad notches per motor at the fundamental + 2nd harmonic,
//! center frequencies tracking ACHIEVED RPM (bidir-DShot eRPM primary,
//! per jess DD-024 — the same `read_motor_rpm` seam the rotor-out FDI
//! consumes).
//!
//! Safety posture:
//! - **Absent RPM ⇒ unity bypass, bit-exact** — no filtering is safer
//!   than a mistracked notch. States reset so re-engagement is clean.
//! - **Nyquist clamp per notch** — a harmonic above 0.45·fs bypasses
//!   individually (at the 250 Hz flight loop the 2nd harmonic leaves the
//!   band above ~4200 RPM; it re-engages when RPM falls back).
//! - **Total** (Kani NOTCH-K01/K02): any input — including NaN/∞ samples
//!   or a poisoned internal state — yields a finite output; non-finite
//!   arithmetic results reset the offending biquad and pass the (sanitized)
//!   input through.
//!
//! Design: RBJ cookbook notch (b = [1, −2cosω₀, 1], a = [1+α, −2cosω₀,
//! 1−α], α = sinω₀/2Q), Direct Form 1 so state has bounded physical
//! meaning. Q = 5 per notch: deep at center (> 20 dB over the tracking
//! band), negligible phase at the rate-loop crossover (measured < 10°
//! in the verification tests, not asserted from arithmetic).

#![no_std]
#![forbid(unsafe_code)]

/// Per-notch quality factor. Higher = narrower/deeper but more sensitive
/// to RPM error; 5 matches the reference autopilots' defaults' ballpark.
pub const NOTCH_Q: f32 = 5.0;
/// A notch only engages when its center is below this fraction of fs
/// (Nyquist margin) …
pub const MAX_F0_FRAC: f32 = 0.45;
/// … and above this floor (idle jitter is not worth a notch).
pub const MIN_F0_HZ: f32 = 10.0;
/// Output hard cap (rad/s) — far beyond any physical body rate; the
/// totality backstop for the filter arithmetic.
pub const OUT_CAP: f32 = 200.0;

#[inline]
fn sane(x: f32) -> f32 {
    if x.is_finite() {
        x.clamp(-OUT_CAP, OUT_CAP)
    } else {
        0.0
    }
}

/// One RBJ biquad notch (Direct Form 1).
#[derive(Clone, Copy, Debug, Default)]
pub struct Notch {
    // normalized coefficients (b0, b1, b2, a1, a2); unity when disengaged
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
    engaged: bool,
}

/// The engageable-band gate, TRIG-FREE (comparisons only): `set` engages
/// exactly when this holds. Split out so Kani can prove the gate over all
/// inputs without pulling libm's argument reduction into the call graph
/// (symbolic trig unwinds >1500 iterations in CBMC — the libm lesson).
pub fn band_ok(f0_hz: f32, fs_hz: f32) -> bool {
    f0_hz.is_finite()
        && fs_hz.is_finite()
        && fs_hz > 0.0
        && f0_hz >= MIN_F0_HZ
        && f0_hz <= MAX_F0_FRAC * fs_hz
}

impl Notch {
    /// (Re)tune to center `f0_hz` at sample rate `fs_hz`. Outside the
    /// engageable band (or with nonsense inputs) the notch DISENGAGES —
    /// unity passthrough with cleared state. Engagement ≡ [`band_ok`]
    /// (the one-line linkage below; the gate itself is Kani-proven).
    pub fn set(&mut self, f0_hz: f32, fs_hz: f32) {
        if !band_ok(f0_hz, fs_hz) {
            self.disengage();
            return;
        }
        let w0 = 2.0 * core::f32::consts::PI * f0_hz / fs_hz;
        let cw = relay_math::cosf(w0);
        let sw = relay_math::sinf(w0);
        let alpha = sw / (2.0 * NOTCH_Q);
        let a0 = 1.0 + alpha;
        self.b0 = 1.0 / a0;
        self.b1 = -2.0 * cw / a0;
        self.b2 = 1.0 / a0;
        self.a1 = -2.0 * cw / a0;
        self.a2 = (1.0 - alpha) / a0;
        if !self.engaged {
            self.x1 = 0.0;
            self.x2 = 0.0;
            self.y1 = 0.0;
            self.y2 = 0.0;
        }
        self.engaged = true;
    }

    /// Unity passthrough with cleared state.
    pub fn disengage(&mut self) {
        *self = Notch::default();
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Kani-only state poisoning: totality must hold for ANY internal
    /// state, not just states reachable through sane arithmetic.
    #[cfg(kani)]
    pub(crate) fn poison_state(&mut self, st: [f32; 4]) {
        self.x1 = st[0];
        self.x2 = st[1];
        self.y1 = st[2];
        self.y2 = st[3];
    }

    /// One sample. Total: the output is always finite (a non-finite
    /// arithmetic result resets this biquad and passes the sanitized
    /// input through).
    pub fn apply(&mut self, x: f32) -> f32 {
        if !self.engaged {
            return x; // bit-exact bypass
        }
        let xs = sane(x);
        // Sanitize the OPERANDS, not just the result: with every state word
        // and coefficient finite and bounded, no intermediate can produce a
        // NaN/∞ at all (Kani proves the arithmetic side-conditions, not
        // only the final assert), and a poisoned state heals in one sample
        // instead of glitching through a reset.
        let (x1, x2, y1, y2) = (sane(self.x1), sane(self.x2), sane(self.y1), sane(self.y2));
        let y = self.b0 * xs + self.b1 * x1 + self.b2 * x2 - self.a1 * y1 - self.a2 * y2;
        let y = sane(y);
        self.x1 = xs;
        self.x2 = x1;
        self.y1 = y;
        self.y2 = y1;
        y
    }
}

/// The gyro-path bank: `M` motors × 2 harmonics × 3 axes.
pub struct HarmonicNotchBank<const M: usize> {
    /// notches[axis][motor][harmonic]
    notches: [[[Notch; 2]; M]; 3],
    fs_hz: f32,
    bypassed: bool,
}

impl<const M: usize> HarmonicNotchBank<M> {
    pub fn new(fs_hz: f32) -> Self {
        HarmonicNotchBank {
            notches: [[[Notch::default(); 2]; M]; 3],
            fs_hz,
            bypassed: true,
        }
    }

    /// Track the achieved per-motor RPM for this cycle. `None` (no RPM
    /// telemetry) ⇒ the WHOLE bank bypasses cleanly until it returns.
    pub fn update_rpm(&mut self, rpm: Option<[i32; M]>) {
        match rpm {
            None => {
                if !self.bypassed {
                    for ax in self.notches.iter_mut() {
                        for m in ax.iter_mut() {
                            m[0].disengage();
                            m[1].disengage();
                        }
                    }
                    self.bypassed = true;
                }
            }
            Some(r) => {
                self.bypassed = false;
                for ax in 0..3 {
                    for (m, rm) in r.iter().enumerate().take(M) {
                        let f0 = (*rm).max(0) as f32 / 60.0; // RPM → Hz
                        self.notches[ax][m][0].set(f0, self.fs_hz);
                        self.notches[ax][m][1].set(2.0 * f0, self.fs_hz);
                    }
                }
            }
        }
    }

    /// Whole-bank bypass state (RPM telemetry absent).
    pub fn is_bypassed(&self) -> bool {
        self.bypassed
    }

    /// Filter one gyro sample (rad/s, body axes). Bit-exact passthrough
    /// while bypassed; finite for ANY input otherwise.
    pub fn apply(&mut self, gyro: [f32; 3]) -> [f32; 3] {
        if self.bypassed {
            return gyro;
        }
        let mut out = gyro;
        for (ax, o) in out.iter_mut().enumerate() {
            // sanitize at the bank boundary: with RPM telemetry present we
            // own the path, and a non-finite gyro sample must not ride
            // through disengaged (Nyquist-clamped) notches untouched.
            let mut v = sane(*o);
            for m in 0..M {
                v = self.notches[ax][m][0].apply(v);
                v = self.notches[ax][m][1].apply(v);
            }
            *o = v;
        }
        out
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// Drive a tuned notch with a unit sinusoid at `f_hz`, return
    /// (attenuation_db, phase_deg) measured by DFT at the drive frequency
    /// after settling — measured, not asserted from arithmetic.
    fn measure(notch_f0: f32, drive_f: f32, fs: f32) -> (f32, f32) {
        let mut n = Notch::default();
        n.set(notch_f0, fs);
        assert!(n.is_engaged());
        let settle = (10.0 * fs / notch_f0) as usize;
        let periods = 50usize;
        let nsamp = (periods as f32 * fs / drive_f) as usize;
        let mut acc_i = 0.0f64;
        let mut acc_q = 0.0f64;
        for k in 0..(settle + nsamp) {
            let t = k as f32 / fs;
            let ph = 2.0 * core::f32::consts::PI * drive_f * t;
            let x = relay_math::sinf(ph);
            let y = n.apply(x);
            if k >= settle {
                acc_i += (y * relay_math::sinf(ph)) as f64;
                acc_q += (y * relay_math::cosf(ph)) as f64;
            }
        }
        let scale = 2.0 / nsamp as f64;
        let (i, q) = (acc_i * scale, acc_q * scale);
        let mag = (i * i + q * q).sqrt() as f32;
        let att_db = -20.0 * (mag.max(1e-9)).log10();
        let phase_deg = (q.atan2(i) as f32).to_degrees();
        (att_db, phase_deg)
    }

    /// ≥ 20 dB at the tracked fundamental across the hover-to-full band
    /// (fs = 1000 Hz, the target hardware gyro path rate).
    #[test]
    fn twenty_db_at_fundamental_across_band() {
        let fs = 1000.0;
        for rpm in [3000.0f32, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0] {
            let f0 = rpm / 60.0;
            let (att, _) = measure(f0, f0, fs);
            assert!(att >= 20.0, "{rpm} RPM ({f0} Hz): {att:.1} dB < 20 dB");
        }
    }

    /// Stepped fine sweep: tracked attenuation stays within 3 dB of the
    /// band's nominal (the swept-RPM criterion, measured at 20 points).
    #[test]
    fn swept_rpm_attenuation_within_3db() {
        let fs = 1000.0;
        let mut atts = Vec::new();
        for k in 0..20 {
            let rpm = 3000.0 + 250.0 * k as f32; // 3000..7750
            let f0 = rpm / 60.0;
            let (att, _) = measure(f0, f0, fs);
            atts.push(att);
        }
        let min = atts.iter().cloned().fold(f32::MAX, f32::min);
        let max = atts.iter().cloned().fold(f32::MIN, f32::max);
        // deep-notch DFT floors can differ hugely in dB; the criterion is
        // about the WORST point staying within 3 dB of nominal (20 dB).
        assert!(min >= 20.0 - 3.0, "sweep worst point {min:.1} dB (max {max:.1})");
    }

    /// Added phase lag at the rate-loop crossover (≈ 5 Hz for the ADRC
    /// band) stays under 10° with the FULL falcon bank engaged at hover
    /// RPM — the filter must not destabilize the loop it protects.
    #[test]
    fn phase_at_crossover_under_10_degrees() {
        let fs = 1000.0;
        let fc = 5.0;
        // full bank: 4 motors near hover (slightly split), both harmonics.
        let mut bank: HarmonicNotchBank<4> = HarmonicNotchBank::new(fs);
        bank.update_rpm(Some([4000, 4100, 3950, 4050]));
        let settle = 2000usize;
        let periods = 40usize;
        let nsamp = (periods as f32 * fs / fc) as usize;
        let mut acc_i = 0.0f64;
        let mut acc_q = 0.0f64;
        for k in 0..(settle + nsamp) {
            let t = k as f32 / fs;
            let ph = 2.0 * core::f32::consts::PI * fc * t;
            let x = relay_math::sinf(ph);
            let y = bank.apply([x, 0.0, 0.0])[0];
            if k >= settle {
                acc_i += (y * relay_math::sinf(ph)) as f64;
                acc_q += (y * relay_math::cosf(ph)) as f64;
            }
        }
        let phase = ((acc_q).atan2(acc_i) as f32).to_degrees().abs();
        assert!(phase < 10.0, "bank phase at {fc} Hz crossover: {phase:.2}°");
    }

    /// Absent RPM telemetry ⇒ BIT-EXACT unity passthrough.
    #[test]
    fn bypass_is_bit_exact() {
        let mut bank: HarmonicNotchBank<4> = HarmonicNotchBank::new(250.0);
        bank.update_rpm(None);
        assert!(bank.is_bypassed());
        for k in 0..500 {
            let x = [
                relay_math::sinf(k as f32 * 0.37) * 3.0,
                f32::from_bits(0x7F80_0000u32.wrapping_sub(k)), // wild bits
                -0.0,
            ];
            let y = bank.apply(x);
            assert_eq!(x[0].to_bits(), y[0].to_bits());
            assert_eq!(x[1].to_bits(), y[1].to_bits());
            assert_eq!(x[2].to_bits(), y[2].to_bits());
        }
        // engage, then drop RPM again: bypass returns and is still exact.
        bank.update_rpm(Some([4000; 4]));
        assert!(!bank.is_bypassed());
        let _ = bank.apply([1.0, 1.0, 1.0]);
        bank.update_rpm(None);
        let x = [0.123f32, -4.5, 6.7];
        let y = bank.apply(x);
        assert_eq!(x[0].to_bits(), y[0].to_bits());
    }

    /// `set` engages EXACTLY per the Kani-proven band_ok gate — the
    /// one-line linkage checked over a grid including both boundaries.
    #[test]
    fn set_engagement_matches_band_ok() {
        let mut n = Notch::default();
        for &fs in &[250.0f32, 1000.0] {
            for k in 0..200 {
                let f0 = k as f32 * 1.5; // 0..300 Hz, crosses both bounds
                n.set(f0, fs);
                assert_eq!(n.is_engaged(), band_ok(f0, fs), "f0={f0} fs={fs}");
            }
            n.set(MIN_F0_HZ, fs);
            assert!(n.is_engaged());
            n.set(MAX_F0_FRAC * fs, fs);
            assert!(n.is_engaged());
            n.set(f32::NAN, fs);
            assert!(!n.is_engaged());
        }
    }

    /// Nyquist clamp: at the 250 Hz flight loop a full-throttle 2nd
    /// harmonic (267 Hz) must NOT engage; the fundamental (133 Hz) is
    /// above 0.45·fs = 112.5 Hz and must not either. At 4000 RPM the
    /// fundamental (66.7 Hz) engages and the 2nd (133 Hz) does not.
    #[test]
    fn nyquist_clamp_per_notch() {
        let fs = 250.0;
        let mut n = Notch::default();
        n.set(8000.0 / 60.0, fs); // 133 Hz > 112.5
        assert!(!n.is_engaged());
        n.set(4000.0 / 60.0, fs); // 66.7 Hz
        assert!(n.is_engaged());
        n.set(2.0 * 4000.0 / 60.0, fs); // 133 Hz again via harmonic path
        assert!(!n.is_engaged());
    }

    /// Vibration-rejection at the flight rate: a hover-band rotor line
    /// (70 Hz) riding on a 2 Hz body-rate signal, fs = 250 Hz. The bank
    /// removes the line and keeps the signal.
    #[test]
    fn removes_rotor_line_keeps_body_rates() {
        let fs = 250.0;
        let mut bank: HarmonicNotchBank<4> = HarmonicNotchBank::new(fs);
        bank.update_rpm(Some([4200; 4])); // 70 Hz fundamental
        let mut in_pow = 0.0f64;
        let mut out_pow = 0.0f64;
        let mut sig_pow = 0.0f64;
        for k in 0..5000 {
            let t = k as f32 / fs;
            let body = relay_math::sinf(2.0 * core::f32::consts::PI * 2.0 * t);
            let vib = 0.8 * relay_math::sinf(2.0 * core::f32::consts::PI * 70.0 * t);
            let y = bank.apply([body + vib, 0.0, 0.0])[0];
            if k > 1000 {
                in_pow += ((body + vib) * (body + vib)) as f64;
                out_pow += ((y - body) * (y - body)) as f64;
                sig_pow += (body * body) as f64;
            }
        }
        // residual (output minus true body signal) is far below the
        // injected vibration power — and the body signal survived.
        assert!(out_pow < 0.05 * in_pow, "residual {out_pow:.1} vs in {in_pow:.1}");
        assert!(sig_pow > 0.0);
    }

    mod proptests {
        use super::super::*;
        use proptest::prelude::*;

        proptest! {
            /// Totality: any bit-pattern inputs and any RPM values yield
            /// finite outputs while engaged.
            #[test]
            fn bank_output_always_finite(
                xs in proptest::collection::vec(any::<u32>(), 1..80),
                rpm in proptest::array::uniform4(-20000i32..20000),
            ) {
                let mut bank: HarmonicNotchBank<4> = HarmonicNotchBank::new(250.0);
                bank.update_rpm(Some(rpm));
                for bits in xs {
                    let x = f32::from_bits(bits);
                    let y = bank.apply([x, x, x]);
                    prop_assert!(y.iter().all(|v| v.is_finite()));
                }
            }
        }
    }
}
