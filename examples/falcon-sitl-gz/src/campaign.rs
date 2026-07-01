//! Monte-Carlo simulation campaigns — broad empirical safety verification.
//!
//! The point-tests in `main.rs` prove each closed-loop behaviour at a SINGLE
//! condition. This module runs the same verified cascade across THOUSANDS of
//! randomised trials and asserts the safety invariant holds across all of them,
//! reporting worst-case margins — the "dispersion deck" discipline (NASA/POST2
//! style) that turns one flight into a statistically-meaningful campaign.
//!
//! Design (see the v1.109 research note): hierarchical splitmix64 seeding so
//! every trial is independently reproducible from `(campaign_seed, index)`;
//! rejection-sample initial conditions inside a stated RECOVERABLE ENVELOPE so
//! "the IC was physically unrecoverable" is never mistaken for "the controller
//! failed"; assert `failures == 0` AND every worst-case margin ≥ its bound.
//!
//! Physics matches the verified `fault_tolerance_chain_recovers_from_rotor_loss`
//! rigid-body sim exactly (same GeoGains, dt, SCALE, hover/floor).

#![allow(dead_code)] // campaign entry points are exercised by #[cfg(test)] gates

use relay_geo::{GeoAtt, GeoGains};
use relay_iekf::RotorFaultDetector;
use relay_mix_quad::{motors_to_torque_signs, QuadMixer};

// ── Deterministic, splittable RNG ───────────────────────────────────────────

/// SplitMix64 — fast, well-distributed, and (crucially) *splittable*: each trial
/// gets an independent stream derived from `(campaign_seed, index)`, so any
/// failing trial is replayable from its index alone.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }
    /// Uniform in [lo, hi].
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }
}

/// Derive an independent per-trial seed from the campaign seed + trial index.
fn trial_rng(campaign_seed: u64, index: u32) -> SplitMix64 {
    // Mix the index in with the golden ratio before seeding so adjacent trials
    // start in decorrelated regions of the stream.
    let mixed = campaign_seed ^ (index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let mut s = SplitMix64(mixed);
    // one warm-up draw
    let _ = s.next_u64();
    s
}

// ── Shared rigid-body attitude sim (identical to the verified point-test) ────

const DT: f32 = 0.002;
const SCALE: f32 = 0.25;
const HOVER: f32 = 0.5;
const FLOOR: f32 = 0.15;

fn integ_rot(r: &[[f32; 3]; 3], w: [f32; 3], dt: f32) -> [[f32; 3]; 3] {
    let wd = [w[0] * dt, w[1] * dt, w[2] * dt];
    let incr = [[1.0, -wd[2], wd[1]], [wd[2], 1.0, -wd[0]], [-wd[1], wd[0], 1.0]];
    let mut m = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for jj in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += r[i][k] * incr[k][jj];
            }
            m[i][jj] = s;
        }
    }
    // Gram-Schmidt on columns to stay in SO(3).
    let col = |mm: &[[f32; 3]; 3], c: usize| [mm[0][c], mm[1][c], mm[2][c]];
    let norm = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let c0 = col(&m, 0);
    let n0 = norm(c0).max(1e-9);
    let e0 = [c0[0] / n0, c0[1] / n0, c0[2] / n0];
    let c1 = col(&m, 1);
    let d = e0[0] * c1[0] + e0[1] * c1[1] + e0[2] * c1[2];
    let p1 = [c1[0] - d * e0[0], c1[1] - d * e0[1], c1[2] - d * e0[2]];
    let n1 = norm(p1).max(1e-9);
    let e1 = [p1[0] / n1, p1[1] / n1, p1[2] / n1];
    let e2 = [
        e0[1] * e1[2] - e0[2] * e1[1],
        e0[2] * e1[0] - e0[0] * e1[2],
        e0[0] * e1[1] - e0[1] * e1[0],
    ];
    [
        [e0[0], e1[0], e2[0]],
        [e0[1], e1[1], e2[1]],
        [e0[2], e1[2], e2[2]],
    ]
}

/// R = Rx(roll)·Ry(pitch) — a tilt with no built-in yaw.
fn tilt_rotation(roll: f32, pitch: f32) -> [[f32; 3]; 3] {
    let (cx, sx) = (roll.cos(), roll.sin());
    let (cy, sy) = (pitch.cos(), pitch.sin());
    let rx = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]];
    let ry = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]];
    let mut m = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for jj in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += rx[i][k] * ry[k][jj];
            }
            m[i][jj] = s;
        }
    }
    m
}

fn tilt_of(r: &[[f32; 3]; 3]) -> f32 {
    r[2][2].clamp(-1.0, 1.0).acos()
}

// ── Motor-out recovery campaign ─────────────────────────────────────────────

/// A single dispersed motor-out trial (the randomised inputs).
#[derive(Clone, Copy, Debug)]
struct MotorOutTrial {
    failed_rotor: usize, // 0..4
    fail_step: u32,      // when the rotor dies
    roll0: f32,          // initial tilt components (rad)
    pitch0: f32,
    omega0: [f32; 3], // initial body rate (rad/s)
}

/// The RECOVERABLE ENVELOPE for a single-rotor-out quad: modest initial tilt +
/// body rate, so the reduced-attitude law has the authority to recover. ICs are
/// rejection-sampled to lie inside this, so a FAIL is a real bug, never
/// "physics said no". Deliberately conservative (inside the true envelope).
const MAX_TILT0: f32 = 0.20; // ~11.5° initial tilt
const MAX_RATE0: f32 = 0.80; // rad/s per axis

fn sample_motor_out(rng: &mut SplitMix64) -> MotorOutTrial {
    // Rejection-sample the tilt so the *combined* tilt magnitude ≤ MAX_TILT0
    // (not just per-axis), keeping strictly inside the envelope.
    let (roll0, pitch0) = loop {
        let rr = rng.range(-MAX_TILT0, MAX_TILT0);
        let pp = rng.range(-MAX_TILT0, MAX_TILT0);
        if (rr * rr + pp * pp).sqrt() <= MAX_TILT0 {
            break (rr, pp);
        }
    };
    MotorOutTrial {
        failed_rotor: (rng.next_u64() % 4) as usize,
        fail_step: 500 + (rng.next_u64() % 1000) as u32, // 1.0 s .. 3.0 s
        roll0,
        pitch0,
        omega0: [
            rng.range(-MAX_RATE0, MAX_RATE0),
            rng.range(-MAX_RATE0, MAX_RATE0),
            rng.range(-MAX_RATE0, MAX_RATE0),
        ],
    }
}

/// Outcome of one recovery episode.
#[derive(Clone, Copy, Debug)]
struct MotorOutOutcome {
    isolated: Option<usize>,
    detect_latency_steps: u32, // steps from fail_step to isolation
    peak_tilt_after: f32,      // worst tilt after the failure settles in
    final_tilt: f32,
    mix_ok: bool, // MIX-P08 held every reconfigured step
    nan: bool,
}

fn run_motor_out_trial(t: MotorOutTrial) -> MotorOutOutcome {
    let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
    let j = GeoGains::FALCON_QUAD.j;
    let b3_d = [0.0f32, 0.0, 1.0];
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    let mut r = tilt_rotation(t.roll0, t.pitch0);
    let mut omega = t.omega0;
    let mut fdi = RotorFaultDetector::new(0.5, 0.1);
    let mut isolated: Option<usize> = None;
    let mut detect_step: Option<u32> = None;
    let mut peak_tilt_after = 0.0f32;
    let mut mix_ok = true;
    let mut nan = false;

    for step in 0..4000u32 {
        let (_torque, motors_cmd) = if let Some(f) = isolated {
            let tq = ctrl.moment_reduced(&r, omega, b3_d);
            (tq, QuadMixer::new().mix_rotor_out(f, tq, HOVER, FLOOR))
        } else {
            let tq = ctrl.moment(&r, omega, &level);
            (tq, QuadMixer::new().mix_thrust_floor(tq, HOVER, FLOOR))
        };

        // MIX-P08 in the loop: failed rotor OFF, healthy ∈ [floor,1].
        if let Some(f) = isolated {
            if motors_cmd[f] != 0.0 {
                mix_ok = false;
            }
            for (i, &v) in motors_cmd.iter().enumerate() {
                if i != f && !(FLOOR - 1e-6..=1.0 + 1e-6).contains(&v) {
                    mix_ok = false;
                }
            }
        }

        // Physics: inject the rotor failure.
        let mut motors_real = motors_cmd;
        if step >= t.fail_step {
            motors_real[t.failed_rotor] = 0.0;
        }
        // FDI on the commanded-vs-achieved residual.
        if isolated.is_none() {
            let mut resid = [0.0f32; 4];
            for i in 0..4 {
                resid[i] = (motors_cmd[i] - motors_real[i]).abs();
            }
            if let Some(f) = fdi.update(resid) {
                isolated = Some(f);
                detect_step = Some(step);
            }
        }

        // Rigid-body update from the ACHIEVED motors.
        let bt = motors_to_torque_signs(motors_real);
        let body_t = [bt[0] * SCALE, bt[1] * SCALE, bt[2] * SCALE];
        let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
        let gyro = [
            omega[1] * jo[2] - omega[2] * jo[1],
            omega[2] * jo[0] - omega[0] * jo[2],
            omega[0] * jo[1] - omega[1] * jo[0],
        ];
        for i in 0..3 {
            omega[i] += DT * (body_t[i] - gyro[i]) / j[i];
        }
        r = integ_rot(&r, omega, DT);

        if !omega[0].is_finite() || !r[2][2].is_finite() {
            nan = true;
        }
        // Track peak tilt once the failure + a short transient has passed.
        if step > t.fail_step + 300 {
            peak_tilt_after = peak_tilt_after.max(tilt_of(&r));
        }
    }

    MotorOutOutcome {
        isolated,
        detect_latency_steps: match (detect_step, Some(t.fail_step)) {
            (Some(d), Some(f)) if d >= f => d - f,
            _ => u32::MAX,
        },
        peak_tilt_after,
        final_tilt: tilt_of(&r),
        mix_ok,
        nan,
    }
}

/// Aggregated campaign statistics — the distribution, not a boolean.
#[derive(Clone, Debug, Default)]
pub struct MotorOutReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_peak_tilt: f32,
    pub worst_final_tilt: f32,
    pub worst_detect_latency_steps: u32,
    /// (index, seed-ish, reason) for each failing trial — the reproducer table.
    pub failing: Vec<(u32, String)>,
}

/// Run `n` dispersed motor-out recovery trials from `campaign_seed`.
pub fn run_motor_out_campaign(n: u32, campaign_seed: u64) -> MotorOutReport {
    let mut rep = MotorOutReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_motor_out(&mut rng);
        let o = run_motor_out_trial(t);

        rep.worst_peak_tilt = rep.worst_peak_tilt.max(o.peak_tilt_after);
        rep.worst_final_tilt = rep.worst_final_tilt.max(o.final_tilt);
        if o.detect_latency_steps != u32::MAX {
            rep.worst_detect_latency_steps =
                rep.worst_detect_latency_steps.max(o.detect_latency_steps);
        }

        // Invariants (all must hold inside the recoverable envelope):
        let mut reason = String::new();
        if o.nan {
            reason = "NaN in cascade".into();
        } else if o.isolated != Some(t.failed_rotor) {
            reason = format!("FDI isolated {:?}, expected {}", o.isolated, t.failed_rotor);
        } else if !o.mix_ok {
            reason = "MIX-P08 violated (failed rotor nonzero / healthy out of band)".into();
        } else if o.peak_tilt_after >= 1.4 {
            reason = format!("tumbled: peak tilt {:.3} rad", o.peak_tilt_after);
        } else if o.final_tilt >= 0.5 {
            reason = format!("did not settle: final tilt {:.3} rad", o.final_tilt);
        }
        if !reason.is_empty() {
            rep.failures += 1;
            if rep.failing.len() < 20 {
                rep.failing.push((i, format!("{t:?}: {reason}")));
            }
        }
    }
    rep
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Campaign seed is checked in so the whole run is reproducible; bump it to
    /// resample. Rule of three: 2000 trials with zero failures ⇒ ≥99.85%
    /// success at 95% confidence, within the stated recoverable envelope.
    const MOTOR_OUT_SEED: u64 = 0x0FA1_C0DE_AD00_0001;
    const MOTOR_OUT_TRIALS: u32 = 2000;

    #[test]
    fn motor_out_monte_carlo_campaign() {
        let rep = run_motor_out_campaign(MOTOR_OUT_TRIALS, MOTOR_OUT_SEED);
        eprintln!(
            "motor-out campaign: {} trials, {} failures | worst peak tilt {:.3} rad, worst final tilt {:.3} rad, worst detect latency {} steps ({:.0} ms)",
            rep.trials, rep.failures, rep.worst_peak_tilt, rep.worst_final_tilt,
            rep.worst_detect_latency_steps, rep.worst_detect_latency_steps as f32 * DT * 1000.0
        );

        // Primary safety assertion: not one trial in the envelope fails.
        assert_eq!(
            rep.failures, 0,
            "motor-out recovery failed in {}/{} dispersed trials; first failures: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Physical safety bounds — the actual invariant: never tumbles
        // (< 1.4 rad ≈ 80°), always settles upright (< 0.5 rad ≈ 29°).
        assert!(
            rep.worst_peak_tilt < 1.4,
            "worst peak tilt across {} trials = {:.3} rad (tumble bound 1.4)",
            rep.trials, rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.5,
            "worst final tilt across {} trials = {:.3} rad (settle bound 0.5)",
            rep.trials, rep.worst_final_tilt
        );
        // Tighter REGRESSION bounds, set just above the measured worst case
        // (peak 0.832, final 0.097, detect 1 step at seed 0xFA1C_0DEAD_0001) —
        // a change that erodes the recovery margin trips these long before it
        // reaches the physical safety bound above.
        assert!(
            rep.worst_peak_tilt < 1.0,
            "REGRESSION: worst peak tilt {:.3} rad exceeded the 1.0 early-warning bound (was ~0.83)",
            rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.2,
            "REGRESSION: worst final tilt {:.3} rad exceeded the 0.2 early-warning bound (was ~0.10)",
            rep.worst_final_tilt
        );
        assert!(
            rep.worst_detect_latency_steps < 25, // < 50 ms
            "REGRESSION: worst FDI detect latency {} steps exceeded 25 (was 1)",
            rep.worst_detect_latency_steps
        );
    }
}

// ── Attitude-stabilisation campaign (random tilt, no fault) ──────────────────

/// A single dispersed attitude-recovery trial: the aircraft starts tilted and
/// spinning, no rotor fault. The FULL-attitude geometric law must null both the
/// tilt and the body rate back to level.
#[derive(Clone, Copy, Debug)]
struct AttStabTrial {
    roll0: f32,       // initial tilt components (rad)
    pitch0: f32,
    omega0: [f32; 3], // initial body rate (rad/s)
}

/// The RECOVERABLE ENVELOPE for a healthy quad recovering to level: a large-ish
/// initial tilt plus a body rate on every axis, all within the authority of the
/// full-attitude law at the FALCON_QUAD gains. ICs are rejection-sampled to lie
/// inside this, so a FAIL is a real controller bug, never "physics said no".
const ATT_MAX_TILT0: f32 = 0.50; // ~29° combined initial tilt
const ATT_MAX_RATE0: f32 = 1.00; // rad/s per axis

fn sample_att_stab(rng: &mut SplitMix64) -> AttStabTrial {
    // Rejection-sample the tilt so the *combined* magnitude ≤ ATT_MAX_TILT0.
    let (roll0, pitch0) = loop {
        let rr = rng.range(-ATT_MAX_TILT0, ATT_MAX_TILT0);
        let pp = rng.range(-ATT_MAX_TILT0, ATT_MAX_TILT0);
        if (rr * rr + pp * pp).sqrt() <= ATT_MAX_TILT0 {
            break (rr, pp);
        }
    };
    AttStabTrial {
        roll0,
        pitch0,
        omega0: [
            rng.range(-ATT_MAX_RATE0, ATT_MAX_RATE0),
            rng.range(-ATT_MAX_RATE0, ATT_MAX_RATE0),
            rng.range(-ATT_MAX_RATE0, ATT_MAX_RATE0),
        ],
    }
}

/// Outcome of one attitude-recovery episode.
#[derive(Clone, Copy, Debug)]
struct AttStabOutcome {
    peak_tilt: f32,  // worst tilt over the whole trajectory (divergence guard)
    final_tilt: f32, // tilt at the end (settle guard)
    final_rate: f32, // |ω| at the end (rate-null guard)
    nan: bool,
}

fn run_att_stab_trial(t: AttStabTrial) -> AttStabOutcome {
    let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
    let j = GeoGains::FALCON_QUAD.j;
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    let mut r = tilt_rotation(t.roll0, t.pitch0);
    let mut omega = t.omega0;
    let mut peak_tilt = 0.0f32;
    let mut nan = false;

    for _ in 0..4000u32 {
        // Healthy full-attitude law → thrust-floor mix (no fault, no FDI).
        let tq = ctrl.moment(&r, omega, &level);
        let motors_cmd = QuadMixer::new().mix_thrust_floor(tq, HOVER, FLOOR);

        // Rigid-body update from the commanded motors (no failure injected).
        let bt = motors_to_torque_signs(motors_cmd);
        let body_t = [bt[0] * SCALE, bt[1] * SCALE, bt[2] * SCALE];
        let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
        let gyro = [
            omega[1] * jo[2] - omega[2] * jo[1],
            omega[2] * jo[0] - omega[0] * jo[2],
            omega[0] * jo[1] - omega[1] * jo[0],
        ];
        for i in 0..3 {
            omega[i] += DT * (body_t[i] - gyro[i]) / j[i];
        }
        r = integ_rot(&r, omega, DT);

        if !omega[0].is_finite() || !r[2][2].is_finite() {
            nan = true;
        }
        peak_tilt = peak_tilt.max(tilt_of(&r));
    }

    let final_rate = (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]).sqrt();
    AttStabOutcome {
        peak_tilt,
        final_tilt: tilt_of(&r),
        final_rate,
        nan,
    }
}

/// Aggregated attitude-stabilisation campaign statistics.
#[derive(Clone, Debug, Default)]
pub struct AttStabReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_peak_tilt: f32,
    pub worst_final_tilt: f32,
    pub worst_final_rate: f32,
    /// (index, reason) for each failing trial — the reproducer table.
    pub failing: Vec<(u32, String)>,
}

/// Run `n` dispersed attitude-recovery trials from `campaign_seed`.
pub fn run_att_stab_campaign(n: u32, campaign_seed: u64) -> AttStabReport {
    let mut rep = AttStabReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_att_stab(&mut rng);
        let o = run_att_stab_trial(t);

        rep.worst_peak_tilt = rep.worst_peak_tilt.max(o.peak_tilt);
        rep.worst_final_tilt = rep.worst_final_tilt.max(o.final_tilt);
        rep.worst_final_rate = rep.worst_final_rate.max(o.final_rate);

        // Invariants (all must hold inside the recoverable envelope):
        let mut reason = String::new();
        if o.nan {
            reason = "NaN in cascade".into();
        } else if o.peak_tilt >= 1.4 {
            reason = format!("diverged: peak tilt {:.3} rad", o.peak_tilt);
        } else if o.final_tilt >= 0.05 {
            reason = format!("did not settle: final tilt {:.3} rad", o.final_tilt);
        } else if o.final_rate >= 0.05 {
            reason = format!("rate not nulled: final |ω| {:.3} rad/s", o.final_rate);
        }
        if !reason.is_empty() {
            rep.failures += 1;
            if rep.failing.len() < 20 {
                rep.failing.push((i, format!("{t:?}: {reason}")));
            }
        }
    }
    rep
}

// ── Hexa-airframe attitude-robustness campaign ───────────────────────────────

/// A single dispersed hexa attitude-recovery trial. Same geometric controller,
/// six rotors (MixerN::hexa_x). No fault — Monte-Carlo over the initial state.
#[derive(Clone, Copy, Debug)]
struct HexaTrial {
    roll0: f32,
    pitch0: f32,
    omega0: [f32; 3],
}

/// The RECOVERABLE ENVELOPE for the hexa recovering to level. Six rotors share
/// the collective (lower per-rotor hover), so the allocatable torque margin per
/// rotor is smaller than the quad's; keep the envelope inside that authority.
const HEXA_MAX_TILT0: f32 = 0.50; // ~29° combined initial tilt
const HEXA_MAX_RATE0: f32 = 1.00; // rad/s per axis
const HEXA_HOVER: f32 = 0.35; // 6 rotors share the collective ⇒ lower per-rotor
const HEXA_SCALE: f32 = 0.30; // matches the hexa_cascade_stabilizes_attitude sim

fn sample_hexa(rng: &mut SplitMix64) -> HexaTrial {
    let (roll0, pitch0) = loop {
        let rr = rng.range(-HEXA_MAX_TILT0, HEXA_MAX_TILT0);
        let pp = rng.range(-HEXA_MAX_TILT0, HEXA_MAX_TILT0);
        if (rr * rr + pp * pp).sqrt() <= HEXA_MAX_TILT0 {
            break (rr, pp);
        }
    };
    HexaTrial {
        roll0,
        pitch0,
        omega0: [
            rng.range(-HEXA_MAX_RATE0, HEXA_MAX_RATE0),
            rng.range(-HEXA_MAX_RATE0, HEXA_MAX_RATE0),
            rng.range(-HEXA_MAX_RATE0, HEXA_MAX_RATE0),
        ],
    }
}

/// Outcome of one hexa attitude-recovery episode.
#[derive(Clone, Copy, Debug)]
struct HexaOutcome {
    peak_tilt: f32,
    final_tilt: f32,
    final_rate: f32,
    motors_ok: bool, // every rotor command stayed in [0,1]
    nan: bool,
}

fn run_hexa_trial(t: HexaTrial) -> HexaOutcome {
    use relay_mix_quad::MixerN;

    let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
    let j = GeoGains::FALCON_QUAD.j;
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let hexa = MixerN::hexa_x();

    let mut r = tilt_rotation(t.roll0, t.pitch0);
    let mut omega = t.omega0;
    let mut peak_tilt = 0.0f32;
    let mut motors_ok = true;
    let mut nan = false;

    for _ in 0..4000u32 {
        // Same verified geometric law; airframe-agnostic 6-rotor allocation.
        let tq = ctrl.moment(&r, omega, &level);
        let motors = hexa.mix(tq, HEXA_HOVER);
        for &m in motors.iter().take(6) {
            if !(0.0..=1.0).contains(&m) {
                motors_ok = false;
            }
        }
        // The wrench the 6 rotors actually produce is the plant input.
        let w = hexa.achieved_wrench(&motors);
        let body_t = [w[1] * HEXA_SCALE, w[2] * HEXA_SCALE, w[3] * HEXA_SCALE];
        let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
        let gyro = [
            omega[1] * jo[2] - omega[2] * jo[1],
            omega[2] * jo[0] - omega[0] * jo[2],
            omega[0] * jo[1] - omega[1] * jo[0],
        ];
        for i in 0..3 {
            omega[i] += DT * (body_t[i] - gyro[i]) / j[i];
        }
        r = integ_rot(&r, omega, DT);

        if !omega[0].is_finite() || !r[2][2].is_finite() {
            nan = true;
        }
        peak_tilt = peak_tilt.max(tilt_of(&r));
    }

    let final_rate = (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]).sqrt();
    HexaOutcome {
        peak_tilt,
        final_tilt: tilt_of(&r),
        final_rate,
        motors_ok,
        nan,
    }
}

/// Aggregated hexa-campaign statistics.
#[derive(Clone, Debug, Default)]
pub struct HexaReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_peak_tilt: f32,
    pub worst_final_tilt: f32,
    pub worst_final_rate: f32,
    pub failing: Vec<(u32, String)>,
}

/// Run `n` dispersed hexa attitude-recovery trials from `campaign_seed`.
pub fn run_hexa_campaign(n: u32, campaign_seed: u64) -> HexaReport {
    let mut rep = HexaReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_hexa(&mut rng);
        let o = run_hexa_trial(t);

        rep.worst_peak_tilt = rep.worst_peak_tilt.max(o.peak_tilt);
        rep.worst_final_tilt = rep.worst_final_tilt.max(o.final_tilt);
        rep.worst_final_rate = rep.worst_final_rate.max(o.final_rate);

        let mut reason = String::new();
        if o.nan {
            reason = "NaN in cascade".into();
        } else if !o.motors_ok {
            reason = "hexa motor command out of [0,1]".into();
        } else if o.peak_tilt >= 1.4 {
            reason = format!("diverged: peak tilt {:.3} rad", o.peak_tilt);
        } else if o.final_tilt >= 0.05 {
            reason = format!("did not settle: final tilt {:.3} rad", o.final_tilt);
        } else if o.final_rate >= 0.05 {
            reason = format!("rate not nulled: final |ω| {:.3} rad/s", o.final_rate);
        }
        if !reason.is_empty() {
            rep.failures += 1;
            if rep.failing.len() < 20 {
                rep.failing.push((i, format!("{t:?}: {reason}")));
            }
        }
    }
    rep
}

#[cfg(test)]
mod campaign2_tests {
    use super::*;

    // Checked-in campaign seeds so the whole run is reproducible; bump to
    // resample. Rule of three: 2000 trials, zero failures ⇒ ≥99.85% success at
    // 95% confidence, within the stated recoverable envelope.
    const ATT_STAB_SEED: u64 = 0x0FA1_C0DE_AD00_0002;
    const ATT_STAB_TRIALS: u32 = 2000;
    const HEXA_SEED: u64 = 0x0FA1_C0DE_AD00_0003;
    const HEXA_TRIALS: u32 = 2000;

    #[test]
    fn att_stab_monte_carlo_campaign() {
        let rep = run_att_stab_campaign(ATT_STAB_TRIALS, ATT_STAB_SEED);
        eprintln!(
            "att-stab campaign: {} trials, {} failures | worst peak tilt {:.4} rad, worst final tilt {:.4} rad, worst final |ω| {:.4} rad/s",
            rep.trials, rep.failures, rep.worst_peak_tilt, rep.worst_final_tilt, rep.worst_final_rate
        );

        // Primary safety assertion: not one trial in the envelope fails.
        assert_eq!(
            rep.failures, 0,
            "attitude recovery failed in {}/{} dispersed trials; first failures: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Physical safety bounds — never diverges (< 1.4 rad ≈ 80°), always
        // settles level (< 0.05 rad ≈ 2.9°) with the body rate nulled.
        assert!(
            rep.worst_peak_tilt < 1.4,
            "worst peak tilt across {} trials = {:.4} rad (divergence bound 1.4)",
            rep.trials, rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.05,
            "worst final tilt across {} trials = {:.4} rad (settle bound 0.05)",
            rep.trials, rep.worst_final_tilt
        );
        assert!(
            rep.worst_final_rate < 0.05,
            "worst final rate across {} trials = {:.4} rad/s (rate-null bound 0.05)",
            rep.trials, rep.worst_final_rate
        );
        // Tighter REGRESSION bounds, set just above the measured worst case
        // (peak 0.5540, final 0.0000, rate 0.0002 at seed 0x0FA1C0DEAD000002).
        // A change that erodes the recovery margin trips these long before the
        // physical safety bound.
        assert!(
            rep.worst_peak_tilt < 0.56,
            "REGRESSION: worst peak tilt {:.4} rad exceeded the 0.56 early-warning bound (was ~0.554)",
            rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.005,
            "REGRESSION: worst final tilt {:.4} rad exceeded the 0.005 early-warning bound (was ~0.0000)",
            rep.worst_final_tilt
        );
        assert!(
            rep.worst_final_rate < 0.005,
            "REGRESSION: worst final rate {:.4} rad/s exceeded the 0.005 early-warning bound (was ~0.0002)",
            rep.worst_final_rate
        );
    }

    #[test]
    fn hexa_monte_carlo_campaign() {
        let rep = run_hexa_campaign(HEXA_TRIALS, HEXA_SEED);
        eprintln!(
            "hexa campaign: {} trials, {} failures | worst peak tilt {:.4} rad, worst final tilt {:.4} rad, worst final |ω| {:.4} rad/s",
            rep.trials, rep.failures, rep.worst_peak_tilt, rep.worst_final_tilt, rep.worst_final_rate
        );

        assert_eq!(
            rep.failures, 0,
            "hexa attitude recovery failed in {}/{} dispersed trials; first failures: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Physical safety bounds — same invariant on the 6-rotor airframe.
        assert!(
            rep.worst_peak_tilt < 1.4,
            "worst peak tilt across {} trials = {:.4} rad (divergence bound 1.4)",
            rep.trials, rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.05,
            "worst final tilt across {} trials = {:.4} rad (settle bound 0.05)",
            rep.trials, rep.worst_final_tilt
        );
        assert!(
            rep.worst_final_rate < 0.05,
            "worst final rate across {} trials = {:.4} rad/s (rate-null bound 0.05)",
            rep.trials, rep.worst_final_rate
        );
        // Tighter REGRESSION bounds, set just above the measured worst case
        // (peak 0.5226, final 0.0000, rate 0.0001 at seed 0x0FA1C0DEAD000003).
        assert!(
            rep.worst_peak_tilt < 0.54,
            "REGRESSION: worst peak tilt {:.4} rad exceeded the 0.54 early-warning bound (was ~0.523)",
            rep.worst_peak_tilt
        );
        assert!(
            rep.worst_final_tilt < 0.005,
            "REGRESSION: worst final tilt {:.4} rad exceeded the 0.005 early-warning bound (was ~0.0000)",
            rep.worst_final_tilt
        );
        assert!(
            rep.worst_final_rate < 0.005,
            "REGRESSION: worst final rate {:.4} rad/s exceeded the 0.005 early-warning bound (was ~0.0002)",
            rep.worst_final_rate
        );
    }
}
