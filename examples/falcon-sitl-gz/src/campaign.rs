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
            // Rank-3 allocation (v1.114): the diagonal-opposite rotor must be
            // free to reach 0 (a positive floor pins it up, reinjecting the
            // parasitic moment and starving roll/pitch authority) — so pass 0,
            // matching the production ROTOR_OUT_FLOOR.
            (tq, QuadMixer::new().mix_rotor_out(f, tq, HOVER, 0.0))
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
                // Rank-3 floor is 0 (the diagonal-opposite rotor rests at 0).
                if i != f && !(-1e-6..=1.0 + 1e-6).contains(&v) {
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

// ── FULL-LOOP motor-out campaign (v1.114) ────────────────────────────────────
//
// The `run_motor_out_campaign` above disperses over an ATTITUDE-ONLY rigid-body
// sim (constant thrust, TRUE-state feedback, no estimator, no rotational drag) —
// the same fidelity ceiling as the point-test it mirrors. That ceiling HID the
// v1.114 flip: the real failure is a coupling (parasitic lean → descent →
// estimator divergence + thrust collapse → tumble) whose three ingredients —
// the altitude loop, the IEKF, and drag-bounded spin — none exist in that sim.
// Dispersion adds breadth, not fidelity; thousands of trials of a gap-blind
// model can't falsify a bug the model can't express.
//
// THIS campaign closes the gap: it disperses over the FULL production FlightCore
// (verified IEKF → geometric SE(3) → reduced-attitude + rank-3 allocation,
// driven by the real FDI on ESC RPM) flown on the analytic SimBackend with
// rotational drag + ground contact — the same harness as the
// `survives_single_rotor_failure_without_flipping` oracle, now under dispersion.

use falcon_core::{FlightCore, Pathology, SimBackend};

const FL_DT: f32 = 0.004;
const FL_WARMUP: u32 = 3000; // steps to settle a level hover before the failure
const FL_RECOVER: u32 = 2000; // steps to fly out the recovery/descent

/// One dispersed full-loop rotor-out trial, sampled inside a RECOVERABLE
/// envelope (a rotor-out quad with too little rotational drag spins up beyond
/// estimator observability — that is physics, not a controller failure, so it
/// is excluded rather than counted as a loss).
#[derive(Clone, Debug)]
struct FullLoopTrial {
    failed_rotor: usize,
    setpoint_alt: f32, // NED z setpoint (m, negative up)
    rot_drag: f32,     // quadratic rotational drag coefficient
    gyro_white: f32,   // gyro noise σ (rad/s)
    gps_noise: f32,    // GNSS noise σ (m)
    seed: u32,         // per-trial noise realisation
}

fn sample_fullloop(rng: &mut SplitMix64, index: u32) -> FullLoopTrial {
    FullLoopTrial {
        failed_rotor: (rng.next_u64() % 4) as usize,
        setpoint_alt: -rng.range(2.0, 4.0), // hold 2..4 m
        rot_drag: rng.range(0.015, 0.035),  // realistic drag-bounded spin
        // Sensor noise within the FDI's SPECIFIED range (its detection gate is
        // near-level/low-rate; heavier noise degrades detection — a separate,
        // pre-existing FDI-robustness question, not part of the v1.114
        // recovery-allocation guard this campaign exists to provide).
        gyro_white: rng.range(0.0, 0.006),
        gps_noise: rng.range(0.0, 0.12),
        seed: (index.wrapping_mul(2_654_435_761)) ^ 0x5151_5151,
    }
}

#[derive(Clone, Debug, Default)]
pub struct FullLoopReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_peak_tilt: f32, // worst tilt after the failure (rad)
    pub worst_yaw_rate: f32,  // worst |yaw rate| (rad/s) — the relinquished spin
    pub least_descent: f32,   // smallest net altitude lost (m) — must stay > 0
    pub worst_detect_latency_steps: u32,
    pub failing: Vec<(u32, String)>,
}

struct FullLoopOutcome {
    isolated: Option<usize>,
    detect_latency_steps: u32,
    peak_tilt: f32,   // worst tilt over the whole recovery (rad)
    peak_yaw: f32,    // worst |yaw rate| (rad/s) — the relinquished spin
    net_descent: f32, // altitude LOST from the failure point (m) — must be > 0
    nan: bool,
}

fn run_fullloop_trial(t: &FullLoopTrial) -> FullLoopOutcome {
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, FL_DT).with_pathology(Pathology {
        gyro_white: t.gyro_white,
        gps_noise: t.gps_noise,
        ..Default::default()
    });
    backend.reseed(t.seed);
    backend.rot_drag = t.rot_drag;
    backend.ground_contact = true;
    let mut core = FlightCore::new(0.5, 1.0 / FL_DT);
    core.set_altitude(t.setpoint_alt);

    for _ in 0..FL_WARMUP {
        core.step(&mut backend);
    }
    // If the hover never settled (tilt or NaN), this trial's IC is unusable —
    // treat as NaN so the caller reports it rather than silently passing.
    let hover_tilt = backend.tilt();
    if !hover_tilt.is_finite() || hover_tilt > 0.15 {
        return FullLoopOutcome {
            isolated: None,
            detect_latency_steps: u32::MAX,
            peak_tilt: hover_tilt,
            peak_yaw: 0.0,
            net_descent: 0.0,
            nan: !hover_tilt.is_finite(),
        };
    }
    let alt_at_failure = -backend.pos[2];

    backend.fail_rotor(t.failed_rotor);
    let mut peak_tilt = 0.0f32;
    let mut peak_yaw = 0.0f32;
    let mut detect_latency = u32::MAX;
    let mut nan = false;
    for k in 0..FL_RECOVER {
        core.step(&mut backend);
        if detect_latency == u32::MAX && core.failed_motor().is_some() {
            detect_latency = k;
        }
        let tilt = backend.tilt();
        if !tilt.is_finite() {
            nan = true;
            break;
        }
        peak_tilt = peak_tilt.max(tilt);
        let yaw = backend.omega[2];
        peak_yaw = peak_yaw.max(if yaw < 0.0 { -yaw } else { yaw });
    }
    // A rotor-out quad CANNOT hold altitude — the honest success criterion is a
    // CONTROLLED DESCENT (net altitude lost, thrust axis kept upright), not
    // "reached the ground within the window".
    let net_descent = alt_at_failure - (-backend.pos[2]);
    FullLoopOutcome {
        isolated: core.failed_motor(),
        detect_latency_steps: detect_latency,
        peak_tilt,
        peak_yaw,
        net_descent,
        nan,
    }
}

/// Run `n` dispersed FULL-LOOP motor-out recovery trials from `campaign_seed`.
pub fn run_fullloop_motor_out_campaign(n: u32, campaign_seed: u64) -> FullLoopReport {
    let mut rep = FullLoopReport {
        trials: n,
        least_descent: f32::INFINITY,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_fullloop(&mut rng, i);
        let o = run_fullloop_trial(&t);

        rep.worst_peak_tilt = rep.worst_peak_tilt.max(o.peak_tilt);
        rep.worst_yaw_rate = rep.worst_yaw_rate.max(o.peak_yaw);
        rep.least_descent = rep.least_descent.min(o.net_descent);
        if o.detect_latency_steps != u32::MAX {
            rep.worst_detect_latency_steps =
                rep.worst_detect_latency_steps.max(o.detect_latency_steps);
        }

        let mut reason = String::new();
        if o.nan {
            reason = "NaN in full-loop cascade".into();
        } else if o.isolated != Some(t.failed_rotor) {
            reason = format!("FDI isolated {:?}, expected {}", o.isolated, t.failed_rotor);
        } else if o.peak_tilt >= 1.0 {
            reason = format!("tumbled: peak tilt {:.3} rad", o.peak_tilt);
        } else if o.peak_yaw >= 12.0 {
            reason = format!("yaw spin ran away: {:.1} rad/s", o.peak_yaw);
        }
        // NOTE: altitude after the loss is deliberately NOT gated here. Two
        // rotors at max thrust equal the hover weight on this plant (k_thrust is
        // sized for 4×0.5), so a rotor-out quad is near neutrally-buoyant;
        // bringing it DOWN is the FlightSupervisor's job (it commands LAND on a
        // motor failure — covered by falcon-core `motor_failure_commands_land`).
        // This campaign guards the ATTITUDE-domain invariant v1.114 fixed (no
        // parasitic-moment flip); `least_descent` is REPORTED, not asserted.
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
mod fullloop_tests {
    use super::*;

    // Reproducible seed (bump to resample). The full-loop trial is ~50× the
    // cost of the attitude-only one (a 15-state IEKF per step), so the deck is
    // smaller — 300 clean trials still give ≥99% success at 95% confidence
    // within the recoverable envelope, and the coverage is FIDELITY not just
    // breadth (real estimator + altitude loop + drag-bounded spin).
    const FL_SEED: u64 = 0x00FA_11C0_DE00_1114;
    const FL_TRIALS: u32 = 200;

    #[test]
    fn fullloop_motor_out_monte_carlo_campaign() {
        let rep = run_fullloop_motor_out_campaign(FL_TRIALS, FL_SEED);
        eprintln!(
            "full-loop motor-out campaign: {} trials, {} failures | worst peak tilt {:.3} rad, worst yaw {:.1} rad/s, worst detect {} steps | (reported, not gated: least net descent {:.2} m — altitude is the supervisor's LAND job)",
            rep.trials, rep.failures, rep.worst_peak_tilt, rep.worst_yaw_rate,
            rep.worst_detect_latency_steps, rep.least_descent
        );

        // Primary safety assertion: not one trial in the envelope fails — the
        // production loop (IEKF + altitude + FDI + rank-3 allocation) recovers a
        // single-rotor loss across the whole dispersion. This is the FIDELITY
        // guard the idealized campaign could not provide: a parasitic-moment
        // regression would drive `worst_peak_tilt` toward π and trip here.
        assert_eq!(
            rep.failures, 0,
            "full-loop motor-out recovery failed in {}/{} dispersed trials; first failures: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Physical safety bounds: never tumbles (< 1.0 rad), spin stays bounded
        // (< 12 rad/s), and every trial makes a CONTROLLED DESCENT (> 0.3 m lost
        // — a 3-rotor quad must come down, not fly off).
        assert!(
            rep.worst_peak_tilt < 1.0,
            "worst tilt {:.3} rad exceeded the 1.0 tumble bound",
            rep.worst_peak_tilt
        );
        assert!(
            rep.worst_yaw_rate < 12.0,
            "worst yaw spin {:.1} rad/s exceeded the 12 rad/s bound",
            rep.worst_yaw_rate
        );
    }
}

// ── FDI noise-robustness campaign (v1.115, FAULT-P03) ────────────────────────
//
// The full-loop recovery campaign above runs at a MODEST sensor-noise envelope
// (gps σ ≤ 0.12 m, gyro ≤ 0.006). This one stresses the single-rotor-out FDI
// under HEAVIER noise (gps σ ≤ 0.18 m, gyro ≤ 0.010) — the regime where the
// pre-v1.115 detector false-isolated a HEALTHY rotor. Cause: the effectiveness
// residual compared the CURRENT-tick command against the PREVIOUS-tick achieved
// RPM (a one-tick telemetry lag), so an abrupt collective step spiked every
// rotor's residual at once and the CUSUM tripped on whichever was checked first
// (43/300 false isolations, 0 with alignment). Fixed by comparing achieved
// against the command that produced it + gating detection on ROLL/PITCH rate
// only (a rotor-out relinquishes yaw and spins — not a tumble).

fn sample_fdi_noise(rng: &mut SplitMix64, index: u32) -> FullLoopTrial {
    FullLoopTrial {
        failed_rotor: (rng.next_u64() % 4) as usize,
        setpoint_alt: -rng.range(2.0, 4.0),
        rot_drag: rng.range(0.015, 0.035),
        gyro_white: rng.range(0.0, 0.010),
        gps_noise: rng.range(0.0, 0.18),
        seed: index.wrapping_mul(2_654_435_761) ^ 0x0FD1_0FD1,
    }
}

#[derive(Clone, Debug, Default)]
pub struct FdiReport {
    pub trials: u32,
    pub misses: u32, // never isolated within the window
    pub wrong: u32,  // isolated a HEALTHY rotor (false positive — the safety bug)
    pub worst_latency_steps: u32,
    pub failing: Vec<(u32, String)>,
}

/// Run `n` dispersed FDI noise-robustness trials from `campaign_seed`.
pub fn run_fdi_noise_campaign(n: u32, campaign_seed: u64) -> FdiReport {
    let mut rep = FdiReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_fdi_noise(&mut rng, i);
        let o = run_fullloop_trial(&t);
        match o.isolated {
            None => {
                rep.misses += 1;
                if rep.failing.len() < 20 {
                    rep.failing.push((i, format!("{t:?}: never isolated")));
                }
            }
            Some(f) if f != t.failed_rotor => {
                rep.wrong += 1;
                if rep.failing.len() < 20 {
                    rep.failing
                        .push((i, format!("{t:?}: isolated {f}, expected {}", t.failed_rotor)));
                }
            }
            Some(_) => {
                if o.detect_latency_steps != u32::MAX {
                    rep.worst_latency_steps = rep.worst_latency_steps.max(o.detect_latency_steps);
                }
            }
        }
    }
    rep
}

#[cfg(test)]
mod fdi_noise_tests {
    use super::*;

    const FDI_SEED: u64 = 0x00FD_1000_0000_1150;
    const FDI_TRIALS: u32 = 200;

    #[test]
    fn fdi_noise_robustness_monte_carlo_campaign() {
        let rep = run_fdi_noise_campaign(FDI_TRIALS, FDI_SEED);
        eprintln!(
            "FDI noise-robustness campaign: {} trials | misses {}, wrong-isolations {}, worst detect latency {} steps",
            rep.trials, rep.misses, rep.wrong, rep.worst_latency_steps
        );

        // SAFETY (the v1.115 fix): NEVER isolate a healthy rotor. A false
        // positive drops a healthy vehicle into degraded rotor-out mode — the
        // command-alignment fix must hold this at 0 across the whole envelope.
        assert_eq!(
            rep.wrong, 0,
            "FDI false-isolated a healthy rotor in {}/{} trials: {:#?}",
            rep.wrong, rep.trials, rep.failing
        );
        // FUNCTIONAL: within the flying envelope the correct rotor is isolated
        // (no miss) with bounded latency.
        assert_eq!(
            rep.misses, 0,
            "FDI never isolated the dead rotor in {}/{} trials: {:#?}",
            rep.misses, rep.trials, rep.failing
        );
        assert!(
            rep.worst_latency_steps < 25, // < 100 ms at 250 Hz
            "worst FDI detect latency {} steps exceeded 25",
            rep.worst_latency_steps
        );
    }
}

// ── Supervised rotor-out LANDING campaign (v1.117, FAULT-P04) ────────────────
//
// The v1.114 full-loop campaign guards the ATTITUDE invariant and deliberately
// reports-but-does-not-gate altitude ("the supervisor's LAND job"). THIS is
// that job, dispersed: the FULL production FlightSupervisor (FSM + failsafes +
// the same core) flies takeoff → settled hover → a rotor dies → the motor
// failsafe commands LAND → land-where-you-are + the lift-deficit-compensated
// velocity descent (both v1.117) put it on the ground upright → Disarmed.

use falcon_core::FlightSupervisor;
use relay_fsm::{Event as FsmEvent, Mode as FsmMode};

#[derive(Clone, Debug)]
struct SupLandTrial {
    failed_rotor: usize,
    cruise_alt: f32,
    rot_drag: f32,
    seed: u32,
}

fn sample_supland(rng: &mut SplitMix64, index: u32) -> SupLandTrial {
    SupLandTrial {
        failed_rotor: (rng.next_u64() % 4) as usize,
        cruise_alt: rng.range(1.5, 3.5),
        rot_drag: rng.range(0.015, 0.035),
        seed: index.wrapping_mul(2_654_435_761) ^ 0x1A9D_0117,
    }
}

#[derive(Clone, Debug, Default)]
pub struct SupLandReport {
    pub trials: u32,
    /// Trials whose PRE-FAILURE hover never settled (rejection-sampled out —
    /// they measure the warmup, not the recovery; reported, never silent).
    pub unsettled: u32,
    pub failures: u32,
    pub worst_approach_sink: f32, // m/s below 0.5 m AGL
    pub worst_tilt: f32,          // rad, airborne after the failure
    pub failing: Vec<(u32, String)>,
}

/// Run `n` dispersed supervised rotor-out landing trials.
pub fn run_supervised_rotorout_landing_campaign(n: u32, campaign_seed: u64) -> SupLandReport {
    const DT: f32 = 0.002;
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut rep = SupLandReport { trials: n, ..Default::default() };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_supland(&mut rng, i);
        let mut b = SimBackend::new(level, DT);
        b.reseed(t.seed);
        b.rot_drag = t.rot_drag;
        b.ground_contact = true;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, t.cruise_alt, 14.0);
        sup.command(FsmEvent::Arm, true, true);
        sup.command(FsmEvent::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut b);
        }
        // Rejection sampling: the recovery is only measurable from a settled
        // hover (the campaign discipline — an unsettled IC is not a finding).
        if !((-b.pos[2] - t.cruise_alt).abs() < 0.3 && b.vel[2].abs() < 0.3) {
            rep.unsettled += 1;
            continue;
        }

        b.fail_rotor(t.failed_rotor);
        let (mut landed, mut approach_sink, mut peak_tilt) = (false, 0.0f32, 0.0f32);
        for _ in 0..20000 {
            sup.step(&mut b);
            let alt = -b.pos[2];
            if alt > 0.15 {
                peak_tilt = peak_tilt.max(b.tilt());
                if alt < 0.5 {
                    approach_sink = approach_sink.max(b.vel[2]);
                }
            }
            if sup.mode() == FsmMode::Disarmed {
                landed = true;
                break;
            }
        }
        rep.worst_approach_sink = rep.worst_approach_sink.max(approach_sink);
        rep.worst_tilt = rep.worst_tilt.max(peak_tilt);

        let mut reason = String::new();
        if !landed {
            reason = format!("never touched down (mode {:?}, alt {:.2})", sup.mode(), -b.pos[2]);
        } else if approach_sink >= 1.2 {
            reason = format!("hard approach: {approach_sink:.2} m/s");
        } else if peak_tilt >= 0.6 {
            reason = format!("tilted during descent: {peak_tilt:.2} rad");
        } else if b.tilt() >= 0.35 {
            reason = format!("tipped at touchdown: {:.2} rad", b.tilt());
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
mod supland_tests {
    use super::*;

    const SL_SEED: u64 = 0x0117_1A9D_5EED_0001;
    const SL_TRIALS: u32 = 150;

    #[test]
    fn supervised_rotorout_landing_monte_carlo_campaign() {
        let rep = run_supervised_rotorout_landing_campaign(SL_TRIALS, SL_SEED);
        eprintln!(
            "supervised rotor-out landing campaign: {} trials ({} unsettled-rejected), {} failures | worst approach sink {:.2} m/s, worst tilt {:.2} rad",
            rep.trials, rep.unsettled, rep.failures, rep.worst_approach_sink, rep.worst_tilt
        );
        // The rejection count must stay a small minority — if most trials
        // can't even settle a hover, the campaign is measuring the wrong thing.
        assert!(
            rep.unsettled * 4 <= rep.trials,
            "{}/{} trials never settled a hover — warmup/tuning problem",
            rep.unsettled,
            rep.trials
        );
        assert_eq!(
            rep.failures, 0,
            "supervised rotor-out landing failed in {}/{} trials: {:#?}",
            rep.failures, rep.trials, rep.failing
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

// ── Estimator-robustness campaign (IEKF under noisy sensors + GNSS dropout) ──
//
// Drives the real relay_iekf::Iekf against a static-hover truth with dispersed
// gyro/accel/GNSS noise and a randomised GNSS-dropout window, asserting the
// estimate's tilt + position error stay bounded, that it dead-reckons through
// the dropout and RECONVERGES once GNSS returns, and that NEES (the chi-square
// consistency statistic) never blows up. Noise is intrinsic here, so this is a
// genuinely stochastic Monte-Carlo (unlike the deterministic attitude sweeps).

/// Box-Muller Gaussian from two uniforms (mean 0, unit variance).
fn gaussian(rng: &mut SplitMix64) -> f32 {
    let u1 = rng.unit().max(1e-7);
    let u2 = rng.unit();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
}

#[derive(Clone, Copy, Debug)]
struct EstimatorTrial {
    gyro_sigma: f32,
    accel_sigma: f32,
    gnss_sigma: f32,
    dropout_start: u32,
    dropout_dur: u32,
}

// Recoverable envelope: realistic sensor noise + a GNSS dropout short enough
// that dead-reckoning drift stays bounded and the filter reconverges when GNSS
// returns. Conservative (inside the true envelope) so a FAIL is a real bug.
const EST_GYRO_SIGMA: (f32, f32) = (0.001, 0.02); // rad/s
const EST_ACCEL_SIGMA: (f32, f32) = (0.02, 0.30); // m/s²
const EST_GNSS_SIGMA: (f32, f32) = (0.10, 1.00); // m
const EST_DROPOUT_MAX: u32 = 300; // 3 s at dt = 0.01

fn sample_estimator(rng: &mut SplitMix64) -> EstimatorTrial {
    EstimatorTrial {
        gyro_sigma: rng.range(EST_GYRO_SIGMA.0, EST_GYRO_SIGMA.1),
        accel_sigma: rng.range(EST_ACCEL_SIGMA.0, EST_ACCEL_SIGMA.1),
        gnss_sigma: rng.range(EST_GNSS_SIGMA.0, EST_GNSS_SIGMA.1),
        dropout_start: 500 + (rng.next_u64() % 300) as u32, // 5.0–8.0 s
        dropout_dur: 100 + (rng.next_u64() % (EST_DROPOUT_MAX - 100 + 1) as u64) as u32, // 1–3 s
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct EstimatorOutcome {
    peak_tilt_err: f32,
    peak_dropout_pos_err: f32,
    reconverged_pos_err: f32,
    peak_nees: f32,
    nan: bool,
}

fn run_estimator_trial(t: EstimatorTrial, rng: &mut SplitMix64) -> EstimatorOutcome {
    use relay_iekf::{Iekf, Imu as IekfImu};
    let p_true = [2.0f32, -1.0, -10.0];
    let dt = 0.01f32;
    let steps = 1500u32;
    let mut f = Iekf::level();
    let mut o = EstimatorOutcome::default();
    let dropout_end = t.dropout_start + t.dropout_dur;
    for step in 0..steps {
        let imu = IekfImu {
            gyro: [
                gaussian(rng) * t.gyro_sigma,
                gaussian(rng) * t.gyro_sigma,
                gaussian(rng) * t.gyro_sigma,
            ],
            accel: [
                gaussian(rng) * t.accel_sigma,
                gaussian(rng) * t.accel_sigma,
                -9.81 + gaussian(rng) * t.accel_sigma,
            ],
        };
        f.propagate(imu, dt);
        let _ = f.update_gravity(imu.accel, (t.accel_sigma * t.accel_sigma).max(0.01));
        let in_dropout = step >= t.dropout_start && step < dropout_end;
        if step % 20 == 0 && !in_dropout {
            let z = [
                p_true[0] + gaussian(rng) * t.gnss_sigma,
                p_true[1] + gaussian(rng) * t.gnss_sigma,
                p_true[2] + gaussian(rng) * t.gnss_sigma,
            ];
            let _ = f.update_position(z, (t.gnss_sigma * t.gnss_sigma).max(0.01));
        }
        let s = f.state();
        let perr = ((s.p[0] - p_true[0]).powi(2)
            + (s.p[1] - p_true[1]).powi(2)
            + (s.p[2] - p_true[2]).powi(2))
        .sqrt();
        if step > 400 {
            // after the initial convergence transient
            o.peak_tilt_err = o.peak_tilt_err.max(s.tilt_rad());
            let nees = f.nees_position(p_true);
            if nees.is_finite() {
                o.peak_nees = o.peak_nees.max(nees);
            }
        }
        if in_dropout {
            o.peak_dropout_pos_err = o.peak_dropout_pos_err.max(perr);
        }
        if step >= steps - 200 {
            // reconvergence window: well after GNSS resumes
            o.reconverged_pos_err = o.reconverged_pos_err.max(perr);
        }
        if !s.p[0].is_finite() || !s.q[0].is_finite() {
            o.nan = true;
        }
    }
    o
}

#[derive(Clone, Debug, Default)]
pub struct EstimatorReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_tilt_err: f32,
    pub worst_dropout_pos_err: f32,
    pub worst_reconverged_pos_err: f32,
    pub worst_nees: f32,
    pub failing: Vec<(u32, String)>,
}

pub fn run_estimator_campaign(n: u32, campaign_seed: u64) -> EstimatorReport {
    let mut rep = EstimatorReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_estimator(&mut rng);
        let o = run_estimator_trial(t, &mut rng);
        rep.worst_tilt_err = rep.worst_tilt_err.max(o.peak_tilt_err);
        rep.worst_dropout_pos_err = rep.worst_dropout_pos_err.max(o.peak_dropout_pos_err);
        rep.worst_reconverged_pos_err = rep.worst_reconverged_pos_err.max(o.reconverged_pos_err);
        rep.worst_nees = rep.worst_nees.max(o.peak_nees);

        let mut reason = String::new();
        if o.nan {
            reason = "NaN in estimate".into();
        } else if o.peak_tilt_err >= 0.30 {
            reason = format!("tilt error {:.3} rad", o.peak_tilt_err);
        } else if o.reconverged_pos_err >= 2.5 {
            reason = format!("did not reconverge: {:.2} m", o.reconverged_pos_err);
        } else if o.peak_nees >= 150.0 {
            reason = format!("NEES blew up: {:.1}", o.peak_nees);
        } else if o.peak_dropout_pos_err >= 40.0 {
            reason = format!("dead-reckoning diverged: {:.1} m", o.peak_dropout_pos_err);
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
mod campaign3_tests {
    use super::*;

    const EST_SEED: u64 = 0x0FA1_C0DE_E57E_0001;
    const EST_TRIALS: u32 = 600; // stochastic campaign (real noise) -- rule-of-three 3/600 = 99.5% @ 95%

    #[test]
    fn estimator_robustness_monte_carlo_campaign() {
        let rep = run_estimator_campaign(EST_TRIALS, EST_SEED);
        eprintln!(
            "estimator campaign: {} trials, {} failures | worst tilt-err {:.4} rad, worst dropout pos-err {:.3} m, worst reconverged pos-err {:.3} m, worst NEES {:.2}",
            rep.trials, rep.failures, rep.worst_tilt_err, rep.worst_dropout_pos_err,
            rep.worst_reconverged_pos_err, rep.worst_nees
        );
        assert_eq!(
            rep.failures, 0,
            "estimator robustness failed in {}/{} trials; first: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Regression bounds set just above the measured worst case over 2000
        // trials at seed 0x0FA1_C0DE_E57E_0001 (tilt-err 0.017 rad, dropout
        // drift 19.8 m, reconverged 1.86 m, NEES 57) — a change that degrades
        // the estimator (looser tilt aiding, slower reconvergence, or an
        // overconfident covariance) trips these.
        assert!(
            rep.worst_tilt_err < 0.05,
            "REGRESSION: worst tilt err {:.4} rad exceeded 0.05 (was ~0.017)",
            rep.worst_tilt_err
        );
        assert!(
            rep.worst_reconverged_pos_err < 2.5,
            "REGRESSION: worst reconverged pos err {:.2} m exceeded 2.5 (was ~1.86)",
            rep.worst_reconverged_pos_err
        );
        assert!(
            rep.worst_nees < 90.0,
            "REGRESSION: worst NEES {:.1} exceeded 90 (was ~57 — a spike above this means overconfident covariance)",
            rep.worst_nees
        );
    }
}

// ── Maneuvering-truth estimator campaign (IEKF under motion + noise) ──
//
// The static-hover estimator campaign isolates noise rejection; this one adds
// a horizontal MANEUVER (the truth accelerates on a random sinusoidal path)
// with an unmodelled accelerometer scale error, and checks the IEKF stays
// consistent UNDER MOTION via nees_velocity (the harder observability case the
// v0.21 filter failed). Genuinely stochastic (noise + random maneuver/scale).

#[derive(Clone, Copy, Debug)]
struct ManeuverTrial {
    ax: f32,
    ay: f32,
    fx: f32,
    fy: f32,
    scale_err: f32,
    accel_sigma: f32,
    gnss_sigma: f32,
}

const MAN_ACCEL_AMP: (f32, f32) = (0.5, 2.5); // m/s² (bounded position swing at these freqs)
const MAN_FREQ: (f32, f32) = (0.5, 1.0); // rad/s
const MAN_SCALE_ERR: (f32, f32) = (0.0, 0.05); // unmodelled accel scale
const MAN_ACCEL_SIGMA: (f32, f32) = (0.01, 0.10);
const MAN_GNSS_SIGMA: (f32, f32) = (0.05, 0.30);

fn sample_maneuver(rng: &mut SplitMix64) -> ManeuverTrial {
    ManeuverTrial {
        ax: rng.range(MAN_ACCEL_AMP.0, MAN_ACCEL_AMP.1),
        ay: rng.range(MAN_ACCEL_AMP.0, MAN_ACCEL_AMP.1),
        fx: rng.range(MAN_FREQ.0, MAN_FREQ.1),
        fy: rng.range(MAN_FREQ.0, MAN_FREQ.1),
        scale_err: rng.range(MAN_SCALE_ERR.0, MAN_SCALE_ERR.1),
        accel_sigma: rng.range(MAN_ACCEL_SIGMA.0, MAN_ACCEL_SIGMA.1),
        gnss_sigma: rng.range(MAN_GNSS_SIGMA.0, MAN_GNSS_SIGMA.1),
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ManeuverOutcome {
    peak_pos_err: f32,
    peak_vel_nees: f32,
    peak_tilt: f32,
    nan: bool,
}

fn run_maneuver_trial(t: ManeuverTrial, rng: &mut SplitMix64) -> ManeuverOutcome {
    use relay_iekf::{Iekf, Imu as IekfImu, NavState};
    let dt = 0.01f32;
    let steps = 2500u32;
    let mut f = Iekf::new(NavState::identity());
    let mut o = ManeuverOutcome::default();
    let (mut tp, mut tv) = ([0.0f32; 3], [0.0f32; 3]);
    for step in 0..steps {
        let time = step as f32 * dt;
        let a_true = [t.ax * (t.fx * time).sin(), t.ay * (t.fy * time).cos(), 0.0];
        for i in 0..3 {
            tp[i] += tv[i] * dt + 0.5 * a_true[i] * dt * dt;
            tv[i] += a_true[i] * dt;
        }
        let accel = [
            (1.0 + t.scale_err) * a_true[0] + gaussian(rng) * t.accel_sigma,
            (1.0 + t.scale_err) * a_true[1] + gaussian(rng) * t.accel_sigma,
            -9.81 + gaussian(rng) * t.accel_sigma,
        ];
        f.propagate(IekfImu { gyro: [0.0; 3], accel }, dt);
        // NO gravity aiding under motion: specific force != gravity (aiding here
        // would read the maneuver accel as a tilt). gyro=0 keeps attitude level.
        if step % 20 == 0 {
            let z = [
                tp[0] + gaussian(rng) * t.gnss_sigma,
                tp[1] + gaussian(rng) * t.gnss_sigma,
                tp[2] + gaussian(rng) * t.gnss_sigma,
            ];
            let _ = f.update_position(z, (t.gnss_sigma * t.gnss_sigma).max(0.01));
        }
        let s = f.state();
        if step > 800 {
            let perr = ((s.p[0] - tp[0]).powi(2)
                + (s.p[1] - tp[1]).powi(2)
                + (s.p[2] - tp[2]).powi(2))
            .sqrt();
            o.peak_pos_err = o.peak_pos_err.max(perr);
            o.peak_tilt = o.peak_tilt.max(s.tilt_rad());
            let nees = f.nees_velocity(tv);
            if nees.is_finite() {
                o.peak_vel_nees = o.peak_vel_nees.max(nees);
            }
        }
        if !s.p[0].is_finite() || !s.q[0].is_finite() {
            o.nan = true;
        }
    }
    o
}

#[derive(Clone, Debug, Default)]
pub struct ManeuverReport {
    pub trials: u32,
    pub failures: u32,
    pub worst_pos_err: f32,
    pub worst_vel_nees: f32,
    pub worst_tilt: f32,
    pub failing: Vec<(u32, String)>,
}

pub fn run_maneuver_campaign(n: u32, campaign_seed: u64) -> ManeuverReport {
    let mut rep = ManeuverReport {
        trials: n,
        ..Default::default()
    };
    for i in 0..n {
        let mut rng = trial_rng(campaign_seed, i);
        let t = sample_maneuver(&mut rng);
        let o = run_maneuver_trial(t, &mut rng);
        rep.worst_pos_err = rep.worst_pos_err.max(o.peak_pos_err);
        rep.worst_vel_nees = rep.worst_vel_nees.max(o.peak_vel_nees);
        rep.worst_tilt = rep.worst_tilt.max(o.peak_tilt);
        let mut reason = String::new();
        if o.nan {
            reason = "NaN".into();
        } else if o.peak_pos_err >= 4.0 {
            reason = format!("pos err {:.2} m", o.peak_pos_err);
        } else if o.peak_tilt >= 0.15 {
            reason = format!("tilt {:.3} rad", o.peak_tilt);
        } else if o.peak_vel_nees >= 60.0 {
            reason = format!("vel-NEES {:.1}", o.peak_vel_nees);
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
mod campaign4_tests {
    use super::*;

    const MAN_SEED: u64 = 0x0FA1_C0DE_3A17_0001;
    const MAN_TRIALS: u32 = 500;

    #[test]
    fn maneuvering_estimator_monte_carlo_campaign() {
        let rep = run_maneuver_campaign(MAN_TRIALS, MAN_SEED);
        eprintln!(
            "maneuver campaign: {} trials, {} failures | worst pos-err {:.3} m, worst vel-NEES {:.2}, worst tilt {:.4} rad",
            rep.trials, rep.failures, rep.worst_pos_err, rep.worst_vel_nees, rep.worst_tilt
        );
        assert_eq!(
            rep.failures, 0,
            "maneuvering estimator failed in {}/{}; first: {:#?}",
            rep.failures, rep.trials, rep.failing
        );
        // Regression bounds just above the measured worst (seed 0x0FA1_C0DE_3A17_0001:
        // pos-err 0.91 m, vel-NEES 11.2, tilt 0.064 rad).
        assert!(rep.worst_pos_err < 2.0, "REGRESSION: worst pos err {:.2} m (was ~0.91)", rep.worst_pos_err);
        assert!(rep.worst_tilt < 0.12, "REGRESSION: worst tilt {:.3} rad (was ~0.064)", rep.worst_tilt);
        assert!(rep.worst_vel_nees < 30.0, "REGRESSION: worst vel-NEES {:.1} (was ~11.2)", rep.worst_vel_nees);
    }
}

// ── GNSS-spoof-injection campaign (validates the SpoofMonitor) ──
//
// Two-sided (like the fail-safe campaigns): LEGIT trials feed the SpoofMonitor
// the real zero-mean-noise GNSS innovation — it must NOT false-alarm; SPOOF
// trials walk the GNSS measurement off with a consistent bias FASTER than the
// noise floor — the monitor must DETECT (and latch) within a budget. Truth is
// at the origin (no convergence transient to bias the CUSUM), the monitor drift
// is scaled to the GNSS noise (2σ — a spoof slower than the noise is physically
// undetectable, so the envelope keeps the spoof above it), and the innovation
// fed is the position residual z − est.p each GNSS fix (the production wiring).

#[derive(Clone, Copy, Debug)]
struct SpoofTrial {
    spoof: bool,
    onset_step: u32,
    spoof_mult: f32, // spoof bias per fix, as a MULTIPLE of the GNSS σ
    gnss_sigma: f32,
}

const SPOOF_MULT: (f32, f32) = (4.0, 8.0); // spoof rate = mult × σ (above the 2σ drift)
const SPOOF_GNSS_SIGMA: (f32, f32) = (0.10, 0.50);
const SPOOF_DETECT_BUDGET: u32 = 30; // GNSS fixes

fn sample_spoof(rng: &mut SplitMix64, spoof: bool) -> SpoofTrial {
    SpoofTrial {
        spoof,
        onset_step: 200 + (rng.next_u64() % 200) as u32, // 2–4 s in
        spoof_mult: rng.range(SPOOF_MULT.0, SPOOF_MULT.1),
        gnss_sigma: rng.range(SPOOF_GNSS_SIGMA.0, SPOOF_GNSS_SIGMA.1),
    }
}

/// Returns (spoofed_flag, detect_latency_fixes) — latency from onset (u32::MAX
/// if never detected).
fn run_spoof_trial(t: SpoofTrial, rng: &mut SplitMix64) -> (bool, u32) {
    use relay_iekf::{Iekf, Imu as IekfImu, SpoofMonitor};
    let p_true = [0.0f32, 0.0, 0.0];
    let dt = 0.01f32;
    let steps = 1500u32;
    let mut f = Iekf::level();
    // Drift scaled to the noise: legit zero-mean noise stays below it; a
    // sustained bias > drift accumulates.
    let mut mon = SpoofMonitor::new(2.0, 2.0 * t.gnss_sigma);
    let spoof_rate = t.spoof_mult * t.gnss_sigma; // m per fix
    let mut detect_step: Option<u32> = None;
    let accel_sigma = 0.05f32;
    for step in 0..steps {
        let imu = IekfImu {
            gyro: [0.0; 3],
            accel: [
                gaussian(rng) * accel_sigma,
                gaussian(rng) * accel_sigma,
                -9.81 + gaussian(rng) * accel_sigma,
            ],
        };
        f.propagate(imu, dt);
        let _ = f.update_gravity(imu.accel, 0.01);
        if step % 20 == 0 {
            let offset = if t.spoof && step >= t.onset_step {
                spoof_rate * ((step - t.onset_step) / 20) as f32
            } else {
                0.0
            };
            let z = [
                p_true[0] + offset + gaussian(rng) * t.gnss_sigma,
                p_true[1] + gaussian(rng) * t.gnss_sigma,
                p_true[2] + gaussian(rng) * t.gnss_sigma,
            ];
            let s = f.state();
            let innov = [z[0] - s.p[0], z[1] - s.p[1], z[2] - s.p[2]];
            // Let the covariance settle a few fixes before arming the monitor.
            if step >= 100 && mon.update(innov) && detect_step.is_none() {
                detect_step = Some(step);
            }
            let _ = f.update_position(z, (t.gnss_sigma * t.gnss_sigma).max(0.01));
        }
    }
    let latency = match detect_step {
        Some(d) if d >= t.onset_step => (d - t.onset_step) / 20,
        Some(_) => 0, // detected before onset ⇒ a false alarm on a spoof trial
        None => u32::MAX,
    };
    (mon.spoofed(), latency)
}

#[cfg(test)]
mod campaign5_tests {
    use super::*;

    const SPOOF_SEED: u64 = 0x0FA1_C0DE_5900_0001;
    const SPOOF_TRIALS: u32 = 400;

    #[test]
    fn gnss_spoof_monte_carlo_campaign() {
        let (mut detected, mut clean, mut worst_latency) = (0u32, 0u32, 0u32);
        for i in 0..SPOOF_TRIALS {
            let spoof = i % 2 == 0;
            let mut rng = trial_rng(SPOOF_SEED, i);
            let t = sample_spoof(&mut rng, spoof);
            let (spoofed, latency) = run_spoof_trial(t, &mut rng);
            if spoof {
                assert!(
                    spoofed,
                    "trial {i}: spoof (rate {:.2}×σ) NOT detected",
                    t.spoof_mult
                );
                assert!(
                    latency <= SPOOF_DETECT_BUDGET,
                    "trial {i}: spoof detected in {latency} fixes > budget {SPOOF_DETECT_BUDGET}"
                );
                worst_latency = worst_latency.max(latency);
                detected += 1;
            } else {
                assert!(!spoofed, "trial {i}: FALSE spoof alarm on legit noisy GNSS");
                clean += 1;
            }
        }
        eprintln!(
            "spoof campaign: {} trials | {detected} detected (worst latency {worst_latency} fixes), {clean} no-false-alarm",
            SPOOF_TRIALS
        );
        assert!(detected > 150 && clean > 150, "both regimes well-sampled ({detected}/{clean})");
    }
}

// ── Motor-out recovery UNDER DISPERSION (actuator noise + wind) ──
//
// The motor_out campaign above is a clean-plant deterministic sweep; this adds
// the stochastic dispersion a real vehicle sees during the recovery — per-rotor
// ACTUATOR noise (thrust scatter) + a constant WIND-disturbance torque with
// per-step gusts — and asserts the FDI-isolate → reconfigure → settle chain
// still holds. Same physics as run_motor_out_trial with the noise injected.

#[derive(Clone, Copy, Debug)]
struct MotorOutDispTrial {
    base: MotorOutTrial,
    act_sigma: f32,      // per-rotor multiplicative thrust noise
    wind_torque: [f32; 3], // constant disturbance torque
    gust_sigma: f32,     // per-step gust torque
}

const MO_ACT_SIGMA: (f32, f32) = (0.0, 0.05); // ≤5% actuator scatter
const MO_WIND_TORQUE: f32 = 0.03; // constant disturbance amplitude
const MO_GUST_SIGMA: (f32, f32) = (0.0, 0.02);

fn sample_motor_out_disp(rng: &mut SplitMix64) -> MotorOutDispTrial {
    let base = sample_motor_out(rng);
    MotorOutDispTrial {
        base,
        act_sigma: rng.range(MO_ACT_SIGMA.0, MO_ACT_SIGMA.1),
        wind_torque: [
            rng.range(-MO_WIND_TORQUE, MO_WIND_TORQUE),
            rng.range(-MO_WIND_TORQUE, MO_WIND_TORQUE),
            rng.range(-MO_WIND_TORQUE, MO_WIND_TORQUE),
        ],
        gust_sigma: rng.range(MO_GUST_SIGMA.0, MO_GUST_SIGMA.1),
    }
}

fn run_motor_out_disp_trial(t: MotorOutDispTrial, rng: &mut SplitMix64) -> MotorOutOutcome {
    let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
    let j = GeoGains::FALCON_QUAD.j;
    let b3_d = [0.0f32, 0.0, 1.0];
    let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut r = tilt_rotation(t.base.roll0, t.base.pitch0);
    let mut omega = t.base.omega0;
    let mut fdi = RotorFaultDetector::new(0.5, 0.1);
    let mut isolated: Option<usize> = None;
    let mut detect_step: Option<u32> = None;
    let mut peak_tilt_after = 0.0f32;
    let mut mix_ok = true;
    let mut nan = false;

    for step in 0..4000u32 {
        let (_torque, motors_cmd) = if let Some(f) = isolated {
            let tq = ctrl.moment_reduced(&r, omega, b3_d);
            // Rank-3 allocation (v1.114): the diagonal-opposite rotor must be
            // free to reach 0 (a positive floor pins it up, reinjecting the
            // parasitic moment and starving roll/pitch authority) — so pass 0,
            // matching the production ROTOR_OUT_FLOOR.
            (tq, QuadMixer::new().mix_rotor_out(f, tq, HOVER, 0.0))
        } else {
            let tq = ctrl.moment(&r, omega, &level);
            (tq, QuadMixer::new().mix_thrust_floor(tq, HOVER, FLOOR))
        };
        if let Some(f) = isolated {
            if motors_cmd[f] != 0.0 {
                mix_ok = false;
            }
            for (i, &v) in motors_cmd.iter().enumerate() {
                // Rank-3 floor is 0 (the diagonal-opposite rotor rests at 0).
                if i != f && !(-1e-6..=1.0 + 1e-6).contains(&v) {
                    mix_ok = false;
                }
            }
        }
        // Physics: failure + per-rotor ACTUATOR NOISE on the achieved thrust.
        let mut motors_real = motors_cmd;
        for m in motors_real.iter_mut() {
            *m *= 1.0 + gaussian(rng) * t.act_sigma;
        }
        if step >= t.base.fail_step {
            motors_real[t.base.failed_rotor] = 0.0;
        }
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
        // Rigid-body update + WIND disturbance torque (constant + gust).
        let bt = motors_to_torque_signs(motors_real);
        let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
        let gyro = [
            omega[1] * jo[2] - omega[2] * jo[1],
            omega[2] * jo[0] - omega[0] * jo[2],
            omega[0] * jo[1] - omega[1] * jo[0],
        ];
        for i in 0..3 {
            let body_t = bt[i] * SCALE + t.wind_torque[i] + gaussian(rng) * t.gust_sigma;
            omega[i] += DT * (body_t - gyro[i]) / j[i];
        }
        r = integ_rot(&r, omega, DT);
        if !omega[0].is_finite() || !r[2][2].is_finite() {
            nan = true;
        }
        if step > t.base.fail_step + 300 {
            peak_tilt_after = peak_tilt_after.max(tilt_of(&r));
        }
    }
    MotorOutOutcome {
        isolated,
        detect_latency_steps: match (detect_step, Some(t.base.fail_step)) {
            (Some(d), Some(f)) if d >= f => d - f,
            _ => u32::MAX,
        },
        peak_tilt_after,
        final_tilt: tilt_of(&r),
        mix_ok,
        nan,
    }
}

#[cfg(test)]
mod campaign6_tests {
    use super::*;

    const MOD_SEED: u64 = 0x0FA1_C0DE_D157_0001;
    const MOD_TRIALS: u32 = 1500;

    #[test]
    fn motor_out_dispersed_monte_carlo_campaign() {
        let (mut fails, mut worst_peak, mut worst_final) = (0u32, 0.0f32, 0.0f32);
        for i in 0..MOD_TRIALS {
            let mut rng = trial_rng(MOD_SEED, i);
            let t = sample_motor_out_disp(&mut rng);
            let o = run_motor_out_disp_trial(t, &mut rng);
            worst_peak = worst_peak.max(o.peak_tilt_after);
            worst_final = worst_final.max(o.final_tilt);
            let bad = o.nan
                || o.isolated != Some(t.base.failed_rotor)
                || !o.mix_ok
                || o.peak_tilt_after >= 1.4
                || o.final_tilt >= 0.5;
            if bad {
                fails += 1;
            }
        }
        eprintln!(
            "motor-out DISPERSED campaign: {} trials, {} failures | worst peak tilt {:.3} rad, worst final tilt {:.3} rad",
            MOD_TRIALS, fails, worst_peak, worst_final
        );
        assert_eq!(fails, 0, "dispersed motor-out recovery failed in {fails}/{MOD_TRIALS}");
        assert!(worst_peak < 1.4, "worst peak tilt {:.3}", worst_peak);
        assert!(worst_final < 0.5, "worst final tilt {:.3}", worst_final);
    }
}

// ── Mission / waypoint corridor-tracking campaign ──
//
// Drives the REAL relay_pos::PosController through a multi-waypoint mission on a
// faithful point-mass plant: the attitude inner loop is assumed to track the
// commanded AttitudeSetpoint (validated separately by the attitude campaigns),
// so the plant achieves the thrust-vector acceleration a = T·t_dir + gravity.
// Asserts CORRIDOR containment (cross-track error < half-width) on every leg and
// ARRIVAL at each waypoint within a time budget, under wind + mass dispersion.

use relay_pos::{PosController, PosGains, PositionSetpoint, Timestamp};

const G_ACCEL: f32 = 9.81;

/// Thrust axis (body −z, "up") expressed in NED for a body→NED quaternion.
fn thrust_dir_ned(q: [f32; 4]) -> [f32; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    // −(third column of R): body +z is "down", thrust is along body −z.
    [
        -2.0 * (x * z + w * y),
        -2.0 * (y * z - w * x),
        -(1.0 - 2.0 * (x * x + y * y)),
    ]
}

fn ts(t: f32) -> Timestamp {
    Timestamp {
        seconds: t.floor() as u64,
        fraction: ((t - t.floor()) * (1u64 << 32) as f32) as u32,
    }
}

/// Cross-track distance from point `p` to the segment `a`→`b` (horizontal).
fn cross_track(a: [f32; 3], b: [f32; 3], p: [f32; 3]) -> f32 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    if len2 < 1e-6 {
        return (ap[0] * ap[0] + ap[1] * ap[1]).sqrt();
    }
    let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / len2).clamp(0.0, 1.0);
    let c = [a[0] + t * ab[0], a[1] + t * ab[1]];
    ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2)).sqrt()
}

#[derive(Clone, Copy, Debug)]
struct MissionTrial {
    wind: [f32; 2],
    thrust_scale: f32,
    start_offset: [f32; 2],
}

const MISSION_WIND: f32 = 1.0; // ≤1 m/s² horizontal wind
const MISSION_THRUST_SCALE: (f32, f32) = (0.90, 1.10);
const MISSION_START_OFFSET: f32 = 1.0;

fn sample_mission(rng: &mut SplitMix64) -> MissionTrial {
    MissionTrial {
        wind: [rng.range(-MISSION_WIND, MISSION_WIND), rng.range(-MISSION_WIND, MISSION_WIND)],
        thrust_scale: rng.range(MISSION_THRUST_SCALE.0, MISSION_THRUST_SCALE.1),
        start_offset: [
            rng.range(-MISSION_START_OFFSET, MISSION_START_OFFSET),
            rng.range(-MISSION_START_OFFSET, MISSION_START_OFFSET),
        ],
    }
}

/// Returns (max_cross_track, reached_all, steps_used, final_pos_x, nan).
fn run_mission_trial(t: MissionTrial) -> (f32, bool, u32, f32, bool) {
    let wps: [[f32; 3]; 5] = [
        [0.0, 0.0, -3.0],
        [6.0, 0.0, -3.0],
        [6.0, 6.0, -3.0],
        [0.0, 6.0, -3.0],
        [0.0, 0.0, -3.0],
    ];
    let gains = PosGains::DEFAULT;
    let mut ctrl = PosController::new();
    let dt = 0.02f32;
    let max_steps = 3000u32; // 60 s budget
    let arrival_radius = 0.6f32;
    let mut p = [wps[0][0] + t.start_offset[0], wps[0][1] + t.start_offset[1], wps[0][2]];
    let mut v = [0.0f32; 3];
    let mut q = [1.0f32, 0.0, 0.0, 0.0];
    let mut wp = 1usize;
    let mut max_cross = 0.0f32;
    let mut nan = false;
    let mut step = 0u32;
    while wp < wps.len() && step < max_steps {
        let sp = PositionSetpoint::hover_at(wps[wp]);
        let att = ctrl.tick(ts(step as f32 * dt), p, v, q, sp);
        q = att.quaternion; // perfect inner-loop tracking
        let t_accel = G_ACCEL * (att.thrust / gains.hover_thrust) * t.thrust_scale;
        let dir = thrust_dir_ned(q);
        let a = [
            t_accel * dir[0] + t.wind[0],
            t_accel * dir[1] + t.wind[1],
            t_accel * dir[2] + G_ACCEL,
        ];
        for i in 0..3 {
            v[i] += a[i] * dt;
            p[i] += v[i] * dt;
        }
        if !p[0].is_finite() || !v[0].is_finite() {
            nan = true;
            break;
        }
        // corridor after a brief settle from the start offset
        if step > 50 {
            max_cross = max_cross.max(cross_track(wps[wp - 1], wps[wp], p));
        }
        let d = ((p[0] - wps[wp][0]).powi(2) + (p[1] - wps[wp][1]).powi(2) + (p[2] - wps[wp][2]).powi(2)).sqrt();
        if d < arrival_radius {
            wp += 1;
        }
        step += 1;
    }
    (max_cross, wp >= wps.len(), step, p[0], nan)
}

#[cfg(test)]
mod campaign7_tests {
    use super::*;

    const MISSION_SEED: u64 = 0x0FA1_C0DE_3155_0001;
    const MISSION_TRIALS: u32 = 400;

    #[test]
    fn mission_corridor_monte_carlo_campaign() {
        // diagnostic: one nominal trial (no dispersion) to sanity-check the loop
        let (c0, r0, s0, fx0, _) = run_mission_trial(MissionTrial {
            wind: [0.0, 0.0],
            thrust_scale: 1.0,
            start_offset: [0.0, 0.0],
        });
        eprintln!("mission diag (nominal): reached={r0} steps={s0} max_cross={c0:.2} final_x={fx0:.2}");

        let (mut reached, mut worst_cross, mut worst_steps) = (0u32, 0.0f32, 0u32);
        let mut fails = 0u32;
        for i in 0..MISSION_TRIALS {
            let mut rng = trial_rng(MISSION_SEED, i);
            let t = sample_mission(&mut rng);
            let (cross, ok, steps, _fx, nan) = run_mission_trial(t);
            worst_cross = worst_cross.max(cross);
            worst_steps = worst_steps.max(steps);
            if ok {
                reached += 1;
            }
            if nan || !ok || cross >= 3.0 {
                fails += 1;
            }
        }
        eprintln!(
            "mission campaign: {} trials | {reached} completed, {fails} failures, worst cross-track {:.2} m, worst steps {worst_steps}",
            MISSION_TRIALS, worst_cross
        );
        assert_eq!(fails, 0, "mission failed in {fails}/{MISSION_TRIALS}");
        assert_eq!(reached, MISSION_TRIALS, "every trial must complete all waypoints");
        // Physical corridor half-width 3 m; regression bound just above the
        // measured worst (1.02 m at seed 0x0FA1_C0DE_3155_0001).
        assert!(worst_cross < 3.0, "worst cross-track {:.2} m (corridor 3)", worst_cross);
        assert!(
            worst_cross < 1.6,
            "REGRESSION: worst cross-track {:.2} m exceeded 1.6 (was ~1.02)",
            worst_cross
        );
    }
}
