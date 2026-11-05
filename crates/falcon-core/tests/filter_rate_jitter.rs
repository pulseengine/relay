//! REVIEW OF v1.140, DEFECT A: frame-period JITTER must not re-design the
//! gyro filters.
//!
//! THE DEFECT. The cascade re-synced its loop rate whenever ONE frame's
//! `1/dt` left a 2% band of the last rate, and every resync REBUILT the gyro
//! low-pass and the harmonic notch bank with zeroed state. A host whose frame
//! period alternates across that band (a 400 Hz loop on a 1 ms OS tick: 2, 3,
//! 2, 3 ms; 1 kHz with +-25 us of jitter; 250 Hz at 4.0/4.1 ms) zeroed both
//! on every tick and lost rate control: supervised, Takeoff -> Land ->
//! Terminated (peak tilt 1.95 rad at 400 Hz 2/3 ms on 725f146).
//!
//! THE FIX (maintainer decision: split the consumers). The FDI / warm-up /
//! guard / stall tick counters keep the per-frame resync exactly as before.
//! Only the FILTER design rate moves to a robust estimate: the median of the
//! last 16 valid frame periods (seeded by the first valid frame), re-designed
//! only when that leaves the 2% band of the filters' current design rate,
//! and re-designed by a coefficient update that KEEPS the filter state.
//!
//! What this checks: (a) the supervised takeoff at 400 Hz 2/3 ms; (b) four
//! jitter patterns hovering with at most 3 filter re-designs; (c) early long
//! frames still isolate no healthy rotor (the FDI path is unchanged). The
//! genuine rate step is a lib test
//! (`a_genuine_rate_step_redesigns_the_filters_within_the_window`), because
//! it changes the plant's own period; the kept state is relay-adrc's
//! `a_retuned_biquad_keeps_its_state` and relay-notch's
//! `set_sample_rate_keeps_state_and_engagement`.
//!
//! SimBackend only: the plant integrates at its own fixed period while the
//! REPORTED frame period jitters, as on a host whose timestamps jitter
//! around a steady physical loop.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{
    FlightBackend, FlightCore, FlightSupervisor, ImuSample, Pathology,
    SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ, SimBackend,
};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The rate the cascade's gyro filters are designed for.
fn filter_hz(c: &FlightCore) -> f32 {
    c.filter_design_hz()
}

/// SimBackend whose `dt()` walks a repeating pattern, one entry per tick (the
/// tick advances on `write_motors`, which every control step calls once).
/// Optionally a late frame `(tick, dt)` list, ticks counted from `armed_at`
/// (the first flying tick), for the FDI cases.
struct Jitter {
    b: SimBackend,
    pat: Vec<f32>,
    i: usize,
    /// Control ticks since spin-up; `None` on the ground.
    k: Option<u32>,
    spikes: Vec<(u32, f32)>,
    /// Rotor reported at 0 RPM on spin-up ticks `0..until` while it spins.
    zero_rpm: Option<(usize, u32)>,
    /// Soft-start: `rotor`'s command scaled by min(1, t / ramp_s) from spin-up.
    ramp: Option<(usize, f32)>,
}

impl Jitter {
    fn new(b: SimBackend, pat: Vec<f32>) -> Self {
        Jitter {
            b,
            pat,
            i: 0,
            k: None,
            spikes: vec![],
            zero_rpm: None,
            ramp: None,
        }
    }
}

impl FlightBackend for Jitter {
    fn read_imu(&mut self) -> ImuSample {
        self.b.read_imu()
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.b.read_position()
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.b.read_mag()
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.b.read_heading()
    }
    fn write_motors(&mut self, m: &[f32]) {
        let mut m4 = [0.0f32; 4];
        m4.copy_from_slice(&m[..4]);
        if let (Some(k), Some((r, ramp_s))) = (self.k, self.ramp) {
            m4[r] *= (k as f32 * self.b.dt() / ramp_s).min(1.0);
        }
        self.b.write_motors(&m4);
        self.i += 1;
    }
    fn dt(&self) -> f32 {
        if let Some(k) = self.k {
            for &(kk, d) in &self.spikes {
                if kk == k {
                    return d;
                }
            }
        }
        self.pat[self.i % self.pat.len()]
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.b.read_battery_v()
    }
    fn read_battery_i(&mut self) -> Option<f32> {
        self.b.read_battery_i()
    }
    fn read_range(&mut self) -> Option<f32> {
        self.b.read_range()
    }
    fn read_baro(&mut self) -> Option<f32> {
        self.b.read_baro()
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        let mut r = self.b.read_motor_rpm()?;
        if let (Some(k), Some((rotor, until))) = (self.k, self.zero_rpm) {
            if k < until {
                r[rotor] = 0;
            }
        }
        Some(r)
    }
}

fn noisy() -> Pathology {
    Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        gps_noise: 0.3,
        turbulence: 1.0,
        ..Pathology::default()
    }
}

fn mean(pat: &[f32]) -> f32 {
    pat.iter().sum::<f32>() / pat.len() as f32
}

/// A deterministic pattern: `base` with a fraction `p_permille` of frames
/// `late` (an LCG, so the case is the same on every run).
fn late_frames(n: usize, base: f32, late: f32, p_permille: u64, seed: u64) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            if ((s >> 33) % 1000) < p_permille {
                late
            } else {
                base
            }
        })
        .collect()
}

#[derive(Debug)]
struct Hover {
    /// Filter re-designs counted by watching the design rate change.
    redesigns: u32,
    filter_hz: f32,
    rp_rate_rms: f32,
    peak_h: f32,
}

/// A FlightCore built at the supervisor's nominal 1 kHz, hovering 2 m up with
/// a 1 m position offset to correct, `secs` of frames of `pat` on a noisy
/// plant stepping at the pattern's mean period.
fn hover(pat: Vec<f32>, secs: f32) -> (Hover, u32) {
    let pdt = mean(&pat);
    let mut b = Jitter::new(SimBackend::new(LEVEL, pdt).with_pathology(noisy()), pat);
    let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    b.b.pos = [1.0, 0.0, -2.0];
    let n = (secs / pdt) as usize;
    let (mut acc, mut cnt, mut peak_h) = (0.0f64, 0usize, 0.0f32);
    let (mut redesigns, mut last) = (0u32, filter_hz(&core));
    for k in 0..n {
        core.step(&mut b);
        if filter_hz(&core) != last {
            redesigns += 1;
            last = filter_hz(&core);
        }
        if !b.b.pos[2].is_finite() {
            acc = f64::INFINITY;
            break;
        }
        if k > n / 3 {
            let w = b.b.omega;
            acc += (w[0] * w[0] + w[1] * w[1]) as f64;
            cnt += 1;
            peak_h = peak_h.max((b.b.pos[0].powi(2) + b.b.pos[1].powi(2)).sqrt());
        }
    }
    (
        Hover {
            redesigns,
            filter_hz: filter_hz(&core),
            rp_rate_rms: (acc / cnt.max(1) as f64).sqrt() as f32,
            peak_h,
        },
        core.filter_redesigns(),
    )
}

/// (a) **The supervised takeoff at 400 Hz on a 1 ms OS tick (2, 3, 2, 3 ms)
/// reaches Loiter level.** On 725f146: Takeoff -> Land -> Terminated, peak
/// tilt 1.95 rad, 12000 filter rebuilds in 30 s.
#[test]
fn a_400_hz_takeoff_on_a_1_ms_tick_reaches_loiter_level() {
    let pat = vec![0.002f32, 0.003];
    let pdt = mean(&pat);
    let mut b = Jitter::new(SimBackend::new(LEVEL, pdt).with_pathology(noisy()), pat);
    b.b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    let s = |x: f32| (x / pdt + 0.5) as usize;
    for _ in 0..s(5.0) {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{:?}", sup.arm_blocked_check());
    let mut modes = vec![sup.mode()];
    let mut peak_tilt = 0.0f32;
    let (mut redesigns, mut last) = (0u32, filter_hz(sup.core()));
    for _ in 0..s(25.0) {
        sup.step(&mut b);
        peak_tilt = peak_tilt.max(b.b.tilt());
        if filter_hz(sup.core()) != last {
            redesigns += 1;
            last = filter_hz(sup.core());
        }
        if modes.last() != Some(&sup.mode()) {
            modes.push(sup.mode());
        }
    }
    eprintln!(
        "400 Hz 2/3 ms supervised: modes {modes:?}, peak tilt {peak_tilt:.3} rad, \
         {redesigns} filter re-designs, filters at {:.1} Hz, FDI counted at {:.1} Hz",
        filter_hz(sup.core()),
        sup.core().designed_loop_hz()
    );
    assert!(
        modes.contains(&Mode::Loiter) && !modes.contains(&Mode::Land),
        "the takeoff must complete into Loiter without a failsafe landing: {modes:?}"
    );
    assert!(
        peak_tilt < 0.1,
        "peak tilt {peak_tilt} rad (fixed 2.5 ms flies at ~0.02)"
    );
    assert!(redesigns <= 3, "{redesigns} filter re-designs");
}

/// (b) **Jitter hovers on at most 3 filter re-designs.** Four patterns, 12 s
/// each, the fixed-period hover as the yardstick (rate rms 0.0075 rad/s,
/// peak horizontal 0.9 m at 1 kHz). On 725f146: 1 kHz +-25 us 12000
/// rebuilds, 3.4 rad/s, 40 m off; 250 Hz 4.0/4.1 ms diverged to non-finite;
/// period-7 (2, 1 x6 ms) non-finite; 10 % of frames 2 ms late 1950 rebuilds.
#[test]
fn jitter_hovers_on_at_most_three_filter_redesigns() {
    let cases: Vec<(&str, Vec<f32>)> = vec![
        ("1 kHz +-25 us", vec![0.000975, 0.001025]),
        ("250 Hz 4.0/4.1 ms", vec![0.004, 0.0041]),
        (
            "1 kHz period-7: 2, 1 x6 ms",
            vec![0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        ),
        (
            "1 kHz, 10% of frames 2 ms late",
            late_frames(20000, 0.001, 0.002, 100, 2),
        ),
    ];
    let mut bad = Vec::new();
    for (name, pat) in cases {
        let (h, counted) = hover(pat, 12.0);
        eprintln!("{name}: {h:?}");
        assert_eq!(
            counted, h.redesigns,
            "{name}: filter_redesigns() must count every observed re-design"
        );
        // NaN-safe: a non-finite hover fails `bounded`.
        let bounded = h.rp_rate_rms < 0.03 && h.peak_h < 2.5 && h.filter_hz.is_finite();
        if h.redesigns > 3 || !bounded {
            bad.push((name, h));
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// The FDI cases' rig: a supervised spin-up from the ground at `hz`, RPM
/// telemetry on, with long frames reported on spin-up ticks `spikes`.
fn fly(
    hz: u32,
    spikes: Vec<(u32, f32)>,
    zero_rpm: Option<(usize, u32)>,
    ramp: Option<(usize, f32)>,
    tau_s: f32,
) -> (Option<(u32, usize)>, bool, f32) {
    let dt = 1.0 / hz as f32;
    let mut sim = SimBackend::new(LEVEL, dt);
    sim.ground_contact = true;
    sim.rot_drag = 0.02;
    sim.rpm_telemetry = true;
    sim.motor_tau = tau_s;
    let mut b = Jitter::new(sim, vec![dt]);
    b.spikes = spikes;
    b.zero_rpm = zero_rpm;
    b.ramp = ramp;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    let s = |x: f32| (x * hz as f32 + 0.5) as u32;
    for _ in 0..s(5.0) {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{:?}", sup.arm_blocked_check());
    let (mut iso, mut loiter, mut peak_tilt) = (None, false, 0.0f32);
    let mut stop = s(40.0);
    for k in 0..s(40.0) {
        b.k = Some(k);
        sup.step(&mut b);
        peak_tilt = peak_tilt.max(b.b.tilt());
        if iso.is_none() {
            if let Some(r) = sup.core().failed_motor() {
                iso = Some((k + 1, r));
            }
        }
        if !loiter && sup.mode() == Mode::Loiter {
            loiter = true;
            stop = stop.min(k + s(1.0));
        }
        if k >= stop {
            break;
        }
    }
    (iso, loiter, peak_tilt)
}

/// (c) **Early long frames still isolate no healthy rotor.** The FDI path is
/// unchanged by the fix (its counters resync per frame exactly as before);
/// this pins that the filter change did not move it. Healthy rotor 2 reads
/// 0 RPM through ~80 ms of spin-up (zero RPM is what the stall check looks
/// for), or soft-starts over 100 ms behind a 50 ms lag, with ONE 50 ms frame
/// at flying tick 1 or TWO 20 ms frames at ticks 3 and 5. Every cell flies
/// to Loiter with no isolation on 725f146 too.
#[test]
fn early_long_frames_isolate_no_healthy_rotor() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        let per = 1.0 / hz as f32;
        for spikes in [vec![(1u32, 0.05f32)], vec![(3, 0.02), (5, 0.02)]] {
            let longest = spikes.iter().map(|s| s.1).fold(0.0f32, f32::max);
            let zero = ((0.08 - longest) / per) as u32 + 1;
            for (zero_rpm, ramp, tau) in [
                (Some((2usize, zero)), None, 0.0f32),
                (None, Some((2usize, 0.1f32)), 0.05),
            ] {
                let r = fly(hz, spikes.clone(), zero_rpm, ramp, tau);
                eprintln!("{hz} Hz spikes {spikes:?} zero-RPM {zero_rpm:?} ramp {ramp:?}: {r:?}");
                if r.0.is_some() || !r.1 {
                    bad.push((hz, spikes.clone(), zero_rpm, ramp, r));
                }
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
