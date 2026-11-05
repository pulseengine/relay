#![allow(dead_code, unused)]
//! REVIEW OF v1.140, DEFECTS A and B: per-frame `dt` JITTER must not
//! resync the loop rate.
//!
//! THE DEFECT (A). The rate resync compared EACH frame's `1/dt` with the
//! last rebuilt rate, inside a 2% band and with no averaging. A host whose
//! frame period ALTERNATES across that band (a 400 Hz loop on a 1 ms OS
//! tick: 2, 3, 2, 3 ms; a 1 kHz loop with +-25 us of jitter; 250 Hz at
//! 4.0/4.1 ms) rebuilt the gyro low-pass and the notch bank, with their
//! state ZEROED, on every tick, and the supervised takeoff lost rate control
//! (Takeoff -> Land -> Terminated, peak tilt 1.95 rad at 400 Hz 2/3 ms).
//!
//! THE DEFECT (B). Under the same per-frame resync every tick-count against a
//! fixed-TIME threshold was rescaled on every frame, so the 0.1 s stall
//! window filled at ~75-78 ms of elapsed time under 1/2 ms alternation at
//! 1 kHz.
//!
//! THE FIX. The first frame still establishes the rate (as before); from
//! then on the resync compares the MEAN PERIOD of a block of frames
//! (blocks of 2, 4, 8, then `RATE_WINDOW_FRAMES` = 16 frames; from 4 frames
//! on the longest and the shortest are dropped)
//! with the 2% band, so an alternation converges to its mean rate and
//! stays there, a genuine rate change still resyncs within a block or two,
//! and the ticks counted inside the block are re-expressed at the block's
//! mean period (they WERE ticks of that period).
//!
//! SimBackend only.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{
    FlightBackend, FlightCore, FlightSupervisor, ImuSample, Pathology,
    SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ, SimBackend,
};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// SimBackend whose `dt()` walks a repeating pattern, one entry per tick
/// (the tick advances on `write_motors`, which every step calls once). The
/// plant itself integrates at its own fixed period: what is under test is
/// the CORE's reaction to the reported period, as on a host whose frame
/// timestamps jitter around a steady physical loop.
struct Jitter {
    b: SimBackend,
    pat: Vec<f32>,
    i: usize,
}

impl Jitter {
    fn dt_now(&self) -> f32 {
        self.pat[self.i % self.pat.len()]
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
        self.b.write_motors(m);
        self.i += 1;
    }
    fn dt(&self) -> f32 {
        self.dt_now()
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
        self.b.read_motor_rpm()
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

/// The patterns of the review. (name, frame periods, the plant's period)
fn patterns() -> Vec<(&'static str, Vec<f32>)> {
    vec![
        ("400 Hz on a 1 ms tick: 2/3 ms", vec![0.002, 0.003]),
        ("1 kHz +-25 us", vec![0.000975, 0.001025]),
        ("250 Hz 4.0/4.1 ms", vec![0.004, 0.0041]),
        ("1 kHz late every other: 1/2 ms", vec![0.001, 0.002]),
        (
            "250 Hz ms-quantized 1-in-10",
            vec![
                0.004, 0.004, 0.004, 0.004, 0.005, 0.003, 0.004, 0.004, 0.004, 0.004,
            ],
        ),
    ]
}

#[derive(Debug)]
struct Hover {
    resyncs: u32,
    designed_hz: f32,
    rp_rate_rms: f32,
    peak_rate: f32,
    peak_h: f32,
}

/// A FlightCore built at the supervisor's nominal 1 kHz, hovering 2 m up
/// with a 1 m position offset to correct, `secs` of frames of `pat`.
fn hover(pat: Vec<f32>, secs: f32) -> Hover {
    let pdt = mean(&pat);
    let mut b = Jitter {
        b: SimBackend::new(LEVEL, pdt).with_pathology(noisy()),
        pat,
        i: 0,
    };
    let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    b.b.pos = [1.0, 0.0, -2.0];
    let n = (secs / pdt) as usize;
    let (mut acc, mut cnt, mut peak_rate, mut peak_h) = (0.0f64, 0usize, 0.0f32, 0.0f32);
    for k in 0..n {
        core.step(&mut b);
        let w = b.b.omega;
        if k > n / 3 {
            let r2 = w[0] * w[0] + w[1] * w[1];
            acc += r2 as f64;
            cnt += 1;
            peak_rate = peak_rate.max(r2.sqrt());
            peak_h = peak_h.max((b.b.pos[0].powi(2) + b.b.pos[1].powi(2)).sqrt());
        }
        if !b.b.pos[2].is_finite() {
            peak_rate = f32::INFINITY;
            break;
        }
    }
    Hover {
        resyncs: core.rate_resyncs(),
        designed_hz: core.designed_loop_hz(),
        rp_rate_rms: (acc / cnt.max(1) as f64).sqrt() as f32,
        peak_rate,
        peak_h,
    }
}


fn lcg_pattern(n: usize, base: f32, late: f32, p_permille: u32, seed: u64) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            if ((s >> 33) % 1000) < p_permille as u64 { late } else { base }
        })
        .collect()
}

#[test]
fn p3_patterns_that_flip_blocks() {
    let mut cases: Vec<(String, Vec<f32>)> = vec![
        ("fixed 1 ms".into(), vec![0.001]),
        ("fixed 3.333 ms".into(), vec![0.0033333]),
        ("300 Hz on 1 ms tick 3,3,4".into(), vec![0.003, 0.003, 0.004]),
        ("875 Hz on 1 ms tick 2,1x6".into(), vec![0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
        ("1 kHz period-5 late 2,1x4".into(), vec![0.002, 0.001, 0.001, 0.001, 0.001]),
        ("1 kHz period-9 late".into(), vec![0.002, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
    ];
    for (p, seed) in [(50u32, 1u64), (100, 2), (200, 3)] {
        cases.push((format!("1 kHz random 2 ms late {p} permille"), lcg_pattern(20000, 0.001, 0.002, p, seed)));
    }
    cases.push(("250 Hz random 8 ms late 100 permille".into(), lcg_pattern(5000, 0.004, 0.008, 100, 4)));
    for (name, pat) in cases {
        let h = hover(pat, 12.0);
        eprintln!("P3 {name}: {h:?}");
    }
}

#[test]
fn p4_takeoff() {
    let mut bad = Vec::new();
    for pat in [
        vec![0.002f32, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        vec![0.002, 0.001, 0.001, 0.001, 0.001],
        lcg_pattern(20000, 0.001, 0.002, 50, 1),
        lcg_pattern(20000, 0.001, 0.002, 100, 2),
        vec![0.003, 0.003, 0.004],
        lcg_pattern(20000, 0.0025, 0.0035, 100, 7),
    ] {
        let pdt = mean(&pat);
        let mut b = Jitter {
            b: SimBackend::new(LEVEL, pdt).with_pathology(noisy()),
            pat: pat.clone(),
            i: 0,
        };
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
        assert_eq!(
            sup.mode(),
            Mode::Takeoff,
            "{pat:?}: {:?}",
            sup.arm_blocked_check()
        );
        let mut modes = vec![sup.mode()];
        let mut peak_tilt = 0.0f32;
        for _ in 0..s(25.0) {
            sup.step(&mut b);
            peak_tilt = peak_tilt.max(b.b.tilt());
            if modes.last() != Some(&sup.mode()) {
                modes.push(sup.mode());
            }
        }
        let resyncs = sup.core().rate_resyncs();
        eprintln!("P4 {:?}", &pat[..pat.len().min(12)]); eprintln!("P4 : modes {modes:?} peak tilt {peak_tilt:.3} rad, {resyncs} resyncs");
        if !modes.contains(&Mode::Loiter) || peak_tilt >= 0.1 {
            bad.push((pat, modes, peak_tilt));
        }
    }
    eprintln!("P4 bad count {}", bad.len());
}

#[test]
fn p5_stall() {
    let mut bad = Vec::new();
    for pat in [
        vec![0.002f32, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        vec![0.002, 0.001, 0.001, 0.001, 0.001],
        lcg_pattern(20000, 0.001, 0.002, 50, 1),
        lcg_pattern(20000, 0.001, 0.002, 100, 2),
        vec![0.003, 0.003, 0.004],
        lcg_pattern(20000, 0.0025, 0.0035, 100, 7),
    ] {
        let pdt = mean(&pat);
        let mut b = Jitter {
            b: SimBackend::new(LEVEL, pdt),
            pat: pat.clone(),
            i: 0,
        };
        b.b.rpm_telemetry = true;
        b.b.pos = [0.0, 0.0, -2.0];
        b.b.fail_rotor(1);
        let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
        core.set_position([0.0, 0.0, -2.0]);
        let mut elapsed = 0.0f32;
        let mut iso = None;
        for tick in 1..=2000u32 {
            let dt = b.dt_now();
            core.step(&mut b);
            if tick >= 2 {
                elapsed += dt;
            }
            if core.failed_motor().is_some() {
                iso = Some((tick, core.failed_motor()));
                break;
            }
        }
        let ms = elapsed * 1000.0;
        eprintln!("P5 {:?}", &pat[..pat.len().min(8)]); eprintln!("P5 : isolated {iso:?} after {ms:.1} ms of stall");
        if iso.map(|i| i.1) != Some(Some(1)) || !(95.0..=105.0).contains(&ms) {
            bad.push((pat, iso, ms));
        }
    }
    }
