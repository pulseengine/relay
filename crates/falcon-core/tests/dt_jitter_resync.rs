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

/// (1) THE REGRESSION TEST. The SUPERVISED takeoff, 400 Hz on a 1 ms OS
/// tick (frames 2, 3, 2, 3 ms), noisy sensors: 5 s on the ground, arm,
/// take off, and it must reach Loiter with the vehicle never tilting past
/// 0.1 rad, the same as at a fixed 2.5 ms.
///
/// FAILS ON 725f146: Takeoff -> Land -> Terminated, peak tilt 1.95 rad.
#[test]
fn a_400_hz_loop_on_a_1_ms_tick_takes_off_to_loiter() {
    let mut bad = Vec::new();
    for pat in [vec![0.0025f32], vec![0.002, 0.003], vec![0.003, 0.002]] {
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
        eprintln!("{pat:?}: modes {modes:?} peak tilt {peak_tilt:.3} rad, {resyncs} resyncs");
        if !modes.contains(&Mode::Loiter) || peak_tilt >= 0.1 {
            bad.push((pat, modes, peak_tilt));
        }
    }
    assert!(bad.is_empty(), "jittered frames lost the takeoff: {bad:?}");
}

/// (2) FlightCore hover, 1 kHz +-25 us and 250 Hz 4.0/4.1 ms: the
/// roll/pitch rate stays as quiet as at the fixed period, and the position
/// error stays bounded.
///
/// FAILS ON 725f146: see the eprintln (rate rms / peak far above fixed).
#[test]
fn jittered_frames_keep_the_hover_bounded() {
    let mut bad = Vec::new();
    for (fixed, jit) in [
        (vec![0.001f32], vec![0.000975f32, 0.001025]),
        (vec![0.004], vec![0.004, 0.0041]),
    ] {
        let f = hover(fixed.clone(), 30.0);
        let j = hover(jit.clone(), 30.0);
        eprintln!("fixed {fixed:?}: {f:?}\njitter {jit:?}: {j:?}");
        if !(j.peak_rate < 0.5
            && j.peak_h < f.peak_h + 0.5
            && j.rp_rate_rms < 2.0 * f.rp_rate_rms + 0.02)
        {
            bad.push((jit, j));
        }
    }
    assert!(bad.is_empty(), "jitter destabilised the hover: {bad:?}");
}

/// (3) An alternating or quantised frame period rebuilds the rate filters at
/// most TWICE over 12 s (the first frame, then the block mean), and the
/// rate it settles on is the MEAN frame rate within the 2% band.
///
/// FAILS ON 725f146: a rebuild on every tick (thousands).
#[test]
fn an_alternating_frame_period_resyncs_at_most_twice() {
    let mut bad = Vec::new();
    for (name, pat) in patterns() {
        let want = 1.0 / mean(&pat);
        let h = hover(pat, 12.0);
        eprintln!(
            "{name}: {} resyncs, designed {:.1} Hz (mean {want:.1})",
            h.resyncs, h.designed_hz
        );
        if h.resyncs > 2 || (h.designed_hz - want).abs() > 0.02 * want {
            bad.push((name, h.resyncs, h.designed_hz, want));
        }
    }
    assert!(
        bad.is_empty(),
        "per-frame jitter resynced the loop rate: {bad:?}"
    );
}

/// (5) DEFECT B. A rotor dead from the first tick of an AIRBORNE start, at the
/// supervisor's nominal 1 kHz, with the frame period alternating 1/2 ms (both
/// phases) and 400 Hz on a 1 ms tick: the 0.1 s stall window must fill at
/// 100 ms +-5 ms of ELAPSED time (the summed frame periods of the stalled
/// ticks; the run starts on tick 2, the first tick has no previous command).
///
/// FAILS ON 725f146: ~75-78 ms.
#[test]
fn the_stall_window_is_0_1_s_of_time_under_alternating_frames() {
    let mut bad = Vec::new();
    for pat in [
        vec![0.001f32, 0.002],
        vec![0.002, 0.001],
        vec![0.002, 0.003],
        vec![0.003, 0.002],
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
        eprintln!("{pat:?}: isolated {iso:?} after {ms:.1} ms of stall");
        if iso.map(|i| i.1) != Some(Some(1)) || !(95.0..=105.0).contains(&ms) {
            bad.push((pat, iso, ms));
        }
    }
    assert!(
        bad.is_empty(),
        "the stall window is not 0.1 s of time: {bad:?}"
    );
}
