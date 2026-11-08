//! KNOWN LIMIT A (#638): a host whose frame period keeps crossing the 2% resync
//! band. On main through falcon-v1.141.0 every crossing rebuilt the gyro
//! low-pass and the notch bank with zeroed state, so a host alternating 2 ms /
//! 3 ms at 400 Hz, or +-25 us at 1 kHz, flew with its gyro effectively
//! unfiltered and tumbled. The alternation guard holds the filter design at
//! the window median while EVERY one of the last 16 frames crossed the band,
//! and lets go on the first frame that does not.
//!
//! The other half of the contract is what must NOT change: a minority of late
//! frames keeps main's per-frame redesign, which matches the notch to rotor
//! vibration sampled at that frame's real interval. Holding the design there
//! instead isolated a healthy rotor in 23-24 of 24 seeds (kill test,
//! 2026-11-08). These tests pin that the guard never engages for such hosts.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightCore, FlightSupervisor, ImuSample, Pathology, SimBackend};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// A `SimBackend` whose reported `dt` follows `pat`, one entry per frame.
struct Pattern {
    inner: SimBackend,
    pat: Vec<f32>,
    i: usize,
}
impl FlightBackend for Pattern {
    fn read_imu(&mut self) -> ImuSample {
        self.inner.read_imu()
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.inner.read_position()
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.inner.read_mag()
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.inner.read_heading()
    }
    fn write_motors(&mut self, m: &[f32]) {
        self.inner.write_motors(m);
        self.i += 1;
    }
    fn dt(&self) -> f32 {
        self.pat[self.i % self.pat.len()]
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.inner.read_battery_v()
    }
    fn read_range(&mut self) -> Option<f32> {
        self.inner.read_range()
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.inner.read_motor_rpm()
    }
}

fn noisy(nominal_dt: f32) -> SimBackend {
    SimBackend::new(LEVEL, nominal_dt).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        gps_noise: 0.3,
        turbulence: 1.0,
        ..Pathology::default()
    })
}

/// Hover a bare `FlightCore` for `secs`; the roll+pitch body-rate RMS over the
/// last two thirds.
fn hover_rate_rms(pat: Vec<f32>, nominal_dt: f32, noise: bool, secs: f32) -> f64 {
    let mut b = Pattern {
        inner: if noise {
            noisy(nominal_dt)
        } else {
            SimBackend::new(LEVEL, nominal_dt)
        },
        pat,
        i: 0,
    };
    let mut core = FlightCore::new(0.5, 1.0 / nominal_dt);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    b.inner.pos[0] = 1.0;
    let n = (secs / nominal_dt) as usize;
    let (mut acc, mut cnt) = (0.0f64, 0usize);
    for k in 0..n {
        core.step(&mut b);
        if k > n / 3 {
            let w = b.inner.omega;
            acc += (w[0] * w[0] + w[1] * w[1]) as f64;
            cnt += 1;
        }
    }
    (acc / cnt.max(1) as f64).sqrt()
}

/// The defect's supervised reproduction: on main it went Takeoff -> Land ->
/// Terminated with a 1.95 rad peak tilt.
#[test]
fn a_400hz_host_alternating_2_and_3_ms_takes_off_and_loiters() {
    let mut b = Pattern {
        inner: noisy(0.0025),
        pat: vec![0.002, 0.003],
        i: 0,
    };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mut peak_tilt = 0.0f32;
    let mut modes = Vec::new();
    for _ in 0..(20.0 / 0.0025) as usize {
        sup.step(&mut b);
        peak_tilt = peak_tilt.max(b.inner.r[2][2].clamp(-1.0, 1.0).acos());
        let m = sup.mode();
        if modes.last() != Some(&m) {
            modes.push(m);
        }
    }
    assert_eq!(
        modes,
        vec![Mode::Takeoff, Mode::Loiter],
        "peak tilt {peak_tilt}"
    );
    assert!(peak_tilt < 0.2, "peak tilt {peak_tilt} rad");
    assert!(
        sup.core().alternation_guard(),
        "the guard holds while the host alternates"
    );
    assert!(
        (sup.core().filter_design_hz() - 400.0).abs() < 1.0,
        "filters designed at the median period: {} Hz",
        sup.core().filter_design_hz()
    );
}

/// +-25 us at 1 kHz: main measured rate RMS 6.155 rad/s (clean) and 3.434
/// (noisy); a fixed 1 ms period measures 0.0003 and 0.0073.
#[test]
fn a_1khz_host_alternating_plus_minus_25_us_hovers_like_a_fixed_period() {
    let alt = vec![0.000975, 0.001025];
    let clean = hover_rate_rms(alt.clone(), 0.001, false, 12.0);
    let noise = hover_rate_rms(alt, 0.001, true, 12.0);
    assert!(clean < 0.01, "clean rate rms {clean}");
    assert!(noise < 0.05, "noisy rate rms {noise}");
}

/// 4.0 / 4.1 ms (2.5%, just past the band) at 250 Hz: on main the noisy run
/// isolated a rotor and went non-finite.
#[test]
fn a_250hz_host_alternating_just_past_the_band_hovers() {
    let r = hover_rate_rms(vec![0.004, 0.0041], 0.004, true, 20.0);
    assert!(r < 0.05, "noisy rate rms {r}");
}

fn lcg(s: &mut u64) -> f64 {
    *s = s
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*s >> 11) as f64) / ((1u64 << 53) as f64)
}

/// A FIXED period never engages the guard, so the filters follow main's
/// design rate bit for bit on every frame.
#[test]
fn a_fixed_period_never_engages_the_guard() {
    let dt = 0.004;
    let mut b = Pattern {
        inner: noisy(dt),
        pat: vec![dt],
        i: 0,
    };
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([0.0, 0.0, -2.0]);
    for k in 0..5000 {
        core.step(&mut b);
        assert!(!core.alternation_guard(), "guard engaged at frame {k}");
        assert_eq!(
            core.filter_design_hz().to_bits(),
            core.designed_loop_hz().to_bits(),
            "frame {k}"
        );
    }
}

/// Like [`Pattern`], but the PLANT integrates each frame's own `dt` (a late
/// frame really is a longer step), which is what exposed the late-frame
/// class in #644.
struct OwnDt {
    inner: SimBackend,
    pat: Vec<f32>,
    i: usize,
}
impl OwnDt {
    fn cur(&self) -> f32 {
        self.pat[self.i % self.pat.len()]
    }
}
impl FlightBackend for OwnDt {
    fn read_imu(&mut self) -> ImuSample {
        self.inner.read_imu()
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.inner.read_position()
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.inner.read_mag()
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.inner.read_heading()
    }
    fn write_motors(&mut self, m: &[f32]) {
        self.inner.set_dt(self.cur());
        self.inner.write_motors(m);
        self.i += 1;
    }
    fn dt(&self) -> f32 {
        self.pat[(self.i + self.pat.len() - 1) % self.pat.len()]
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.inner.read_battery_v()
    }
    fn read_range(&mut self) -> Option<f32> {
        self.inner.read_range()
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.inner.read_motor_rpm()
    }
}

/// THE OUTCOME main gets right and the guard must keep. Late frames at
/// 250 Hz under rotor vibration (0.4 rad/s): 1 in 10 at 1.5x, and 4 in 10 at
/// 1.5x. The vehicle loiters for 30 s with a healthy rotor that must NOT be
/// isolated, and the guard must never engage. Freezing the filter design
/// isolated it in 24 of 24 seeds (1 in 10); a looser guard (12 of 16, out at
/// 4) isolated it in 25 of 48 (4 in 10; main 1 of 48).
#[test]
fn late_frames_under_rotor_vibration_never_engage_the_guard_or_isolate_a_healthy_rotor() {
    let hz = 250.0f32;
    let dt = 1.0 / hz;
    for (frac, seed) in [
        (0.1, 0u64),
        (0.1, 1),
        (0.4, 2),
        (0.4, 3),
        (0.4, 4),
        (0.4, 5),
    ] {
        let mut s = seed * 7919 + 7;
        let pat: Vec<f32> = (0..20011)
            .map(|_| if lcg(&mut s) < frac { dt * 1.5 } else { dt })
            .collect();
        let mut sim = SimBackend::new(LEVEL, pat[0]).with_pathology(Pathology {
            gyro_white: 0.02,
            vibration: 0.5,
            ..Pathology::default()
        });
        sim.ground_contact = true;
        sim.rpm_telemetry = true;
        sim.reseed(seed as u32 + 1);
        let mut b = OwnDt {
            inner: sim,
            pat,
            i: 0,
        };
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.set_calibration(falcon_core::relay_calib::CalParams {
            gyro_bias: [0.001, 0.0, 0.0],
            ..falcon_core::relay_calib::CalParams::identity()
        });
        for _ in 0..(5.0 * hz) as usize {
            sup.step(&mut b);
        }
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        let mut loiter_ticks = 0usize;
        for _ in 0..(45.0 * hz) as usize {
            sup.step(&mut b);
            if sup.mode() == Mode::Loiter {
                b.inner.rotor_vib = 0.4;
                loiter_ticks += 1;
            }
            assert!(
                !sup.core().alternation_guard(),
                "late {frac}, seed {seed}: guard engaged"
            );
            assert_eq!(
                sup.core().failed_motor(),
                None,
                "late {frac}, seed {seed}: a healthy rotor was isolated in {:?}",
                sup.mode()
            );
        }
        assert!(
            loiter_ticks as f32 > 30.0 * hz,
            "seed {seed}: loitered {loiter_ticks} ticks"
        );
    }
}
