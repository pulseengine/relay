#![allow(dead_code, unused)]
//! Reconstructed copy of scratchpad/probe_review.rs as it was at 18:26 on
//! 2026-11-05 (P1, P2, P6, P7); the scratchpad file was later overwritten by
//! a reviewer's probe. Text restored verbatim from the session transcript.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, SimBackend};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

const GUARD_S: f32 = 0.3;
const STALL_S: f32 = 0.1;

#[derive(Debug, Clone, Copy, PartialEq)]
enum RpmFeed {
    Always,
    Never,
    EveryOther,
}

struct Asym {
    b: SimBackend,
    rotor: usize,
    ramp_s: f32,
    k: Option<u32>,
    kill_at: Option<(u32, usize)>,
    weak: Option<(usize, f32)>,
    rpm: RpmFeed,
    dt: f32,
    zero_rpm: Option<([u32; 4], [u32; 4])>,
    dt_spike: Vec<(u32, f32)>,
}

impl FlightBackend for Asym {
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
    fn write_motors(&mut self, motors: &[f32]) {
        let mut m = [0.0f32; 4];
        m.copy_from_slice(&motors[..4]);
        if let Some(k) = self.k {
            if let Some((kk, r)) = self.kill_at {
                if k == kk {
                    self.b.fail_rotor(r);
                }
            }
            if self.ramp_s > 0.0 {
                m[self.rotor] *= (k as f32 * self.dt / self.ramp_s).min(1.0);
            }
            if let Some((r, g)) = self.weak {
                m[r] *= g;
            }
        }
        self.b.write_motors(&m);
    }
    fn dt(&self) -> f32 {
        if let Some(k) = self.k {
            for &(kk, d) in &self.dt_spike {
                if kk == k {
                    return d;
                }
            }
        }
        self.b.dt()
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
        let rpm = self.read_motor_rpm_feed();
        match (rpm, self.k, self.zero_rpm) {
            (Some(mut r), Some(k), Some((start, until))) => {
                for i in 0..4 {
                    if k >= start[i] && k < until[i] {
                        r[i] = 0;
                    }
                }
                Some(r)
            }
            _ => rpm,
        }
    }
}

impl Asym {
    fn read_motor_rpm_feed(&mut self) -> Option<[i32; 4]> {
        match self.rpm {
            RpmFeed::Always => self.b.read_motor_rpm(),
            RpmFeed::Never => None,
            RpmFeed::EveryOther => match self.k {
                Some(k) if k % 2 == 1 => None,
                _ => self.b.read_motor_rpm(),
            },
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct Flight {
    iso: Option<(u32, usize, Mode)>,
    loiter: bool,
    peak_tilt: f32,
    tilt_before_iso: f32,
}

#[allow(clippy::too_many_arguments)]
fn fly_full(
    hz: u32,
    ramp_ms: u32,
    tau_ms: u32,
    rotor: usize,
    dead: Option<usize>,
    weak: Option<(usize, f32)>,
    rpm: RpmFeed,
    after_loiter_s: f32,
    zero_rpm: Option<([u32; 4], [u32; 4])>,
    dt_spike: Vec<(u32, f32)>,
) -> (Flight, FlightSupervisor, Asym) {
    let faulted = dead.is_some() || weak.is_some();
    let dt = 1.0 / hz as f32;
    let mut b = SimBackend::new(LEVEL, dt);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    b.rpm_telemetry = true;
    b.motor_tau = tau_ms as f32 / 1000.0;
    let mut a = Asym {
        b,
        rotor,
        ramp_s: ramp_ms as f32 / 1000.0,
        k: None,
        kill_at: dead.map(|r| (0, r)),
        weak,
        rpm,
        dt,
        zero_rpm,
        dt_spike,
    };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    let s = |x: f32| (x * hz as f32 + 0.5) as u32;
    for _ in 0..s(5.0) {
        sup.step(&mut a);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{:?}", sup.arm_blocked_check());
    let mut f = Flight::default();
    let mut stop = s(40.0);
    for k in 0..s(40.0) {
        a.k = Some(k);
        sup.step(&mut a);
        let tilt = a.b.tilt();
        f.peak_tilt = f.peak_tilt.max(tilt);
        if f.iso.is_none() {
            if faulted {
                f.tilt_before_iso = f.tilt_before_iso.max(tilt);
            }
            if let Some(r) = sup.core().failed_motor() {
                f.iso = Some((k + 1, r, sup.mode()));
                if faulted {
                    stop = stop.min(k + s(0.5));
                }
            }
        }
        if !f.loiter && sup.mode() == Mode::Loiter {
            f.loiter = true;
            stop = stop.min(k + s(after_loiter_s));
        }
        if k >= stop {
            break;
        }
    }
    (f, sup, a)
}

use falcon_core::{FlightCore, SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ};

fn airborne(kill: u32, spikes: &[(u32, f32)]) -> (Option<(u32, usize)>, f32, f32) {
    let dt = 0.001;
    let mut b = SimBackend::new(LEVEL, dt);
    b.rpm_telemetry = true;
    b.pos = [0.0, 0.0, -2.0];
    let mut a = Asym {
        b,
        rotor: 0,
        ramp_s: 0.0,
        k: None,
        kill_at: Some((kill, 1)),
        weak: None,
        rpm: RpmFeed::Always,
        dt,
        zero_rpm: None,
        dt_spike: spikes.to_vec(),
    };
    let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
    core.set_position([0.0, 0.0, -2.0]);
    let mut elapsed = 0.0f32;
    for k in 0..2000u32 {
        a.k = Some(k);
        let d = a.dt();
        core.step(&mut a);
        if k > kill {
            elapsed += d;
        }
        if let Some(r) = core.failed_motor() {
            return (Some((k, r)), elapsed * 1000.0, core.designed_loop_hz());
        }
    }
    (None, elapsed * 1000.0, core.designed_loop_hz())
}

#[test]
fn p1_spikes_before_kill_shorten_stall_window() {
    for (kill, spikes) in [
        (36u32, vec![]),
        (36, vec![(33u32, 0.05f32)]),
        (36, vec![(33, 0.05), (35, 0.05)]),
        (36, vec![(33, 0.01), (35, 0.01)]),
        (36, vec![(33, 0.005), (35, 0.005)]),
        (36, vec![(33, 0.003), (35, 0.003)]),
        (40, vec![(33, 0.05), (35, 0.05)]),
        (44, vec![(33, 0.02), (35, 0.02), (38, 0.02)]),
        (60, vec![(33, 0.05), (35, 0.05)]),
    ] {
        let (iso, ms, hz) = airborne(kill, &spikes);
        eprintln!("P1 kill {kill} spikes {spikes:?}: iso {iso:?} after {ms:.1} ms reported, designed {hz:.1}");
    }
}

#[test]
fn p2_spinup_spikes_isolate_healthy_rotor() {
    for hz in [1000u32, 250] {
        for (zero, spikes) in [
            (60u32, vec![]),
            (60, vec![(20u32, 0.05f32), (22, 0.05)]),
            (60, vec![(20, 0.01), (22, 0.01)]),
            (40, vec![(3, 0.05), (5, 0.05)]),
            (40, vec![(3, 0.02), (5, 0.02)]),
            (20, vec![(3, 0.05), (5, 0.05)]),
            (30, vec![(8, 0.05), (10, 0.05)]),
        ] {
            let zero = if hz == 250 { zero / 4 } else { zero };
            let (f, _sup, _) = fly_full(
                hz,
                0,
                0,
                0,
                None,
                None,
                RpmFeed::Always,
                1.0,
                Some(([u32::MAX, u32::MAX, 0, u32::MAX], [zero; 4])),
                spikes.clone(),
            );
            eprintln!("P2 {hz} Hz rotor2 0 RPM {zero} ticks, spikes {spikes:?}: {f:?}");
        }
    }
}

#[test]
fn p6_two_long_first_frames() {
    let mut n_bad = 0;
    for spikes_k in [vec![0u32], vec![1], vec![0, 1], vec![0, 2], vec![1, 2], vec![0, 1, 2], vec![3, 5], vec![4, 6]] {
        for hz in [250u32, 1000] {
            for d in [0.02f32, 0.05] {
                for (ramp, tau, rotor) in [(100u32, 50u32, 2usize), (200, 50, 1), (150, 0, 3)] {
                    let spikes: Vec<(u32, f32)> = spikes_k.iter().map(|&k| (k, d)).collect();
                    let (f, _, _) = fly_full(hz, ramp, tau, rotor, None, None, RpmFeed::Always, 1.0, None, spikes);
                    if f.iso.is_some() || !f.loiter {
                        n_bad += 1;
                        eprintln!("P6 BAD long frames at {spikes_k:?} {hz} Hz {:.0} ms rotor {rotor} {ramp} ms lag {tau}: {f:?}", d * 1000.0);
                    }
                }
            }
        }
        eprintln!("P6 done {spikes_k:?}");
    }
    eprintln!("P6 total bad {n_bad}");
}

#[test]
fn p7_one_late_frame_in_the_two_block() {
    for hz in [1000u32, 250] {
        for spike_k in [1u32, 2, 0, 5] {
            for d in [0.02f32, 0.05] {
                let per = 1.0 / hz as f32;
                let zero = ((0.08 - d) / per) as u32 + 1;
                let (f, _, _) = fly_full(hz, 0, 0, 0, None, None, RpmFeed::Always, 1.0,
                    Some(([u32::MAX, u32::MAX, 0, u32::MAX], [zero; 4])), vec![(spike_k, d)]);
                let summed = (zero - 1) as f32 * per + d;
                eprintln!("P7 {hz} Hz one {:.0} ms frame at tick {spike_k}, rotor2 0 RPM ticks [0,{zero}) = {:.0} ms summed: {f:?}", d * 1e3, summed * 1e3);
            }
        }
    }
}
