// Adversarial review probe (uncommitted). Same file runs on parent and PR.
extern crate std;
use std::prelude::v1::*;
use std::{vec, println, format};
use crate::relay_fsm::{Event, Mode};
use crate::{
    FlightBackend, FlightSupervisor, ImuSample, Pathology, SimBackend,
};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

struct Pb {
    b: SimBackend,
    pat: Vec<f32>,
    i: usize,
    reads: usize,
    kick: Option<(usize, usize, Vec3)>,
    /// (read index, value) injected into gyro x
    poison: Option<(usize, f32)>,
}

impl FlightBackend for Pb {
    fn read_imu(&mut self) -> ImuSample {
        let mut s = self.b.read_imu();
        self.reads += 1;
        if let Some((a, z, g)) = self.kick {
            if self.reads >= a && self.reads < z {
                s.gyro = [s.gyro[0] + g[0], s.gyro[1] + g[1], s.gyro[2] + g[2]];
            }
        }
        if let Some((at, v)) = self.poison {
            if self.reads == at {
                s.gyro[0] = v;
            }
        }
        s
    }
    fn read_position(&mut self) -> Option<Vec3> { self.b.read_position() }
    fn read_mag(&mut self) -> Option<Vec3> { self.b.read_mag() }
    fn read_heading(&mut self) -> Option<f32> { self.b.read_heading() }
    fn write_motors(&mut self, m: &[f32]) {
        let mut m4 = [0.0f32; 4];
        m4.copy_from_slice(&m[..4]);
        self.b.write_motors(&m4);
        self.i += 1;
    }
    fn dt(&self) -> f32 { self.pat[self.i % self.pat.len()] }
    fn read_battery_v(&mut self) -> Option<f32> { self.b.read_battery_v() }
    fn read_battery_i(&mut self) -> Option<f32> { self.b.read_battery_i() }
    fn read_range(&mut self) -> Option<f32> { self.b.read_range() }
    fn read_baro(&mut self) -> Option<f32> { self.b.read_baro() }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> { self.b.read_motor_rpm() }
}

fn mean(p: &[f32]) -> f32 { p.iter().sum::<f32>() / p.len() as f32 }

fn rand2(lo: f32, hi: f32, seed: u64) -> Vec<f32> {
    let mut s = seed;
    (0..200000).map(|_| {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        if ((s >> 33) % 2) == 0 { hi } else { lo }
    }).collect()
}

#[derive(Clone)]
struct Cfg {
    pat: Vec<f32>,
    rotor_vib: f32,
    bump: Option<Vec3>,
    /// at flight time t (s), switch to (pattern, plant dt)
    step: Option<(f32, Vec<f32>)>,
    poison: Option<(f32, f32)>, // (flight time s, value)
    fly_s: f32,
}

fn sup_new() -> FlightSupervisor {
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(crate::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..crate::relay_calib::CalParams::identity()
    });
    sup
}

/// returns (final mode, modes seen, peak tilt over whole flight, peak tilt after step/poison, rate rms last third)
fn run(c: &Cfg) -> (Mode, Vec<Mode>, f32, f32, f32) {
    let pdt = mean(&c.pat);
    let hz = 1.0 / pdt;
    let pre = (5.0 * hz) as usize;
    let mut sim = SimBackend::new(LEVEL, pdt).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        ..Pathology::default()
    });
    sim.ground_contact = true;
    sim.rotor_vib = c.rotor_vib;
    let mut b = Pb { b: sim, pat: c.pat.clone(), i: 0, reads: 0, kick: None, poison: None };
    b.kick = c.bump.map(|g| (pre - (0.02 * hz) as usize, pre, g));
    let mut sup = sup_new();
    for _ in 0..pre { sup.step(&mut b); }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    if sup.mode() != Mode::Takeoff { return (sup.mode(), vec![sup.mode()], f32::NAN, f32::NAN, f32::NAN); }
    let mut modes = vec![sup.mode()];
    let (mut peak, mut peak_after) = (0.0f32, 0.0f32);
    let mut t = 0.0f32;
    let mut stepped = false;
    let mut poisoned = c.poison.is_none();
    let (mut acc, mut cnt) = (0.0f64, 0usize);
    while t < c.fly_s {
        if !stepped {
            if let Some((ts, ref np)) = c.step {
                if t >= ts {
                    b.pat = np.clone();
                    b.i = 0;
                    b.b.dt = mean(np);
                    stepped = true;
                }
            }
        }
        if !poisoned {
            if let Some((tp, v)) = c.poison {
                if t >= tp { b.poison = Some((b.reads + 1, v)); poisoned = true; }
            }
        }
        let d = b.dt();
        sup.step(&mut b);
        t += b.b.dt;
        let _ = d;
        let tl = b.b.tilt();
        peak = if tl.is_finite() { peak.max(tl) } else { f32::INFINITY };
        if stepped || (c.poison.is_some() && poisoned) { peak_after = if tl.is_finite() { peak_after.max(tl) } else { f32::INFINITY }; }
        if t > c.fly_s * 2.0 / 3.0 {
            let w = b.b.omega;
            acc += (w[0] * w[0] + w[1] * w[1]) as f64;
            cnt += 1;
        }
        if modes.last() != Some(&sup.mode()) { modes.push(sup.mode()); }
    }
    (sup.mode(), modes, peak, peak_after, (acc / cnt.max(1) as f64).sqrt() as f32)
}

fn report(name: &str, c: &Cfg) {
    let (m, ms, p, pa, rms) = run(c);
    println!("CELL|{name}|{m:?}|{p:.4}|{pa:.4}|{rms:.5}|{ms:?}");
}

#[test]
fn probe_boundary_fixed_rates() {
    let mut hz = 125.0f32;
    while hz <= 160.0 {
        for vib in [0.0f32, 0.8] {
            for bump in [None, Some([3.0f32, -2.0, 1.0])] {
                let c = Cfg { pat: vec![1.0 / hz], rotor_vib: vib, bump, step: None, poison: None, fly_s: 20.0 };
                report(&format!("fixed {hz:.1}Hz vib{vib} bump{}", bump.is_some()), &c);
            }
        }
        hz += 1.0;
    }
}

#[test]
fn probe_wide_rates() {
    for hz in [10.0f32, 15.0, 20.0, 30.0, 40.0, 50.0, 60.0, 75.0, 90.0, 100.0, 110.0, 200.0, 250.0, 300.0, 350.0, 400.0, 600.0, 1000.0, 2000.0, 5000.0, 10000.0] {
        for vib in [0.0f32, 0.8] {
            for bump in [None, Some([3.0f32, -2.0, 1.0])] {
                let c = Cfg { pat: vec![1.0 / hz], rotor_vib: vib, bump, step: None, poison: None, fly_s: 20.0 };
                report(&format!("fixed {hz:.1}Hz vib{vib} bump{}", bump.is_some()), &c);
            }
        }
    }
}

#[test]
fn probe_rate_steps() {
    let pairs: &[(f32, f32)] = &[
        (140.0, 125.0), (125.0, 140.0), (135.0, 131.0), (131.0, 135.0), (160.0, 128.0), (128.0, 160.0),
        (1000.0, 140.0), (1000.0, 125.0), (1000.0, 100.0), (1000.0, 50.0), (1000.0, 200.0), (1000.0, 400.0),
        (400.0, 140.0), (250.0, 125.0), (250.0, 100.0), (140.0, 1000.0), (100.0, 1000.0), (50.0, 1000.0),
        (500.0, 150.0), (300.0, 150.0), (150.0, 300.0),
    ];
    for &(a, z) in pairs {
        for vib in [0.0f32, 0.8] {
            let c = Cfg { pat: vec![1.0 / a], rotor_vib: vib, bump: None, step: Some((10.0, vec![1.0 / z])), poison: None, fly_s: 20.0 };
            report(&format!("step {a}->{z}Hz vib{vib}"), &c);
        }
    }
}

#[test]
fn probe_straddle_jitter() {
    let cases: Vec<(&str, Vec<f32>)> = vec![
        ("alt 7.4/7.6ms", vec![0.0074, 0.0076]),
        ("rand 7.4/7.6ms s1", rand2(0.0074, 0.0076, 1)),
        ("rand 7.4/7.6ms s2", rand2(0.0074, 0.0076, 2)),
        ("rand 7.2/7.8ms s1", rand2(0.0072, 0.0078, 1)),
        ("rand 7.0/8.0ms s3", rand2(0.0070, 0.0080, 3)),
        ("rand 6.5/7.5ms s3", rand2(0.0065, 0.0075, 3)),
        ("rand 6.0/9.0ms s4", rand2(0.0060, 0.0090, 4)),
        ("rand 6/7ms s5", rand2(0.0060, 0.0070, 5)),
        ("rand 4/10ms s6", rand2(0.004, 0.010, 6)),
        ("rand 6.6/6.9ms (notch f0 edge ~148Hz) s7", rand2(0.0066, 0.0069, 7)),
        ("rand 3.3/3.5ms (h2 edge ~296Hz) s8", rand2(0.0033, 0.0035, 8)),
        ("fixed 7.5ms", vec![0.0075]),
        ("fixed 6.75ms", vec![0.00675]),
        ("fixed 3.4ms", vec![0.0034]),
        ("fixed 7ms", vec![0.007]),
    ];
    for (name, pat) in cases {
        for vib in [0.0f32, 0.8] {
            for bump in [None, Some([3.0f32, -2.0, 1.0])] {
                let c = Cfg { pat: pat.clone(), rotor_vib: vib, bump, step: None, poison: None, fly_s: 20.0 };
                report(&format!("{name} vib{vib} bump{}", bump.is_some()), &c);
            }
        }
    }
}

#[test]
fn probe_poison() {
    for (name, pat) in [("fixed 1kHz", vec![0.001f32]), ("fixed 250Hz", vec![0.004]), ("fixed 140Hz", vec![1.0 / 140.0]), ("rand 2/3ms", rand2(0.002, 0.003, 9)), ("alt 2/3ms", vec![0.002, 0.003])] {
        for v in [f32::NAN, f32::INFINITY, 1.0e30, 1.0e6, 50.0] {
            let c = Cfg { pat: pat.clone(), rotor_vib: 0.0, bump: None, step: None, poison: Some((10.0, v)), fly_s: 20.0 };
            report(&format!("poison {name} {v}"), &c);
        }
    }
}

/// Two flights: fly at A, land, disarm, idle at B, take off again.
#[test]
fn probe_second_flight() {
    for &(a, z) in &[(1000.0f32, 1000.0f32), (1000.0, 140.0), (1000.0, 250.0), (250.0, 1000.0), (140.0, 1000.0), (400.0, 125.0), (1000.0, 100.0)] {
        for bump in [None, Some([3.0f32, -2.0, 1.0])] {
            let pdt = 1.0 / a;
            let mut sim = SimBackend::new(LEVEL, pdt).with_pathology(Pathology { gyro_white: 0.02, vibration: 0.5, ..Pathology::default() });
            sim.ground_contact = true;
            let mut b = Pb { b: sim, pat: vec![pdt], i: 0, reads: 0, kick: None, poison: None };
            let mut sup = sup_new();
            for _ in 0..((5.0 * a) as usize) { sup.step(&mut b); }
            sup.command(Event::Arm, true, true);
            sup.command(Event::RequestTakeoff, true, true);
            for _ in 0..((15.0 * a) as usize) { sup.step(&mut b); }
            let m1 = sup.mode();
            sup.command(Event::RequestLand, true, true);
            let mut landed = false;
            for _ in 0..((40.0 * a) as usize) {
                sup.step(&mut b);
                if sup.mode() == Mode::Disarmed { landed = true; break; }
            }
            if !landed { sup.command(Event::RequestDisarm, true, true); }
            // switch rate while on the ground
            let ndt = 1.0 / z;
            b.pat = vec![ndt]; b.i = 0; b.b.dt = ndt;
            let pre = (5.0 * z) as usize;
            b.kick = bump.map(|g| (b.reads + pre - (0.02 * z) as usize, b.reads + pre, g));
            for _ in 0..pre { sup.step(&mut b); }
            sup.command(Event::Arm, true, true);
            sup.command(Event::RequestTakeoff, true, true);
            let mut modes = vec![sup.mode()];
            let mut peak = 0.0f32;
            for _ in 0..((20.0 * z) as usize) {
                sup.step(&mut b);
                let tl = b.b.tilt();
                peak = if tl.is_finite() { peak.max(tl) } else { f32::INFINITY };
                if modes.last() != Some(&sup.mode()) { modes.push(sup.mode()); }
            }
            println!("CELL|second {a}->{z}Hz bump{} f1={m1:?} landed={landed}|{:?}|{peak:.4}|-|-|{modes:?}", bump.is_some(), sup.mode());
        }
    }
}

#[test]
fn probe2_vib_boundary() {
    let mut hz = 118.0f32;
    while hz <= 152.0 {
        for vib in [0.05f32, 0.1, 0.2, 0.3] {
            let c = Cfg { pat: vec![1.0 / hz], rotor_vib: vib, bump: None, step: None, poison: None, fly_s: 20.0 };
            report(&format!("fixed {hz:.1}Hz vib{vib}"), &c);
        }
        hz += 1.0;
    }
    for vib in [0.05f32, 0.1, 0.2, 0.3] {
        for hz in [100.0f32, 110.0, 160.0, 200.0, 250.0, 300.0, 400.0, 1000.0] {
            let c = Cfg { pat: vec![1.0 / hz], rotor_vib: vib, bump: None, step: None, poison: None, fly_s: 20.0 };
            report(&format!("fixed {hz:.1}Hz vib{vib}"), &c);
        }
    }
}

/// Unsupervised hover (no failsafes) under rotor vibration: rate rms + motor thrash.
#[test]
fn probe4_hover_vib() {
    use crate::{FlightCore, SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ};
    let hzs: Vec<f32> = if std::env::var("FINE").is_ok() {
        (0..16).map(|i| 126.0 + 0.5 * i as f32).collect()
    } else {
        vec![118.0f32, 122.0, 125.0, 128.0, 130.0, 132.0, 133.0, 134.0, 136.0, 140.0, 150.0, 250.0]
    };
    let vibs: Vec<f32> = if std::env::var("FINE").is_ok() { vec![0.15, 0.2, 0.25, 0.3, 0.35] } else { vec![0.1, 0.3, 0.5, 0.8] };
    for &hz in &hzs {
        for &vib in &vibs {
            let dt = 1.0 / hz;
            let mut b = SimBackend::new(LEVEL, dt).with_pathology(Pathology { gyro_white: 0.02, vibration: 0.5, ..Pathology::default() });
            b.rotor_vib = vib;
            b.pos = [0.0, 0.0, -2.0];
            let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
            core.set_position([0.0, 0.0, -2.0]);
            let n = (12.0 * hz) as usize;
            let (mut acc, mut cnt, mut peak) = (0.0f64, 0usize, 0.0f32);
            let (mut s1, mut s2) = (0.0f64, 0.0f64);
            for k in 0..n {
                core.step(&mut b);
                let tl = b.tilt();
                peak = if tl.is_finite() { peak.max(tl) } else { f32::INFINITY };
                if k > n / 3 {
                    let w = b.omega;
                    acc += (w[0] * w[0] + w[1] * w[1]) as f64;
                    let m = b.motor_state[0] as f64;
                    s1 += m; s2 += m * m;
                    cnt += 1;
                }
            }
            let c = cnt.max(1) as f64;
            let thr = (s2 / c - (s1 / c).powi(2)).max(0.0).sqrt();
            println!("CELL|hover {hz}Hz vib{vib}|Loiter|{peak:.4}|0|{:.5}|thrash {thr:.5}", (acc / c).sqrt());
        }
    }
}

/// Supervised: clean takeoff to Loiter, then rotor vibration switched on at 10 s.
#[test]
fn probe5_sup_vib_after_loiter() {
    for hz in [125.0f32, 127.0, 128.0, 129.0, 130.0, 131.0, 132.0, 133.0, 134.0, 136.0, 140.0, 250.0] {
        for vib in [0.15f32, 0.2, 0.25, 0.3] {
            let pdt = 1.0 / hz;
            let mut sim = SimBackend::new(LEVEL, pdt).with_pathology(Pathology { gyro_white: 0.02, vibration: 0.5, ..Pathology::default() });
            sim.ground_contact = true;
            let mut b = Pb { b: sim, pat: vec![pdt], i: 0, reads: 0, kick: None, poison: None };
            let mut sup = sup_new();
            for _ in 0..((5.0 * hz) as usize) { sup.step(&mut b); }
            sup.command(Event::Arm, true, true);
            sup.command(Event::RequestTakeoff, true, true);
            let mut modes = vec![sup.mode()];
            let mut peak = 0.0f32;
            for k in 0..((25.0 * hz) as usize) {
                if k == (10.0 * hz) as usize { b.b.rotor_vib = vib; }
                sup.step(&mut b);
                if k >= (10.0 * hz) as usize {
                    let tl = b.b.tilt();
                    peak = if tl.is_finite() { peak.max(tl) } else { f32::INFINITY };
                }
                if modes.last() != Some(&sup.mode()) { modes.push(sup.mode()); }
            }
            println!("CELL|supvib {hz}Hz vib{vib}|{:?}|{peak:.4}|0|0|{modes:?}", sup.mode());
        }
    }
}

#[test]
fn probe3_vib_transitions() {
    for vib in [0.03f32, 0.05] {
        let cases: Vec<(&str, Vec<f32>)> = vec![
            ("alt 7.4/7.6ms", vec![0.0074, 0.0076]),
            ("rand 7.4/7.6ms s1", rand2(0.0074, 0.0076, 1)),
            ("rand 7.2/7.8ms s1", rand2(0.0072, 0.0078, 1)),
            ("rand 7.0/8.0ms s3", rand2(0.0070, 0.0080, 3)),
            ("rand 6.5/7.5ms s3", rand2(0.0065, 0.0075, 3)),
            ("rand 6.0/9.0ms s4", rand2(0.0060, 0.0090, 4)),
            ("rand 6.6/6.9ms s7", rand2(0.0066, 0.0069, 7)),
            ("rand 3.3/3.5ms s8", rand2(0.0033, 0.0035, 8)),
            ("rand 2/3ms s9", rand2(0.002, 0.003, 9)),
            ("fixed 7.5ms", vec![0.0075]),
            ("fixed 6.75ms", vec![0.00675]),
            ("fixed 3.4ms", vec![0.0034]),
        ];
        for (name, pat) in cases {
            for bump in [None, Some([3.0f32, -2.0, 1.0])] {
                let c = Cfg { pat: pat.clone(), rotor_vib: vib, bump, step: None, poison: None, fly_s: 20.0 };
                report(&format!("{name} vib{vib} bump{}", bump.is_some()), &c);
            }
        }
        let pairs: &[(f32, f32)] = &[
            (140.0, 125.0), (125.0, 140.0), (135.0, 131.0), (131.0, 135.0), (160.0, 128.0), (128.0, 160.0),
            (1000.0, 140.0), (1000.0, 125.0), (1000.0, 100.0), (400.0, 140.0), (250.0, 125.0),
            (140.0, 1000.0), (100.0, 1000.0), (500.0, 150.0), (300.0, 150.0), (150.0, 300.0), (300.0, 145.0), (145.0, 300.0),
        ];
        for &(a, z) in pairs {
            let c = Cfg { pat: vec![1.0 / a], rotor_vib: vib, bump: None, step: Some((10.0, vec![1.0 / z])), poison: None, fly_s: 20.0 };
            report(&format!("step {a}->{z}Hz vib{vib}"), &c);
        }
    }
    let mut hz = 122.0f32;
    while hz <= 136.0 {
        for vib in [0.06f32, 0.07, 0.08, 0.09] {
            let c = Cfg { pat: vec![1.0 / hz], rotor_vib: vib, bump: None, step: None, poison: None, fly_s: 20.0 };
            report(&format!("fixed {hz:.1}Hz vib{vib}"), &c);
        }
        hz += 2.0;
    }
}
