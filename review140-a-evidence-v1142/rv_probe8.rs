// Adversarial review probe (PR #644 @ 86a44a9 vs 725f146). Common public API only.
// Cells: rate x rpm-scale x rotor-vib x dt pattern x scenario. The rotor lines are
// generated HERE at the reported RPM (scaled by k), so the notch bank sees
// hover RPM 2000-8000; the plant integrates each frame's OWN dt (real jitter).
extern crate std;
use std::{vec, vec::Vec, string::String, format};
use crate::relay_fsm::{Event, Mode};
use crate::{FlightBackend, FlightSupervisor, ImuSample, Pathology, SimBackend};
use std::io::Write;

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

struct K {
    b: SimBackend,
    pat: Vec<f32>,
    i: usize,
    k: f32,
    vib: f32,
    ph: [f32; 4],
    last_m: [f32; 4],
    thrash: f64,
    nthr: u64,
}
impl K {
    fn cur(&self) -> f32 {
        self.pat[self.i % self.pat.len()]
    }
}
impl FlightBackend for K {
    fn read_imu(&mut self) -> ImuSample {
        let mut s = self.b.read_imu();
        if self.vib > 0.0 {
            if let Some(r) = self.b.read_motor_rpm() {
                let dt = self.cur();
                for m in 0..4 {
                    let f = r[m] as f32 * self.k / 60.0;
                    self.ph[m] += 2.0 * std::f32::consts::PI * f * dt;
                    while self.ph[m] > 2.0 * std::f32::consts::PI {
                        self.ph[m] -= 2.0 * std::f32::consts::PI;
                    }
                    s.gyro[0] += self.vib * relay_math::sinf(self.ph[m]);
                    s.gyro[1] += self.vib * relay_math::cosf(self.ph[m]);
                    // 2nd harmonic, half amplitude
                    s.gyro[0] += 0.5 * self.vib * relay_math::sinf(2.0 * self.ph[m]);
                    s.gyro[1] += 0.5 * self.vib * relay_math::cosf(2.0 * self.ph[m]);
                }
            }
        }
        s
    }
    fn read_position(&mut self) -> Option<Vec3> { self.b.read_position() }
    fn read_mag(&mut self) -> Option<Vec3> { self.b.read_mag() }
    fn read_heading(&mut self) -> Option<f32> { self.b.read_heading() }
    fn write_motors(&mut self, m: &[f32]) {
        self.b.dt = self.cur();
        let mut d = 0.0f64;
        for j in 0..4 { d += {let q=(m[j] - self.last_m[j]) as f64; q*q}; self.last_m[j] = m[j]; }
        self.thrash += d; self.nthr += 1;
        self.b.write_motors(m);
        self.i += 1;
    }
    fn dt(&self) -> f32 { self.cur() }
    fn read_battery_v(&mut self) -> Option<f32> { self.b.read_battery_v() }
    fn read_battery_i(&mut self) -> Option<f32> { self.b.read_battery_i() }
    fn read_range(&mut self) -> Option<f32> { self.b.read_range() }
    fn read_baro(&mut self) -> Option<f32> { self.b.read_baro() }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.b.read_motor_rpm().map(|r| { let mut o = r; for j in 0..4 { o[j] = (r[j] as f32 * self.k) as i32; } o })
    }
}

fn lcg(s: &mut u64) -> f64 {
    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    ((*s >> 11) as f64) / ((1u64 << 53) as f64)
}

fn pattern(kind0: &str, hz: f32) -> Vec<f32> {
    let dt = 1.0 / hz;
    let (kind, seed) = match kind0.rfind('s') { Some(i) if kind0.starts_with("late") || kind0.starts_with("fixed") => (&kind0[..i], kind0[i+1..].parse::<u64>().unwrap()), _ => (kind0, 0u64) };
    let mut s = seed * 7919 + 7;
    match kind {
        "late10" => (0..40009).map(|_| if lcg(&mut s) < 0.1 { dt * 1.5 } else { dt }).collect(),
        "late20" => (0..40009).map(|_| if lcg(&mut s) < 0.05 { dt * 2.0 } else { dt }).collect(),
        "late5" => (0..40009).map(|_| if lcg(&mut s) < 0.2 { dt * 1.25 } else { dt }).collect(),
        "fixed" => vec![dt],
        "alt" => vec![dt * 0.875, dt * 1.125],
        "rand12" => (0..40009).map(|_| if lcg(&mut s) < 0.5 { dt * 0.875 } else { dt * 1.125 }).collect(),
        // uniform +-1.5 % (inside the 2 % band: the parent never resynced)
        "jit1" => (0..40009).map(|_| dt * (1.0 + 0.03 * (lcg(&mut s) as f32 - 0.5))).collect(),
        // uniform +-4 % continuous
        "jit4" => (0..40009).map(|_| dt * (1.0 + 0.08 * (lcg(&mut s) as f32 - 0.5))).collect(),
        // a minority of late frames: 1 in 10 is 1.5x
        "late" => (0..40009).map(|_| if lcg(&mut s) < 0.1 { dt * 1.5 } else { dt }).collect(),
        _ => panic!(),
    }
}

struct Out { iso: Option<String>, tick: u64, mode: Mode, loiter: bool, peak: f32, rms_h: f64, rms_tilt: f64, thrash: f64, extra: String }

fn rig(hz: f32, pat_kind: &str, k: f32) -> (K, FlightSupervisor) {
    let pat = pattern(pat_kind, hz);
    let mut sim = SimBackend::new(LEVEL, pat[0]).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        ..Pathology::default()
    });
    sim.ground_contact = true;
    sim.rpm_telemetry = true;
    let b = K { b: sim, pat, i: 0, k, vib: 0.0, ph: [0.0; 4], last_m: [0.0; 4], thrash: 0.0, nthr: 0 };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(crate::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..crate::relay_calib::CalParams::identity()
    });
    (b, sup)
}

fn fly(b: &mut K, sup: &mut FlightSupervisor, secs: f32, hz: f32, vib: f32, o: &mut Out, measure: bool) {
    let n = (secs * hz) as usize;
    let (mut sh, mut st, mut cnt) = (0.0f64, 0.0f64, 0u64);
    for _ in 0..n {
        sup.step(b);
        o.tick += 1;
        if o.iso.is_none() { if let Some(m) = sup.core().failed_motor() { o.iso = Some(format!("iso@{:.2}s:{:?}:m{}", o.tick as f32 / hz, sup.mode(), m)); } }
        if !o.loiter && sup.mode() == Mode::Loiter { o.loiter = true; b.b.rotor_vib = vib; }
        if o.loiter && measure {
            let t = b.b.tilt();
            o.peak = if t.is_finite() && o.peak.is_finite() { o.peak.max(t) } else { f32::INFINITY };
            let p = b.b.pos;
            sh += (p[0] as f64)*(p[0] as f64) + (p[1] as f64)*(p[1] as f64);
            st += (t as f64)*(t as f64);
            cnt += 1;
        }
    }
    if cnt > 0 { o.rms_h = (sh / cnt as f64).sqrt(); o.rms_tilt = (st / cnt as f64).sqrt(); }
}

fn cell(scn: &str, hz: f32, k: f32, vib: f32, pat_kind: &str) -> String {
    let (mut b, mut sup) = rig(hz, pat_kind, k);
    if let Some(i) = pat_kind.rfind('s') { b.b.reseed(pat_kind[i+1..].parse::<u32>().unwrap_or(1) + 1); }
    let pert = std::env::var("RV_PERTURB").map(|_| 1).unwrap_or(0);
    let pre = (5.0 * hz) as usize + pert;
    for _ in 0..pre { sup.step(&mut b); }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mut o = Out { iso: None, tick: 0, mode: sup.mode(), loiter: false, peak: 0.0, rms_h: 0.0, rms_tilt: 0.0, thrash: 0.0, extra: String::new() };
    match scn {
        "long" => { fly(&mut b, &mut sup, 60.0, hz, vib, &mut o, true); o.extra = format!("fm={:?}", sup.core().failed_motor()); }
        "second" => {
            fly(&mut b, &mut sup, 25.0, hz, vib, &mut o, false);
            let m1 = sup.mode();
            sup.command(Event::RequestLand, true, false);
            let mut disarmed = false;
            for _ in 0..(60.0 * hz) as usize { sup.step(&mut b); o.tick += 1; if o.iso.is_none() { if let Some(m) = sup.core().failed_motor() { o.iso = Some(format!("iso@{:.2}s:LANDING:{:?}:m{}", o.tick as f32 / hz, sup.mode(), m)); } } if sup.mode() == Mode::Disarmed { disarmed = true; break; } }
            b.b.rotor_vib = 0.0;
            for _ in 0..(5.0 * hz) as usize { sup.step(&mut b); }
            sup.command(Event::Arm, true, true);
            sup.command(Event::RequestTakeoff, true, true);
            let m2 = sup.mode();
            o.loiter = false; o.peak = 0.0;
            fly(&mut b, &mut sup, 30.0, hz, vib, &mut o, true);
            o.extra = format!("f1={m1:?} disarmed={disarmed} start2={m2:?} fm={:?}", sup.core().failed_motor());
        }
        "rotorout" => {
            fly(&mut b, &mut sup, 25.0, hz, vib, &mut o, true);
            let m1 = sup.mode();
            b.b.fail_rotor(1);
            o.peak = 0.0;
            fly(&mut b, &mut sup, 15.0, hz, vib, &mut o, true);
            o.extra = format!("before={m1:?} fm={:?} z={:.2}", sup.core().failed_motor(), b.b.pos[2]);
        }
        "step" => {
            // a genuine rate change mid-flight: hz -> hz/4 (or x4 if hz<=250) after 20 s
            fly(&mut b, &mut sup, 20.0, hz, vib, &mut o, true);
            let to = if hz <= 250.0 { hz * 4.0 } else { hz / 4.0 };
            let off = b.i;
            let np: Vec<f32> = b.pat.iter().map(|d| d * hz / to).collect();
            b.pat = np; b.i = off;
            fly(&mut b, &mut sup, 20.0, to, vib, &mut o, true);
            fly(&mut b, &mut sup, 0.0, to, vib, &mut o, true);
        }
        _ => panic!(),
    }
    o.mode = sup.mode();
    let iso_s = o.iso.clone().unwrap_or_else(|| String::from("iso=none"));
    o.thrash = if b.nthr > 0 { (b.thrash / b.nthr as f64).sqrt() } else { 0.0 };
    format!(
        "CELL {scn} {hz} {k} {vib} {pat_kind} loiter={} mode={:?} peak={:.4} rmsh={:.4} rmstilt={:.5} thrash={:.5} {}",
        o.loiter, o.mode, o.peak, o.rms_h, o.rms_tilt, o.thrash, o.extra
    ) + " " + &iso_s
}

#[test]
fn rv_grid8() {
    let out = std::env::var("RV_OUT").unwrap();
    let mut jobs: Vec<(&'static str, f32, f32, f32, String)> = Vec::new();
    for &hz in &[250.0f32, 400.0, 1000.0] {
        for &vib in &[0.25f32, 0.3, 0.4] {
            for seed in 0..24u32 {
                for pat in ["fixed", "late10", "late5"] {
                    jobs.push(("second", hz, 1.0, vib, format!("{pat}s{seed}")));
                    jobs.push(("long", hz, 1.0, vib, format!("{pat}s{seed}")));
                }
            }
        }
    }
    let n = jobs.len();
    let jobs = std::sync::Arc::new(std::sync::Mutex::new(jobs));
    let results = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut hs = Vec::new();
    for _ in 0..9 {
        let (j, r) = (jobs.clone(), results.clone());
        hs.push(std::thread::spawn(move || loop {
            let job = j.lock().unwrap().pop();
            let Some((scn, hz, k, vib, pat)) = job else { break };
            let line = cell(scn, hz, k, vib, &pat);
            r.lock().unwrap().push(line);
        }));
    }
    for h in hs { h.join().unwrap(); }
    let mut r = results.lock().unwrap().clone();
    r.sort();
    assert_eq!(r.len(), n);
    let mut f = std::fs::File::create(out).unwrap();
    for l in r { writeln!(f, "{l}").unwrap(); }
}
