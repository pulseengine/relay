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
    rep: Vec<f32>,
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
    fn dt(&self) -> f32 { if self.rep.is_empty() { self.cur() } else { self.rep[self.i % self.rep.len()] } }
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

fn pattern(kind0: &str, hz: f32) -> (Vec<f32>, Vec<f32>) {
    let dt = 1.0 / hz;
    let i = kind0.rfind('s').unwrap();
    let (kind, seed) = (&kind0[..i], kind0[i+1..].parse::<u64>().unwrap());
    let mut s = seed * 7919 + 7;
    let n = 400009usize.min((140.0 * hz) as usize + 7);
    let alt = |j: usize| if j % 2 == 0 { dt * 0.875 } else { dt * 1.125 };
    let late = |s: &mut u64, p: f64, m: f32| if lcg(s) < p { dt * m } else { dt };
    let burst = (1.0 * hz) as usize;
    let mut rep = Vec::new();
    let pat: Vec<f32> = match kind {
        "fixed" => vec![dt],
        "alt" => vec![dt * 0.875, dt * 1.125],
        "p3" => vec![dt, dt, dt * 1.5],
        "p3b" => vec![dt, dt * 1.5, dt * 1.5],
        "p5" => vec![dt, dt * 1.5, dt, dt * 1.5, dt],
        "jit" => (0..n).map(|_| dt * (1.0 + 0.08 * (lcg(&mut s) as f32 - 0.5))).collect(),
        "late5" => (0..n).map(|_| late(&mut s, 0.2, 1.25)).collect(),
        "late10" => (0..n).map(|_| late(&mut s, 0.1, 1.5)).collect(),
        "late30" => (0..n).map(|_| late(&mut s, 0.3, 1.5)).collect(),
        "late40" => (0..n).map(|_| late(&mut s, 0.4, 1.5)).collect(),
        "late30q" => (0..n).map(|_| late(&mut s, 0.3, 1.25)).collect(),
        // 1 s of alternation (boot jitter, pre-arm) then a minority-late / period-3 host
        "bl5" => (0..n).map(|j| if j < burst { alt(j) } else { late(&mut s, 0.2, 1.25) }).collect(),
        "bl10" => (0..n).map(|j| if j < burst { alt(j) } else { late(&mut s, 0.1, 1.5) }).collect(),
        "bp3" => (0..n).map(|j| if j < burst { alt(j) } else { [dt, dt, dt * 1.5][j % 3] }).collect(),
        // 0.1 s alternation bursts every 2 s on a fixed base (repeated entry/exit)
        "mb" => (0..n).map(|j| if j % (2 * hz as usize) < (0.1 * hz) as usize { alt(j) } else { dt }).collect(),
        // ... on a minority-late base (entries + hysteresis hold)
        "mbl" => (0..n).map(|j| if j % (2 * hz as usize) < (0.1 * hz) as usize { alt(j) } else { late(&mut s, 0.2, 1.25) }).collect(),
        // slow drift +-3% over 10 s plus uniform +-1% jitter: flickers at the band edge
        "drift" => (0..n).map(|j| { let t = j as f32 / hz; dt * (1.0 + 0.03 * relay_math::sinf(2.0 * std::f32::consts::PI * t / 10.0) + 0.02 * (lcg(&mut s) as f32 - 0.5)) }).collect(),
        // alternation, every 4th REPORTED dt is NaN (plant still integrates the real period)
        "altnan" => { rep = (0..4).map(|j| if j == 3 { f32::NAN } else { alt(j) }).collect(); vec![dt * 0.875, dt * 1.125, dt * 0.875, dt * 1.125] }
        // fixed, reported dt 0 / negative / NaN on 5% of frames
        "badrep" => { let p: Vec<f32> = vec![dt; n]; rep = (0..n).map(|_| { let u = lcg(&mut s); if u < 0.017 { 0.0 } else if u < 0.034 { -dt } else if u < 0.05 { f32::NAN } else { dt } }).collect(); p }

        // ---- rv_adv2 (review of 2f9314b) ----
        // N alternation frames, then the last period repeated M times (non-crossing): guard flapping
        k if k.starts_with("flap") => {
            let body = &k[4..];
            let (nn, mm) = match body.find('m') { Some(p) => (body[..p].parse::<usize>().unwrap(), body[p+1..].parse::<usize>().unwrap()), None => (body.parse::<usize>().unwrap(), 1) };
            let mut v = Vec::new();
            for j in 0..nn { v.push(alt(j)); }
            let last = *v.last().unwrap();
            for _ in 0..mm { v.push(last); }
            v
        }
        // altc<centre%>a<half-amplitude permille>: two-level alternation dt*c*(1-+a)
        k if k.starts_with("altc") => { let q = &k[4..]; let ai = q.find('a').unwrap(); let c = q[..ai].parse::<f32>().unwrap() / 100.0; let a = q[ai+1..].parse::<f32>().unwrap() / 1000.0; vec![dt * c * (1.0 - a), dt * c * (1.0 + a)] }
        // fixc<centre%>: fixed period dt*c (control: is it the rate, or the alternation?)
        k if k.starts_with("fixc") => vec![dt * k[4..].parse::<f32>().unwrap() / 100.0],
        "alt23" => vec![dt * 0.8, dt * 1.2],
        "alt5" => vec![dt * 0.97, dt * 1.03],
        "altoff" => vec![dt * 1.3 * 0.875, dt * 1.3 * 1.125],
        "altoffd" => vec![dt * 0.7 * 0.875, dt * 0.7 * 1.125],
        "l3" => vec![dt * 0.8, dt, dt * 1.2],
        "l3b" => vec![dt * 0.8, dt * 1.2, dt * 1.6],
        "l3c" => vec![dt, dt * 1.03, dt * 1.06],
        "saw" => (0..16).map(|j| dt * (1.0 + 0.03 * j as f32)).collect(),
        "asym" => vec![dt, dt * 1.05, dt * 2.0],
        "asym2" => vec![dt * 0.97, dt, dt * 0.97, dt, dt * 0.97, dt, dt * 1.6],
        "asym3" => vec![dt * 0.9, dt * 1.1, dt * 0.9, dt * 1.1, dt * 0.9, dt * 1.1, dt * 0.9, dt * 2.0],
        // alternation whose centre drifts +-15% with period T s
        "altdrift2" | "altdrift10" => { let tp = if kind == "altdrift2" { 2.0 } else { 10.0 }; (0..n).map(|j| { let t = j as f32 / hz; let c = 1.0 + 0.15 * relay_math::sinf(2.0 * std::f32::consts::PI * t / tp); dt * c * if j % 2 == 0 { 0.875 } else { 1.125 } }).collect() }
        // alternation whose amplitude breathes 0..+-20% (crossing and not) with period 4 s
        "altamp" => (0..n).map(|j| { let t = j as f32 / hz; let a = 0.1 + 0.1 * relay_math::sinf(2.0 * std::f32::consts::PI * t / 4.0); dt * if j % 2 == 0 { 1.0 - a } else { 1.0 + a } }).collect(),
        // alternation + 10% random 1.5x-late frames
        "altlate" => (0..n).map(|j| { let b0 = alt(j); if lcg(&mut s) < 0.1 { b0 * 1.5 } else { b0 } }).collect(),
        // large uniform jitter: crosses most but not all frames (natural flapper)
        "jit30" => (0..n).map(|_| dt * (1.0 + 0.30 * (lcg(&mut s) as f32 - 0.5))).collect(),
        "jit50" => (0..n).map(|_| dt * (1.0 + 0.50 * (lcg(&mut s) as f32 - 0.5))).collect(),
        "jit15" => (0..n).map(|_| dt * (1.0 + 0.15 * (lcg(&mut s) as f32 - 0.5))).collect(),
        // alternation; reported dt invalid (NaN / 0) on p of frames at random (plant integrates the real period)
        "altbad1" | "altbad5" => { let p = if kind == "altbad1" { 0.02 } else { 0.05 }; let pp: Vec<f32> = (0..n).map(|j| alt(j)).collect(); rep = (0..n).map(|j| { let u = lcg(&mut s); if u < p * 0.5 { f32::NAN } else if u < p { 0.0 } else { pp[j] } }).collect(); pp }
        // 2 s alternation bursts every 4 s on a fixed base
        "mb2" => (0..n).map(|j| if j % (4 * hz as usize) < (2.0 * hz) as usize { alt(j) } else { dt }).collect(),
        // 0.5 s bursts every 1 s on a late5 base
        "mbl2" => (0..n).map(|j| if j % (hz as usize) < (0.5 * hz) as usize { alt(j) } else { late(&mut s, 0.2, 1.25) }).collect(),
        _ => panic!("{kind}"),
    };
    (pat, rep)
}

struct Out { iso: Option<String>, tick: u64, mode: Mode, loiter: bool, peak: f32, rms_h: f64, rms_tilt: f64, thrash: f64, extra: String }

fn rig(hz: f32, pat_kind: &str, k: f32) -> (K, FlightSupervisor) {
    let (pat, rep) = pattern(pat_kind, hz);
    let mut sim = SimBackend::new(LEVEL, pat[0]).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        ..Pathology::default()
    });
    sim.ground_contact = true;
    sim.rpm_telemetry = true;
    let b = K { b: sim, pat, rep, i: 0, k, vib: 0.0, ph: [0.0; 4], last_m: [0.0; 4], thrash: 0.0, nthr: 0 };
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
            let pre_iso = o.iso.clone();
            b.b.fail_rotor(1);
            o.peak = 0.0;
            let t0 = o.tick;
            let mut lat = -1.0f32;
            for _ in 0..(15.0 * hz) as usize {
                sup.step(&mut b); o.tick += 1;
                if lat < 0.0 && sup.core().failed_motor().is_some() { lat = (o.tick - t0) as f32 / hz; }
                let t = b.b.tilt(); o.peak = if t.is_finite() && o.peak.is_finite() { o.peak.max(t) } else { f32::INFINITY };
            }
            o.extra = format!("before={m1:?} preiso={} fm={:?} lat={lat:.3} z={:.2}", pre_iso.is_some(), sup.core().failed_motor(), b.b.pos[2]);
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

        "swfix" | "swalt" | "fixsw" => {
            // swfix: pattern for 20 s, then FIXED at 1.6x period (genuine rate change down); swalt: then alternation at 1.6x period
            // fixsw: fixed nominal for 20 s then the pattern
            let dt0 = 1.0 / hz;
            if scn == "fixsw" { let real = core::mem::replace(&mut b.pat, vec![dt0]); let realrep = core::mem::take(&mut b.rep); fly(&mut b, &mut sup, 20.0, hz, vib, &mut o, true); b.pat = real; b.rep = realrep; fly(&mut b, &mut sup, 25.0, hz, vib, &mut o, true); }
            else {
                fly(&mut b, &mut sup, 20.0, hz, vib, &mut o, true);
                b.pat = if scn == "swfix" { vec![dt0 * 1.6] } else { vec![dt0 * 1.6 * 0.875, dt0 * 1.6 * 1.125] };
                b.rep.clear(); b.i = 0;
                fly(&mut b, &mut sup, 25.0, hz / 1.6, vib, &mut o, true);
            }
            o.extra = format!("fm={:?}", sup.core().failed_motor());
        }
        "cycles" => {
            // three arm/takeoff/loiter/land/disarm cycles; measure flight 3
            let mut ms = String::new();
            for c in 0..3 {
                if c > 0 {
                    for _ in 0..(5.0 * hz) as usize { sup.step(&mut b); }
                    sup.command(Event::Arm, true, true);
                    sup.command(Event::RequestTakeoff, true, true);
                    o.loiter = false; o.peak = 0.0;
                }
                fly(&mut b, &mut sup, 15.0, hz, vib, &mut o, c == 2);
                ms += &format!("{:?},", sup.mode());
                sup.command(Event::RequestLand, true, false);
                for _ in 0..(60.0 * hz) as usize { sup.step(&mut b); o.tick += 1; if o.iso.is_none() { if let Some(m) = sup.core().failed_motor() { o.iso = Some(format!("iso@{:.2}s:LANDc{c}:{:?}:m{}", o.tick as f32 / hz, sup.mode(), m)); } } if sup.mode() == Mode::Disarmed { break; } }
                ms += &format!("{:?};", sup.mode());
                b.b.rotor_vib = 0.0;
            }
            o.extra = format!("cyc={ms} fm={:?}", sup.core().failed_motor());
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
fn rv_adv2() {
    let out = std::env::var("RV_OUT").unwrap();
    let pats: Vec<&str> = std::env::var("RV_PATS").map(|v| v.leak().split(',').collect()).unwrap_or(vec!["fixed","alt","p3","p3b","p5","late30","late40","late30q","bl5","bl10","bp3","mb","mbl","drift","altnan","badrep"]);
    let rates: Vec<f32> = std::env::var("RV_RATES").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![250.0, 400.0, 1000.0]);
    let vibs: Vec<f32> = std::env::var("RV_VIBS").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![0.0, 0.3]);
    let scns: Vec<&str> = std::env::var("RV_SCNS").map(|v| v.leak().split(',').collect()).unwrap_or(vec!["long", "rotorout"]);
    let nseed: u32 = std::env::var("RV_SEEDS").map(|v| v.parse().unwrap()).unwrap_or(8);
    let mut jobs: Vec<(&'static str, f32, f32, f32, String)> = Vec::new();
    for &hz in &rates { for &vib in &vibs { for seed in 0..nseed { for pat in &pats { for scn in &scns {
        let scn: &'static str = match *scn { "long" => "long", "rotorout" => "rotorout", "second" => "second", "step" => "step", "swfix" => "swfix", "swalt" => "swalt", "fixsw" => "fixsw", "cycles" => "cycles", _ => panic!() };
        jobs.push((scn, hz, 1.0, vib, format!("{pat}s{seed}")));
    }}}}}
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
