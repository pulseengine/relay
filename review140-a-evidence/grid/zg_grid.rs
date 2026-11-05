// ONE grid over every plant input the reviews of PR #644 touched. Common
// API only (compiles on 725f146 and on the branch). Supervised takeoff after
// a 5 s hold, 20 s flown; rotor vibration switched on at the first Loiter.
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, Pathology, SimBackend};
use std::io::Write;

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

struct K {
    b: SimBackend,
    pat: Vec<f32>,
    i: usize,
    reads: usize,
    kick: Option<(usize, usize, Vec3)>,
}
impl FlightBackend for K {
    fn read_imu(&mut self) -> ImuSample {
        let mut s = self.b.read_imu();
        self.reads += 1;
        if let Some((a, z, g)) = self.kick {
            if self.reads >= a && self.reads < z {
                s.gyro = [s.gyro[0] + g[0], s.gyro[1] + g[1], s.gyro[2] + g[2]];
            }
        }
        s
    }
    fn read_position(&mut self) -> Option<Vec3> { self.b.read_position() }
    fn read_mag(&mut self) -> Option<Vec3> { self.b.read_mag() }
    fn read_heading(&mut self) -> Option<f32> { self.b.read_heading() }
    fn write_motors(&mut self, m: &[f32]) { self.b.write_motors(m); self.i += 1; }
    fn dt(&self) -> f32 { self.pat[self.i % self.pat.len()] }
    fn read_battery_v(&mut self) -> Option<f32> { self.b.read_battery_v() }
    fn read_battery_i(&mut self) -> Option<f32> { self.b.read_battery_i() }
    fn read_range(&mut self) -> Option<f32> { self.b.read_range() }
    fn read_baro(&mut self) -> Option<f32> { self.b.read_baro() }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> { self.b.read_motor_rpm() }
}

fn pattern(kind: &str, hz: f32) -> Vec<f32> {
    let dt = 1.0 / hz;
    let (lo, hi) = (dt * 0.875, dt * 1.125);
    match kind {
        "fixed" => vec![dt],
        "alt" => vec![lo, hi],
        _ => {
            let mut s = 11u64;
            (0..20011).map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                if ((s >> 33) % 2) == 0 { hi } else { lo }
            }).collect()
        }
    }
}

fn cell(hz: f32, vib: f32, pre_kind: &str, pat_kind: &str) -> String {
    let pat = pattern(pat_kind, hz);
    let pdt = pat.iter().sum::<f32>() / pat.len() as f32;
    let rate = 1.0 / pdt;
    // GRID_PERTURB=1: hold one tick longer (a physically meaningless change,
    // to measure how far trajectories are sensitive to a 1-tick difference).
    let pre = (5.0 * rate) as usize + std::env::var("GRID_PERTURB").map(|_| 1).unwrap_or(0);
    let kick = match pre_kind {
        "plain" => None,
        "bump" => Some((pre - (0.02 * rate) as usize, pre, [3.0f32, -2.0, 1.0])),
        _ => Some((1usize, pre, [0.5f32, 0.0, 0.0])),
    };
    let mut sim = SimBackend::new(LEVEL, pdt).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        ..Pathology::default()
    });
    sim.ground_contact = true;
    let mut b = K { b: sim, pat, i: 0, reads: 0, kick };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    for _ in 0..pre { sup.step(&mut b); }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mode0 = sup.mode();
    let mut peak = 0.0f32;
    let mut vib_on = false;
    let mut loiter_seen = false;
    for _ in 0..((20.0 * rate) as usize) {
        sup.step(&mut b);
        let t = b.b.tilt();
        peak = if t.is_finite() && peak.is_finite() { peak.max(t) } else { f32::INFINITY };
        if !vib_on && sup.mode() == Mode::Loiter {
            b.b.rotor_vib = vib;
            vib_on = true;
            loiter_seen = true;
        }
    }
    format!("CELL {hz:.3} {vib} {pre_kind} {pat_kind} start={mode0:?} loiter={loiter_seen} mode={:?} peak={peak:.4}", sup.mode())
}

#[test]
fn zg_grid() {
    let out = std::env::var("GRID_OUT").unwrap();
    let mut rates: Vec<f32> = (1..=10).map(|k| 10.0 * k as f32).collect();
    rates.extend((1..=200).map(|k| 100.0 + 0.5 * k as f32));
    rates.extend((0..=20).map(|k| 120.0 + 0.001 * k as f32));
    rates.extend([250.0f32, 300.0, 400.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0]);
    let mut jobs = Vec::new();
    for &hz in &rates {
        for vib in [0.0f32, 0.1, 0.25, 0.3] {
            for pre in ["plain", "bump", "rot"] {
                for pat in ["fixed", "alt", "rand"] {
                    jobs.push((hz, vib, pre, pat));
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
            let Some((hz, vib, pre, pat)) = job else { break };
            let line = cell(hz, vib, pre, pat);
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
