//! MEASUREMENT PROBE (uncommitted): rotor-out FDI false isolation of a HEALTHY
//! rotor under rotor vibration, by flight phase, at a FIXED loop period.
//!
//! Rig mirrors the reviewer's rv_probe9 "second" scenario on the public API:
//! SimBackend, Pathology{gyro_white 0.02, vibration 0.5}, ground contact, RPM
//! telemetry, motor_tau 0 (default), seed = reseed(seed+1); 5 s pre-arm
//! (+PROBE_PERTURB ticks); Arm + RequestTakeoff; rotor_vib switched on at the
//! first Loiter tick; 25 s from takeoff; RequestLand; up to 60 s to Disarmed;
//! 5 s on the ground (rotor_vib 0); Arm attempt.
//!
//! env: PROBE_OUT=<file>  PROBE_MODE=grid|trace  PROBE_PERTURB=<ticks>
//!      PROBE_VIB_FROM_ARM=1 (vib on from arm instead of from Loiter)
//!      PROBE_TRACE=hz,vib,seed  (trace mode)

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, Pathology, SimBackend};
use std::io::Write;

const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn rig(hz: f32, seed: u32) -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, 1.0 / hz).with_pathology(Pathology {
        gyro_white: 0.02,
        vibration: 0.5,
        ..Pathology::default()
    });
    b.ground_contact = true;
    b.rpm_telemetry = true;
    b.reseed(seed + 1);
    if let Some(t) = std::env::var("PROBE_TAU").ok().and_then(|s| s.parse::<f32>().ok()) {
        b.motor_tau = t; // sensitivity only; default rig keeps the reviewer's tau = 0
    }
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    (b, sup)
}

struct Iso {
    t_since_takeoff: f32,
    t_since_land: f32, // negative = before RequestLand
    mode: Mode,
    rotor: usize,
    alt: f32,
    vz: f32, // NED +down
    phase: &'static str,
}

fn phase_of(mode: Mode, alt: f32, landing: bool) -> &'static str {
    match mode {
        Mode::Takeoff => "takeoff",
        Mode::Loiter => "hover",
        Mode::Land if !landing => "rotorout-land", // a Land the FDI itself drove
        Mode::Land if alt > 0.01 => "descent",
        Mode::Land => "touchdown",
        Mode::Disarmed | Mode::Armed => "ground",
        _ => "other",
    }
}

fn run(hz: f32, vib: f32, seed: u32, perturb: usize, vib_from_arm: bool, trace: Option<&mut Vec<String>>) -> String {
    let (mut b, mut sup) = rig(hz, seed);
    for _ in 0..((5.0 * hz) as usize + perturb) {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mut trace = trace;
    let mut iso: Option<Iso> = None;
    let mut tick = 0u64;
    let mut land_tick: Option<u64> = None;
    let mut vib_on = false;
    if vib_from_arm {
        b.rotor_vib = vib;
        vib_on = true;
    }
    let mut max_tilt_land = 0.0f32;
    let mut max_wz_land = 0.0f32;
    let mut nan_at: Option<(f32, &'static str)> = None;
    let mut disarmed = false;
    let mut shadow = [0.0f32; 4];
    let flight_ticks = (25.0 * hz) as u64;
    let land_limit = (60.0 * hz) as u64;
    loop {
        if land_tick.is_none() && tick == flight_ticks {
            sup.command(Event::RequestLand, true, false);
            land_tick = Some(tick);
        }
        if let Some(lt) = land_tick {
            if tick - lt >= land_limit {
                break;
            }
        }
        sup.step(&mut b);
        tick += 1;
        if !vib_on && sup.mode() == Mode::Loiter {
            b.rotor_vib = vib;
            vib_on = true;
        }
        let alt = -b.pos[2];
        if nan_at.is_none() && !(b.pos[0].is_finite() && b.omega[2].is_finite()) {
            nan_at = Some((land_tick.map(|lt| (tick - lt) as f32 / hz).unwrap_or(-1.0), "airborne-loop"));
        }
        if land_tick.is_some() {
            let t = b.tilt();
            max_tilt_land = if t.is_finite() { max_tilt_land.max(t) } else { f32::INFINITY };
            max_wz_land = max_wz_land.max(b.omega[2].abs());
        }
        if let Some(tr) = trace.as_deref_mut() {
            // per-tick from 1 s before RequestLand onwards
            if tick + hz as u64 >= flight_ticks {
                let (r2, tc, gate, res) = sup.core().fdi_diag();
                if gate && sup.core().failed_motor().is_none() {
                    for i in 0..4 {
                        shadow[i] = (shadow[i] + res[i] - 0.1).max(0.0);
                    }
                }
                let rpm = b.read_motor_rpm().unwrap_or([0; 4]);
                tr.push(format!(
                    "{:.4} {:?} alt={:.3} vz={:.3} tilt={:.3} wtrue=[{:.3},{:.3}] rp2={:.3} tcos={:.3} gate={} res=[{:.3},{:.3},{:.3},{:.3}] cus=[{:.3},{:.3},{:.3},{:.3}] mot=[{:.3},{:.3},{:.3},{:.3}] fm={:?}",
                    (tick as f32 - flight_ticks as f32) / hz,
                    sup.mode(), alt, b.vel[2], b.tilt(), b.omega[0], b.omega[1], r2, tc, gate as u8,
                    res[0], res[1], res[2], res[3], shadow[0], shadow[1], shadow[2], shadow[3],
                    rpm[0] as f32 / 8000.0, rpm[1] as f32 / 8000.0, rpm[2] as f32 / 8000.0, rpm[3] as f32 / 8000.0,
                    sup.core().failed_motor()
                ));
            }
        }
        if iso.is_none() {
            if let Some(m) = sup.core().failed_motor() {
                let mode = sup.mode();
                iso = Some(Iso {
                    t_since_takeoff: tick as f32 / hz,
                    t_since_land: land_tick.map(|lt| (tick - lt) as f32 / hz).unwrap_or(-1.0),
                    mode,
                    rotor: m,
                    alt,
                    vz: b.vel[2],
                    phase: phase_of(mode, alt, land_tick.is_some()),
                });
                if trace.is_some() && land_tick.is_some() {
                    // keep 0.3 s after isolation then stop the trace
                }
            }
        }
        if land_tick.is_some() && sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
        if land_tick.is_none() && sup.mode() == Mode::Disarmed {
            // a rotor-out Land that already disarmed before the land command
            disarmed = true;
            break;
        }
    }
    let land_secs = land_tick.map(|lt| (tick - lt) as f32 / hz).unwrap_or(-1.0);
    let wz_at_disarm = b.omega[2];
    b.rotor_vib = 0.0;
    let mut wz_max_ground = 0.0f32;
    for k in 0..(5.0 * hz) as usize {
        sup.step(&mut b);
        wz_max_ground = wz_max_ground.max(b.omega[2].abs());
        if nan_at.is_none() && !(b.pos[0].is_finite() && b.omega[2].is_finite()) {
            nan_at = Some((k as f32 / hz, "after-disarm"));
        }
    }
    sup.command(Event::Arm, true, true);
    let rearm_mode = sup.mode();
    let blocked = sup.arm_blocked_check();
    use falcon_core::relay_preflight::CheckId as C;
    let ct = sup.check_table();
    let rows = format!(
        "motorsOK={} fenceOK={} estOK={} truexy=[{:.2},{:.2}] estxy=[{:.2},{:.2}]",
        ct.passed(C::MotorsHealthy) as u8, ct.passed(C::InsideGeofence) as u8, ct.passed(C::EstimatorConverged) as u8,
        b.pos[0], b.pos[1], sup.core().state().p[0], sup.core().state().p[1]
    );
    let flipped = !(max_tilt_land < 1.0);
    let end = if !disarmed {
        if flipped { "flipped-not-disarmed" } else { "not-disarmed" }
    } else if flipped {
        "disarmed-after-flip"
    } else if rearm_mode == Mode::Armed {
        "landed+disarmed+rearm-ok"
    } else {
        "landed+disarmed+REARM-REFUSED"
    };
    let iso_s = match &iso {
        None => "iso=none".to_string(),
        Some(i) => format!(
            "iso=m{} phase={} mode={:?} t_to={:.3} t_land={:.3} alt={:.3} vz={:.3}",
            i.rotor, i.phase, i.mode, i.t_since_takeoff, i.t_since_land, i.alt, i.vz
        ),
    };
    format!(
        "CELL hz={hz} vib={vib} seed={seed} end={end} blocked={:?} {rows} wz_disarm={wz_at_disarm:.2} wzmax_land={max_wz_land:.2} wzmax_ground={wz_max_ground:.3e} nan={} land_s={land_secs:.2} maxtilt_land={max_tilt_land:.3} {iso_s}",
        blocked,
        match nan_at { None => "none".to_string(), Some((t, w)) => format!("{w}@{t:.3}") }
    )
}

#[test]
fn probe() {
    let out = match std::env::var("PROBE_OUT") {
        Ok(o) => o,
        Err(_) => return, // inert unless asked
    };
    let perturb: usize = std::env::var("PROBE_PERTURB").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let vfa = std::env::var("PROBE_VIB_FROM_ARM").is_ok();
    let mode = std::env::var("PROBE_MODE").unwrap_or_else(|_| "grid".into());
    if mode == "trace" {
        let spec = std::env::var("PROBE_TRACE").unwrap();
        let v: Vec<&str> = spec.split(',').collect();
        let (hz, vib, seed) = (v[0].parse().unwrap(), v[1].parse().unwrap(), v[2].parse().unwrap());
        let mut tr = Vec::new();
        let line = run(hz, vib, seed, perturb, vfa, Some(&mut tr));
        let mut f = std::fs::File::create(out).unwrap();
        writeln!(f, "{line}").unwrap();
        for l in tr {
            writeln!(f, "{l}").unwrap();
        }
        return;
    }
    let hzs: Vec<f32> = std::env::var("PROBE_HZ").map(|s| s.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![250.0, 400.0, 1000.0]);
    let vibs: Vec<f32> = std::env::var("PROBE_VIBS").map(|s| s.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![0.0, 0.1, 0.25, 0.3]);
    let mut jobs = Vec::new();
    for &hz in &hzs {
        for &vib in &vibs {
            for seed in 0..24u32 {
                jobs.push((hz, vib, seed));
            }
        }
    }
    let n = jobs.len();
    let jobs = std::sync::Arc::new(std::sync::Mutex::new(jobs));
    let res = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let nt = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4).saturating_sub(1).max(1);
    let mut hs = Vec::new();
    for _ in 0..nt {
        let (j, r) = (jobs.clone(), res.clone());
        hs.push(std::thread::spawn(move || loop {
            let Some((hz, vib, seed)) = j.lock().unwrap().pop() else { break };
            let l = run(hz, vib, seed, perturb, vfa, None);
            r.lock().unwrap().push(l);
        }));
    }
    for h in hs {
        h.join().unwrap();
    }
    let mut r = res.lock().unwrap().clone();
    r.sort();
    assert_eq!(r.len(), n);
    let mut f = std::fs::File::create(out).unwrap();
    for l in r {
        writeln!(f, "{l}").unwrap();
    }
}
