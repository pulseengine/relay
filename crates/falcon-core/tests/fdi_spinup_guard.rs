//! #609 finding 18: the rotor-out FDI's SPIN-UP GUARD against an
//! asymmetric ESC ramp, and against a rotor that is dead from the first tick.
//!
//! THE DEFECT. One ESC whose output ramps up more slowly than the other
//! three (a soft-start) gives that rotor a DIFFERENTIAL residual during
//! spin-up: it is a healthy rotor that looks, to the FDI, exactly like a
//! failing one. Finding 17's common-mode subtraction removes only what all
//! four share, so it cannot help. With a 0.2 s guard and the raw
//! command-aligned residual, rotor 2 ramping over 100 ms behind a 50 ms rotor
//! lag was isolated ~0.25 s into the FIRST takeoff at 1 kHz, and the vehicle
//! flipped.
//!
//! THE FIX (maintainer decision 2026-11-04, "T300+M50"): the guard is 0.3 s
//! (`FDI_SPINUP_GUARD_S`, fixed TIME at the resynced rate) and the residual is
//! computed against a first-order 50 ms lag model of the command
//! (`FDI_ROTOR_LAG_TAU_S`, an ASSUMPTION pending real ESC data), keeping
//! finding 17's per-tick common-mode subtraction.
//!
//! Every pre-existing FDI suite starts AFTER the guard has released (a
//! settled Loiter, or a second flight), so none of them exercised it. These
//! fly the SUPERVISED spin-up from the ground, as a vehicle does.
//!
//! WHAT THIS DOES NOT COVER. SimBackend only. Its ground contact holds the
//! airframe while a dead rotor spins up; that is what keeps the dead-at-
//! spin-up tilt bounded for 0.3 s, and it has not been checked on gz or on
//! hardware. An AIRBORNE start is a different case: see the lib test
//! `the_fdi_engages_after_a_fixed_time_at_250_hz_and_1_khz`.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, SimBackend};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The production guard, restated so a change to it is a visible change
/// here too.
const GUARD_S: f32 = 0.3;

/// SimBackend with one ASYMMETRIC ESC: from spin-up on, rotor `rotor`
/// receives its command scaled by `min(1, t / ramp_s)` (a soft-start the other
/// three ESCs do not have). Optionally a rotor is failed at spin-up tick
/// `kill_at`.
struct Asym {
    b: SimBackend,
    rotor: usize,
    ramp_s: f32,
    /// Ticks since spin-up (0 = the first flying tick); None on the ground.
    k: Option<u32>,
    kill_at: Option<(u32, usize)>,
    dt: f32,
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
        }
        self.b.write_motors(&m);
    }
    fn dt(&self) -> f32 {
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
        self.b.read_motor_rpm()
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct Flight {
    /// (spin-up ticks counted as the FDI counts them, i.e. 1 on the first
    /// flying tick; rotor; mode then)
    iso: Option<(u32, usize, Mode)>,
    /// Reached Loiter (the takeoff completed).
    loiter: bool,
    /// Peak TRUE tilt (rad) over the flight.
    peak_tilt: f32,
    /// Peak true tilt (rad) from the kill to the isolation.
    tilt_before_iso: f32,
}

/// One supervised flight from the ground on a SimBackend with rotor lag
/// `tau_ms`, one ESC ramping over `ramp_ms`, optionally a rotor dead from
/// the first flying tick. Flies to Loiter + `after_loiter_s`, or, with a
/// kill, to the isolation + 0.5 s.
fn fly(
    hz: u32,
    ramp_ms: u32,
    tau_ms: u32,
    rotor: usize,
    dead: Option<usize>,
    after_loiter_s: f32,
) -> Flight {
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
        dt,
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
            if dead.is_some() {
                f.tilt_before_iso = f.tilt_before_iso.max(tilt);
            }
            if let Some(r) = sup.core().failed_motor() {
                f.iso = Some((k + 1, r, sup.mode()));
                if dead.is_some() {
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
    f
}

/// THE REGRESSION TEST. A healthy vehicle whose rotor 2 ESC soft-starts over
/// 100/150/200 ms, all rotors lagging 50 ms: no rotor is isolated and the
/// takeoff reaches Loiter, at 250 Hz and at 1 kHz.
///
/// FAILS ON THE BASE (0.2 s guard, raw residual): at 1 kHz every cell
/// isolates rotor 2 (0.25-0.26 s) and the vehicle flips; at 250 Hz the 150
/// and 200 ms ramps do (100 ms passes there).
#[test]
fn an_asymmetric_esc_spin_up_isolates_no_rotor() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        for ramp in [100u32, 150, 200] {
            let f = fly(hz, ramp, 50, 2, None, 1.0);
            eprintln!("{hz} Hz ramp {ramp} ms lag 50 ms rotor 2: {f:?}");
            if f.iso.is_some() || !f.loiter {
                bad.push((hz, ramp, f.iso, f.loiter, f.peak_tilt));
            }
        }
    }
    assert!(
        bad.is_empty(),
        "isolated a healthy, slowly-ramping rotor: {bad:?}"
    );
}

/// The other side of the guard. A rotor dead from the FIRST flying tick
/// can only be caught when the guard releases, so the guard is its
/// detection latency. Every rotor, rotor lag 0 and 50 ms, at 250 Hz and
/// 1 kHz: the correct rotor is isolated within 0.3 s of spin-up, with the
/// true tilt below 0.5 rad until then (the bound
/// `survives_single_rotor_failure_without_flipping` uses).
///
/// Measured: isolated on the guard's own tick (75 at 250 Hz, 300 at 1 kHz),
/// tilt <= 0.33 rad. A 0.4 s guard flips the vehicle first (measured in the
/// finding-18 sweep), so this is the test that holds the guard down.
#[test]
fn a_rotor_dead_from_the_first_tick_is_isolated_within_the_guard() {
    for hz in [250u32, 1000] {
        let guard = (GUARD_S * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            for r in 0..4 {
                let f = fly(hz, 0, tau, 0, Some(r), 0.0);
                eprintln!(
                    "{hz} Hz lag {tau} ms rotor {r} dead at spin-up: iso {:?} ({:.3} s), tilt before iso {:.3} rad",
                    f.iso,
                    f.iso.map(|i| i.0 as f32 / hz as f32).unwrap_or(f32::NAN),
                    f.tilt_before_iso
                );
                let (ticks, got, _) = f
                    .iso
                    .unwrap_or_else(|| panic!("{hz} Hz lag {tau}: dead rotor {r} never isolated"));
                assert_eq!(got, r, "{hz} Hz lag {tau}: isolated the WRONG rotor");
                assert!(
                    ticks <= guard,
                    "{hz} Hz lag {tau} rotor {r}: {ticks} ticks > {guard} (0.3 s)"
                );
                assert!(
                    f.tilt_before_iso < 0.5,
                    "{hz} Hz lag {tau} rotor {r}: tilted {:.3} rad before isolation",
                    f.tilt_before_iso
                );
            }
        }
    }
}

/// KNOWN LIMIT, kept visible: a 300 ms ESC soft-start is STILL isolated as
/// a rotor-out with the 0.3 s guard and the 50 ms lag model. The sweep below
/// measured 4/8/12 of 192 cells at 250/500/1000 Hz failing, every one a
/// 300 ms ramp (a ramp as long as the guard is not over when it releases).
/// This asserts the 1 kHz, 50 ms-lag cell still behaves that way; when a
/// change fixes it, this test fails and the limit is retired here, in the
/// sweep's allowlist and in FV-FALCON-FAULT-004.
#[test]
fn known_limit_a_300_ms_esc_soft_start_is_still_isolated() {
    let f = fly(1000, 300, 50, 2, None, 1.0);
    eprintln!("1000 Hz ramp 300 ms lag 50 ms rotor 2: {f:?}");
    assert_eq!(
        f.iso.map(|i| i.1),
        Some(2),
        "the 300 ms soft-start no longer isolates rotor 2: retire this known limit"
    );
}

/// The ramp x lag x rotor x rate SWEEP the guard was chosen from, as a
/// DOCUMENTED BENCH (ignored: 576 supervised flights, minutes in release).
///
/// ```text
/// cargo test --release -p falcon-core --test fdi_spinup_guard -- --ignored --nocapture
/// ```
///
/// Prints the grid per rate (rows ESC ramp ms, columns common lag ms; `ok`
/// or `R<rotor>@<s>`, `F` = tilt > 1 rad) and FAILS on any false isolation
/// outside the named known limits.
#[test]
#[ignore = "bench: 576 supervised flights; run with --ignored --release"]
fn spinup_sweep_bench() {
    const RAMPS: [u32; 8] = [0, 25, 50, 75, 100, 150, 200, 300];
    const TAUS: [u32; 6] = [0, 20, 30, 40, 50, 80];
    // (hz, ramp, tau): measured 2026-11-04 on the T300+M50 prototype.
    const KNOWN_LIMIT: [(u32, u32, u32); 6] = [
        (250, 300, 80),
        (500, 300, 0),
        (500, 300, 80),
        (1000, 300, 0),
        (1000, 300, 50),
        (1000, 300, 80),
    ];
    let mut unexpected = Vec::new();
    for hz in [250u32, 500, 1000] {
        let mut n = 0;
        for rotor in 0..4 {
            println!("hz={hz} ramping rotor={rotor}   rows=ESC ramp ms, cols=common lag ms");
            print!("  ramp\\lag");
            for t in TAUS {
                print!("{t:>12}");
            }
            println!();
            for ramp in RAMPS {
                print!("  {ramp:>8}");
                for tau in TAUS {
                    let f = fly(hz, ramp, tau, rotor, None, 3.0);
                    let cell = match f.iso {
                        None if f.loiter => "ok".to_string(),
                        None => "noLtr".to_string(),
                        Some((t, r, _)) => {
                            format!(
                                "R{r}@{:.3}{}",
                                t as f32 / hz as f32,
                                if f.peak_tilt > 1.0 { "F" } else { "" }
                            )
                        }
                    };
                    print!("{cell:>12}");
                    if f.iso.is_some() || !f.loiter {
                        n += 1;
                        if !KNOWN_LIMIT.contains(&(hz, ramp, tau)) {
                            unexpected.push((hz, rotor, ramp, tau, cell));
                        }
                    }
                }
                println!();
            }
        }
        println!("SUMMARY hz={hz}: false isolation / no Loiter {n}/192");
    }
    assert!(
        unexpected.is_empty(),
        "false isolations outside the known limits: {unexpected:?}"
    );
}
