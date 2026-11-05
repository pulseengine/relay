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
//! finding 17's per-tick common-mode subtraction. Inside the guard a
//! SPIN-UP STALL CHECK (decision "(c)", `FDI_STALL_RPM_FRAC`,
//! `FDI_STALL_CMD_MIN`, `FDI_STALL_WINDOW_S`: RPM below 5 % of full scale
//! while commanded above 20 %, for 0.1 s unbroken, all assumptions pending
//! BOM ESC startup data) isolates a FULLY stalled rotor without waiting for
//! the guard.
//!
//! Every pre-existing FDI suite starts AFTER the guard has released (a
//! settled Loiter, or a second flight), so none of them exercised it. These
//! fly the SUPERVISED spin-up from the ground, as a vehicle does.
//!
//! WHAT THIS DOES NOT COVER. SimBackend only. Its ground contact holds the
//! airframe while a WEAK (not stalled) rotor spins up until the guard
//! releases; that has not been checked on gz or on hardware. The AIRBORNE
//! stalled-rotor start is the lib test
//! `an_airborne_rotor_dead_from_the_first_tick_is_caught_by_the_stall_check`.
//! A sensorless ESC that reads zero RPM for longer than the window while it
//! starts commutating would be isolated as dead: not modelled here.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, SimBackend};

type Vec3 = [f32; 3];
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The production guard, restated so a change to it is a visible change
/// here too.
const GUARD_S: f32 = 0.3;
/// The production stall window (`FDI_STALL_WINDOW_S`), restated likewise.
const STALL_S: f32 = 0.1;

/// How the rig delivers ESC RPM telemetry.
#[derive(Debug, Clone, Copy, PartialEq)]
enum RpmFeed {
    /// Every tick (the default).
    Always,
    /// Never: `read_motor_rpm` is `None` (the seam's `motor-rpm: none`).
    Never,
    /// Only on even spin-up ticks; `None` in between (a lossy link).
    EveryOther,
}

/// SimBackend with one ASYMMETRIC ESC: from spin-up on, rotor `rotor`
/// receives its command scaled by `min(1, t / ramp_s)` (a soft-start the other
/// three ESCs do not have). Optionally a rotor is failed at spin-up tick
/// `kill_at`, or is WEAK from spin-up (its command scaled by a constant
/// gain: it spins, so it is not stalled), and the RPM feed can be lossy.
struct Asym {
    b: SimBackend,
    rotor: usize,
    ramp_s: f32,
    /// Ticks since spin-up (0 = the first flying tick); None on the ground.
    k: Option<u32>,
    kill_at: Option<(u32, usize)>,
    weak: Option<(usize, f32)>,
    rpm: RpmFeed,
    dt: f32,
    /// Rotor `i` reports 0 RPM on spin-up ticks `start[i]..until[i]` (an
    /// unpopulated telemetry bus, or an ESC that has not started
    /// commutating), while its rotor really spins. `u32::MAX` = never.
    zero_rpm: Option<([u32; 4], [u32; 4])>,
    /// `dt()` returns this on that one spin-up tick (a late frame).
    dt_spike: Option<(u32, f32)>,
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
        match (self.k, self.dt_spike) {
            (Some(k), Some((kk, d))) if k == kk => d,
            _ => self.b.dt(),
        }
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
    fly_rig(
        hz,
        ramp_ms,
        tau_ms,
        rotor,
        dead,
        None,
        RpmFeed::Always,
        after_loiter_s,
    )
}

/// [`fly`] with a weak rotor and a chosen RPM feed.
#[allow(clippy::too_many_arguments)]
fn fly_rig(
    hz: u32,
    ramp_ms: u32,
    tau_ms: u32,
    rotor: usize,
    dead: Option<usize>,
    weak: Option<(usize, f32)>,
    rpm: RpmFeed,
    after_loiter_s: f32,
) -> Flight {
    fly_full(
        hz,
        ramp_ms,
        tau_ms,
        rotor,
        dead,
        weak,
        rpm,
        after_loiter_s,
        None,
        None,
    )
    .0
}

/// [`fly_rig`] with a faked zero-RPM window (`Asym::zero_rpm`) and a late
/// frame (`Asym::dt_spike`), returning the supervisor and the rig too so a
/// test can land and re-arm.
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
    dt_spike: Option<(u32, f32)>,
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
/// must not wait out a long guard. Every rotor, rotor lag 0 and 50 ms, at 250 Hz and
/// 1 kHz: the correct rotor is isolated within 0.3 s of spin-up, with the
/// true tilt below 0.5 rad until then (the bound
/// `survives_single_rotor_failure_without_flipping` uses).
///
/// Since the stall check this is the OUTER bound (the stall window test
/// below is the tight one): measured 0.104 s / 0.101 s, tilt <= 0.046 rad.
/// Guard alone it was the guard's own tick (0.3 s), tilt <= 0.33 rad, and a
/// 0.4 s guard flipped the vehicle first.
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

/// THE STALL CHECK (#609 finding 18, decision "(c)"), on the ground: a rotor
/// dead from the first flying tick is isolated by the spin-up stall check
/// once its 0.1 s window has elapsed, not before and within 2 ticks of it,
/// every rotor, rotor lag 0 and 50 ms, at 250 Hz and 1 kHz, with the true
/// tilt below 0.1 rad. FAILS on the guard-only design (isolated at 0.3 s,
/// tilt up to 0.33 rad). The airborne case is the lib test
/// `an_airborne_rotor_dead_from_the_first_tick_is_caught_by_the_stall_check`.
#[test]
fn a_stalled_rotor_is_isolated_within_the_stall_window() {
    for hz in [250u32, 1000] {
        let window = (STALL_S * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            for r in 0..4 {
                let f = fly(hz, 0, tau, 0, Some(r), 0.0);
                eprintln!(
                    "{hz} Hz lag {tau} ms rotor {r} stalled at spin-up: iso {:?}, tilt before iso {:.3} rad",
                    f.iso, f.tilt_before_iso
                );
                let (ticks, got, _) = f.iso.unwrap_or_else(|| {
                    panic!("{hz} Hz lag {tau}: stalled rotor {r} never isolated")
                });
                assert_eq!(got, r, "{hz} Hz lag {tau}: isolated the WRONG rotor");
                assert!(
                    ticks >= window && ticks <= window + 2,
                    "{hz} Hz lag {tau} rotor {r}: isolated at tick {ticks}, window {window}"
                );
                assert!(
                    f.tilt_before_iso < 0.1,
                    "{hz} Hz lag {tau} rotor {r}: tilt {:.3}",
                    f.tilt_before_iso
                );
            }
        }
    }
}

/// The stall check must NEVER fire on a healthy 300 ms ESC soft-start (the
/// slowest ramp measured; its longest near-zero run was 68 ms against the
/// 100 ms window), at any lag, at 250/500/1000 Hz. The stall check acts only
/// inside the guard, so "never fired" is: no isolation before the guard's
/// tick. (These cells may still be isolated AFTER the guard, by the
/// residual: that is the separate known limit below.)
#[test]
fn a_300_ms_soft_start_never_trips_the_stall_check() {
    let mut bad = Vec::new();
    for hz in [250u32, 500, 1000] {
        let guard = (GUARD_S * hz as f32 + 0.5) as u32;
        for tau in [0u32, 20, 30, 40, 50, 80] {
            for rotor in [0usize, 2] {
                let f = fly(hz, 300, tau, rotor, None, 0.5);
                if let Some((t, r, _)) = f.iso {
                    if t < guard {
                        bad.push((hz, tau, rotor, t, r));
                    }
                }
            }
        }
    }
    assert!(
        bad.is_empty(),
        "the stall check fired on a healthy soft-start: {bad:?}"
    );
}

/// Only a FULLY stalled rotor is caught early. A rotor that spins with 40 %
/// effectiveness from spin-up is not stalled; the residual catches it, and
/// only once the 0.3 s guard has released (the guard timing the old lib
/// test asserted for a dead rotor). Both rates, lag 0 and 50 ms.
#[test]
fn a_weak_rotor_is_left_to_the_guard() {
    for hz in [250u32, 1000] {
        let guard = (GUARD_S * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            let f = fly_rig(hz, 0, tau, 0, None, Some((3, 0.4)), RpmFeed::Always, 0.0);
            eprintln!(
                "{hz} Hz lag {tau} ms rotor 3 at 40 %: iso {:?}, tilt before iso {:.3}",
                f.iso, f.tilt_before_iso
            );
            let (ticks, got, _) = f
                .iso
                .unwrap_or_else(|| panic!("{hz} Hz lag {tau}: weak rotor never isolated"));
            assert_eq!(got, 3, "{hz} Hz lag {tau}: wrong rotor");
            assert!(
                ticks >= guard && ticks <= guard + (0.02 * hz as f32) as u32,
                "{hz} Hz lag {tau}: isolated at tick {ticks}, the guard releases at {guard}"
            );
        }
    }
}

/// MISSING RPM is not evidence of a stall. (1) No RPM at all
/// (`motor-rpm: none` every tick) with a rotor dead from spin-up: nothing
/// is isolated (the FDI is inert without telemetry, as before). (2) A lossy
/// feed, RPM present only every other tick: each missing tick resets the
/// stall runs, so the stall check never completes its window and the dead
/// rotor is isolated by the residual at the guard instead, never before it.
#[test]
fn missing_rpm_never_trips_the_stall_check() {
    for hz in [250u32, 1000] {
        let guard = (GUARD_S * hz as f32 + 0.5) as u32;
        let none = fly_rig(hz, 0, 0, 0, Some(2), None, RpmFeed::Never, 0.0);
        eprintln!("{hz} Hz no RPM, rotor 2 dead: iso {:?}", none.iso);
        assert_eq!(none.iso, None, "{hz} Hz: isolated with NO RPM telemetry");
        let lossy = fly_rig(hz, 0, 0, 0, Some(2), None, RpmFeed::EveryOther, 0.0);
        eprintln!(
            "{hz} Hz RPM every other tick, rotor 2 dead: iso {:?}",
            lossy.iso
        );
        if let Some((t, r, _)) = lossy.iso {
            assert!(
                t >= guard,
                "{hz} Hz: lossy RPM isolated at tick {t} < guard {guard}: the stall check fired on gaps"
            );
            assert_eq!(r, 2, "{hz} Hz: wrong rotor");
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

/// REVIEW OF v1.141, FINDING 2: COMMON-MODE STALL. All four rotors report
/// 0 RPM while commanded and spinning (a telemetry bus that is up but not
/// yet populated; bidirectional DShot before the ESCs commutate), for 0.2 s
/// of the spin-up (longer than the 0.1 s stall window, shorter than the
/// 0.3 s guard) and for the whole flight. A stall every rotor shares is not
/// a single-rotor fault: no rotor may be isolated, the takeoff must reach
/// Loiter, and after Land and Disarm the vehicle must re-arm.
///
/// FAILS ON 5020977: the stall loop isolated rotor 0 (checked first) at the
/// window, permanently, and `MotorsHealthy` then refused to arm.
#[test]
fn a_common_mode_zero_rpm_isolates_no_rotor_and_rearms() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        let s = |x: f32| (x * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            for until in [s(0.2), u32::MAX] {
                let (f, mut sup, mut a) = fly_full(
                    hz,
                    0,
                    tau,
                    0,
                    None,
                    None,
                    RpmFeed::Always,
                    1.0,
                    Some(([0; 4], [until; 4])),
                    None,
                );
                eprintln!("{hz} Hz lag {tau} ms all four 0 RPM until {until}: {f:?}");
                if f.iso.is_some() || !f.loiter {
                    bad.push(format!(
                        "{hz} Hz lag {tau} until {until}: iso {:?} loiter {}",
                        f.iso, f.loiter
                    ));
                    continue;
                }
                a.k = None;
                sup.command(Event::RequestLand, true, false);
                let mut disarmed = false;
                for _ in 0..s(60.0) {
                    sup.step(&mut a);
                    if sup.mode() == Mode::Disarmed {
                        disarmed = true;
                        break;
                    }
                }
                for _ in 0..s(5.0) {
                    sup.step(&mut a);
                }
                let blocked = sup.arm_blocked_check();
                let failed = sup.core().failed_motor();
                eprintln!(
                    "  landed: disarmed {disarmed}, failed {failed:?}, arm blocked {blocked:?}"
                );
                if !disarmed || failed.is_some() || blocked.is_some() {
                    bad.push(format!(
                        "{hz} Hz lag {tau} until {until}: after landing disarmed {disarmed} failed {failed:?} blocked {blocked:?}"
                    ));
                }
            }
        }
    }
    assert!(
        bad.is_empty(),
        "a stall common to all four rotors isolated a rotor or blocked re-arm: {bad:?}"
    );
}

/// FINDING 2, STAGGERED: the common-mode stall need not start on the same
/// tick for every rotor. Rotor 0 reads 0 RPM from spin-up, rotors 1-3 from
/// spin-up tick 3 (lag 0, so they really spin), for 0.2 s and for the whole
/// flight. A rule that only refuses when several rotors have FILLED the
/// window would still isolate rotor 0 (it fills first, alone); the rule is
/// "no other rotor is stalled NOW". FAILS ON 5020977 (rotor 0 isolated).
#[test]
fn a_staggered_common_mode_stall_isolates_no_rotor() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        let s = |x: f32| (x * hz as f32 + 0.5) as u32;
        for until in [s(0.2), u32::MAX] {
            let (f, _, _) = fly_full(
                hz,
                0,
                0,
                0,
                None,
                None,
                RpmFeed::Always,
                1.0,
                Some(([0, 3, 3, 3], [until; 4])),
                None,
            );
            eprintln!("{hz} Hz staggered 0 RPM until {until}: {f:?}");
            if f.iso.is_some() || !f.loiter {
                bad.push((hz, until, f.iso, f.loiter));
            }
        }
    }
    assert!(
        bad.is_empty(),
        "a staggered common-mode stall isolated a rotor: {bad:?}"
    );
}

/// FINDING 2, NEGATIVE CONTROL: the common-mode rule must not swallow a
/// real single dead rotor whose healthy siblings are briefly near zero too.
/// Rotor `r` dead from spin-up, rotor `r+1` a HEALTHY 100 ms ESC soft-start
/// (it reads near zero while commanded for the first tens of ms, a
/// co-stall), rotor lag 0 and 50 ms, 250 Hz and 1 kHz: the dead rotor is
/// still isolated at the stall window, within 2 ticks, tilt below 0.1 rad.
/// (The plain ground and airborne controls are
/// `a_stalled_rotor_is_isolated_within_the_stall_window` and the lib test
/// `an_airborne_rotor_dead_from_the_first_tick_is_caught_by_the_stall_check`,
/// both unchanged.)
#[test]
fn a_stalled_rotor_beside_a_slow_healthy_rotor_is_still_isolated_in_the_window() {
    for hz in [250u32, 1000] {
        let window = (STALL_S * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            for r in 0..4 {
                let slow = (r + 1) % 4;
                let f = fly(hz, 100, tau, slow, Some(r), 0.0);
                eprintln!(
                    "{hz} Hz lag {tau} ms rotor {r} dead, rotor {slow} 100 ms soft-start: iso {:?}, tilt {:.3}",
                    f.iso, f.tilt_before_iso
                );
                let (ticks, got, _) = f
                    .iso
                    .unwrap_or_else(|| panic!("{hz} Hz lag {tau}: dead rotor {r} never isolated"));
                assert_eq!(got, r, "{hz} Hz lag {tau}: isolated the WRONG rotor");
                assert!(
                    ticks >= window && ticks <= window + 2,
                    "{hz} Hz lag {tau} rotor {r}: isolated at tick {ticks}, window {window}"
                );
                assert!(
                    f.tilt_before_iso < 0.1,
                    "{hz} Hz lag {tau} rotor {r}: tilt {:.3}",
                    f.tilt_before_iso
                );
            }
        }
    }
}

/// REVIEW OF v1.141, FINDING 3: ONE LATE FRAME. A healthy rotor 2 reports 0
/// RPM for its first 60 ms (inside the 0.1 s window), lag 0. One frame whose
/// `dt` is twice the loop period, at 55 ms, makes the core resync to half
/// the rate, which halves the window in ticks; the in-progress stall run,
/// still counted in old-rate ticks, must not then exceed it. 1 kHz (one
/// 2 ms frame at spin-up tick 55) and 250 Hz (one 8 ms frame at tick 14):
/// no isolation, Loiter reached.
///
/// FAILS ON 5020977: rotor 2 was isolated on the late frame, permanently.
#[test]
fn a_late_frame_does_not_shorten_the_stall_window() {
    let mut bad = Vec::new();
    for (hz, zero_until, spike_k) in [(1000u32, 60u32, 55u32), (250, 15, 14)] {
        let dt_spike = 2.0 / hz as f32;
        let (f, _, _) = fly_full(
            hz,
            0,
            0,
            0,
            None,
            None,
            RpmFeed::Always,
            1.0,
            Some(([u32::MAX, u32::MAX, 0, u32::MAX], [zero_until; 4])),
            Some((spike_k, dt_spike)),
        );
        eprintln!(
            "{hz} Hz rotor 2 0 RPM for {zero_until} ticks, one {:.0} ms frame at tick {spike_k}: {f:?}",
            dt_spike * 1000.0
        );
        if f.iso.is_some() || !f.loiter {
            bad.push((hz, f.iso, f.loiter));
        }
    }
    assert!(
        bad.is_empty(),
        "one late frame isolated a healthy rotor: {bad:?}"
    );
}

/// FINDING 2, THE SHARED WINDOW IS DISCARDED: all four rotors read 0 RPM
/// from spin-up; rotors 1-3 recover at 0.15 s, rotor 0 at 0.2 s (one ESC
/// slower to report). Rotor 0 then stands alone with a run already past the
/// 0.1 s window, but that window filled while the stall was shared, so it
/// must earn a fresh one on its own (it is alone for only 50 ms here): no
/// isolation, Loiter reached. Lag 0 and 50 ms, 250 Hz and 1 kHz. Without
/// the `fdi_stall_common` reset rotor 0 is isolated the tick the others
/// recover.
#[test]
fn a_rotor_left_silent_after_a_shared_stall_must_earn_its_own_window() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        let s = |x: f32| (x * hz as f32 + 0.5) as u32;
        for tau in [0u32, 50] {
            let (f, _, _) = fly_full(
                hz,
                0,
                tau,
                0,
                None,
                None,
                RpmFeed::Always,
                1.0,
                Some(([0; 4], [s(0.2), s(0.15), s(0.15), s(0.15)])),
                None,
            );
            eprintln!("{hz} Hz lag {tau} ms shared stall, rotor 0 silent 50 ms longer: {f:?}");
            if f.iso.is_some() || !f.loiter {
                bad.push((hz, tau, f.iso, f.loiter));
            }
        }
    }
    assert!(
        bad.is_empty(),
        "a window filled during a shared stall isolated the last rotor to recover: {bad:?}"
    );
}

/// KNOWN LIMIT (c), kept visible (review of v1.140, defect C; maintainer:
/// pin and disclose, fix in v1.142): a rotor dead from the first tick of an
/// AIRBORNE start, beside a HEALTHY sibling whose RPM telemetry reads 0 for
/// the whole flight (an unpopulated bus slot, a dead telemetry wire), is
/// NEVER isolated. Two rotors read stalled on every tick, so the stall check
/// takes it for a common-mode stall and discards every window
/// (`fdi_stall_common`), and once the guard releases the residual path is
/// shut by the rate gate as the vehicle rolls off. Every (dead, silent)
/// ordered pair, at 250 Hz and 1 kHz, built as the supervisor builds the
/// core (nominal 1 kHz), hovering 2 m up: no isolation in 2 s, and the
/// vehicle falls (printed, not asserted). POSITIVE CONTROL on the same rig:
/// with every sibling reporting, the same dead rotor IS isolated at the
/// stall window (so the pin is the silent sibling, not the rig). This is a DOUBLE fault (one rotor
/// dead, one telemetry channel dead), outside the single-rotor isolation;
/// v1.142 is to disambiguate co-stalled rotors by the body-rate / attitude
/// response. When a change fixes it, this test fails and the limit is
/// retired here and in FV-FALCON-FAULT-004.
#[test]
fn known_limit_c_dead_rotor_beside_a_silent_sibling_is_not_isolated_airborne() {
    use falcon_core::{FlightCore, SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ};
    let mut isolated = Vec::new();
    let mut control_missed = Vec::new();
    for hz in [250u32, 1000] {
        let dt = 1.0 / hz as f32;
        for dead in 0..4usize {
            // silent == 4: no silent sibling (the positive control).
            for silent in 0..5usize {
                if silent == dead {
                    continue;
                }
                let mut b = SimBackend::new(LEVEL, dt);
                b.rpm_telemetry = true;
                b.pos = [0.0, 0.0, -2.0];
                let mut start = [u32::MAX; 4];
                if silent < 4 {
                    start[silent] = 0;
                }
                let mut a = Asym {
                    b,
                    rotor: 0,
                    ramp_s: 0.0,
                    k: None,
                    kill_at: Some((0, dead)),
                    weak: None,
                    rpm: RpmFeed::Always,
                    dt,
                    zero_rpm: Some((start, [u32::MAX; 4])),
                    dt_spike: None,
                };
                let mut core =
                    FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
                core.set_position([0.0, 0.0, -2.0]);
                let mut iso = None;
                let mut tilt = 0.0f32;
                for k in 0..(2 * hz) {
                    a.k = Some(k);
                    core.step(&mut a);
                    tilt = tilt.max(a.b.tilt());
                    if iso.is_none() {
                        iso = core.failed_motor().map(|r| (k + 1, r));
                    }
                }
                let sib = if silent < 4 {
                    format!("rotor {silent} RPM 0")
                } else {
                    "every sibling reporting".to_string()
                };
                eprintln!(
                    "{hz} Hz rotor {dead} dead, {sib}: iso {iso:?}, peak tilt {tilt:.2} rad, alt {:.1} m after 2 s",
                    -a.b.pos[2]
                );
                if silent == 4 {
                    if iso.map(|i| i.1) != Some(dead) {
                        control_missed.push((hz, dead, iso));
                    }
                } else if iso.is_some() {
                    isolated.push((hz, dead, silent, iso));
                }
            }
        }
    }
    assert!(
        control_missed.is_empty(),
        "positive control: with every sibling reporting the dead rotor must be isolated: {control_missed:?}"
    );
    assert!(
        isolated.is_empty(),
        "a dead rotor beside a silent sibling WAS isolated: known limit (c) may be fixed, retire it: {isolated:?}"
    );
}

/// A LONG FIRST FRAME (a host's first tick after start-up, or the
/// cascade's first flying tick, 20 / 50 ms) makes the core resync every
/// fixed-time threshold to 50 / 20 Hz on that frame. If a rate seeded by
/// that one frame were held for many ticks, the stall window would be only
/// 2-5 ticks long, and a healthy ESC soft-starting behind the others is
/// alone below 5 % RPM for longer than that. Regression guard for any change
/// to how the rate is resynced: 250 Hz and 1 kHz, the cascade's first
/// flying frame 20 or 50 ms, healthy rotor soft-starts of 100/200/150 ms
/// (lag 50/50/0 ms): no isolation, Loiter reached. Passes on 725f146 (the
/// per-frame resync re-judges the rate on the next frame); not fail-first.
#[test]
fn a_long_first_frame_isolates_no_healthy_rotor() {
    let mut bad = Vec::new();
    for hz in [250u32, 1000] {
        for d in [0.02f32, 0.05] {
            for (ramp, tau, rotor) in [(100u32, 50u32, 2usize), (200, 50, 1), (150, 0, 3)] {
                let (f, _, _) = fly_full(
                    hz,
                    ramp,
                    tau,
                    rotor,
                    None,
                    None,
                    RpmFeed::Always,
                    1.0,
                    None,
                    Some((0, d)),
                );
                eprintln!(
                    "{hz} Hz first flying frame {:.0} ms, rotor {rotor} {ramp} ms soft-start, lag {tau}: {f:?}",
                    d * 1000.0
                );
                if f.iso.is_some() || !f.loiter {
                    bad.push((hz, d, ramp, rotor, f.iso, f.loiter));
                }
            }
        }
    }
    assert!(
        bad.is_empty(),
        "a long first frame isolated a healthy rotor: {bad:?}"
    );
}
