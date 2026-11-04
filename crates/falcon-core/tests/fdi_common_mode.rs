//! #609 finding 17: the rotor-out FDI must reject a COMMON-MODE residual.
//!
//! THE DEFECT. The FDI residual is |previous command − achieved| per rotor,
//! command-aligned for ONE tick of telemetry lag (FV-FALCON-FAULT-004). A real
//! ESC/rotor lags its command by a first-order time constant of tens of
//! milliseconds, so any abrupt COLLECTIVE step leaves all four rotors behind
//! their commands by the same amount. The supervised landing makes exactly
//! that step: when the velocity landing law engages (a few ticks into Land,
//! ~2 m above ground, NOT at touchdown) the collective drops from hover
//! (~0.5) to ~0.3. Measured at tau = 50 ms: every rotor's residual was
//! 0.194, 0.186, 0.179, ... (equal to 1e-6), the CUSUM crossed on tick 7, and
//! rotor 0 was isolated only because it is checked first. The rest of the
//! landing was flown on the degraded 3-rotor law and `MotorsHealthy` refused
//! every later arm. Every healthy supervised landing at tau >= 40 ms did this.
//!
//! THE FIX. A single rotor out is DIFFERENTIAL: one rotor's residual rises,
//! the other three stay near zero. Subtracting the per-tick MINIMUM residual
//! removes what all four share and leaves a single-rotor fault untouched.
//!
//! What must still hold: a dead rotor is isolated, the CORRECT one, within
//! the FAULT-P03 latency, in a hover AND during the landing step, under lag,
//! and also when the rotor dies gradually rather than instantly.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, SimBackend};

type Vec3 = [f32; 3];

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const LAGS: [f32; 6] = [0.0, 0.02, 0.03, 0.04, 0.05, 0.08];

/// SimBackend with a GRADUALLY dying rotor: from `kill()` on, rotor `rotor`
/// receives its command scaled by a factor that decays exponentially with
/// time constant `decay_s` (a failing ESC / motor losing thrust), so its
/// reported RPM sinks gradually instead of dropping to 0 in one tick. (Dual
/// GNSS is not forwarded: `plant` never enables it.)
struct Decaying {
    b: SimBackend,
    rotor: usize,
    decay_s: f32,
    gain: Option<f32>,
}

impl Decaying {
    fn kill(&mut self) {
        self.gain = Some(1.0);
    }
}

impl FlightBackend for Decaying {
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
        if let Some(g) = self.gain.as_mut() {
            *g *= 1.0 - DT / self.decay_s;
            m[self.rotor] *= *g;
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

fn plant(tau: f32) -> SimBackend {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    b.rpm_telemetry = true;
    b.motor_tau = tau;
    b
}

/// A calibrated supervisor in a settled 2 m Loiter on `b`, no rotor isolated.
fn loitering<B: FlightBackend>(b: &mut B) -> FlightSupervisor {
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    for _ in 0..2500 {
        sup.step(b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{:?}", sup.arm_blocked_check());
    for _ in 0..15_000 {
        sup.step(b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter, "never reached Loiter");
    for _ in 0..5000 {
        sup.step(b);
    }
    assert_eq!(sup.core().failed_motor(), None, "isolated before the test");
    sup
}

/// Healthy supervised landings, six rotor lags: no rotor is ever isolated,
/// and the vehicle re-arms afterwards. FAILED before the fix at 40, 50 and
/// 80 ms (rotor 0 isolated 6-9 ticks into Land, re-arm refused by
/// MotorsHealthy).
#[test]
fn a_healthy_landing_under_rotor_lag_isolates_no_rotor() {
    let mut bad = Vec::new();
    for tau in LAGS {
        let mut b = plant(tau);
        let mut sup = loitering(&mut b);
        sup.command(Event::RequestLand, true, false);
        let mut phantom = None;
        for i in 0..30_000u32 {
            sup.step(&mut b);
            if phantom.is_none() {
                if let Some(r) = sup.core().failed_motor() {
                    phantom = Some((i, r, -sup.core().state().p[2]));
                }
            }
            if sup.mode() == Mode::Disarmed {
                break;
            }
        }
        assert_eq!(sup.mode(), Mode::Disarmed, "tau {tau}: never touched down");
        for _ in 0..2500 {
            sup.step(&mut b);
        }
        sup.command(Event::Arm, true, true);
        eprintln!(
            "tau {tau}: phantom isolation {phantom:?}, re-arm {:?} ({:?})",
            sup.mode(),
            sup.arm_blocked_check()
        );
        if phantom.is_some() || sup.mode() != Mode::Armed {
            bad.push((tau, phantom, sup.arm_blocked_check()));
        }
    }
    assert!(
        bad.is_empty(),
        "phantom isolation / refused re-arm: {bad:?}"
    );
}

/// Kill each rotor at each lag, in a settled Loiter and at the Land step
/// (the collective drop the common-mode residual comes from): the CORRECT
/// rotor is isolated within the measured bounds (Loiter 3 ticks; Land entry
/// <= 12 ticks, 24 ms, against FAULT-P03's 25-step bound).
#[test]
fn a_dead_rotor_is_isolated_correctly_and_fast_under_lag() {
    for tau in LAGS {
        for (what, land, bound) in [("loiter", false, 3u32), ("land entry", true, 12)] {
            for rotor in 0..4 {
                let mut b = plant(tau);
                let mut sup = loitering(&mut b);
                if land {
                    sup.command(Event::RequestLand, true, false);
                }
                b.fail_rotor(rotor);
                let mut got = None;
                for i in 1..=3000u32 {
                    sup.step(&mut b);
                    if let Some(r) = sup.core().failed_motor() {
                        got = Some((i, r));
                        break;
                    }
                }
                eprintln!("tau {tau} {what} rotor {rotor}: {got:?}");
                let (ticks, r) = got.unwrap_or_else(|| {
                    panic!("tau {tau} {what}: dead rotor {rotor} never isolated")
                });
                assert_eq!(r, rotor, "tau {tau} {what}: isolated the WRONG rotor");
                assert!(
                    ticks <= bound,
                    "tau {tau} {what} rotor {rotor}: {ticks} ticks > {bound}"
                );
            }
        }
    }
}

/// A rotor that dies GRADUALLY (its thrust decays with a 100 ms time
/// constant) is still isolated, correctly, in Loiter, with 50 ms rotor lag.
/// SimBackend's `fail_rotor` drops the RPM to 0 in one tick, which flatters
/// any residual scheme; this does not.
#[test]
fn a_gradually_dying_rotor_is_isolated_correctly() {
    for rotor in 0..4 {
        let mut d = Decaying {
            b: plant(0.05),
            rotor,
            decay_s: 0.1,
            gain: None,
        };
        let mut sup = loitering(&mut d);
        d.kill();
        let mut got = None;
        for i in 1..=3000u32 {
            sup.step(&mut d);
            if let Some(r) = sup.core().failed_motor() {
                got = Some((i, r));
                break;
            }
            assert_ne!(sup.mode(), Mode::Terminated, "rotor {rotor}: terminated");
        }
        eprintln!("gradual decay (100 ms) rotor {rotor}: isolated {got:?}");
        let (ticks, r) = got.unwrap_or_else(|| panic!("decaying rotor {rotor} never isolated"));
        assert_eq!(r, rotor, "isolated the WRONG rotor");
        // 250 ms: FAULT-P01's budget is 500 ms from onset; a 100 ms decay
        // has lost ~90% of its thrust by 230 ms.
        assert!(ticks <= 125, "rotor {rotor}: {ticks} ticks");
    }
}
