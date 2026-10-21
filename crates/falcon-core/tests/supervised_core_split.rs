//! THE SUPERVISOR OVER SEPARATELY-OWNED HALVES IS THE SAME VEHICLE (#575).
//!
//! `FlightSupervisor` is generic over `SupervisedCore` so the `cascade-split`
//! deployment can supervise an estimator and a control partition that live in
//! two other components. The wasm-level proof of that is the twin differential
//! in `tests/cascade-sitl-wasm` (bench-only: it needs the component toolchain).
//! This is its in-crate counterpart, which `cargo test` runs everywhere.
//!
//! It builds the two halves the way the split components do — from the `pub`
//! constructors, `FlightCore::warmup_steps` and the `SUPERVISOR_CORE_*`
//! constants — drives them through the trait with the split's semantics (the
//! `NavState` copied across the boundary, the in-flight edge passed as an
//! `Option` that is `None` on estimate-only ticks), and requires the result to
//! match a stock `FlightSupervisor` BIT FOR BIT through arming, takeoff, a
//! mission, the mission-complete RTL, the landing, the touchdown disarm and the
//! estimate-only ticks after it.
//!
//! What it does NOT cover, and the twin differential does: the WIT seam itself
//! (sanitising, the frame round-trip, the published state).

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{
    CascadePartition, EstimatorPartition, FlightBackend, FlightCore, FlightSupervisor, NavState,
    SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ, SimBackend, SupervisedCore,
};

/// The two partitions, owned separately — the shape `cascade-split` has, minus
/// the component boundary.
struct Halves {
    est: EstimatorPartition,
    casc: CascadePartition,
}

impl Halves {
    fn new() -> Self {
        let warmup = FlightCore::warmup_steps(SUPERVISOR_CORE_LOOP_HZ);
        Halves {
            est: EstimatorPartition::new(warmup),
            casc: CascadePartition::new(
                SUPERVISOR_CORE_HOVER_THRUST,
                SUPERVISOR_CORE_LOOP_HZ,
                warmup,
            ),
        }
    }

    /// The estimator side of the seam: `in_flight` is `Some` on a control tick
    /// and `None` on an estimate-only tick, exactly as the WIT `tick` takes it.
    fn est_tick<B: FlightBackend>(
        &mut self,
        b: &mut B,
        in_flight: Option<bool>,
        raw: falcon_core::ImuSample,
        heading: Option<f32>,
        dt: f32,
    ) -> NavState {
        if let Some(f) = in_flight {
            self.est.set_in_flight(f);
        }
        self.est.step(b, raw, heading, dt)
    }
}

impl SupervisedCore for Halves {
    fn state(&self) -> NavState {
        self.est.state()
    }
    fn tilt_uncertainty(&self) -> f32 {
        self.est.tilt_uncertainty()
    }
    fn calibration(&self) -> falcon_core::relay_calib::CalParams {
        // Neither half is ever given a calibration here, as across the seam.
        falcon_core::relay_calib::CalParams::identity()
    }
    fn nav_compromised(&self) -> bool {
        self.est.nav_compromised()
    }
    fn gnss_diverged(&self) -> bool {
        self.est.gnss_diverged()
    }
    fn failed_motor(&self) -> Option<usize> {
        self.casc.failed_motor()
    }
    fn max_motor(&self) -> f32 {
        self.casc.max_motor()
    }
    fn set_landing(&mut self, on: bool) {
        self.casc.set_landing(on)
    }
    fn set_position(&mut self, ned: [f32; 3]) {
        self.casc.set_position(ned)
    }
    fn step<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let raw = b.read_imu();
        let heading = b.read_heading();
        // The feedback edge, read BEFORE the estimator ticks (#452).
        let flying = self.casc.flying();
        let ns = self.est_tick(b, Some(flying), raw, heading, dt);
        // Copied across the boundary, as the seam does.
        let across: NavState = ns;
        self.casc.step(b, &across, raw, heading.is_some(), dt);
    }
    fn step_estimate_only<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let raw = b.read_imu();
        let heading = b.read_heading();
        let _ = self.est_tick(b, None, raw, heading, dt);
        self.casc.keep_warm(raw);
        b.write_motors(&[0.0, 0.0, 0.0, 0.0]);
    }
}

fn same_bits(a: &[f32], b: &[f32]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

#[test]
fn a_supervisor_over_split_halves_is_bit_exact_with_the_fused_one() {
    let dt = 0.004;
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant_f = SimBackend::new(identity, dt);
    let mut plant_s = SimBackend::new(identity, dt);

    let (home, fence, cruise, low) = ([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    let mut fused = FlightSupervisor::new(home, fence, cruise, low);
    let mut split = FlightSupervisor::with_core(Halves::new(), home, fence, cruise, low);

    for s in [&mut fused as &mut dyn Cmd, &mut split] {
        s.cmd(Event::Arm);
        s.cmd(Event::RequestTakeoff);
    }

    let mut saw = [false; 8];
    let mut was_airborne = false;
    let mut ground_ticks = 0u32;
    let mut mission_sent = false;
    let mut loiter = 0u32;
    for tick in 0..30_000u32 {
        fused.step(&mut plant_f);
        split.step(&mut plant_s);

        let (nf, ns) = (fused.state(), split.state());
        let flat = |n: &NavState| {
            let mut v = Vec::with_capacity(16);
            v.extend_from_slice(&n.q);
            v.extend_from_slice(&n.v);
            v.extend_from_slice(&n.p);
            v.extend_from_slice(&n.b_g);
            v.extend_from_slice(&n.b_a);
            v
        };
        assert!(
            same_bits(&flat(&nf), &flat(&ns)),
            "NavState differs at tick {tick}"
        );
        assert!(
            same_bits(&plant_f.pos, &plant_s.pos) && same_bits(&plant_f.vel, &plant_s.vel),
            "plant diverged at tick {tick} — the motor commands differed"
        );
        assert_eq!(fused.mode(), split.mode(), "mode differs at tick {tick}");
        assert_eq!(
            fused.failsafe_flags(),
            split.failsafe_flags(),
            "failsafe word differs at tick {tick}"
        );
        assert_eq!(
            fused.core().failed_motor(),
            split.core().failed_motor(),
            "failed rotor differs at tick {tick}"
        );

        let mode = fused.mode();
        saw[mode as usize] = true;
        loiter = if mode == Mode::Loiter { loiter + 1 } else { 0 };
        if loiter >= 250 && !mission_sent {
            fused.set_mission([3.0, 0.0, -2.0]);
            split.set_mission([3.0, 0.0, -2.0]);
            fused.command(Event::RequestMission, true, true);
            split.command(Event::RequestMission, true, true);
            mission_sent = true;
        }
        if fused.fsm_is_airborne() {
            was_airborne = true;
        }
        if was_airborne && mode == Mode::Disarmed {
            // Estimate-only ticks after touchdown, where the estimator's
            // in-flight flag is deliberately left as the last control tick set it.
            ground_ticks += 1;
            if ground_ticks >= 500 {
                break;
            }
        }
    }
    for (m, name) in [
        (Mode::Takeoff, "takeoff"),
        (Mode::Loiter, "loiter"),
        (Mode::Mission, "mission"),
        (Mode::Rtl, "rtl"),
        (Mode::Land, "land"),
    ] {
        assert!(saw[m as usize], "the run never reached {name}");
    }
    assert!(
        ground_ticks >= 500,
        "the run never touched down and disarmed (ground ticks {ground_ticks})"
    );
}

/// `command` takes (event, level, throttle_low); the bring-up arm here is level
/// with throttle low, before the first step, as the seam harnesses do.
trait Cmd {
    fn cmd(&mut self, ev: Event);
}
impl<C: SupervisedCore> Cmd for FlightSupervisor<C> {
    fn cmd(&mut self, ev: Event) {
        self.command(ev, true, true);
    }
}

/// `fsm.is_airborne()` is not exposed; the airborne modes are.
trait Airborne {
    fn fsm_is_airborne(&self) -> bool;
}
impl<C: SupervisedCore> Airborne for FlightSupervisor<C> {
    fn fsm_is_airborne(&self) -> bool {
        matches!(
            self.mode(),
            Mode::Takeoff | Mode::Loiter | Mode::Mission | Mode::Rtl | Mode::Land
        )
    }
}
