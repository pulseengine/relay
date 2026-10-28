//! THE PRODUCTION SPLIT FORWARDING, UNDER THE SUPERVISOR, IS THE SAME VEHICLE
//! (#575).
//!
//! `cascade-split`'s supervisor is `FlightSupervisor<SplitCore<_>>`, and
//! `falcon_core::split` holds both its forwarding (`SplitCore`) and what each
//! partition component runs behind the seam (`estimator_tick`, `control_tick`,
//! ...). The orchestrator plugs WIT imports into `SplitCore`; this test plugs
//! real in-process partitions into the SAME `SplitCore`, through the SAME
//! partition-side functions, so the code under test is the production
//! forwarding rather than a copy of it. Only the WIT encoding is left to the
//! bench-only wasm twin differential (tests/cascade-sitl-wasm).
//!
//! Both supervisors are driven exactly as a component drives its own: the plant
//! is sampled ONCE per tick through the production `SeamFrame::read` and served
//! from the frame, so noise draws cannot desynchronise the two sides.
//!
//! THE SCENARIO IS BUILT TO MAKE EACH FORWARD DECISIVE:
//!
//! - `keep_warm`: the vehicle is ARMED before the first step and HELD in Armed
//!   for a while — estimate-only ticks, `keep_warm` feeding the control half's
//!   gyro low-pass with a NOISY gyro — and only then takes off, with no reset in
//!   between. At the 1000 Hz design rate the first control tick does not
//!   rebuild that filter, so its history reaches the motors.
//! - estimator health: compared DIRECTLY, every tick, through
//!   `core().tilt_uncertainty()` etc. on both supervisors, so the forward itself
//!   is measured whatever the pre-arm table makes of it.
//! - THE CALIBRATION (#586): a non-identity calibration is installed through
//!   `FlightSupervisor::set_calibration` on both supervisors — and re-installed
//!   after the rebuild, as the components do — so it must reach BOTH split
//!   halves (the control half de-biases its own copy of the gyro) and be READ
//!   BACK from the estimator for the `CalibrationPresent` row. Two mutants drop
//!   one of those each, and must diverge.
//!
//! Every negative control below is a `#[test]` that requires the mutated
//! seam to DIVERGE: a comparison that cannot fail proves nothing.

use falcon_core::relay_calib::CalParams;
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::split::{
    self, ControlStatus, EstimatorHealth, FrameBackend, PartitionSeam, SeamFrame, SplitCore,
};
use falcon_core::{
    CascadePartition, EstimatorPartition, FlightBackend, FlightSupervisor, ImuSample, NavState,
    Pathology, SimBackend, SupervisedCore,
};

/// 1000 Hz: the design rate the supervisor builds its core at, so the first
/// control tick keeps the gyro filter `keep_warm` warmed.
const DT: f32 = 0.001;
const HOME: [f32; 3] = [0.0, 0.0, 0.0];
const FENCE: f32 = 50.0;
const CRUISE: f32 = 2.0;
const LOW_BATT: f32 = 14.0;

/// The real partitions, in process, behind the production partition-side code.
struct InProcess {
    est: EstimatorPartition,
    casc: CascadePartition,
}

impl InProcess {
    fn new() -> Self {
        InProcess {
            est: split::new_estimator(),
            casc: split::new_control(),
        }
    }
}

impl PartitionSeam for InProcess {
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState {
        split::estimator_tick(&mut self.est, f, in_flight)
    }
    fn estimator_nav(&self) -> NavState {
        self.est.state()
    }
    fn estimator_health(&self) -> EstimatorHealth {
        split::estimator_health(&self.est)
    }
    fn control_tick(&mut self, nav: &NavState, f: &SeamFrame) -> [f32; 4] {
        split::control_tick(&mut self.casc, nav, f)
    }
    fn control_keep_warm(&mut self, imu: ImuSample) {
        self.casc.keep_warm(imu);
    }
    fn control_set_position(&mut self, ned: [f32; 3]) {
        self.casc.set_position(ned);
    }
    fn control_set_landing(&mut self, on: bool) {
        self.casc.set_landing(on);
    }
    fn control_status(&self) -> ControlStatus {
        split::control_status(&self.casc)
    }
    fn estimator_set_calibration(&mut self, c: CalParams) {
        split::estimator_set_calibration(&mut self.est, c);
    }
    fn estimator_calibration(&self) -> CalParams {
        split::estimator_calibration(&self.est)
    }
    fn control_set_calibration(&mut self, c: CalParams) {
        split::control_set_calibration(&mut self.casc, c);
    }
}

/// A deliberately broken seam, for the negative controls.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Mutation {
    None,
    /// The #452 edge never arrives: the estimator always hears "not flying".
    EdgeCut,
    /// Estimate-only ticks write the flag (`Some(false)`) instead of leaving it.
    EstimateOnlyClearsFlag,
    /// The control half's `keep_warm` is dropped.
    NoKeepWarm,
    /// The tilt-uncertainty forward is stubbed to 0.
    TiltStub,
    /// The supervisor is rebuilt WITHOUT rebuilding the partitions.
    NoReset,
    /// The calibration reaches the estimator but NOT the control half.
    CalibEstimatorOnly,
    /// The calibration read-back is stubbed to the identity (the pre-#586
    /// `SplitCore::calibration`), so `CalibrationPresent` reads a copy that is
    /// not what the estimator applies.
    CalibReadIdentity,
}

struct Mutant {
    inner: InProcess,
    m: Mutation,
}

impl PartitionSeam for Mutant {
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState {
        let in_flight = match (self.m, in_flight) {
            (Mutation::EdgeCut, Some(_)) => Some(false),
            (Mutation::EstimateOnlyClearsFlag, None) => Some(false),
            (_, x) => x,
        };
        self.inner.estimator_tick(f, in_flight)
    }
    fn estimator_nav(&self) -> NavState {
        self.inner.estimator_nav()
    }
    fn estimator_health(&self) -> EstimatorHealth {
        let mut h = self.inner.estimator_health();
        if self.m == Mutation::TiltStub {
            h.tilt_uncertainty = 0.0;
        }
        h
    }
    fn control_tick(&mut self, nav: &NavState, f: &SeamFrame) -> [f32; 4] {
        self.inner.control_tick(nav, f)
    }
    fn control_keep_warm(&mut self, imu: ImuSample) {
        if self.m != Mutation::NoKeepWarm {
            self.inner.control_keep_warm(imu);
        }
    }
    fn control_set_position(&mut self, ned: [f32; 3]) {
        self.inner.control_set_position(ned);
    }
    fn control_set_landing(&mut self, on: bool) {
        self.inner.control_set_landing(on);
    }
    fn control_status(&self) -> ControlStatus {
        self.inner.control_status()
    }
    fn estimator_set_calibration(&mut self, c: CalParams) {
        self.inner.estimator_set_calibration(c);
    }
    fn estimator_calibration(&self) -> CalParams {
        if self.m == Mutation::CalibReadIdentity {
            return CalParams::identity();
        }
        self.inner.estimator_calibration()
    }
    fn control_set_calibration(&mut self, c: CalParams) {
        if self.m != Mutation::CalibEstimatorOnly {
            self.inner.control_set_calibration(c);
        }
    }
}

/// A non-identity calibration (a gyro zero-rate bias of about 0.1 deg/s, as
/// the wasm harness injects) — what `pilot.configure-calibration` installs.
const CAL: CalParams = CalParams {
    gyro_bias: [0.002, -0.0015, 0.001],
    accel_bias: [0.0; 3],
    accel_scale: [1.0; 3],
    mag_offset: [0.0; 3],
    mag_scale: [1.0; 3],
};

fn cal_bits(c: &CalParams) -> Vec<u32> {
    let mut v = Vec::new();
    for a in [
        &c.gyro_bias,
        &c.accel_bias,
        &c.accel_scale,
        &c.mag_offset,
        &c.mag_scale,
    ] {
        v.extend(bits(a));
    }
    v
}

fn plant() -> SimBackend {
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    // A NOISY gyro, so the estimate-only ticks give `keep_warm` something to
    // remember. Deterministic (counter-seeded LCG); both plants draw alike.
    SimBackend::new(identity, DT).with_pathology(Pathology {
        gyro_white: 0.01,
        ..Pathology::default()
    })
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

fn nav_bits(n: &NavState) -> Vec<u32> {
    let mut v = Vec::new();
    for a in [&n.q[..], &n.v, &n.p, &n.b_g, &n.b_a] {
        v.extend(bits(a));
    }
    v
}

/// Coverage of one run, so a pass is about a flight and not about a hover.
#[derive(Default, Debug)]
struct Seen {
    modes: [bool; 8],
    armed_hold_ticks: u32,
    flights: u32,
    touchdowns: u32,
}

/// Fly the fused supervisor and the split one side by side; `Err` names the
/// first tick and quantity that differs.
fn run(m: Mutation) -> Result<Seen, String> {
    let mut seen = Seen::default();
    let mut pf = plant();
    let mut ps = plant();
    let mut fused = FlightSupervisor::new(HOME, FENCE, CRUISE, LOW_BATT);
    let mut split = FlightSupervisor::with_core(
        SplitCore(Mutant {
            inner: InProcess::new(),
            m,
        }),
        HOME,
        FENCE,
        CRUISE,
        LOW_BATT,
    );

    fused.set_calibration(CAL);
    split.set_calibration(CAL);

    // FLIGHT 1: arm before the first step, HOLD ARMED, then take off — the
    // estimate-only stretch `keep_warm` must survive into a flight.
    fused.command(Event::Arm, true, true);
    split.command(Event::Arm, true, true);

    let mut phase = 0u32; // 0 hold, 1 flight 1, 2 ground, 3 flight 2 (after rebuild)
    let mut phase_ticks = 0u32;
    let mut loiter = 0u32;
    let mut mission_sent = false;
    for tick in 0..200_000u32 {
        match (phase, phase_ticks) {
            (0, 400) => {
                fused.command(Event::RequestTakeoff, true, true);
                split.command(Event::RequestTakeoff, true, true);
                phase = 1;
                phase_ticks = 0;
                seen.flights += 1;
            }
            (2, 300) => {
                // RESET: the supervisor is rebuilt (as `configure` /
                // `configure-supervisor` do) and so are both partitions —
                // unless the mutant keeps the old ones.
                fused = FlightSupervisor::new(HOME, FENCE, CRUISE, LOW_BATT);
                let SplitCore(old) = core::mem::replace(
                    split.core_mut(),
                    SplitCore(Mutant {
                        inner: InProcess::new(),
                        m,
                    }),
                );
                split = FlightSupervisor::with_core(
                    if m == Mutation::NoReset {
                        SplitCore(old)
                    } else {
                        SplitCore(Mutant {
                            inner: InProcess::new(),
                            m,
                        })
                    },
                    HOME,
                    FENCE,
                    CRUISE,
                    LOW_BATT,
                );
                // The components re-install the host's calibration on every
                // rebuild (`ensure_supervisor`); so does this.
                fused.set_calibration(CAL);
                split.set_calibration(CAL);
                for s in [Event::Arm, Event::RequestTakeoff] {
                    fused.command(s, true, true);
                    split.command(s, true, true);
                }
                phase = 3;
                phase_ticks = 0;
                seen.flights += 1;
            }
            (3, 3000) => break,
            _ => {}
        }

        // One sample per plant per tick, through the production frame read.
        let mut bf = FrameBackend::new(SeamFrame::read(&mut pf));
        let mut bs = FrameBackend::new(SeamFrame::read(&mut ps));
        fused.step(&mut bf);
        split.step(&mut bs);

        let fc = fused.core();
        let sc = split.core();
        let checks: [(&str, Vec<u32>, Vec<u32>); 7] = [
            (
                "calibration",
                cal_bits(&fused.calibration()),
                cal_bits(&split.calibration()),
            ),
            ("motors", bits(&bf.motors), bits(&bs.motors)),
            (
                "nav-state",
                nav_bits(&fused.state()),
                nav_bits(&split.state()),
            ),
            (
                "tilt-uncertainty",
                bits(&[SupervisedCore::tilt_uncertainty(fc)]),
                bits(&[sc.tilt_uncertainty()]),
            ),
            (
                "max-motor",
                bits(&[SupervisedCore::max_motor(fc)]),
                bits(&[sc.max_motor()]),
            ),
            (
                "mode/failsafe/rotor",
                vec![
                    fused.mode() as u32,
                    fused.failsafe_flags(),
                    fc.failed_motor().map_or(0xFF, |i| i as u32),
                ],
                vec![
                    split.mode() as u32,
                    split.failsafe_flags(),
                    sc.failed_motor().map_or(0xFF, |i| i as u32),
                ],
            ),
            (
                "nav-compromised/gnss-diverged",
                vec![
                    SupervisedCore::nav_compromised(fc) as u32,
                    SupervisedCore::gnss_diverged(fc) as u32,
                ],
                vec![sc.nav_compromised() as u32, sc.gnss_diverged() as u32],
            ),
        ];
        for (what, a, b) in checks {
            if a != b {
                return Err(format!("{what} differs at tick {tick} (phase {phase})"));
            }
        }
        // BY CONSTRUCTION, NOT MEASURED: through this seam neither flag can
        // rise. The spoof CUSUM and the divergence latch are fed only on the
        // dual-GNSS path, and the seam carries no dual-GNSS channel. Pinned, so
        // the day a seam carries it this fails and asks for a decisive test.
        assert!(
            !SupervisedCore::nav_compromised(fc) && !SupervisedCore::gnss_diverged(fc),
            "nav integrity flag rose at tick {tick}: the seam now carries a channel \
             that can raise it, so its forward needs a decisive test"
        );

        pf.write_motors(&bf.motors);
        ps.write_motors(&bs.motors);

        let mode = fused.mode();
        seen.modes[mode as usize] = true;
        if phase == 0 && mode == Mode::Armed {
            seen.armed_hold_ticks += 1;
        }
        loiter = if mode == Mode::Loiter { loiter + 1 } else { 0 };
        if phase == 1 && loiter >= 1000 && !mission_sent {
            fused.set_mission([3.0, 0.0, -CRUISE]);
            split.set_mission([3.0, 0.0, -CRUISE]);
            fused.command(Event::RequestMission, true, true);
            split.command(Event::RequestMission, true, true);
            mission_sent = true;
        }
        if phase == 1 && mode == Mode::Disarmed {
            seen.touchdowns += 1;
            phase = 2;
            phase_ticks = 0;
        }
        phase_ticks += 1;
    }
    if phase != 3 {
        return Err(format!(
            "scenario did not complete (stopped in phase {phase})"
        ));
    }
    Ok(seen)
}

#[test]
fn production_split_core_is_bit_exact_with_the_fused_core() {
    let seen = run(Mutation::None).unwrap_or_else(|e| panic!("MISMATCH: {e}"));
    assert!(
        seen.armed_hold_ticks >= 400,
        "armed hold not flown: {seen:?}"
    );
    for (m, name) in [
        (Mode::Armed, "armed"),
        (Mode::Takeoff, "takeoff"),
        (Mode::Loiter, "loiter"),
        (Mode::Mission, "mission"),
        (Mode::Rtl, "rtl"),
        (Mode::Land, "land"),
        (Mode::Disarmed, "disarmed"),
    ] {
        assert!(seen.modes[m as usize], "never reached {name}: {seen:?}");
    }
    assert_eq!(seen.touchdowns, 1, "{seen:?}");
    assert_eq!(seen.flights, 2, "{seen:?}");
}

fn must_diverge(m: Mutation) {
    match run(m) {
        Err(e) if e.contains("differs") => eprintln!("{m:?}: {e}"),
        other => panic!("{m:?} did NOT diverge — the comparison cannot see it: {other:?}"),
    }
}

#[test]
fn negative_control_flying_edge_cut_diverges() {
    must_diverge(Mutation::EdgeCut);
}

#[test]
fn negative_control_estimate_only_clearing_the_flag_diverges() {
    must_diverge(Mutation::EstimateOnlyClearsFlag);
}

#[test]
fn negative_control_dropped_keep_warm_diverges() {
    must_diverge(Mutation::NoKeepWarm);
}

#[test]
fn negative_control_stubbed_tilt_uncertainty_diverges() {
    must_diverge(Mutation::TiltStub);
}

#[test]
fn negative_control_rebuild_without_partition_reset_diverges() {
    must_diverge(Mutation::NoReset);
}

#[test]
fn negative_control_calibration_not_forwarded_to_control_diverges() {
    must_diverge(Mutation::CalibEstimatorOnly);
}

#[test]
fn negative_control_calibration_read_back_stubbed_diverges() {
    must_diverge(Mutation::CalibReadIdentity);
}
