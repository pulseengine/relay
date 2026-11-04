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
//!   BACK from the estimator for `FlightSupervisor::calibration`. Two mutants
//!   drop one of those each, and must diverge.
//! - THE HOST (#609 single-source refactor): `cascade-split` holds
//!   `SupervisorHost<SplitCore<_>>`, the same lifecycle rules as the published
//!   `cascade` (`SupervisorHost<FlightCore>`). The last test drives both hosts
//!   through configure / calibrate / arm / fly / rebuild / revoke and requires
//!   them bit-exact; its negative control drops the tuning in the split's
//!   per-core hook and must diverge.
//!
//! Every negative control below is a `#[test]` that requires the mutated
//! seam to DIVERGE: a comparison that cannot fail proves nothing.

use falcon_core::host::{self, Configured, SupervisorHost, VehicleTuning};
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

impl Default for InProcess {
    fn default() -> Self {
        Self::new()
    }
}

impl PartitionSeam for InProcess {
    /// What `cascade-estimator`'s and `cascade-control`'s `configure` run.
    fn reset_partitions(&mut self, tuning: Option<&VehicleTuning>) {
        *self = InProcess::new();
        if let Some(t) = tuning {
            split::apply_estimator_tuning(&mut self.est, &split::estimator_tuning(t));
            split::apply_control_tuning(&mut self.casc, &split::control_tuning(t));
        }
    }
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
    fn control_rearm_fdi(&mut self) {
        split::control_rearm_fdi(&mut self.casc);
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
    /// Estimate-only ticks LEAVE the flag (`None`) instead of writing it false
    /// — the pre-#609-finding-8 contract. Applied where the call can only be
    /// an estimate-only tick: `Some(false)` while the control half's previous
    /// command reads flying (a control tick would have sent `Some(true)`).
    EstimateOnlyLeavesFlag,
    /// The control half's `keep_warm` is dropped.
    NoKeepWarm,
    /// The tilt-uncertainty forward is stubbed to 0.
    TiltStub,
    /// The supervisor is rebuilt WITHOUT rebuilding the partitions.
    NoReset,
    /// The calibration reaches the estimator but NOT the control half.
    CalibEstimatorOnly,
    /// The calibration read-back is stubbed to the identity (the pre-#586
    /// `SplitCore::calibration`), so `FlightSupervisor::calibration` reports a
    /// value the estimator does not apply.
    CalibReadIdentity,
}

struct Mutant {
    inner: InProcess,
    m: Mutation,
}

impl PartitionSeam for Mutant {
    fn reset_partitions(&mut self, tuning: Option<&VehicleTuning>) {
        if self.m != Mutation::NoReset {
            self.inner.reset_partitions(tuning);
        }
    }
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState {
        let in_flight = match (self.m, in_flight) {
            (Mutation::EdgeCut, Some(_)) => Some(false),
            (Mutation::EstimateOnlyLeavesFlag, Some(false))
                if self.inner.control_status().flying =>
            {
                None
            }
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
    fn control_rearm_fdi(&mut self) {
        self.inner.control_rearm_fdi();
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
    // A small soft-iron cross-coupling (positive definite), so the 18th..20th
    // f32 of `calibration-params` are exercised too.
    mag_offdiag: [0.01, -0.02, 0.005],
};

fn cal_bits(c: &CalParams) -> Vec<u32> {
    let mut v = Vec::new();
    for a in [
        &c.gyro_bias,
        &c.accel_bias,
        &c.accel_scale,
        &c.mag_offset,
        &c.mag_scale,
        &c.mag_offdiag,
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
fn negative_control_estimate_only_leaving_the_flag_diverges() {
    must_diverge(Mutation::EstimateOnlyLeavesFlag);
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

// ── the host over each core: the SAME lifecycle rules (#609, #575) ──────────

/// The split seam for the host-level test, with a switch for its negative
/// control: `TUNE = false` drops the tuning in the per-core hook.
#[derive(Default)]
struct HostSeam<const TUNE: bool>(InProcess);

impl<const TUNE: bool> PartitionSeam for HostSeam<TUNE> {
    fn reset_partitions(&mut self, tuning: Option<&VehicleTuning>) {
        self.0.reset_partitions(if TUNE { tuning } else { None });
    }
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState {
        self.0.estimator_tick(f, in_flight)
    }
    fn estimator_nav(&self) -> NavState {
        self.0.estimator_nav()
    }
    fn estimator_health(&self) -> EstimatorHealth {
        self.0.estimator_health()
    }
    fn control_tick(&mut self, nav: &NavState, f: &SeamFrame) -> [f32; 4] {
        self.0.control_tick(nav, f)
    }
    fn control_keep_warm(&mut self, imu: ImuSample) {
        self.0.control_keep_warm(imu)
    }
    fn control_set_position(&mut self, ned: [f32; 3]) {
        self.0.control_set_position(ned)
    }
    fn control_set_landing(&mut self, on: bool) {
        self.0.control_set_landing(on)
    }
    fn control_rearm_fdi(&mut self) {
        self.0.control_rearm_fdi()
    }
    fn control_status(&self) -> ControlStatus {
        self.0.control_status()
    }
    fn estimator_set_calibration(&mut self, c: CalParams) {
        self.0.estimator_set_calibration(c)
    }
    fn estimator_calibration(&self) -> CalParams {
        self.0.estimator_calibration()
    }
    fn control_set_calibration(&mut self, c: CalParams) {
        self.0.control_set_calibration(c)
    }
}

/// A tuning that differs from falcon-core's built-in defaults in every field,
/// so a hook that drops any of it diverges.
const VEHICLE: host::VehicleConfig = host::VehicleConfig {
    hover_thrust: 0.49,
    loop_rate_hz: 1000.0,
    pos_var: 0.02,
    process_floor_vel: 0.001,
    process_floor_pos: 0.001,
    altitude_kp: 0.06,
    altitude_kd: 0.35,
    altitude_ki: 0.01,
    position_ki: 0.03,
};
const SUPCFG: host::SupervisorConfig = host::SupervisorConfig {
    home: HOME,
    fence_radius: FENCE,
    cruise_alt: CRUISE,
    low_batt_v: LOW_BATT,
};

/// Drive a fused host and a split host through the SAME calls; `Err` names the
/// first tick and quantity that differs. Covers: configure before any build,
/// an arm refused before the first step, arm on the measured table, takeoff,
/// a reconfigure REJECTED in flight, land, a rebuild on the ground with the
/// calibration surviving it, and an invalid calibration revoking the earlier
/// one (arm refused, calibration no longer configured, on both).
fn run_hosts<P: PartitionSeam + Default>() -> Result<[u32; 4], String> {
    let mut pf = plant();
    let mut ps = plant();
    let mut fused: SupervisorHost = SupervisorHost::new();
    let mut split: SupervisorHost<SplitCore<P>> = SupervisorHost::new();
    // [arms accepted, rejected configures, refused-after-revoke ticks, ticks]
    let mut seen = [0u32; 4];

    let both = |f: &mut SupervisorHost, s: &mut SupervisorHost<SplitCore<P>>, ev: Event| {
        f.command(ev, true, true);
        s.command(ev, true, true);
    };
    for (a, b) in [
        (
            fused.configure_vehicle(VEHICLE),
            split.configure_vehicle(VEHICLE),
        ),
        (
            fused.configure_supervisor(SUPCFG),
            split.configure_supervisor(SUPCFG),
        ),
        (
            fused.configure_calibration(CAL),
            split.configure_calibration(CAL),
        ),
    ] {
        if a != b || a != Configured::Accepted {
            return Err(format!(
                "configure verdicts differ or refused: {a:?} vs {b:?}"
            ));
        }
    }
    // Before the first step: refused on both (#609 finding 2).
    both(&mut fused, &mut split, Event::Arm);
    if fused.supervisor().unwrap().mode() != Mode::Disarmed
        || split.supervisor().unwrap().mode() != Mode::Disarmed
    {
        return Err("an arm before the first step was accepted".into());
    }

    let mut phase = 0u32; // 0 arm, 1 fly, 2 land, 3 ground/rebuild, 4 revoked
    let mut phase_ticks = 0u32;
    for tick in 0..60_000u32 {
        let mut bf = FrameBackend::new(SeamFrame::read(&mut pf));
        let mut bs = FrameBackend::new(SeamFrame::read(&mut ps));
        let (fm, ff, fn_) = {
            let s = fused.step([0.0, 0.0, -CRUISE], &mut bf);
            (s.mode(), s.failsafe_flags(), nav_bits(&s.state()))
        };
        let (sm, sf, sn) = {
            let s = split.step([0.0, 0.0, -CRUISE], &mut bs);
            (s.mode(), s.failsafe_flags(), nav_bits(&s.state()))
        };
        if bits(&bf.motors) != bits(&bs.motors) {
            return Err(format!("motors differ at tick {tick} (phase {phase})"));
        }
        if (fm, ff) != (sm, sf) {
            return Err(format!(
                "mode/failsafe differ at tick {tick} (phase {phase}): {fm:?}/{ff:#x} vs {sm:?}/{sf:#x}"
            ));
        }
        if fn_ != sn {
            return Err(format!("nav-state differs at tick {tick} (phase {phase})"));
        }
        let fc = fused.supervisor().unwrap().calibration();
        let sc = split.supervisor().unwrap().calibration();
        if cal_bits(&fc) != cal_bits(&sc) {
            return Err(format!(
                "calibration differs at tick {tick} (phase {phase})"
            ));
        }
        pf.write_motors(&bf.motors);
        ps.write_motors(&bs.motors);
        seen[3] += 1;
        phase_ticks += 1;

        match phase {
            0 if fm == Mode::Disarmed => both(&mut fused, &mut split, Event::Arm),
            0 if fm == Mode::Armed => {
                seen[0] += 1;
                both(&mut fused, &mut split, Event::RequestTakeoff);
                phase = 1;
                phase_ticks = 0;
            }
            1 if phase_ticks == 3000 => {
                // In flight: a reconfigure must be REJECTED on both (#609
                // finding 1), and the vehicle keeps flying.
                let a = fused.configure_supervisor(SUPCFG);
                let b = split.configure_supervisor(SUPCFG);
                if a != b || a != Configured::Rejected {
                    return Err(format!("in-flight configure: {a:?} vs {b:?}"));
                }
                seen[1] += 1;
                both(&mut fused, &mut split, Event::RequestLand);
                phase = 2;
                phase_ticks = 0;
            }
            2 if fm == Mode::Disarmed => {
                phase = 3;
                phase_ticks = 0;
            }
            // `request-land` is accepted from Loiter, not mid-Takeoff: re-send.
            2 if fm != Mode::Land => both(&mut fused, &mut split, Event::RequestLand),
            3 if phase_ticks == 200 => {
                // On the ground: a rebuild is permitted, and the calibration
                // must survive it (re-installed on BOTH halves of the split).
                let a = fused.configure_vehicle(VEHICLE);
                let b = split.configure_vehicle(VEHICLE);
                if a != b || a != Configured::Accepted {
                    return Err(format!("ground reconfigure: {a:?} vs {b:?}"));
                }
            }
            3 if phase_ticks == 2200 => {
                // The rebuilt supervisors: the calibration survived the rebuild
                // on both (compared bit for bit every tick above), and both
                // judge an arm by the same row. (The plant has no ground, so a
                // disarmed vehicle is falling and is not expected to arm.)
                let (f, s) = (fused.supervisor().unwrap(), split.supervisor().unwrap());
                if !f.calibration_configured() || !s.calibration_configured() {
                    return Err("the calibration did not survive the rebuild".into());
                }
                if f.arm_blocked_check() != s.arm_blocked_check() {
                    return Err(format!(
                        "rebuilt: refusing row differs: {:?} vs {:?}",
                        f.arm_blocked_check(),
                        s.arm_blocked_check()
                    ));
                }
                // An INVALID calibration revokes on both.
                let bad = CalParams {
                    accel_scale: [1.0, f32::NAN, 1.0],
                    ..CAL
                };
                let (a, b) = (
                    fused.configure_calibration(bad),
                    split.configure_calibration(bad),
                );
                if a != b || a != Configured::Invalid {
                    return Err(format!("invalid calibration: {a:?} vs {b:?}"));
                }
                phase = 4;
                phase_ticks = 0;
            }
            4 if phase_ticks < 200 => {
                both(&mut fused, &mut split, Event::Arm);
                let rf = fused.supervisor().unwrap().arm_blocked_check();
                let rs = split.supervisor().unwrap().arm_blocked_check();
                if rf != rs {
                    return Err(format!("refusing row differs: {rf:?} vs {rs:?}"));
                }
                let (f, s) = (fused.supervisor().unwrap(), split.supervisor().unwrap());
                if fm == Mode::Disarmed
                    && rf.is_some()
                    && !f.calibration_configured()
                    && !s.calibration_configured()
                {
                    seen[2] += 1;
                }
            }
            4 => return Ok(seen),
            _ => {}
        }
    }
    let s = fused.supervisor().unwrap();
    Err(format!(
        "scenario did not complete (phase {phase}, mode {:?}, d {:.2})",
        s.mode(),
        s.state().p[2]
    ))
}

#[test]
fn split_host_runs_the_same_lifecycle_rules_bit_exactly() {
    let seen = run_hosts::<HostSeam<true>>().unwrap_or_else(|e| panic!("MISMATCH: {e}"));
    let [arms, rejected, revoked, ticks] = seen;
    assert_eq!(arms, 1, "{seen:?}");
    assert_eq!(rejected, 1, "{seen:?}");
    assert!(revoked >= 150, "revocation not held: {seen:?}");
    eprintln!("host twin: {ticks} ticks bit-exact");
}

#[test]
fn negative_control_split_host_without_tuning_diverges() {
    match run_hosts::<HostSeam<false>>() {
        Err(e) if e.contains("differ") => eprintln!("NoTuning: {e}"),
        other => panic!("dropping the tuning did NOT diverge: {other:?}"),
    }
}

// ── the host over each core in MISSION (live follow) and a SECOND SPIN-UP ────
//
// #609 findings 4 and 16, after the single-source refactor. Both live in code
// the fused and the split host SHARE (`SupervisorHost::step` delivers the
// follow target; `FlightSupervisor` re-arms the FDI on a spin-up), but the
// FDI itself is in the CONTROL HALF, so the split reaches it only through the
// seam's `control_rearm_fdi`, and the follow target reaches the control half
// only as the setpoint the supervisor derives. So this twin flies both hosts
// through: a Mission with a MOVING streamed target; the stream going stale
// (non-finite targets) into the offboard-stale HOLD with failsafe bit 8; a
// landing; and a SECOND takeoff in the same process with an ASYMMETRIC
// spin-up (one rotor's ESC soft-starts) and ESC RPM telemetry, the case the
// re-arm exists for. Bit-exact
// every tick, and each negative control below must diverge.

/// What the follow twin's split side is given, for its negative controls.
#[derive(Clone, Copy, PartialEq, Debug)]
enum FollowMut {
    None,
    /// The split host gets the PREVIOUS tick's target (a one-frame lag).
    TargetLag,
    /// The split host keeps getting the last finite target when the stream
    /// goes non-finite, so it never goes stale.
    NeverStale,
}

/// The split seam for the follow twin: `REARM = false` drops the spin-up hook.
#[derive(Default)]
struct FollowSeam<const REARM: bool>(InProcess);

impl<const REARM: bool> PartitionSeam for FollowSeam<REARM> {
    fn reset_partitions(&mut self, tuning: Option<&VehicleTuning>) {
        self.0.reset_partitions(tuning);
    }
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState {
        self.0.estimator_tick(f, in_flight)
    }
    fn estimator_nav(&self) -> NavState {
        self.0.estimator_nav()
    }
    fn estimator_health(&self) -> EstimatorHealth {
        self.0.estimator_health()
    }
    fn control_tick(&mut self, nav: &NavState, f: &SeamFrame) -> [f32; 4] {
        self.0.control_tick(nav, f)
    }
    fn control_keep_warm(&mut self, imu: ImuSample) {
        self.0.control_keep_warm(imu)
    }
    fn control_set_position(&mut self, ned: [f32; 3]) {
        self.0.control_set_position(ned)
    }
    fn control_set_landing(&mut self, on: bool) {
        self.0.control_set_landing(on)
    }
    fn control_rearm_fdi(&mut self) {
        if REARM {
            self.0.control_rearm_fdi()
        }
    }
    fn control_status(&self) -> ControlStatus {
        self.0.control_status()
    }
    fn estimator_set_calibration(&mut self, c: CalParams) {
        self.0.estimator_set_calibration(c)
    }
    fn estimator_calibration(&self) -> CalParams {
        self.0.estimator_calibration()
    }
    fn control_set_calibration(&mut self, c: CalParams) {
        self.0.control_set_calibration(c)
    }
}

/// A plant that lands (ground contact) and reports ESC RPM (the FDI's input),
/// with rotor drag so the FDI's residual is meaningful.
fn landing_plant() -> SimBackend {
    let mut p = plant();
    p.ground_contact = true;
    p.rot_drag = 0.02;
    p.rpm_telemetry = true;
    p
}

/// What the follow twin covered.
#[derive(Default, Debug)]
struct FollowSeen {
    ticks: u32,
    mission_ticks: u32,
    /// Largest horizontal distance of the streamed target from the hold
    /// point Mission began at.
    target_travel: f32,
    /// Ticks from the first non-finite target to the HOLD (Loiter + bit 8).
    stale_after: Option<u32>,
    flights: u32,
    touchdowns: u32,
}

/// ASYMMETRIC SPIN-UP (#609 finding 17). Since the residual rejects what all
/// four rotors share, a uniform spin-up lag no longer produces a residual at
/// all, so a dropped re-arm stopped diverging. One rotor that spins up LATER
/// than the rest does: rotor `ASYM_ROTOR`'s ESC ramps its output linearly over
/// `ASYM_SOFTSTART_S` from the takeoff command (a soft-start the other three
/// ESCs do not have). Its residual is differential, which is what a rotor-out
/// looks like, and ONLY the spin-up guard keeps it from being isolated: the
/// ramp is over well inside the guard's 0.3 s (0.2 s before #609 finding
/// 18). Both plants are fed alike.
///
/// Until finding 17 this twin flew each takeoff with a UNIFORM 50 ms rotor
/// lag instead; that transient is common-mode and now produces no residual,
/// so it no longer discriminates. The plant here has NO rotor lag: combined
/// with a 50 ms lag, the same soft-start is isolated ~0.23 s into the FIRST
/// takeoff, past the guard, with or without the common-mode rejection (a
/// separate, pre-existing FDI sensitivity: #609 finding 18, addressed by the
/// 0.3 s guard and the lag-modelled residual, tests/fdi_spinup_guard.rs).
const ASYM_ROTOR: usize = 2;
const ASYM_SOFTSTART_S: f32 = 0.1;
const HOLD_TARGET: [f32; 3] = [0.0, 0.0, -CRUISE];

fn run_hosts_follow<P: PartitionSeam + Default>(mutation: FollowMut) -> Result<FollowSeen, String> {
    let mut pf = landing_plant();
    let mut ps = landing_plant();
    let mut fused: SupervisorHost = SupervisorHost::new();
    let mut split: SupervisorHost<SplitCore<P>> = SupervisorHost::new();
    let mut seen = FollowSeen::default();
    for (a, b) in [
        (
            fused.configure_vehicle(VEHICLE),
            split.configure_vehicle(VEHICLE),
        ),
        (
            fused.configure_supervisor(SUPCFG),
            split.configure_supervisor(SUPCFG),
        ),
        (
            fused.configure_calibration(CAL),
            split.configure_calibration(CAL),
        ),
    ] {
        if a != b || a != Configured::Accepted {
            return Err(format!(
                "configure verdicts differ or refused: {a:?} vs {b:?}"
            ));
        }
    }
    let both = |f: &mut SupervisorHost, s: &mut SupervisorHost<SplitCore<P>>, ev: Event| {
        f.command(ev, true, true);
        s.command(ev, true, true);
    };

    let nan = [f32::NAN; 3];
    // 0 ground, 1 takeoff, 2 loiter, 3 mission (moving target), 4 stale ->
    // HOLD, 5 held, 6 land, 7 ground, 8 flight-2 takeoff, 9 flight-2 loiter
    let mut phase = 0u32;
    let mut phase_ticks = 0u32;
    let mut prev_target = HOLD_TARGET;
    let mut last_finite = HOLD_TARGET;
    for tick in 0..200_000u32 {
        // THE STREAM. In Mission a moving target (a 3 m step north, then a
        // slow ellipse around it); from the stale phase on the non-finite
        // "no target this frame"; otherwise the hold point.
        let target = match phase {
            3 => {
                let t = phase_ticks as f32 * DT;
                let w = 0.6;
                [3.0 + 1.5 * (w * t).sin(), 1.0 - (w * t).cos(), -CRUISE]
            }
            4 | 5 => nan,
            _ => HOLD_TARGET,
        };
        if target.iter().all(|x| x.is_finite()) {
            last_finite = target;
        }
        let split_target = match mutation {
            FollowMut::None => target,
            FollowMut::TargetLag => prev_target,
            FollowMut::NeverStale => {
                if target[0].is_finite() {
                    target
                } else {
                    last_finite
                }
            }
        };
        prev_target = target;

        let mut bf = FrameBackend::new(SeamFrame::read(&mut pf));
        let mut bs = FrameBackend::new(SeamFrame::read(&mut ps));
        let (fm, ff, fnav, fr) = {
            let s = fused.step(target, &mut bf);
            (
                s.mode(),
                s.failsafe_flags(),
                nav_bits(&s.state()),
                s.core().failed_motor(),
            )
        };
        let (sm, sf, snav, sr) = {
            let s = split.step(split_target, &mut bs);
            (
                s.mode(),
                s.failsafe_flags(),
                nav_bits(&s.state()),
                s.core().failed_motor(),
            )
        };
        if bits(&bf.motors) != bits(&bs.motors) {
            return Err(format!("motors differ at tick {tick} (phase {phase})"));
        }
        if (fm, ff, fr) != (sm, sf, sr) {
            return Err(format!(
                "mode/failsafe/rotor differ at tick {tick} (phase {phase}): \
                 {fm:?}/{ff:#x}/{fr:?} vs {sm:?}/{sf:#x}/{sr:?}"
            ));
        }
        if fnav != snav {
            return Err(format!("nav-state differs at tick {tick} (phase {phase})"));
        }
        let (mut mf, mut ms) = (bf.motors, bs.motors);
        if matches!(phase, 1 | 8) {
            let ramp = (phase_ticks as f32 * DT / ASYM_SOFTSTART_S).min(1.0);
            mf[ASYM_ROTOR] *= ramp;
            ms[ASYM_ROTOR] *= ramp;
        }
        pf.write_motors(&mf);
        ps.write_motors(&ms);
        seen.ticks += 1;
        phase_ticks += 1;
        if fr.is_some() {
            return Err(format!(
                "a healthy rotor was isolated at tick {tick} (phase {phase}) on both"
            ));
        }
        let stale = ff & falcon_core::failsafe_flags::TARGET_STALE != 0;
        macro_rules! go {
            ($p:expr) => {{
                phase = $p;
                phase_ticks = 0;
            }};
        }
        match phase {
            0 | 7 if fm == Mode::Armed => {
                both(&mut fused, &mut split, Event::RequestTakeoff);
                seen.flights += 1;
                go!(if phase == 0 { 1 } else { 8 });
            }
            0 | 7 if phase_ticks >= 2000 && fm == Mode::Disarmed => {
                both(&mut fused, &mut split, Event::Arm)
            }
            1 | 8 if fm == Mode::Loiter => {
                go!(if phase == 1 { 2 } else { 9 });
            }
            1 | 8 if fm != Mode::Takeoff => {
                return Err(format!("left Takeoff for {fm:?} at tick {tick}"));
            }
            2 if phase_ticks >= 1000 => {
                both(&mut fused, &mut split, Event::RequestMission);
                go!(3);
            }
            3 => {
                if fm != Mode::Mission {
                    return Err(format!(
                        "left Mission for {fm:?} at tick {tick} while streaming"
                    ));
                }
                if stale {
                    return Err(format!("target-stale raised while streaming, tick {tick}"));
                }
                seen.mission_ticks += 1;
                let d = (target[0] - HOLD_TARGET[0]).hypot(target[1] - HOLD_TARGET[1]);
                seen.target_travel = seen.target_travel.max(d);
                if phase_ticks >= 8000 {
                    go!(4);
                }
            }
            4 if fm == Mode::Loiter && stale => {
                seen.stale_after = Some(phase_ticks);
                go!(5);
            }
            4 if fm != Mode::Mission || phase_ticks > 2000 => {
                return Err(format!(
                    "the stale stream did not take the HOLD: {fm:?}/{ff:#x} after {phase_ticks} ticks"
                ));
            }
            5 if phase_ticks >= 6000 => {
                both(&mut fused, &mut split, Event::RequestLand);
                go!(6);
            }
            6 if fm == Mode::Disarmed => {
                seen.touchdowns += 1;
                go!(7);
            }
            6 if fm != Mode::Land => both(&mut fused, &mut split, Event::RequestLand),
            9 if phase_ticks >= 2000 => return Ok(seen),
            _ => {}
        }
    }
    Err(format!("follow scenario did not complete (phase {phase})"))
}

#[test]
fn split_host_follows_goes_stale_and_re_arms_the_fdi_bit_exactly() {
    let seen = run_hosts_follow::<FollowSeam<true>>(FollowMut::None)
        .unwrap_or_else(|e| panic!("MISMATCH: {e}"));
    assert_eq!(seen.flights, 2, "{seen:?}");
    assert_eq!(seen.touchdowns, 1, "{seen:?}");
    assert!(seen.mission_ticks >= 8000, "{seen:?}");
    assert!(
        seen.target_travel > 3.0,
        "the target did not move: {seen:?}"
    );
    // FOLLOW_TARGET_TIMEOUT_US = 1.0 s = 1000 ticks at 1 kHz.
    let after = seen.stale_after.expect("never went stale");
    assert!(
        (1000..=1002).contains(&after),
        "stale HOLD after {after} ticks: {seen:?}"
    );
    eprintln!("follow twin: {seen:?}");
}

fn follow_must_diverge<P: PartitionSeam + Default>(m: FollowMut, what: &str) {
    match run_hosts_follow::<P>(m) {
        Err(e) if e.contains("differ") => eprintln!("{what}: {e}"),
        other => panic!("{what} did NOT diverge: {other:?}"),
    }
}

#[test]
fn negative_control_split_target_lag_diverges_in_mission() {
    follow_must_diverge::<FollowSeam<true>>(FollowMut::TargetLag, "one-frame target lag");
}

#[test]
fn negative_control_split_never_stale_diverges() {
    follow_must_diverge::<FollowSeam<true>>(FollowMut::NeverStale, "never-stale split");
}

#[test]
fn negative_control_split_without_fdi_rearm_diverges() {
    follow_must_diverge::<FollowSeam<false>>(FollowMut::None, "dropped control_rearm_fdi");
}
