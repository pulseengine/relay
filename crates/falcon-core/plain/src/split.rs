//! THE SPLIT DEPLOYMENT'S FORWARDING, in one place a native test can drive
//! (#575).
//!
//! `cascade-split` supervises an estimator partition and a control partition
//! that live in two other components. The code that turns the supervisor's
//! [`SupervisedCore`] calls into calls on those two halves — and the code each
//! half runs when called — used to live inside the cdylib component crates,
//! where only the bench-only wasm differential could reach it. It lives here
//! now, and the components are thin WIT adapters over it:
//!
//! - [`SplitCore`] is the orchestrator's core: `FlightSupervisor<SplitCore<_>>`,
//!   held by `host::SupervisorHost<SplitCore<_>>` so the seam's lifecycle rules
//!   are the SAME code as the published `cascade` component's (#609
//!   single-source refactor). It is generic over [`PartitionSeam`], the twelve
//!   calls that cross the seam.
//!   The orchestrator implements `PartitionSeam` with WIT imports; a native
//!   test implements it with real in-process partitions, and so exercises THIS
//!   forwarding code, not a copy of it.
//! - [`estimator_tick`], [`estimator_health`], [`control_tick`],
//!   [`control_status`] and the tuning/constructor helpers are what the two
//!   partition components run behind the seam.
//!
//! Nothing here changes `FlightCore` or `FlightSupervisor`.
//!
//! THE CALIBRATION ROW. `CalibrationPresent` is the SUPERVISOR's
//! `calibration_configured` flag (a VALID calibration was configured, the
//! identity included), exactly as for the fused core. The estimator read-back
//! ([`SupervisedCore::calibration`]) is what `FlightSupervisor::calibration`
//! reports and what the CI-run test compares bit for bit; it does not decide
//! the row.

use crate::host::{HostCore, VehicleTuning};
use crate::{
    CascadePartition, EstimatorPartition, FlightBackend, FlightSupervisor, ImuSample, NavState,
    SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ, SupervisedCore,
};

type Vec3 = [f32; 3];

/// One tick's sensors as the seam carries them (the WIT `sensor-frame` minus
/// its encoding). GNSS-dual, baro and rangefinder are NOT carried, so every
/// backend built from a frame answers `None` for them — on both sides of any
/// comparison.
#[derive(Clone, Copy, Debug)]
pub struct SeamFrame {
    pub imu: ImuSample,
    pub dt: f32,
    pub position: Option<Vec3>,
    pub mag: Option<Vec3>,
    pub heading: Option<f32>,
    pub motor_rpm: Option<[i32; 4]>,
    pub battery_v: Option<f32>,
}

impl SeamFrame {
    /// Sample a backend ONCE, in a fixed order. A component's backend is a
    /// frame and its reads are pure, so this is exact there; a simulated
    /// backend whose reads draw noise must be sampled through this exactly
    /// once per tick and then served from the frame, on every side of a
    /// comparison.
    pub fn read<B: FlightBackend>(b: &mut B) -> Self {
        let dt = b.dt();
        let imu = b.read_imu();
        let heading = b.read_heading();
        let position = b.read_position();
        let mag = b.read_mag();
        let motor_rpm = b.read_motor_rpm();
        let battery_v = b.read_battery_v();
        SeamFrame {
            imu,
            dt,
            position,
            mag,
            heading,
            motor_rpm,
            battery_v,
        }
    }
}

/// A frame served as a backend: pure reads, the motor command caught.
pub struct FrameBackend {
    pub frame: SeamFrame,
    pub motors: [f32; 4],
}

impl FrameBackend {
    pub fn new(frame: SeamFrame) -> Self {
        FrameBackend {
            frame,
            motors: [0.0; 4],
        }
    }
}

impl FlightBackend for FrameBackend {
    fn read_imu(&mut self) -> ImuSample {
        self.frame.imu
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.frame.position
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.frame.mag
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.frame.heading
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.frame.motor_rpm
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.frame.battery_v
    }
    fn write_motors(&mut self, motors: &[f32]) {
        for (i, m) in self.motors.iter_mut().enumerate() {
            *m = motors.get(i).copied().unwrap_or(0.0);
        }
    }
    fn dt(&self) -> f32 {
        self.frame.dt
    }
}

/// The estimator facts the supervisor reads.
#[derive(Clone, Copy, Debug)]
pub struct EstimatorHealth {
    pub tilt_uncertainty: f32,
    pub nav_compromised: bool,
    pub gnss_diverged: bool,
}

/// The control facts the supervisor and the estimator read.
#[derive(Clone, Copy, Debug)]
pub struct ControlStatus {
    /// `CascadePartition::flying()` — the #452 feedback edge.
    pub flying: bool,
    pub max_motor: f32,
    pub failed_rotor: Option<usize>,
}

/// The thirteen calls that cross the estimator/control seam.
pub trait PartitionSeam {
    /// REBUILD BOTH PARTITIONS from scratch, each with its share of `tuning`
    /// (`None` = the built-in defaults) — what constructing a fresh fused core
    /// does. Called only from [`HostCore::build_supervisor`], i.e. exactly when
    /// the host (re)builds the supervisor.
    fn reset_partitions(&mut self, tuning: Option<&VehicleTuning>);
    /// See [`estimator_tick`].
    fn estimator_tick(&mut self, f: &SeamFrame, in_flight: Option<bool>) -> NavState;
    fn estimator_nav(&self) -> NavState;
    fn estimator_health(&self) -> EstimatorHealth;
    /// See [`control_tick`].
    fn control_tick(&mut self, nav: &NavState, f: &SeamFrame) -> [f32; 4];
    fn control_keep_warm(&mut self, imu: ImuSample);
    fn control_set_position(&mut self, ned: Vec3);
    fn control_set_landing(&mut self, on: bool);
    /// See [`control_rearm_fdi`]: the supervisor's spin-up hook (#609
    /// finding 16), forwarded to the control half, which owns the FDI.
    fn control_rearm_fdi(&mut self);
    fn control_status(&self) -> ControlStatus;
    /// THE CALIBRATION (#586). Installed on BOTH halves — the fused core's
    /// `set_calibration` writes `est.calib` and `casc.calib`, and the control
    /// half de-biases its own copy of the gyro stream with it — and read back
    /// from the ESTIMATOR for `FlightSupervisor::calibration`.
    fn estimator_set_calibration(&mut self, c: relay_calib::CalParams);
    fn estimator_calibration(&self) -> relay_calib::CalParams;
    fn control_set_calibration(&mut self, c: relay_calib::CalParams);
}

/// The core `cascade-split`'s supervisor drives: the two partitions, through
/// the seam. Nothing is cached — the supervisor reads its core BEFORE stepping
/// it, including on the first tick after a reset, so a cache would need seeding
/// from a hand-written default, which is the value that drifts.
pub struct SplitCore<P>(pub P);

impl<P: PartitionSeam> SupervisedCore for SplitCore<P> {
    fn state(&self) -> NavState {
        self.0.estimator_nav()
    }
    fn tilt_uncertainty(&self) -> f32 {
        self.0.estimator_health().tilt_uncertainty
    }
    /// The ESTIMATOR's live calibration, read across the seam (#586) — not a
    /// cached copy. (`CalibrationPresent` is the supervisor's own
    /// `calibration_configured` flag; see the module docs.)
    fn calibration(&self) -> relay_calib::CalParams {
        self.0.estimator_calibration()
    }
    /// Both halves, as `FlightCore::set_calibration` does.
    fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        self.0.estimator_set_calibration(calib);
        self.0.control_set_calibration(calib);
    }
    fn nav_compromised(&self) -> bool {
        self.0.estimator_health().nav_compromised
    }
    fn gnss_diverged(&self) -> bool {
        self.0.estimator_health().gnss_diverged
    }
    fn failed_motor(&self) -> Option<usize> {
        self.0.control_status().failed_rotor
    }
    fn max_motor(&self) -> f32 {
        self.0.control_status().max_motor
    }
    fn set_landing(&mut self, on: bool) {
        self.0.control_set_landing(on);
    }
    fn set_position(&mut self, ned: Vec3) {
        self.0.control_set_position(ned);
    }
    fn rearm_fdi(&mut self) {
        self.0.control_rearm_fdi();
    }

    /// `FlightCore::step`, across the seam.
    fn step<B: FlightBackend>(&mut self, b: &mut B) {
        let f = SeamFrame::read(b);
        // THE FEEDBACK EDGE (#452): the control half's PREVIOUS command, read
        // before the estimator ticks — one tick of delay, as
        // `est.set_in_flight(casc.flying())` in `FlightCore::step`.
        let flying = self.0.control_status().flying;
        let nav = self.0.estimator_tick(&f, Some(flying));
        let m = self.0.control_tick(&nav, &f);
        b.write_motors(&m);
    }

    /// `FlightCore::step_estimate_only`, across the seam. The in-flight flag is
    /// written FALSE (`Some(false)`), as the fused core writes it: this tick
    /// commands zero motors (#609 finding 8). Until #609 both left it as the
    /// last control tick set it — TRUE at a touchdown — and the grounded
    /// estimate never got its gravity reference back.
    fn step_estimate_only<B: FlightBackend>(&mut self, b: &mut B) {
        let f = SeamFrame::read(b);
        let _ = self.0.estimator_tick(&f, Some(false));
        self.0.control_keep_warm(f.imu);
        b.write_motors(&[0.0, 0.0, 0.0, 0.0]);
    }
}

/// The split's per-core hook of `SupervisorHost` (#575): reset BOTH partitions
/// with the tuning, then build the supervisor over them, into the slot. The
/// rest of the host's build (calibration to both halves, row declarations) is
/// the shared code.
impl<P: PartitionSeam + Default> HostCore for SplitCore<P> {
    fn build_supervisor(
        slot: &mut Option<FlightSupervisor<Self>>,
        (home, fence, cruise, low_batt): (Vec3, f32, f32, f32),
        tuning: Option<VehicleTuning>,
    ) -> &mut FlightSupervisor<Self> {
        slot.get_or_insert_with(|| {
            let mut seam = P::default();
            seam.reset_partitions(tuning.as_ref());
            FlightSupervisor::with_core(SplitCore(seam), home, fence, cruise, low_batt)
        })
    }
}

/// The estimator's share of a validated [`VehicleTuning`], verbatim.
pub fn estimator_tuning(t: &VehicleTuning) -> EstimatorTuning {
    EstimatorTuning {
        pos_var: t.pos_var,
        process_floor_vel: t.process_floor_vel,
        process_floor_pos: t.process_floor_pos,
    }
}

/// The control partition's share of a validated [`VehicleTuning`], verbatim.
pub fn control_tuning(t: &VehicleTuning) -> ControlTuning {
    ControlTuning {
        hover_thrust: t.hover_thrust,
        altitude_kp: t.altitude_kp,
        altitude_kd: t.altitude_kd,
        altitude_ki: t.altitude_ki,
        position_ki: t.position_ki,
    }
}

// ── behind the seam: what each partition component runs ─────────────────────

/// A fresh estimator partition, built as `FlightSupervisor::new` builds its
/// core's.
pub fn new_estimator() -> EstimatorPartition {
    EstimatorPartition::new(SUPERVISOR_CORE_LOOP_HZ)
}

/// A fresh control partition, built as `FlightSupervisor::new` builds its
/// core's.
pub fn new_control() -> CascadePartition {
    CascadePartition::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ)
}

/// The estimator's share of the vehicle tuning, already validated
/// (`host::VehicleConfig::tuning`).
#[derive(Clone, Copy, Debug)]
pub struct EstimatorTuning {
    pub pos_var: f32,
    pub process_floor_vel: f32,
    pub process_floor_pos: f32,
}

/// The control partition's share of the vehicle tuning, already validated
/// (`host::VehicleConfig::tuning`).
#[derive(Clone, Copy, Debug)]
pub struct ControlTuning {
    pub hover_thrust: f32,
    pub altitude_kp: f32,
    pub altitude_kd: f32,
    pub altitude_ki: f32,
    pub position_ki: f32,
}

pub fn apply_estimator_tuning(est: &mut EstimatorPartition, t: &EstimatorTuning) {
    est.set_pos_var(t.pos_var);
    est.set_process_floor(t.process_floor_vel, t.process_floor_pos);
}

/// `set_hover_thrust_core` clamps to [0, 1] like
/// `FlightSupervisor::set_hover_thrust`; they differ only on a non-finite
/// value, which a validated tuning (hover thrust in (0, 1]) never carries, so
/// no clamp engages.
pub fn apply_control_tuning(casc: &mut CascadePartition, t: &ControlTuning) {
    casc.set_hover_thrust_core(t.hover_thrust);
    casc.set_altitude_gains(t.altitude_kp, t.altitude_kd);
    casc.set_altitude_integral_gain(t.altitude_ki);
    casc.set_position_integral_gain(t.position_ki);
}

/// The estimator half of `FlightCore::step` (`in_flight = Some`) or of
/// `FlightCore::step_estimate_only` (`None`, flag untouched).
pub fn estimator_tick(
    est: &mut EstimatorPartition,
    f: &SeamFrame,
    in_flight: Option<bool>,
) -> NavState {
    if let Some(flying) = in_flight {
        est.set_in_flight(flying);
    }
    let mut b = FrameBackend::new(*f);
    est.step(&mut b, f.imu, f.heading, f.dt)
}

pub fn estimator_health(est: &EstimatorPartition) -> EstimatorHealth {
    EstimatorHealth {
        tilt_uncertainty: est.tilt_uncertainty(),
        nav_compromised: est.nav_compromised(),
        gnss_diverged: est.gnss_diverged(),
    }
}

/// The control half of `FlightCore::step`.
pub fn control_tick(casc: &mut CascadePartition, nav: &NavState, f: &SeamFrame) -> [f32; 4] {
    let mut b = FrameBackend::new(*f);
    casc.step(&mut b, nav, f.imu, f.heading.is_some(), f.dt);
    b.motors
}

/// The estimator half of `FlightCore::set_calibration`.
pub fn estimator_set_calibration(est: &mut EstimatorPartition, c: relay_calib::CalParams) {
    est.calib = c;
}

/// What the estimator applies to raw samples — `FlightCore::calibration`.
pub fn estimator_calibration(est: &EstimatorPartition) -> relay_calib::CalParams {
    est.calib
}

/// `FlightCore::rearm_fdi` — the FDI lives in the control half (#609 finding
/// 16). The supervisor calls it on its first flying tick after motors-off.
pub fn control_rearm_fdi(casc: &mut CascadePartition) {
    casc.rearm_fdi();
}

/// The control half of `FlightCore::set_calibration`.
pub fn control_set_calibration(casc: &mut CascadePartition, c: relay_calib::CalParams) {
    casc.calib = c;
}

pub fn control_status(casc: &CascadePartition) -> ControlStatus {
    ControlStatus {
        flying: casc.flying(),
        max_motor: casc.max_motor(),
        failed_rotor: casc.failed_motor(),
    }
}
