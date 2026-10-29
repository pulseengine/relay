//! Lifecycle and configuration tests of the `pilot` / `controller` seam
//! (#609 findings 1-3), run against BOTH sides of the equivalence test:
//!
//!   * the SHIPPED component, through wasmtime and the Component Model ABI —
//!     `#[ignore]`d unless a built artifact is named, because `cargo test` in
//!     this crate does not build wasm. Run with
//!     `CASCADE_WASM=<falcon_cascade_cm.wasm> cargo test -- --ignored`.
//!     An ignored test is REPORTED as ignored, never as passed, so an absent
//!     artifact cannot masquerade as a green run.
//!   * the native mirror (`native_mirror.rs`), which runs on every
//!     `cargo test` and so keeps the mirror's copy of the lifecycle rules from
//!     drifting away from the component's.
//!
//! Every scenario is written ONCE, generically over [`Seam`], so the two sides
//! are held to the same assertions by construction.

use crate::native_mirror::{Calib, NativeCascade, SensorCal, SupCalib};
use crate::physics::{MockPhysics, Physics};
use falcon_core::relay_fsm::Event;
use falcon_core::relay_preflight::CheckId;

const DT: f32 = 0.0025;
const PACK_V: f32 = 16.8;
const LOW_BATT_V: f32 = 14.0;

const DISARMED: u8 = 0;
const ARMED: u8 = 1;
const TAKEOFF: u8 = 2;
const LOITER: u8 = 3;
const TERMINATED: u8 = 7;

fn mock_calib() -> Calib {
    // The mock-plant numbers main.rs installs (hover 0.49, falcon-core defaults
    // elsewhere).
    Calib {
        hover_thrust: 0.49,
        loop_rate_hz: 1.0 / DT,
        pos_var: 0.01,
        process_floor_vel: 0.0,
        process_floor_pos: 0.0,
        altitude_kp: 0.05,
        altitude_kd: 0.30,
        altitude_ki: 0.0,
        position_ki: 0.02,
    }
}

/// The simulated gyro bias the rig adds to every sample, and the calibration
/// that cancels it — same rationale as main.rs's `SITL_GYRO_BIAS`.
const GYRO_BIAS: [f32; 3] = [0.002, -0.0015, 0.001];

fn identity_cal() -> SensorCal {
    SensorCal {
        gyro_bias: [0.0; 3],
        accel_bias: [0.0; 3],
        accel_scale: [1.0; 3],
        mag_offset: [0.0; 3],
        mag_scale: [1.0; 3],
        mag_offdiag: [0.0; 3],
    }
}

fn good_cal() -> SensorCal {
    SensorCal {
        gyro_bias: GYRO_BIAS,
        ..identity_cal()
    }
}

fn good_sup() -> SupCalib {
    SupCalib {
        home: [0.0, 0.0, 0.0],
        fence_radius: 100.0,
        cruise_alt: 2.0,
        low_batt_v: LOW_BATT_V,
    }
}

/// One tick's frame, in the mirror's own types.
struct Frame {
    accel: [f32; 3],
    gyro: [f32; 3],
    position: Option<[f32; 3]>,
    mag: Option<[f32; 3]>,
    heading: Option<f32>,
    rpm: Option<[i32; 4]>,
    battery_v: Option<f32>,
}

/// The seam surface both sides expose.
trait Seam {
    fn configure(&mut self, c: Calib);
    fn configure_supervisor(&mut self, c: SupCalib);
    fn configure_calibration(&mut self, c: SensorCal);
    fn command(&mut self, ev: Event);
    fn step(&mut self, f: &Frame, target: [f32; 3]) -> [f32; 4];
    /// Published `mode`, `None` while not measured.
    fn mode(&mut self) -> Option<u8>;
    /// Published `failed-rotor` (0xFF = none), `None` while not measured.
    fn failed_rotor(&mut self) -> Option<u8>;
}

impl Seam for NativeCascade {
    fn configure(&mut self, c: Calib) {
        NativeCascade::configure(self, c)
    }
    fn configure_supervisor(&mut self, c: SupCalib) {
        NativeCascade::configure_supervisor(self, c)
    }
    fn configure_calibration(&mut self, c: SensorCal) {
        NativeCascade::configure_calibration(self, c)
    }
    fn command(&mut self, ev: Event) {
        NativeCascade::command(self, ev, true, true)
    }
    fn step(&mut self, f: &Frame, target: [f32; 3]) -> [f32; 4] {
        NativeCascade::step(
            self,
            f.accel,
            f.gyro,
            target,
            f.position,
            f.mag,
            f.heading,
            f.rpm,
            f.battery_v,
            DT,
        )
    }
    fn mode(&mut self) -> Option<u8> {
        NativeCascade::mode(self)
    }
    fn failed_rotor(&mut self) -> Option<u8> {
        NativeCascade::failed_rotor(self)
    }
}

/// The shipped component, instantiated exactly as `main` does (empty linker,
/// no WASI).
struct WasmSeam {
    store: wasmtime::Store<()>,
    b: crate::ComposedCascade,
}

impl WasmSeam {
    fn load() -> Self {
        use wasmtime::component::{Component, Linker};
        use wasmtime::{Config, Engine, Store};
        let path = std::env::var("CASCADE_WASM").expect(
            "CASCADE_WASM must name a built falcon_cascade_cm.wasm to run the \
             component half of these tests",
        );
        let mut cfg = Config::new();
        cfg.wasm_component_model(true);
        let engine = Engine::new(&cfg).unwrap();
        let component = Component::from_file(&engine, &path).unwrap();
        let linker: Linker<()> = Linker::new(&engine);
        let mut store = Store::new(&engine, ());
        let b = crate::ComposedCascade::instantiate(&mut store, &component, &linker).unwrap();
        WasmSeam { store, b }
    }
    fn state(&mut self) -> crate::pulseengine::falcon_cascade::types::FlightState {
        self.b
            .pulseengine_falcon_cascade_observer()
            .call_read_state(&mut self.store)
            .unwrap()
    }
}

impl Seam for WasmSeam {
    fn configure(&mut self, c: Calib) {
        use crate::pulseengine::falcon_cascade::types::VehicleConfig;
        self.b
            .pulseengine_falcon_cascade_controller()
            .call_configure(
                &mut self.store,
                VehicleConfig {
                    hover_thrust: c.hover_thrust,
                    loop_rate_hz: c.loop_rate_hz,
                    pos_var: c.pos_var,
                    process_floor_vel: c.process_floor_vel,
                    process_floor_pos: c.process_floor_pos,
                    altitude_kp: c.altitude_kp,
                    altitude_kd: c.altitude_kd,
                    altitude_ki: c.altitude_ki,
                    position_ki: c.position_ki,
                },
            )
            .unwrap();
    }
    fn configure_supervisor(&mut self, c: SupCalib) {
        use crate::exports::pulseengine::falcon_cascade::pilot::SupervisorConfig;
        use crate::pulseengine::falcon_cascade::types::Vec3;
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_configure_supervisor(
                &mut self.store,
                SupervisorConfig {
                    home: Vec3 {
                        x: c.home[0],
                        y: c.home[1],
                        z: c.home[2],
                    },
                    fence_radius: c.fence_radius,
                    cruise_alt: c.cruise_alt,
                    low_batt_v: c.low_batt_v,
                },
            )
            .unwrap();
    }
    fn configure_calibration(&mut self, k: SensorCal) {
        use crate::exports::pulseengine::falcon_cascade::pilot::SensorCalibration;
        use crate::pulseengine::falcon_cascade::types::Vec3;
        let v = |a: [f32; 3]| Vec3 {
            x: a[0],
            y: a[1],
            z: a[2],
        };
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_configure_calibration(
                &mut self.store,
                SensorCalibration {
                    gyro_bias: v(k.gyro_bias),
                    accel_bias: v(k.accel_bias),
                    accel_scale: v(k.accel_scale),
                    mag_offset: v(k.mag_offset),
                    mag_scale: v(k.mag_scale),
                    mag_offdiag: v(k.mag_offdiag),
                },
            )
            .unwrap();
    }
    fn command(&mut self, ev: Event) {
        use crate::exports::pulseengine::falcon_cascade::pilot::PilotCommand as P;
        let cmd = match ev {
            Event::Arm => P::Arm,
            Event::RequestTakeoff => P::RequestTakeoff,
            Event::RequestMission => P::RequestMission,
            Event::RequestLoiter => P::RequestLoiter,
            Event::RequestRtl => P::RequestRtl,
            Event::RequestLand => P::RequestLand,
            Event::RequestDisarm => P::RequestDisarm,
            Event::Terminate => P::Terminate,
            other => panic!("{other:?} is not a pilot command"),
        };
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_command(&mut self.store, cmd, true, true)
            .unwrap();
    }
    fn step(&mut self, f: &Frame, target: [f32; 3]) -> [f32; 4] {
        use crate::pulseengine::falcon_cascade::types::{
            ImuSample, RotorRpm, SensorFrame, Vec3, Waypoint,
        };
        let v = |a: [f32; 3]| Vec3 {
            x: a[0],
            y: a[1],
            z: a[2],
        };
        let frame = SensorFrame {
            imu: ImuSample {
                ax: f.accel[0],
                ay: f.accel[1],
                az: f.accel[2],
                gx: f.gyro[0],
                gy: f.gyro[1],
                gz: f.gyro[2],
            },
            dt_s: DT,
            position_ned: f.position.map(v),
            mag_body: f.mag.map(v),
            heading_rad: f.heading,
            motor_rpm: f.rpm.map(|r| RotorRpm {
                m1: r[0],
                m2: r[1],
                m3: r[2],
                m4: r[3],
            }),
            battery_v: f.battery_v,
        };
        let wp = Waypoint {
            north: target[0],
            east: target[1],
            down: target[2],
            yaw: 0.0,
        };
        let m = self
            .b
            .pulseengine_falcon_cascade_controller()
            .call_step(&mut self.store, frame, wp)
            .unwrap();
        [m.m1, m.m2, m.m3, m.m4]
    }
    fn mode(&mut self) -> Option<u8> {
        let s = self.state();
        (s.valid & (1 << 7) != 0).then_some(s.mode)
    }
    fn failed_rotor(&mut self) -> Option<u8> {
        let s = self.state();
        (s.valid & (1 << 6) != 0).then_some(s.failed_rotor)
    }
}

/// The mock plant plus the frame-building the harness does, so a scenario can
/// fly. GNSS at 5 Hz as in `main`.
struct Rig {
    plant: MockPhysics,
    tick: u32,
    motors: [f32; 4],
    battery_v: Option<f32>,
    /// When set, the accelerometer the FRAME reports instead of the plant's —
    /// a dead / free-falling sensor for the estimator-health scenarios.
    accel_override: Option<[f32; 3]>,
    /// When set, a fixed magnetometer reading the frame carries (the mock
    /// plant has no magnetometer) — so a mag calibration has something to act
    /// on.
    mag_override: Option<[f32; 3]>,
}

impl Rig {
    fn new() -> Self {
        let mut plant = MockPhysics::at_rest();
        // A floor, so a Disarmed (zero-motor) vehicle rests rather than falls.
        plant.set_ground_contact(true);
        Rig {
            plant,
            tick: 0,
            motors: [0.0; 4],
            battery_v: Some(PACK_V),
            accel_override: None,
            mag_override: None,
        }
    }
    /// Advance `n` ticks; returns the last motor command.
    fn fly<S: Seam>(&mut self, s: &mut S, n: u32) -> [f32; 4] {
        for _ in 0..n {
            let (imu, p) = self.plant.measure(0.0);
            let g = imu.gyro_body;
            let f = Frame {
                accel: self.accel_override.unwrap_or(imu.accel_body),
                // The SENSOR's gyro: plant rate + the bias `good_cal` removes.
                gyro: [
                    g[0] + GYRO_BIAS[0],
                    g[1] + GYRO_BIAS[1],
                    g[2] + GYRO_BIAS[2],
                ],
                position: self.tick.is_multiple_of(80).then_some(p),
                mag: self.mag_override.or(self.plant.mag_body_ned()),
                heading: self.plant.heading_ned(),
                rpm: self.plant.motor_rpm(),
                battery_v: self.battery_v,
            };
            self.motors = s.step(&f, [0.0, 0.0, -2.0]);
            self.plant.step(self.motors, DT);
            self.tick += 1;
        }
        self.motors
    }
    fn altitude(&self) -> f32 {
        -self.plant.p_ned[2]
    }
}

/// Step on the ground and send `arm` every tick until the MEASURED pre-arm
/// table allows it — the only way the component arms since #609 finding 2.
/// Returns the tick count it took, or `None` if it never armed within `max`.
fn arm_on_measured_table<S: Seam>(s: &mut S, rig: &mut Rig, max: u32) -> Option<u32> {
    for i in 0..max {
        rig.fly(s, 1);
        if s.mode() == Some(ARMED) {
            return Some(i);
        }
        s.command(Event::Arm);
    }
    None
}

/// Configure, arm on the measured table and request takeoff the way a host
/// does, then fly until airborne in Takeoff/Loiter. Panics if it cannot.
fn launch<S: Seam>(s: &mut S, rig: &mut Rig) {
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    let armed = arm_on_measured_table(s, rig, 4000);
    assert!(
        armed.is_some(),
        "precondition: arms on the measured table; mode {:?}",
        s.mode()
    );
    s.command(Event::RequestTakeoff);
    rig.fly(s, 2000);
    let m = s.mode();
    assert!(
        matches!(m, Some(TAKEOFF) | Some(LOITER)) && rig.altitude() > 1.0,
        "precondition: the vehicle should be airborne after launch; mode {m:?}, \
         altitude {:.2} m",
        rig.altitude()
    );
}

// ── #609 finding 1 ─────────────────────────────────────────────────────────

/// A re-configure (re-tune AND new supervisor config) while airborne must not
/// touch the live supervisor: still flying, motors still commanded.
fn reconfigure_in_flight_keeps_flying<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    launch(s, &mut rig);
    let before = s.mode();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(SupCalib {
        home: [5.0, 0.0, 0.0],
        ..good_sup()
    });
    let m = rig.fly(s, 200);
    let thrust: f32 = m.iter().sum();
    assert_eq!(
        s.mode(),
        before,
        "an in-flight configure changed the mode (a rebuild lands in Disarmed \
         and cuts the motors)"
    );
    assert!(
        thrust > 1.0,
        "an in-flight configure cut the motors: {m:?} (sum {thrust:.3})"
    );
    assert!(
        rig.altitude() > 1.0,
        "the vehicle fell after an in-flight configure: {:.2} m",
        rig.altitude()
    );
}

/// `Terminated` is absorbing: no configure path leads out of it, so a
/// subsequent `arm` cannot produce `Armed`.
fn terminated_is_absorbing_through_configure<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    rig.fly(s, 1);
    s.command(Event::Terminate);
    rig.fly(s, 1);
    assert_eq!(s.mode(), Some(TERMINATED), "precondition: terminated");
    s.configure_supervisor(good_sup());
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.command(Event::Arm);
    rig.fly(s, 5);
    assert_eq!(
        s.mode(),
        Some(TERMINATED),
        "configure + arm escaped the absorbing Terminated state"
    );
}

/// The #565 dead-motor interlock survives a configure. Fly, lose rotor 0 (the
/// FDI isolates it and the supervisor lands), touch down and disarm — then a
/// configure for "the next sortie" must not rebuild the supervisor and reset
/// `failed_motor` to `None`, and the following arm must be refused.
///
/// Measured on the mock plant: isolation within 0.25 s, Land, Disarmed after
/// ~3.25 s. The scenario waits up to 10 s for the disarm.
fn isolated_rotor_survives_configure<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    launch(s, &mut rig);
    rig.plant.fail_rotor(0);
    let mut disarmed = false;
    for _ in 0..40 {
        rig.fly(s, 100);
        if s.mode() == Some(DISARMED) {
            disarmed = true;
            break;
        }
    }
    assert!(
        disarmed,
        "precondition: landed and disarmed; mode {:?}",
        s.mode()
    );
    assert_eq!(s.failed_rotor(), Some(0), "precondition: rotor 0 isolated");

    s.configure_supervisor(good_sup());
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.command(Event::Arm);
    s.command(Event::RequestTakeoff);
    rig.fly(s, 5);
    assert_eq!(
        s.failed_rotor(),
        Some(0),
        "a configure reset the isolated-rotor latch (#565)"
    );
    assert_eq!(
        s.mode(),
        Some(DISARMED),
        "a vehicle with an isolated rotor re-armed after a configure"
    );
}

// ── #609 finding 3 ─────────────────────────────────────────────────────────

/// Every invalid safety configuration, each of which `sup_params` used to map
/// to a PERMISSIVE value (geofence off, battery failsafe unreachable, 2 m
/// cruise). Units/signs read from the WIT: fence-radius m (> 0), cruise-alt m
/// AGL POSITIVE, low-batt-v whole-pack V (> 0), home NED m (finite).
fn invalid_sup_configs() -> Vec<(&'static str, SupCalib)> {
    let g = good_sup();
    vec![
        (
            "fence NaN",
            SupCalib {
                fence_radius: f32::NAN,
                ..g
            },
        ),
        (
            "fence 0",
            SupCalib {
                fence_radius: 0.0,
                ..g
            },
        ),
        (
            "fence -5",
            SupCalib {
                fence_radius: -5.0,
                ..g
            },
        ),
        (
            "fence +inf",
            SupCalib {
                fence_radius: f32::INFINITY,
                ..g
            },
        ),
        (
            "low-batt NaN",
            SupCalib {
                low_batt_v: f32::NAN,
                ..g
            },
        ),
        (
            "low-batt -14",
            SupCalib {
                low_batt_v: -14.0,
                ..g
            },
        ),
        (
            "low-batt 0",
            SupCalib {
                low_batt_v: 0.0,
                ..g
            },
        ),
        (
            "low-batt +inf",
            SupCalib {
                low_batt_v: f32::INFINITY,
                ..g
            },
        ),
        (
            "cruise -30",
            SupCalib {
                cruise_alt: -30.0,
                ..g
            },
        ),
        (
            "cruise 0",
            SupCalib {
                cruise_alt: 0.0,
                ..g
            },
        ),
        (
            "cruise NaN",
            SupCalib {
                cruise_alt: f32::NAN,
                ..g
            },
        ),
        (
            "home.x NaN",
            SupCalib {
                home: [f32::NAN, 0.0, 0.0],
                ..g
            },
        ),
    ]
}

/// Every invalid CALIBRATION (`relay_calib::CalParams::is_valid`: finite, accel
/// scale > 0, soft-iron positive definite). The IDENTITY is NOT here: since
/// jess#167 an explicitly configured identity is VALID ("calibrated upstream")
/// and arms — see `identity_calibration_arms`.
fn bad_calibrations() -> Vec<(&'static str, SensorCal)> {
    let c = good_cal();
    vec![
        (
            "gyro-bias NaN",
            SensorCal {
                gyro_bias: [f32::NAN, 0.0, 0.0],
                ..c
            },
        ),
        (
            "accel-bias +inf",
            SensorCal {
                accel_bias: [0.0, f32::INFINITY, 0.0],
                ..c
            },
        ),
        (
            "accel-scale 0",
            SensorCal {
                accel_scale: [1.0, 1.0, 0.0],
                ..c
            },
        ),
        (
            "mag-scale -1",
            SensorCal {
                mag_scale: [-1.0, 1.0, 1.0],
                ..c
            },
        ),
        (
            "mag-offset NaN",
            SensorCal {
                mag_offset: [0.0, 0.0, f32::NAN],
                ..c
            },
        ),
        (
            "mag-offdiag NaN",
            SensorCal {
                mag_offdiag: [0.0, f32::NAN, 0.0],
                ..c
            },
        ),
        (
            // Finite, every scale > 0, but S = [[1,2,0],[2,1,0],[0,0,1]] is
            // indefinite (eigenvalues 3, -1, 1): it MIRRORS a field direction.
            "soft-iron not positive definite",
            SensorCal {
                mag_offdiag: [2.0, 0.0, 0.0],
                ..c
            },
        ),
    ]
}

/// Ticks a VALID configuration is given on the mock rig before the measured
/// table must allow `arm` (measured: 2), with a wide margin; every refusal
/// case below is given the same window.
const ARM_WINDOW: u32 = 4000;

/// Configure (with whatever supervisor config / calibration the case wants),
/// then try BOTH arm paths: before the first step, and on the measured table
/// for `ARM_WINDOW` ticks. Returns how it armed, if it did.
fn try_arm<S: Seam>(s: &mut S, sup: SupCalib, cal: SensorCal) -> Option<String> {
    try_arm_with(s, sup, Some(cal))
}

/// `try_arm`, where `None` = `configure-calibration` is never called.
fn try_arm_with<S: Seam>(s: &mut S, sup: SupCalib, cal: Option<SensorCal>) -> Option<String> {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    if let Some(cal) = cal {
        s.configure_calibration(cal);
    }
    s.configure_supervisor(sup);
    s.command(Event::Arm);
    s.command(Event::RequestTakeoff);
    rig.fly(s, 5);
    let m = s.mode();
    if m != Some(DISARMED) {
        return Some(format!("pre-step -> mode {m:?}"));
    }
    arm_on_measured_table(s, &mut rig, ARM_WINDOW)
        .map(|t| format!("measured -> armed after {t} ticks"))
}

/// An invalid safety configuration must leave the vehicle unarmable, not
/// armable with a failsafe silently switched off — WITH A VALID CALIBRATION, so
/// `CalibrationPresent` passes and the refusal can only come from
/// `FailsafeConfigured` (the mirror test below names the row).
fn invalid_supervisor_config_refuses_arm<S: Seam>(mk: impl Fn() -> S) {
    // Every case is run and the offenders reported together.
    let mut armed = Vec::new();
    for (name, bad) in invalid_sup_configs() {
        if let Some(how) = try_arm(&mut mk(), bad, good_cal()) {
            armed.push(format!("{name}: {how}"));
        }
    }
    assert!(
        armed.is_empty(),
        "invalid supervisor configs were ARMED: {armed:?}"
    );
}

/// An INVALID calibration must leave the vehicle unarmable even with a VALID
/// supervisor config.
fn bad_calibration_refuses_arm<S: Seam>(mk: impl Fn() -> S) {
    let mut armed = Vec::new();
    for (name, bad) in bad_calibrations() {
        if let Some(how) = try_arm(&mut mk(), good_sup(), bad) {
            armed.push(format!("{name}: {how}"));
        }
    }
    assert!(armed.is_empty(), "bad calibrations were ARMED: {armed:?}");
}

/// NEVER configured: `CalibrationPresent` is false, so `arm` is refused on
/// both paths (before the first step, and on the measured table).
fn never_configured_calibration_refuses_arm<S: Seam>(s: &mut S) {
    let how = try_arm_with(s, good_sup(), None);
    assert!(
        how.is_none(),
        "armed with NO calibration ever configured: {how:?}"
    );
}

/// An explicitly configured IDENTITY is a VALID calibration ("calibrated
/// upstream, no residual", jess#167) and ARMS. The rig's sensor carries
/// `GYRO_BIAS`, which the identity leaves in — harmless at 0.1 °/s, and it is
/// the host's assertion that matters to the pre-arm row, not the residual.
fn identity_calibration_arms<S: Seam>(s: &mut S) {
    let how = try_arm(s, good_sup(), identity_cal());
    assert!(
        how.is_some(),
        "an explicitly configured IDENTITY calibration was REFUSED; mode {:?}",
        s.mode()
    );
}

/// THE CONTROL for every refusal in this file: the SAME rig with a valid
/// configuration AND a valid calibration arms on the measured table.
fn valid_config_arms_on_the_measured_table<S: Seam>(s: &mut S) -> u32 {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    let t = arm_on_measured_table(s, &mut rig, ARM_WINDOW);
    assert!(
        t.is_some(),
        "a VALID configuration did not arm on the measured table within \
         {ARM_WINDOW} ticks; mode {:?}",
        s.mode()
    );
    t.unwrap()
}

// ── configure-calibration lifecycle (relay#586) ─────────────────────────────

/// An INVALID calibration (it would revoke) — the observable probe for
/// "was this call applied?": applied while Disarmed, the next arm is refused.
fn invalid_cal() -> SensorCal {
    SensorCal {
        accel_scale: [0.0, 1.0, 1.0],
        ..good_cal()
    }
}

/// Accepted only while Disarmed: an INVALID calibration sent in flight must be
/// discarded (had it been applied it would have revoked the valid one), so
/// after landing the vehicle is still calibrated. Observable through the seam
/// as: land, disarm, and the next arm still succeeds. (It used the identity
/// as the probe until jess#167 made the identity valid — then the re-arm
/// would succeed either way and the test could not fail.)
fn calibration_is_rejected_while_not_disarmed<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    launch(s, &mut rig);
    s.configure_calibration(invalid_cal());
    let mut disarmed = false;
    for _ in 0..80 {
        // RequestLand is only honoured from Loiter/Mission/Rtl, and `launch`
        // may still be in Takeoff — so it is re-sent until it takes.
        s.command(Event::RequestLand);
        rig.fly(s, 100);
        if s.mode() == Some(DISARMED) {
            disarmed = true;
            break;
        }
    }
    assert!(
        disarmed,
        "precondition: landed and disarmed; mode {:?}",
        s.mode()
    );
    assert!(
        arm_on_measured_table(s, &mut rig, ARM_WINDOW).is_some(),
        "the in-flight invalid calibration was APPLIED (re-arm refused) — it \
         must have been rejected because the vehicle was not disarmed"
    );
}

/// On the ground it IS applied, in place: an invalid calibration sent while
/// Disarmed makes the next arm fail, and a valid one — here the IDENTITY —
/// restores it.
fn calibration_applies_in_place_on_the_ground<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    rig.fly(s, 10);
    s.configure_calibration(invalid_cal());
    assert!(
        arm_on_measured_table(s, &mut rig, ARM_WINDOW).is_none(),
        "an invalid calibration sent while Disarmed was not applied"
    );
    s.configure_calibration(identity_cal());
    assert!(
        arm_on_measured_table(s, &mut rig, 10).is_some(),
        "a valid (identity) calibration sent while Disarmed was not applied in place"
    );
}

/// An INVALID calibration REVOKES an earlier valid one — the vehicle must not
/// fly on a calibration the host has just tried to replace.
fn invalid_calibration_clears_a_previous_valid_one<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    s.configure_calibration(SensorCal {
        accel_scale: [0.0, 1.0, 1.0],
        ..good_cal()
    });
    assert!(
        arm_on_measured_table(s, &mut rig, ARM_WINDOW).is_none(),
        "an invalid calibration left the earlier valid one in force"
    );
}

/// A rebuild by configure-supervisor keeps the calibration (sent first).
fn calibration_survives_a_rebuild<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure_calibration(good_cal());
    s.configure(mock_calib());
    s.configure_supervisor(good_sup());
    rig.fly(s, 10);
    s.configure_supervisor(good_sup()); // Disarmed, unlatched -> rebuild
    assert!(
        arm_on_measured_table(s, &mut rig, ARM_WINDOW).is_some(),
        "the calibration was lost in a configure-supervisor rebuild"
    );
}

// ── relay#586 acceptance: estimator-health rows as the DECIDING row ────────

/// `EstimatorConverged` decides. With calibration flowing and everything else
/// valid, an accelerometer that reports NOTHING (|a| < 1e-3 — dead sensor or
/// free fall) starves the IEKF gravity update, the tilt covariance grows past
/// `PREARM_TILT_UNCERT_MAX` (it STARTS exactly at it: p0 = 0.1 rad per axis,
/// 0.01 + 0.01 = 0.02, and the row is `< 0.02`), and arming is refused — while
/// the same rig with the plant's accelerometer arms.
fn dead_accelerometer_refuses_arm<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    rig.accel_override = Some([0.0, 0.0, 0.0]);
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    assert!(
        arm_on_measured_table(s, &mut rig, ARM_WINDOW).is_none(),
        "armed with a dead accelerometer (EstimatorConverged should refuse)"
    );
}

// ── #609 finding 2 ─────────────────────────────────────────────────────────

/// `arm` on a supervisor that has never been stepped is judged against
/// nothing measured, so it must be REFUSED — not passed on the constructor's
/// forced-true legacy rows with every breadth row undeclared.
fn fresh_supervisor_refuses_arm_before_first_step<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    s.command(Event::Arm);
    s.command(Event::RequestTakeoff);
    rig.fly(s, 5);
    assert_eq!(
        s.mode(),
        Some(DISARMED),
        "a never-stepped supervisor ARMED on its bring-up defaults"
    );
}

/// The same through the REBUILD path: a configure on the ground (permitted)
/// rebuilds the supervisor, and the next `arm` must not be judged against the
/// rebuilt supervisor's defaults.
fn rebuilt_supervisor_refuses_arm_before_first_step<S: Seam>(s: &mut S) {
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_calibration(good_cal());
    s.configure_supervisor(good_sup());
    rig.fly(s, 20);
    assert_eq!(s.mode(), Some(DISARMED), "precondition: on the ground");
    s.configure_supervisor(good_sup());
    s.command(Event::Arm);
    s.command(Event::RequestTakeoff);
    rig.fly(s, 5);
    assert_eq!(
        s.mode(),
        Some(DISARMED),
        "a configure-rebuilt supervisor ARMED on its bring-up defaults"
    );
}

// ── mag-offdiag (jess#167) ──────────────────────────────────────────────────

/// A valid calibration WITH a soft-iron off-diagonal: positive definite
/// (diagonally dominant), every term distinct.
fn offdiag_cal() -> SensorCal {
    SensorCal {
        mag_offset: [0.01, -0.02, 0.015],
        mag_scale: [1.05, 0.95, 1.1],
        mag_offdiag: [0.04, -0.03, 0.02],
        ..good_cal()
    }
}

/// Launch on `cal` and return the bit pattern of every motor command over a
/// fixed flight.
fn motor_trace<S: Seam>(s: &mut S, cal: SensorCal) -> Vec<u32> {
    let mut rig = Rig::new();
    // A body-frame field with all three components non-zero, so every
    // off-diagonal term multiplies something.
    rig.mag_override = Some([0.21, 0.07, 0.43]);
    s.configure(mock_calib());
    s.configure_calibration(cal);
    s.configure_supervisor(good_sup());
    let armed = arm_on_measured_table(s, &mut rig, ARM_WINDOW);
    assert!(armed.is_some(), "precondition: arms; mode {:?}", s.mode());
    s.command(Event::RequestTakeoff);
    let mut out = Vec::new();
    for _ in 0..1500 {
        out.extend(rig.fly(s, 1).iter().map(|m| m.to_bits()));
    }
    out
}

/// The off-diagonal CROSSES THE SEAM and is applied identically on both
/// sides: the shipped component and the native mirror, given the same
/// off-diagonal calibration, command bit-identical motors — AND the channel is
/// LIVE: the same flight with the off-diagonal zeroed commands different
/// motors. (Without the second half this would be the equivalence test that
/// is blind to a channel neither side receives.)
#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_mag_offdiag_crosses_the_seam_bit_identically() {
    let wasm = motor_trace(&mut WasmSeam::load(), offdiag_cal());
    let native = motor_trace(&mut NativeCascade::new(), offdiag_cal());
    assert_eq!(wasm.len(), native.len());
    let first = wasm.iter().zip(&native).position(|(a, b)| a != b);
    assert!(
        first.is_none(),
        "wasm and native disagree with a non-zero mag-offdiag, first at motor word {first:?}"
    );
    let zeroed = motor_trace(
        &mut NativeCascade::new(),
        SensorCal {
            mag_offdiag: [0.0; 3],
            ..offdiag_cal()
        },
    );
    assert_ne!(
        native, zeroed,
        "zeroing mag-offdiag changed NOTHING — the comparison above is blind to the field"
    );
}

/// Mirror-side: a positive-definite off-diagonal calibration is VALID (arms).
#[test]
fn mirror_offdiag_calibration_is_accepted() {
    let mut n = NativeCascade::new();
    let _ = motor_trace(&mut n, offdiag_cal());
}

// ── configure-calibration soak: the SPILLED argument area (#574) ────────────

const SOAK_CALLS: u32 = 10_000;

/// `configure-calibration` is 18 flat f32 > MAX_FLAT_PARAMS, so EVERY call
/// makes the host `cabi_realloc` a 72 B argument area in the component's 8 KiB
/// bump arena; without per-call reclamation the arena is exhausted after
/// ~113 calls. 10 000 calls through the Component Model API — cycling a
/// valid, an invalid and an off-diagonal calibration — must neither trap nor
/// leave the component unable to arm and fly on the LAST (valid) one.
///
/// Named outside the `arena_soak` filter on purpose: FV-FALCON-OCI-006 runs
/// `--ignored arena_soak` with only the five leaf `FALCON_*_WASM` variables.
#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_configure_calibration_soak_10k_calls() {
    let mut s = WasmSeam::load();
    let mut rig = Rig::new();
    s.configure(mock_calib());
    s.configure_supervisor(good_sup());
    let invalid = SensorCal {
        mag_offdiag: [2.0, 0.0, 0.0],
        ..good_cal()
    };
    for i in 0..SOAK_CALLS {
        let c = match i % 3 {
            0 => good_cal(),
            1 => invalid,
            _ => offdiag_cal(),
        };
        s.configure_calibration(c);
        if i % 1000 == 0 {
            rig.fly(&mut s, 1); // interleave a spilled `controller#step` too
        }
    }
    s.configure_calibration(offdiag_cal());
    assert!(
        arm_on_measured_table(&mut s, &mut rig, ARM_WINDOW).is_some(),
        "after {SOAK_CALLS} configure-calibration calls the component no longer arms"
    );
    s.command(Event::RequestTakeoff);
    rig.fly(&mut s, 2000);
    assert!(
        rig.altitude() > 1.0,
        "after the soak the vehicle did not take off; altitude {:.2} m",
        rig.altitude()
    );
}

/// The same at the CORE level, emulating the host exactly as the canonical
/// ABI does it (`cabi_realloc(0, 0, 4, 72)`, write the record, call the core
/// export with the pointer) — the oracle that sees exhaustion the component
/// API hides (see arena_soak.rs: wasmtime would accept a NULL as address 0).
/// Asserts the core export takes ONE i32 (the indirect lowering) and the
/// argument area never moves and is never NULL across 10 000 calls.
#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_configure_calibration_core_area_is_pinned() {
    use wasmtime::{Engine, Instance, Module, Store, ValType};
    let path = std::env::var("CASCADE_WASM").expect("CASCADE_WASM must name the component");
    let bytes = std::fs::read(path).unwrap();
    let engine = Engine::default();
    let module = Module::new(&engine, crate::arena_soak::core_module(&bytes)).unwrap();
    assert_eq!(
        module.imports().len(),
        0,
        "cascade core module imports nothing"
    );
    let name = module
        .exports()
        .map(|e| e.name().to_string())
        .find(|n| n.ends_with("#configure-calibration"))
        .expect("no configure-calibration core export");
    let ty = module.get_export(&name).unwrap().func().unwrap().clone();
    let params: Vec<ValType> = ty.params().collect();
    assert!(
        params.len() == 1 && matches!(params[0], ValType::I32) && ty.results().len() == 0,
        "configure-calibration is expected to lower INDIRECTLY to (param i32); \
         measured {} params",
        params.len()
    );
    let mut store = Store::new(&engine, ());
    let inst = Instance::new(&mut store, &module, &[]).unwrap();
    let memory = inst.get_memory(&mut store, "memory").unwrap();
    let realloc = inst
        .get_typed_func::<(i32, i32, i32, i32), i32>(&mut store, "cabi_realloc")
        .unwrap();
    let call = inst.get_typed_func::<i32, ()>(&mut store, &name).unwrap();
    let mut first = None;
    for i in 0..SOAK_CALLS {
        let p = realloc
            .call(&mut store, (0, 0, 4, 72))
            .unwrap_or_else(|e| panic!("call {i}: cabi_realloc trapped: {e}"));
        assert_ne!(p, 0, "call {i}: cabi_realloc returned NULL");
        assert_eq!(
            p,
            *first.get_or_insert(p),
            "call {i}: the 72 B area moved (leak)"
        );
        let c = if i % 2 == 0 {
            good_cal()
        } else {
            offdiag_cal()
        };
        let vals: Vec<f32> = [
            c.gyro_bias,
            c.accel_bias,
            c.accel_scale,
            c.mag_offset,
            c.mag_scale,
            c.mag_offdiag,
        ]
        .concat();
        let raw: Vec<u8> = vals.iter().flat_map(|v| v.to_le_bytes()).collect();
        assert_eq!(raw.len(), 72);
        memory.write(&mut store, p as usize, &raw).unwrap();
        call.call(&mut store, p)
            .unwrap_or_else(|e| panic!("call {i}: configure-calibration trapped: {e}"));
    }
    println!(
        "configure-calibration: {SOAK_CALLS} core calls, 72 B area pinned at {:#x}",
        first.unwrap()
    );
}

// ── native mirror: every `cargo test` ─────────────────────────────────────

/// One `#[test]` per scenario on the mirror, and an `#[ignore]`d twin on the
/// shipped component — written once here so the two lists cannot drift.
macro_rules! both {
    ($mirror:ident, $wasm:ident, $body:expr) => {
        #[test]
        fn $mirror() {
            let f: fn(&mut NativeCascade) = $body;
            f(&mut NativeCascade::new());
        }
        #[test]
        #[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
        fn $wasm() {
            let f: fn(&mut WasmSeam) = $body;
            f(&mut WasmSeam::load());
        }
    };
}

both!(
    mirror_reconfigure_in_flight_keeps_flying,
    wasm_reconfigure_in_flight_keeps_flying,
    |s| reconfigure_in_flight_keeps_flying(s)
);
both!(
    mirror_terminated_is_absorbing_through_configure,
    wasm_terminated_is_absorbing_through_configure,
    |s| terminated_is_absorbing_through_configure(s)
);
both!(
    mirror_isolated_rotor_survives_configure,
    wasm_isolated_rotor_survives_configure,
    |s| isolated_rotor_survives_configure(s)
);
both!(
    mirror_fresh_supervisor_refuses_arm_before_first_step,
    wasm_fresh_supervisor_refuses_arm_before_first_step,
    |s| fresh_supervisor_refuses_arm_before_first_step(s)
);
both!(
    mirror_rebuilt_supervisor_refuses_arm_before_first_step,
    wasm_rebuilt_supervisor_refuses_arm_before_first_step,
    |s| rebuilt_supervisor_refuses_arm_before_first_step(s)
);
both!(
    mirror_valid_config_arms_on_the_measured_table,
    wasm_valid_config_arms_on_the_measured_table,
    |s| {
        valid_config_arms_on_the_measured_table(s);
    }
);
both!(
    mirror_calibration_is_rejected_while_not_disarmed,
    wasm_calibration_is_rejected_while_not_disarmed,
    |s| calibration_is_rejected_while_not_disarmed(s)
);
both!(
    mirror_calibration_applies_in_place_on_the_ground,
    wasm_calibration_applies_in_place_on_the_ground,
    |s| calibration_applies_in_place_on_the_ground(s)
);
both!(
    mirror_invalid_calibration_clears_a_previous_valid_one,
    wasm_invalid_calibration_clears_a_previous_valid_one,
    |s| invalid_calibration_clears_a_previous_valid_one(s)
);
both!(
    mirror_calibration_survives_a_rebuild,
    wasm_calibration_survives_a_rebuild,
    |s| calibration_survives_a_rebuild(s)
);
both!(
    mirror_never_configured_calibration_refuses_arm,
    wasm_never_configured_calibration_refuses_arm,
    |s| never_configured_calibration_refuses_arm(s)
);
both!(
    mirror_identity_calibration_arms,
    wasm_identity_calibration_arms,
    |s| identity_calibration_arms(s)
);
both!(
    mirror_dead_accelerometer_refuses_arm,
    wasm_dead_accelerometer_refuses_arm,
    |s| dead_accelerometer_refuses_arm(s)
);

#[test]
fn mirror_invalid_supervisor_config_refuses_arm() {
    invalid_supervisor_config_refuses_arm(NativeCascade::new);
}
#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_invalid_supervisor_config_refuses_arm() {
    invalid_supervisor_config_refuses_arm(WasmSeam::load);
}
#[test]
fn mirror_bad_calibration_refuses_arm() {
    bad_calibration_refuses_arm(NativeCascade::new);
}
#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_bad_calibration_refuses_arm() {
    bad_calibration_refuses_arm(WasmSeam::load);
}

/// Names the row: with nothing measured, the FIRST row in gate order
/// (`SensorsHealthy`) refuses — the legacy rows are no longer forced true.
#[test]
fn mirror_fresh_supervisor_is_refused_by_an_unmeasured_row() {
    let mut n = NativeCascade::new();
    n.configure(mock_calib());
    n.configure_calibration(good_cal());
    n.configure_supervisor(good_sup());
    n.command(Event::Arm, true, true);
    assert_eq!(n.arm_blocked_check(), Some(CheckId::SensorsHealthy));
}

/// Warm a mirror on the rig for as long as a valid config needs (+ margin) and
/// return the row refusing `arm` (None = allowed).
fn deciding_row(sup: SupCalib, cal: SensorCal, accel: Option<[f32; 3]>) -> Option<CheckId> {
    deciding_row_with(sup, Some(cal), accel)
}

/// `deciding_row`, where `None` = `configure-calibration` is never called.
fn deciding_row_with(
    sup: SupCalib,
    cal: Option<SensorCal>,
    accel: Option<[f32; 3]>,
) -> Option<CheckId> {
    let warm = valid_config_arms_on_the_measured_table(&mut NativeCascade::new());
    let mut n = NativeCascade::new();
    let mut rig = Rig::new();
    rig.accel_override = accel;
    n.configure(mock_calib());
    if let Some(cal) = cal {
        n.configure_calibration(cal);
    }
    n.configure_supervisor(sup);
    rig.fly(&mut n, warm + 50);
    n.arm_blocked_check()
}

/// THE DECIDING ROW, NAMED, for every refusal scenario the seam can produce —
/// the relay#586 acceptance criterion, and the reason `FailsafeConfigured` is
/// now independently tested: with a VALID calibration flowing,
/// `CalibrationPresent` passes, so an invalid supervisor config is refused BY
/// `FailsafeConfigured` and nothing earlier in gate order.
#[test]
fn mirror_each_refusal_is_decided_by_the_expected_row() {
    // The control: nothing refuses.
    assert_eq!(deciding_row(good_sup(), good_cal(), None), None, "control");
    for (name, bad) in invalid_sup_configs() {
        assert_eq!(
            deciding_row(bad, good_cal(), None),
            Some(CheckId::FailsafeConfigured),
            "invalid supervisor config {name}"
        );
    }
    for (name, bad) in bad_calibrations() {
        assert_eq!(
            deciding_row(good_sup(), bad, None),
            Some(CheckId::CalibrationPresent),
            "bad calibration {name}"
        );
    }
    // Never configured -> CalibrationPresent; an explicit identity -> nothing.
    assert_eq!(
        deciding_row_with(good_sup(), None, None),
        Some(CheckId::CalibrationPresent),
        "never-configured calibration"
    );
    assert_eq!(
        deciding_row(good_sup(), identity_cal(), None),
        None,
        "an explicitly configured identity calibration must not refuse"
    );
    // Estimator health: a dead accelerometer -> EstimatorConverged decides.
    assert_eq!(
        deciding_row(good_sup(), good_cal(), Some([0.0, 0.0, 0.0])),
        Some(CheckId::EstimatorConverged),
        "dead accelerometer"
    );
}
