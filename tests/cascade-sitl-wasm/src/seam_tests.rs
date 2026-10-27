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

use crate::native_mirror::{Calib, NativeCascade, SupCalib};
use crate::physics::{MockPhysics, Physics};
use falcon_core::relay_fsm::Event;

const DT: f32 = 0.0025;
const PACK_V: f32 = 16.8;
const LOW_BATT_V: f32 = 14.0;

const DISARMED: u8 = 0;
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
        }
    }
    /// Advance `n` ticks; returns the last motor command.
    fn fly<S: Seam>(&mut self, s: &mut S, n: u32) -> [f32; 4] {
        for _ in 0..n {
            let (imu, p) = self.plant.measure(0.0);
            let f = Frame {
                accel: imu.accel_body,
                gyro: imu.gyro_body,
                position: self.tick.is_multiple_of(80).then_some(p),
                mag: self.plant.mag_body_ned(),
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

/// Configure, arm and request takeoff the way a host does today, then fly
/// until airborne in Takeoff/Loiter. Panics (with the mode) if it cannot.
fn launch<S: Seam>(s: &mut S, rig: &mut Rig) {
    s.configure(mock_calib());
    s.configure_supervisor(good_sup());
    s.command(Event::Arm);
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
    s.configure_supervisor(good_sup());
    rig.fly(s, 1);
    s.command(Event::Terminate);
    rig.fly(s, 1);
    assert_eq!(s.mode(), Some(TERMINATED), "precondition: terminated");
    s.configure_supervisor(good_sup());
    s.configure(mock_calib());
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

// ── native mirror: every `cargo test` ─────────────────────────────────────

#[test]
fn mirror_reconfigure_in_flight_keeps_flying() {
    reconfigure_in_flight_keeps_flying(&mut NativeCascade::new());
}

#[test]
fn mirror_terminated_is_absorbing_through_configure() {
    terminated_is_absorbing_through_configure(&mut NativeCascade::new());
}

#[test]
fn mirror_isolated_rotor_survives_configure() {
    isolated_rotor_survives_configure(&mut NativeCascade::new());
}

// ── the shipped component: `CASCADE_WASM=... cargo test -- --ignored` ────

#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_reconfigure_in_flight_keeps_flying() {
    reconfigure_in_flight_keeps_flying(&mut WasmSeam::load());
}

#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_terminated_is_absorbing_through_configure() {
    terminated_is_absorbing_through_configure(&mut WasmSeam::load());
}

#[test]
#[ignore = "needs CASCADE_WASM=<built falcon_cascade_cm.wasm>"]
fn wasm_isolated_rotor_survives_configure() {
    isolated_rotor_survives_configure(&mut WasmSeam::load());
}
