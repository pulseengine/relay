//! The flight core, run NATIVELY — the control side of the equivalence test.
//!
//! This exists for one claim: **you develop in wasm and deploy that same wasm,
//! unchanged**. That is only true if the Component Model is transparent — if the
//! identical Rust, reached through a wasm component boundary, computes the
//! identical answer.
//!
//! v0.8 rework. The previous mirror wired five stage crates (relay-pos /
//! relay-att / relay-rate / relay-iekf / relay-mix-quad) because that is what
//! the cascade component wrapped. #393 replaced that component with one wrapping
//! `falcon_core::FlightCore`, so this mirror follows it. Keeping the old mirror
//! would have compared two different programs and reported a difference meaning
//! nothing — a differential is only evidence while both sides run the same code.
//!
//! @0.12.0 rework, and it is the same lesson arriving a second time
//! (SUPERVISOR-P01). The component now wraps `falcon_core::FlightSupervisor`,
//! so this mirror does too. It was NOT updated in the first pass of that change
//! and the gz gate caught it immediately: wasm and native disagreed by
//! 0.069364190 at tick 6249 — a real difference reported honestly, about two
//! different programs rather than about the boundary. That is the differential
//! doing exactly its job, and the reason the gate is required.
//!
//! WHAT MIRRORING NOW REQUIRES, beyond swapping the type:
//!   * the supervisor's own configuration (home / fence / cruise / low-batt),
//!   * a battery reading, because `read_battery_v` defaults to `None` and the
//!     pre-arm table ANDs `BatteryOk`,
//!   * the arm + takeoff commands, because `FlightSupervisor::step` commands
//!     ZERO motors while Disarmed — an unarmed mirror would sit on the ground
//!     while the wasm side flew, and report that as a boundary failure,
//!   * construction through `FlightSupervisor::new`, NOT `FlightCore::new`.
//!     That one is subtle: the supervisor hardcodes `FlightCore::new(0.5,
//!     1000.0)` internally, and the warm-up count is seeded as
//!     `(loop_hz * 0.2).max(10)` — 200 at 1000 Hz. The old mirror built at
//!     `1/dt` (80 at 400 Hz) and does not resync, because only the loop RATE
//!     resyncs from the backend, not the warm-up. A mirror that kept building
//!     the core directly would differ in FDI/innovation warm-up alone.
//!
//! #609 single-source refactor: the supervisor, its construction and every
//! seam rule now come from `falcon_core::host::SupervisorHost` — the SAME type
//! and code the component holds — so both bullets above are satisfied by
//! construction rather than by a hand-kept copy.

use falcon_core::host::{self, SupervisorHost};
use falcon_core::relay_fsm::Event;
#[cfg(test)]
use falcon_core::relay_preflight::CheckId;
use falcon_core::{FlightBackend, ImuSample as CoreImu};

type Vec3 = [f32; 3];

/// One tick's sensors in, motors out — the same shim `wasm/cm/cascade` uses.
///
/// `FlightSupervisor` pulls from a backend and pushes motors into it, which is
/// the seam that lets the same code run against SITL, Gazebo or hardware.
/// Mirroring the component's shim exactly (including returning `None` where the
/// frame carried nothing, so the core skips that fusion rather than being handed
/// a fabricated zero) is what keeps the comparison honest.
struct FrameBackend {
    imu: CoreImu,
    position: Option<Vec3>,
    mag: Option<Vec3>,
    heading: Option<f32>,
    motor_rpm: Option<[i32; 4]>,
    /// @0.12.0. The component overrides this now, so the mirror must too —
    /// otherwise the mirror's battery failsafe and `BatteryOk` row see a
    /// different world from the component's.
    battery_v: Option<f32>,
    dt: f32,
    motors: [f32; 4],
}

impl FlightBackend for FrameBackend {
    fn read_imu(&mut self) -> CoreImu {
        self.imu
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.mag
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.heading
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.motor_rpm
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.position
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.battery_v
    }
    fn write_motors(&mut self, motors: &[f32]) {
        for (i, m) in self.motors.iter_mut().enumerate() {
            *m = motors.get(i).copied().unwrap_or(0.0);
        }
    }
    fn dt(&self) -> f32 {
        self.dt
    }
}

/// The mirror of the WIT `vehicle-config` record. Deliberately a separate type
/// rather than the generated one: the mirror must not be able to compile against
/// a seam the component does not actually export.
#[derive(Clone, Copy)]
pub struct Calib {
    pub hover_thrust: f32,
    pub loop_rate_hz: f32,
    pub pos_var: f32,
    pub process_floor_vel: f32,
    pub process_floor_pos: f32,
    pub altitude_kp: f32,
    pub altitude_kd: f32,
    pub altitude_ki: f32,
    pub position_ki: f32,
}

/// The mirror of the WIT `supervisor-config` record (@0.12.0), separate for the
/// same reason as `Calib`.
///
/// `cruise_alt` is metres AGL and POSITIVE — the one altitude in the seam that
/// is not NED-signed. The supervisor negates it itself when deriving setpoints.
#[derive(Clone, Copy)]
pub struct SupCalib {
    pub home: Vec3,
    pub fence_radius: f32,
    pub cruise_alt: f32,
    pub low_batt_v: f32,
}

/// The mirror of the WIT `sensor-calibration` record — `relay_calib::CalParams`
/// field for field. Separate type for the same reason as `Calib`.
#[derive(Clone, Copy)]
pub struct SensorCal {
    pub gyro_bias: Vec3,
    pub accel_bias: Vec3,
    pub accel_scale: Vec3,
    pub mag_offset: Vec3,
    pub mag_scale: Vec3,
    /// `[xy, xz, yz]`, as the WIT field.
    pub mag_offdiag: Vec3,
}

/// THE SAME `SupervisorHost` THE COMPONENT HOLDS (#609, single-source
/// refactor). Until then this file re-implemented the component's seam rules
/// (`supcfg_valid`, `cal_params`, `sup_params`, `ROWS_FED_BY_STEP`,
/// `rebuild_permitted`, `ensure_supervisor`, the configure guards) line for
/// line, so the differential compared two copies of the logic as well as the
/// boundary. Now both sides call `falcon_core::host`, and what this mirror
/// still owns is exactly what the component owns: the field-for-field record
/// conversions and the frame backend. A difference is therefore about the ABI.
pub struct NativeCascade {
    host: SupervisorHost,
}

impl Default for NativeCascade {
    fn default() -> Self {
        Self::new()
    }
}

impl NativeCascade {
    pub fn new() -> Self {
        Self {
            host: SupervisorHost::new(),
        }
    }

    /// Mirrors `Component::configure`: field-for-field into
    /// `SupervisorHost::configure_vehicle`.
    pub fn configure(&mut self, cfg: Calib) {
        self.host.configure_vehicle(host::VehicleConfig {
            hover_thrust: cfg.hover_thrust,
            loop_rate_hz: cfg.loop_rate_hz,
            pos_var: cfg.pos_var,
            process_floor_vel: cfg.process_floor_vel,
            process_floor_pos: cfg.process_floor_pos,
            altitude_kp: cfg.altitude_kp,
            altitude_kd: cfg.altitude_kd,
            altitude_ki: cfg.altitude_ki,
            position_ki: cfg.position_ki,
        });
    }

    /// Mirrors `PilotGuest::configure_supervisor`.
    pub fn configure_supervisor(&mut self, cfg: SupCalib) {
        self.host.configure_supervisor(host::SupervisorConfig {
            home: cfg.home,
            fence_radius: cfg.fence_radius,
            cruise_alt: cfg.cruise_alt,
            low_batt_v: cfg.low_batt_v,
        });
    }

    /// Mirrors `PilotGuest::configure_calibration`.
    pub fn configure_calibration(&mut self, cal: SensorCal) {
        self.host
            .configure_calibration(falcon_core::relay_calib::CalParams {
                gyro_bias: cal.gyro_bias,
                accel_bias: cal.accel_bias,
                accel_scale: cal.accel_scale,
                mag_offset: cal.mag_offset,
                mag_scale: cal.mag_scale,
                mag_offdiag: cal.mag_offdiag,
            });
    }

    /// The published `mode` byte, or `None` before any supervisor exists.
    /// (Test-only readers: the differential compares motors, not these.)
    #[cfg(test)]
    pub fn mode(&self) -> Option<u8> {
        self.host.supervisor().map(|s| s.mode() as u8)
    }

    /// The published `failed-rotor` byte (0xFF = none).
    #[cfg(test)]
    pub fn failed_rotor(&self) -> Option<u8> {
        self.host
            .supervisor()
            .map(|s| s.core().failed_motor().map(|i| i as u8).unwrap_or(0xFF))
    }

    /// The first pre-arm row refusing an arm right now. THAT a row refuses is
    /// published through the seam (failsafe bit 7, while `disarmed`); WHICH
    /// rows fail is published too since @0.13.0 (`prearm-failing`, below).
    #[cfg(test)]
    pub fn arm_blocked_check(&self) -> Option<CheckId> {
        self.host.supervisor().and_then(|s| s.arm_blocked_check())
    }

    /// The published `prearm-failing` mask (@0.13.0, TRANSPORT-P02), or
    /// `None` before any supervisor exists.
    #[cfg(test)]
    pub fn prearm_failing(&self) -> Option<u32> {
        self.host.supervisor().map(|s| s.prearm_failing_mask())
    }

    /// Mirrors `PilotGuest::command`.
    pub fn command(&mut self, ev: Event, level: bool, throttle_low: bool) {
        self.host.command(ev, level, throttle_low);
    }

    /// Mirrors `wasm/cm/cascade::step` call for call, including the dt clamp and
    /// the lazy construction — the component cannot build its supervisor until a
    /// host has configured it, and a mirror that built eagerly would diverge for
    /// a reason that says nothing about the Component Model.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        accel: [f32; 3],
        gyro: [f32; 3],
        target: [f32; 3],
        position: Option<Vec3>,
        mag: Option<Vec3>,
        heading: Option<f32>,
        motor_rpm: Option<[i32; 4]>,
        battery_v: Option<f32>,
        dt_s: f32,
    ) -> [f32; 4] {
        let dt = if dt_s.is_finite() {
            dt_s.clamp(0.0001, 0.1)
        } else {
            0.001
        };
        let mut b = FrameBackend {
            imu: CoreImu { accel, gyro },
            position,
            mag,
            heading,
            motor_rpm,
            battery_v,
            dt,
            motors: [0.0; 4],
        };
        // `target` is the MISSION-MODE setpoint at @0.12.0 (a live follow
        // setpoint in Mission, #609 finding 4), not a position command — the
        // supervisor owns the setpoint; `SupervisorHost::step` holds the rule.
        self.host.step(target, &mut b);
        b.motors
    }
}

// #609 finding 4 (Decision A): the follow-target contract, driven through this
// mirror so the tests see exactly what a host sees. A child module so the tests
// can read the supervisor's mode without widening the mirror's API.
#[cfg(test)]
#[path = "follow_target_tests.rs"]
mod follow_target_tests;
