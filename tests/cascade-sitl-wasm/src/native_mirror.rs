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

use falcon_core::relay_fsm::Event;
use falcon_core::relay_preflight::CheckId;
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample as CoreImu};

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
}

impl SensorCal {
    /// Mirrors the component's `cal_params`: `None` if any component is
    /// non-finite or any scale is not > 0. Never clamped.
    fn to_core(self) -> Option<falcon_core::relay_calib::CalParams> {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        let finite3 = |v: &Vec3| v.iter().all(|x| x.is_finite());
        let pos3 = |v: &Vec3| v.iter().all(|x| pos(*x));
        let ok = finite3(&self.gyro_bias)
            && finite3(&self.accel_bias)
            && pos3(&self.accel_scale)
            && finite3(&self.mag_offset)
            && pos3(&self.mag_scale);
        ok.then_some(falcon_core::relay_calib::CalParams {
            gyro_bias: self.gyro_bias,
            accel_bias: self.accel_bias,
            accel_scale: self.accel_scale,
            mag_offset: self.mag_offset,
            mag_scale: self.mag_scale,
        })
    }
}

/// Mirrors the component's unconfigured-supervisor constants. A supervisor on
/// defaults is worse than no supervisor, so "unset" means the
/// configuration-dependent failsafes are INACTIVE, never zero.
const SUP_FENCE_UNSET: f32 = 1.0e9;
const SUP_LOW_BATT_UNSET: f32 = 0.0;
const SUP_CRUISE_UNSET: f32 = 2.0;

pub struct NativeCascade {
    sup: Option<FlightSupervisor>,
    cfg: Option<Calib>,
    supcfg: Option<SupCalib>,
    /// Mirrors the component's `CALIB`: last valid calibration, else identity.
    calib: falcon_core::relay_calib::CalParams,
}

impl Default for NativeCascade {
    fn default() -> Self {
        Self::new()
    }
}

/// Mirrors the component's `sane()`. Same fallbacks, same positivity rules —
/// a mirror that sanitised differently would report a difference that is about
/// the mirror, not about the Component Model.
fn sane(v: f32, default: f32, positive: bool) -> f32 {
    if v.is_finite() && (!positive || v > 0.0) {
        v
    } else {
        default
    }
}

/// Mirrors the component's `supcfg_valid` (#609 finding 3): home finite;
/// fence-radius, cruise-alt (m AGL, POSITIVE) and low-batt-v finite and > 0.
fn supcfg_valid(c: &SupCalib) -> bool {
    let pos = |v: f32| v.is_finite() && v > 0.0;
    c.home.iter().all(|v| v.is_finite())
        && pos(c.fence_radius)
        && pos(c.cruise_alt)
        && pos(c.low_batt_v)
}

/// Mirrors the component's `usable_supcfg`: an invalid configuration counts
/// as none — never clamped into a permissive one.
fn usable_supcfg(cfg: Option<&SupCalib>) -> Option<&SupCalib> {
    cfg.filter(|c| supcfg_valid(c))
}

/// Mirrors the component's `sup_params`.
fn sup_params(cfg: Option<&SupCalib>) -> (Vec3, f32, f32, f32) {
    match usable_supcfg(cfg) {
        None => (
            [0.0, 0.0, 0.0],
            SUP_FENCE_UNSET,
            SUP_CRUISE_UNSET,
            SUP_LOW_BATT_UNSET,
        ),
        Some(c) => (c.home, c.fence_radius, c.cruise_alt, c.low_batt_v),
    }
}

/// Mirrors the component's `ROWS_FED_BY_STEP`: exactly the rows
/// `update_preflight` refreshes every step, declared FALSE on construction.
const ROWS_FED_BY_STEP: [CheckId; 12] = [
    CheckId::SensorsHealthy,
    CheckId::EstimatorConverged,
    CheckId::CalibrationPresent,
    CheckId::GeofenceLoaded,
    CheckId::BatteryOk,
    CheckId::FailsafeConfigured,
    CheckId::EstimatorInnovation,
    CheckId::GnssAgreement,
    CheckId::GeofenceSane,
    CheckId::BatteryTakeoffMargin,
    CheckId::MotorsHealthy,
    CheckId::NoFailsafeLatched,
];

/// Mirrors the component's `rebuild_permitted` (#609 finding 1): a configure
/// may drop the supervisor only if none exists, or it is Disarmed and holds no
/// latch a rebuild would reset (isolated rotor, nav-integrity, battery).
fn rebuild_permitted(slot: &Option<FlightSupervisor>) -> bool {
    match slot {
        None => true,
        Some(sup) => {
            let batt = sup.battery();
            sup.mode() == falcon_core::relay_fsm::Mode::Disarmed
                && sup.core().failed_motor().is_none()
                && !sup.core().nav_compromised()
                && !batt.low
                && !batt.critical
        }
    }
}

impl NativeCascade {
    pub fn new() -> Self {
        Self {
            sup: None,
            cfg: None,
            supcfg: None,
            calib: falcon_core::relay_calib::CalParams::identity(),
        }
    }

    /// Mirrors `Component::configure`: rebuild from the new configuration only
    /// while `rebuild_permitted`, otherwise discard it (#609 finding 1).
    pub fn configure(&mut self, cfg: Calib) {
        if !rebuild_permitted(&self.sup) {
            return;
        }
        self.cfg = Some(cfg);
        self.sup = None;
    }

    /// Mirrors `PilotGuest::configure_supervisor`, same guard.
    pub fn configure_supervisor(&mut self, cfg: SupCalib) {
        if !rebuild_permitted(&self.sup) {
            return;
        }
        self.supcfg = Some(cfg);
        self.sup = None;
    }

    /// Mirrors `PilotGuest::configure_calibration`: installed IN PLACE, only
    /// while Disarmed (or not yet built); otherwise rejected and discarded. An
    /// invalid one replaces any earlier calibration with the identity.
    pub fn configure_calibration(&mut self, cal: SensorCal) {
        if let Some(sup) = self.sup.as_ref() {
            if sup.mode() != falcon_core::relay_fsm::Mode::Disarmed {
                return;
            }
        }
        self.calib = cal
            .to_core()
            .unwrap_or(falcon_core::relay_calib::CalParams::identity());
        if let Some(sup) = self.sup.as_mut() {
            sup.set_calibration(self.calib);
        }
    }

    /// The published `mode` byte, or `None` before any supervisor exists.
    /// (Test-only readers: the differential compares motors, not these.)
    #[cfg(test)]
    pub fn mode(&self) -> Option<u8> {
        self.sup.as_ref().map(|s| s.mode() as u8)
    }

    /// The published `failed-rotor` byte (0xFF = none).
    #[cfg(test)]
    pub fn failed_rotor(&self) -> Option<u8> {
        self.sup
            .as_ref()
            .map(|s| s.core().failed_motor().map(|i| i as u8).unwrap_or(0xFF))
    }

    /// The first pre-arm row refusing an arm right now. THAT a row refuses is
    /// published through the seam (failsafe bit 7, while `disarmed`); WHICH row
    /// is visible only natively, which is why the mirror tests can NAME it.
    #[cfg(test)]
    pub fn arm_blocked_check(&self) -> Option<CheckId> {
        self.sup.as_ref().and_then(|s| s.arm_blocked_check())
    }

    /// Mirrors the component's `ensure_supervisor` exactly, declaration included.
    fn ensure(&mut self) -> &mut FlightSupervisor {
        let fresh = self.sup.is_none();
        let (home, fence, cruise, low_batt) = sup_params(self.supcfg.as_ref());
        let cfg = self.cfg;
        let sup = self
            .sup
            .get_or_insert_with(|| FlightSupervisor::new(home, fence, cruise, low_batt));
        if fresh {
            if let Some(c) = cfg {
                sup.set_hover_thrust(sane(c.hover_thrust, 0.5, true));
                let core = sup.core_mut();
                core.set_pos_var(sane(c.pos_var, 0.01, true));
                core.set_process_floor(
                    sane(c.process_floor_vel, 0.0, false),
                    sane(c.process_floor_pos, 0.0, false),
                );
                core.set_altitude_gains(
                    sane(c.altitude_kp, 0.05, false),
                    sane(c.altitude_kd, 0.30, false),
                );
                core.set_altitude_integral_gain(sane(c.altitude_ki, 0.0, false));
                core.set_position_integral_gain(sane(c.position_ki, 0.02, false));
            }
            // Mirrors the component: the stored calibration survives a rebuild.
            sup.set_calibration(self.calib);
            // Mirrors the component's `ROWS_FED_BY_STEP` (#609 finding 2):
            // nothing a step feeds has been measured yet, so all of it fails.
            for id in ROWS_FED_BY_STEP {
                sup.set_check(id, false);
            }
        }
        let configured = usable_supcfg(self.supcfg.as_ref()).is_some();
        let sup = self.sup.as_mut().expect("just inserted");
        sup.set_check(CheckId::FailsafeConfigured, configured);
        sup
    }

    /// Mirrors `PilotGuest::command`.
    pub fn command(&mut self, ev: Event, level: bool, throttle_low: bool) {
        let sup = self.ensure();
        sup.command(ev, level, throttle_low);
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
        let configured = usable_supcfg(self.supcfg.as_ref()).is_some();
        let sup = self.ensure();
        // `target` is a MISSION LEG at @0.12.0, not a position command: the
        // supervisor owns the setpoint and derives one per mode, so writing the
        // core's setpoint here would be overwritten inside the same tick.
        // Outside Mission it is ignored; Takeoff/Loiter's latched hold point
        // lives inside `FlightSupervisor`, so the mirror shares it as is.
        sup.set_mission(target);
        sup.step(&mut b);
        // Mirrors the component's RE-DECLARATION after the step:
        // `update_preflight` rewrites `FailsafeConfigured` from a default-true
        // field, so without this the mirror's table would disagree with what
        // the component's `command` (and `read-state`) sees.
        sup.set_check(CheckId::FailsafeConfigured, configured);
        b.motors
    }
}
