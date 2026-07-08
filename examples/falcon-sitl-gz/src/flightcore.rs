//! `FlightCore`-in-the-loop — flying the PRODUCTION core through the SITL plant.
//!
//! Every other `run_*` scenario in this bench re-implements the flight cascade
//! (estimator → attitude → rate → mixer) by hand against the [`Physics`] trait.
//! That means the sim exercises a *parallel* controller, not the one that
//! ships. The production controller — [`falcon_core::FlightCore`], the verified
//! IEKF → geometric-SE(3) → ADRC → mixer cascade with the single-rotor-out FDI
//! and degraded-allocator recovery — lives behind its OWN hardware seam,
//! [`falcon_core::FlightBackend`], and was flown only against falcon-core's
//! in-crate `SimBackend`.
//!
//! This module closes that gap with an **adapter**, not a rewrite: [`SitlBackend`]
//! implements `FlightBackend` on top of a `&mut dyn Physics` plant, so the same
//! byte-for-byte production core flies the SITL plant (mock today, real Gazebo
//! under `--features gazebo`). What the sim validates is now what flies.
//!
//! ## Seam mapping
//!
//! `FlightBackend` is a *sensor/actuator port* (the core pulls sensors, pushes
//! motors); `Physics` is a *plant* (push motor PWMs + `step(dt)`, then
//! `measure()`). The adapter reconciles them by advancing the plant inside
//! `write_motors` — the tail of one `FlightCore::step` — so each control tick is
//! exactly one physics step, in causal order: sense state(t) → decide →
//! actuate-and-advance to state(t+dt).

use crate::physics::Physics;
use falcon_core::{FlightBackend, ImuSample as CoreImu};
use relay_iekf::Vec3;

/// Adapts a SITL [`Physics`] plant to falcon-core's [`FlightBackend`] seam.
///
/// Owns the plant for the whole run. `read_imu` samples the plant and caches
/// the true position so the same-tick `read_position` returns a coherent fix;
/// `write_motors` is the one place the plant is stepped, once per control tick.
pub struct SitlBackend<'a> {
    plant: &'a mut dyn Physics,
    dt: f32,
    /// IMU measurement noise σ passed to `Physics::measure` (0 for the mock).
    imu_noise: f32,
    /// True position cached by the most recent `read_imu`, surfaced to
    /// `read_position` (and to the caller for evidence logging).
    last_pos: [f32; 3],
    /// GNSS is slower than the control loop; a position fix is offered only
    /// every `gnss_div` ticks (1 = every tick).
    gnss_div: u32,
    tick: u32,
    /// The last per-rotor command the core wrote (for evidence logging).
    last_motors: [f32; 4],
    /// The last IMU sample handed to the core (for evidence logging).
    last_imu: CoreImu,
}

impl<'a> SitlBackend<'a> {
    /// New adapter. `dt` is the control period (s); `imu_noise` the measurement
    /// σ; `gnss_div` the loop-ticks-per-GNSS-fix divisor.
    pub fn new(plant: &'a mut dyn Physics, dt: f32, imu_noise: f32, gnss_div: u32) -> Self {
        SitlBackend {
            plant,
            dt,
            imu_noise,
            last_pos: [0.0; 3],
            gnss_div: gnss_div.max(1),
            tick: 0,
            last_motors: [0.0; 4],
            last_imu: CoreImu {
                accel: [0.0; 3],
                gyro: [0.0; 3],
            },
        }
    }

    /// The true NED position sampled by the last `read_imu` (for evidence).
    pub fn last_true_pos(&self) -> [f32; 3] {
        self.last_pos
    }

    /// The per-rotor command from the most recent control tick (for evidence).
    pub fn last_motors(&self) -> [f32; 4] {
        self.last_motors
    }

    /// The IMU sample from the most recent control tick (for evidence).
    pub fn last_imu(&self) -> ([f32; 3], [f32; 3]) {
        (self.last_imu.accel, self.last_imu.gyro)
    }

    /// Stage a single-rotor failure on the underlying plant (the scenario calls
    /// this at the injection time; the adapter owns the plant, so the fault must
    /// route through here rather than a separate borrow).
    pub fn fail_rotor(&mut self, rotor: usize) {
        self.plant.fail_rotor(rotor);
    }
}

impl FlightBackend for SitlBackend<'_> {
    fn read_imu(&mut self) -> CoreImu {
        // `measure` returns (IMU, true position); cache the position so the
        // same tick's `read_position` is coherent with this IMU sample.
        let (s, p) = self.plant.measure(self.imu_noise);
        self.last_pos = p;
        let sample = CoreImu {
            accel: s.accel_body,
            gyro: s.gyro_body,
        };
        self.last_imu = sample;
        sample
    }

    fn read_position(&mut self) -> Option<Vec3> {
        // Offer a fix only on GNSS ticks; between fixes the core dead-reckons
        // on the IMU (exactly as it must against a real 5 Hz receiver).
        self.tick = self.tick.wrapping_add(1);
        if self.tick % self.gnss_div == 0 {
            Some(self.last_pos)
        } else {
            None
        }
    }

    fn read_mag(&mut self) -> Option<Vec3> {
        // MockPhysics has no magnetometer (returns None); the real gz bridge
        // supplies one. Forward whatever the plant exposes.
        self.plant.mag_body_ned()
    }

    fn read_heading(&mut self) -> Option<f32> {
        // The clean absolute yaw (NED). MockPhysics has none; the gz bridge
        // resolves it from the Pose_V truth orientation — the "compass" that
        // makes yaw observable (the raw gz-mag frame is unvalidated, so this is
        // the heading reference the hover uses).
        self.plant.heading_ned()
    }

    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        // Carry the plant's per-rotor ESC telemetry to the core's rotor-out FDI.
        // Without this the FDI is inert in SITL (a rotor loss goes undetected),
        // so this is the plumbing a recordable rotor-out flight depends on.
        self.plant.motor_rpm()
    }

    fn write_motors(&mut self, motors: &[f32]) {
        // The tail of one control tick: actuate and advance the plant by dt.
        // `mix` yields 4 quad motors; pad defensively in case a backend ever
        // hands us fewer.
        let m = [
            motors.first().copied().unwrap_or(0.0),
            motors.get(1).copied().unwrap_or(0.0),
            motors.get(2).copied().unwrap_or(0.0),
            motors.get(3).copied().unwrap_or(0.0),
        ];
        self.last_motors = m;
        self.plant.step(m, self.dt);
    }

    fn dt(&self) -> f32 {
        self.dt
    }
}
