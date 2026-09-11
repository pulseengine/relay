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
//! It is now a much smaller file, which is the point: there is one flight core,
//! and both sides call it.

use falcon_core::{FlightBackend, FlightCore, ImuSample as CoreImu};

type Vec3 = [f32; 3];

/// One tick's sensors in, motors out — the same shim `wasm/cm/cascade` uses.
///
/// `FlightCore` pulls from a backend and pushes motors into it, which is the
/// seam that lets the same code run against SITL, Gazebo or hardware. Mirroring
/// the component's shim exactly (including returning `None` where the frame
/// carried nothing, so the core skips that fusion rather than being handed a
/// fabricated zero) is what keeps the comparison honest.
struct FrameBackend {
    imu: CoreImu,
    position: Option<Vec3>,
    mag: Option<Vec3>,
    heading: Option<f32>,
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
    fn read_position(&mut self) -> Option<Vec3> {
        self.position
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

pub struct NativeCascade {
    core: Option<FlightCore>,
    cfg: Option<Calib>,
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

impl NativeCascade {
    pub fn new() -> Self {
        Self { core: None, cfg: None }
    }

    /// Mirrors `Component::configure`, INCLUDING dropping the core so the next
    /// step rebuilds from the new calibration.
    pub fn configure(&mut self, cfg: Calib) {
        self.cfg = Some(cfg);
        self.core = None;
    }

    /// Mirrors `wasm/cm/cascade::step` call for call, including the dt clamp and
    /// the lazy construction on the FIRST frame's rate — the component cannot
    /// build its core until a host states its period, and a mirror that built
    /// eagerly on a guessed rate would diverge for a reason that says nothing
    /// about the Component Model.
    pub fn step(
        &mut self,
        accel: [f32; 3],
        gyro: [f32; 3],
        target: [f32; 3],
        position: Option<Vec3>,
        mag: Option<Vec3>,
        heading: Option<f32>,
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
            dt,
            motors: [0.0; 4],
        };
        // Built from the calibration when the host supplied one, and from the
        // v0.8 defaults otherwise — identical to the component's `build_core`.
        let cfg = self.cfg;
        let core = self.core.get_or_insert_with(|| match cfg {
            None => FlightCore::new(0.5, 1.0 / dt),
            Some(c) => {
                let hz = sane(c.loop_rate_hz, 1.0 / dt, true);
                let mut core = FlightCore::new(sane(c.hover_thrust, 0.5, true), hz);
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
                core
            }
        });
        core.set_position(target);
        core.step(&mut b);
        b.motors
    }
}
