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
    dt: f32,
    motors: [f32; 4],
}

impl FlightBackend for FrameBackend {
    fn read_imu(&mut self) -> CoreImu {
        self.imu
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.position
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        None
    }
    fn read_heading(&mut self) -> Option<f32> {
        None
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

pub struct NativeCascade {
    core: Option<FlightCore>,
}

impl Default for NativeCascade {
    fn default() -> Self {
        Self::new()
    }
}

impl NativeCascade {
    pub fn new() -> Self {
        Self { core: None }
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
            dt,
            motors: [0.0; 4],
        };
        // hover_thrust 0.5, loop rate from the frame — identical to the component.
        let core = self.core.get_or_insert_with(|| FlightCore::new(0.5, 1.0 / dt));
        core.set_position(target);
        core.step(&mut b);
        b.motors
    }
}
