//! `Physics` trait — the abstraction a real Gazebo bridge implements.
//!
//! Same architectural pattern as the HITL harness's `HitlBench` trait
//! and `FrameSource`: one tiny interface decouples the verified
//! cascade from the IO that gives it sensor data + consumes its
//! motor commands. Verified path (`relay-ekf` → `relay-pos` →
//! `relay-att` → `relay-rate` → `relay-mix-quad`) is unchanged.
//!
//! `MockPhysics` is the in-process reference implementation —
//! a copy of `examples/falcon-sitl-hover`'s `Plant` struct, kept
//! small so the scaffold exercises end-to-end. `GazeboPhysics` is
//! the stub for the real bridge.

use libm::sqrtf;
use relay_ekf::{quat_mul, ImuSample};

// Same physical constants the falcon-sitl-hover SITL uses.
pub const INERTIA: f32 = 0.0125; // kg·m²
pub const FRICTION: f32 = 0.005;
pub const THRUST_SCALE: f32 = 20.0; // N at full PWM
pub const GRAVITY: f32 = 9.81;
pub const DRAG_COEFFICIENT: f32 = 0.05;

/// What the verified cascade needs from "the world".
///
/// `step` advances the physics by `dt` under the given motor PWMs;
/// `measure` reads back an IMU sample + true position for the
/// estimator + safety chain. `position_ned_m` is what the
/// `relay-lc::Geofence::check` sees post-conversion.
pub trait Physics {
    /// Backend name for the verdict log.
    fn name(&self) -> &'static str;

    /// Advance physics by `dt` seconds under the given motor PWMs
    /// (4 values, each in [0, 1]). For the falcon-quad airframe.
    fn step(&mut self, motor_pwm: [f32; 4], dt: f32);

    /// Read IMU body-frame samples + true NED position (m).
    /// `noise_std` lets the impl add Gaussian noise; the
    /// `MockPhysics` impl uses a tiny xorshift; real gz-sim would
    /// already have noise baked into its IMU model so the parameter
    /// becomes a no-op there.
    fn measure(&mut self, noise_std: f32) -> (ImuSample, [f32; 3]);
}

/// In-process reference impl — same toy integrator as
/// `examples/falcon-sitl-hover`'s `Plant`. Kept here so the
/// scaffold is runnable without external dependencies.
pub struct MockPhysics {
    /// Body-frame angular velocity (rad/s).
    pub omega: [f32; 3],
    /// Body-to-NED unit quaternion.
    pub q: [f32; 4],
    /// Position in NED frame (m).
    pub p_ned: [f32; 3],
    /// Velocity in NED frame (m/s).
    pub v_ned: [f32; 3],
    /// xorshift state for the IMU noise generator.
    pub rng: u64,
}

impl MockPhysics {
    pub fn at_rest() -> Self {
        Self {
            omega: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            p_ned: [0.0; 3],
            v_ned: [0.0; 3],
            rng: 0xCAFE_BABE_DEAD_BEEF,
        }
    }

    fn next_unit_normal(&mut self) -> f32 {
        // Box-Muller from two uniforms in (0, 1].
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u1 = ((self.rng >> 11) as f32 / (1u64 << 53) as f32).max(1e-9);
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let u2 = (self.rng >> 11) as f32 / (1u64 << 53) as f32;
        let r = libm::sqrtf(-2.0 * libm::logf(u1));
        let theta = 2.0 * std::f32::consts::PI * u2;
        r * libm::cosf(theta)
    }
}

impl Physics for MockPhysics {
    fn name(&self) -> &'static str { "mock" }

    fn step(&mut self, motor_pwm: [f32; 4], dt: f32) {
        // Sum the four motor PWMs into a normalised collective thrust;
        // mixer's torque is approximated as zero in this scaffold (the
        // full mixer-to-physics torque mapping lives in falcon-sitl-
        // hover; this stub keeps the cascade running so the scaffold
        // ends with a complete loop).
        let thrust_normalised =
            ((motor_pwm[0] + motor_pwm[1] + motor_pwm[2] + motor_pwm[3]) / 4.0).clamp(0.0, 1.0);

        // Rotational dynamics under (zero) torque + friction.
        for i in 0..3 {
            self.omega[i] += ((-FRICTION * self.omega[i]) / INERTIA) * dt;
        }
        // Integrate quaternion from angular velocity.
        let qdot = quat_mul(self.q, [0.0, self.omega[0], self.omega[1], self.omega[2]]);
        let mut q_new = [
            self.q[0] + 0.5 * qdot[0] * dt,
            self.q[1] + 0.5 * qdot[1] * dt,
            self.q[2] + 0.5 * qdot[2] * dt,
            self.q[3] + 0.5 * qdot[3] * dt,
        ];
        let n = sqrtf(
            q_new[0] * q_new[0] + q_new[1] * q_new[1]
                + q_new[2] * q_new[2] + q_new[3] * q_new[3],
        );
        if n > 1.0e-12 {
            q_new = [q_new[0] / n, q_new[1] / n, q_new[2] / n, q_new[3] / n];
            self.q = q_new;
        }

        // Translational — thrust body up rotated into NED, plus gravity, minus drag.
        let t = thrust_normalised * THRUST_SCALE;
        let thrust_body = [0.0, 0.0, -t];
        let qv = [0.0, thrust_body[0], thrust_body[1], thrust_body[2]];
        let qc = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let t1 = quat_mul(self.q, quat_mul(qv, qc));
        let thrust_ned = [t1[1], t1[2], t1[3]];
        for i in 0..3 {
            let g = if i == 2 { GRAVITY } else { 0.0 };
            let drag = DRAG_COEFFICIENT * self.v_ned[i];
            let a = thrust_ned[i] + g - drag;
            self.v_ned[i] += a * dt;
            self.p_ned[i] += self.v_ned[i] * dt;
        }
    }

    fn measure(&mut self, noise_std: f32) -> (ImuSample, [f32; 3]) {
        let gyro_body = [
            self.omega[0] + noise_std * self.next_unit_normal(),
            self.omega[1] + noise_std * self.next_unit_normal(),
            self.omega[2] + noise_std * self.next_unit_normal(),
        ];
        // Body-frame accel: gravity rotated into body via q. Simplified
        // — at small attitude angles the accel reads [0, 0, -g] body
        // plus thrust contribution; we approximate as the body-frame
        // thrust the controller would feel for closed-loop testing.
        let accel_body = [
            noise_std * self.next_unit_normal(),
            noise_std * self.next_unit_normal(),
            -GRAVITY + noise_std * self.next_unit_normal(),
        ];
        let sample = ImuSample {
            time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
            accel_body,
            gyro_body,
        };
        (sample, self.p_ned)
    }
}

// ─── GazeboPhysics ────────────────────────────────────────────────────
//
// Two implementations live behind a feature flag. Default-feature
// builds get the stub (v0.16.1 contract); `--features gazebo` builds
// get the real gz-transport-rs-backed impl (v0.18.0).
//
// The stub is preserved so cargo workspace builds stay lean — pulling
// in gz-transport-rs drags in tokio + libzmq (compiled from C source
// via zeromq-src), ~30-60 s extra build time. The stub keeps the
// scaffold contract usable for users without gz-sim installed.

/// Stub for the Gazebo Sim bridge. Used when the `gazebo` feature is
/// OFF. Records `world` / `model` for log output; `step()` warns and
/// no-ops, `measure()` returns zeros. The verdict prints FAIL — that's
/// the correct signal that `--features gazebo` is needed (or the
/// bench wire-up).
#[cfg(not(feature = "gazebo"))]
pub struct GazeboPhysics {
    pub world_name: String,
    pub model_name: String,
}

#[cfg(not(feature = "gazebo"))]
impl GazeboPhysics {
    pub fn new(world: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            world_name: world.into(),
            model_name: model.into(),
        }
    }
}

#[cfg(not(feature = "gazebo"))]
impl Physics for GazeboPhysics {
    fn name(&self) -> &'static str { "gazebo (stub)" }

    fn step(&mut self, _motor_pwm: [f32; 4], _dt: f32) {
        eprintln!(
            "GazeboPhysics::step is a stub — world={} model={}; rebuild with --features gazebo for the real bridge",
            self.world_name, self.model_name,
        );
    }

    fn measure(&mut self, _noise_std: f32) -> (ImuSample, [f32; 3]) {
        (
            ImuSample {
                time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                accel_body: [0.0; 3],
                gyro_body: [0.0; 3],
            },
            [0.0, 0.0, 0.0],
        )
    }
}

// ─── Real GazeboPhysics (feature = "gazebo") ──────────────────────────
//
// Uses `gz-transport-rs` to subscribe to `gz.msgs.IMU` on the standard
// imu_sensor topic and publish `gz.msgs.Double` to each rotor's
// cmd_vel topic. Pure-Rust (no Gazebo C++ install needed at build
// time — gz-transport-rs vendors libzmq via zeromq-src).
//
// Architecturally the same shape as the MAVLink bridge: an async
// subscription updates a shared snapshot; `measure()` reads from the
// snapshot synchronously; `step()` fires a sync `publish` per rotor.

#[cfg(feature = "gazebo")]
mod gz_real {
    use super::{Physics, ImuSample};
    use std::sync::{Arc, Mutex};
    use tokio::sync::mpsc;

    /// gz-sim uses ENU body frame (X forward, Y left, Z up);
    /// falcon uses NED body frame (X forward, Y right, Z down).
    /// Conversion: (x, y, z)_ned = (x, -y, -z)_enu. Same for
    /// accel + gyro since both are body-frame vectors.
    fn enu_to_ned(v: [f32; 3]) -> [f32; 3] {
        [v[0], -v[1], -v[2]]
    }

    /// Launch-site anchor for NavSat lat/lon/alt → NED projection.
    /// Same equirectangular pattern as `MavlinkBench`'s `Home` from
    /// v0.12 — small-error within a few-km bench range.
    #[derive(Clone, Copy, Debug)]
    pub struct Home {
        pub lat_deg: f64,
        pub lon_deg: f64,
        pub alt_m: f64,
    }

    impl Home {
        /// World-origin default — useful for SDF worlds whose vehicle
        /// spawns at lat/lon (0, 0) and want raw deltas without an
        /// anchor.
        pub const ORIGIN: Self = Self { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 };

        /// Equirectangular projection of (lat_deg, lon_deg, alt_m)
        /// to local NED in metres. Down is positive — alt above
        /// home is negative D.
        pub fn project_to_ned_m(&self, lat_deg: f64, lon_deg: f64, alt_m: f64) -> [f32; 3] {
            const EARTH_R_M: f64 = 6_371_000.0;
            const PI: f64 = std::f64::consts::PI;
            let d_lat = (lat_deg - self.lat_deg) * PI / 180.0;
            let d_lon = (lon_deg - self.lon_deg) * PI / 180.0;
            let lat0 = self.lat_deg * PI / 180.0;
            let north_m = d_lat * EARTH_R_M;
            let east_m = d_lon * EARTH_R_M * lat0.cos();
            let down_m = -(alt_m - self.alt_m);
            [north_m as f32, east_m as f32, down_m as f32]
        }
    }

    pub struct GazeboPhysics {
        pub world_name: String,
        pub model_name: String,
        /// Launch-site anchor for the NavSat → NED projection.
        pub home: Home,
        /// Latest IMU sample observed on the imu_sensor topic.
        /// `None` until first frame arrives.
        latest_imu: Arc<Mutex<Option<ImuSample>>>,
        /// Latest NED position (m). v0.18.0: stub `[0,0,0]`. v0.18.1:
        /// populated from `gz.msgs.NavSat` on the navsat topic, via
        /// `Home::project_to_ned_m`.
        latest_position_ned_m: Arc<Mutex<[f32; 3]>>,
        /// One mpsc sender per rotor; the receiver task owns the
        /// gz-transport Publisher and emits `gz.msgs.Double` per send.
        rotor_tx: [mpsc::UnboundedSender<f32>; 4],
        /// Tokio runtime kept alive for the duration of this instance.
        /// Dropped on shutdown which joins subscriber + publisher tasks.
        _runtime: tokio::runtime::Runtime,
    }

    impl GazeboPhysics {
        /// Alias for `connect` — mirrors the stub `GazeboPhysics::new`
        /// signature so the CLI binary uses the same call regardless
        /// of feature flag. Panics on connect failure (the connect
        /// attempt is a programmer error in the CLI surface; library
        /// users should call `connect` directly for `Result`).
        pub fn new(
            world: impl Into<String>,
            model: impl Into<String>,
        ) -> Self {
            Self::connect_with_home(world, model, Home::ORIGIN)
                .expect("GazeboPhysics::new: gz-transport connect failed; is `gz sim` running?")
        }

        /// Connect to the gz-transport network and start subscriber
        /// + publisher tasks. Blocks until the Node is online.
        /// Home anchor defaults to world origin (0,0,0); use
        /// `connect_with_home` to supply a launch-site lat/lon/alt.
        pub fn connect(
            world: impl Into<String>,
            model: impl Into<String>,
        ) -> Result<Self, gz_transport_rs::Error> {
            Self::connect_with_home(world, model, Home::ORIGIN)
        }

        /// v0.18.1 — connect + supply the launch-site home for the
        /// NavSat → NED projection. Without this the NavSat
        /// subscriber still runs but `measure()` returns positions
        /// relative to `Home::ORIGIN` (lat=0, lon=0, alt=0), which
        /// is almost certainly not what you want for a bench run.
        pub fn connect_with_home(
            world: impl Into<String>,
            model: impl Into<String>,
            home: Home,
        ) -> Result<Self, gz_transport_rs::Error> {
            let world = world.into();
            let model = model.into();

            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("tokio runtime");

            let latest_imu: Arc<Mutex<Option<ImuSample>>> = Arc::new(Mutex::new(None));
            let latest_position_ned_m = Arc::new(Mutex::new([0.0_f32; 3]));

            let (tx0, rx0) = mpsc::unbounded_channel::<f32>();
            let (tx1, rx1) = mpsc::unbounded_channel::<f32>();
            let (tx2, rx2) = mpsc::unbounded_channel::<f32>();
            let (tx3, rx3) = mpsc::unbounded_channel::<f32>();

            // Tasks run on the runtime; the result of the setup
            // (Node + publishers) returns to the caller, errors
            // propagate.
            let imu_ref = latest_imu.clone();
            let position_ref = latest_position_ned_m.clone();
            let home_for_setup = home;
            let world_for_setup = world.clone();
            let model_for_setup = model.clone();
            runtime.block_on(async move {
                use gz_transport_rs::Node;
                use gz_transport_rs::msgs::{Double, Imu, NavSat};

                let mut node = Node::new(None).await?;
                let imu_topic = format!(
                    "/world/{world_for_setup}/model/{model_for_setup}/link/base_link/sensor/imu_sensor/imu"
                );
                let mut sub = node.subscribe::<Imu>(&imu_topic).await?;
                tokio::spawn(async move {
                    while let Some((msg, _meta)) = sub.recv().await {
                        let (ax, ay, az) = msg.linear_acceleration
                            .as_ref()
                            .map(|v| (v.x as f32, v.y as f32, v.z as f32))
                            .unwrap_or((0.0, 0.0, 0.0));
                        let (gx, gy, gz) = msg.angular_velocity
                            .as_ref()
                            .map(|v| (v.x as f32, v.y as f32, v.z as f32))
                            .unwrap_or((0.0, 0.0, 0.0));
                        let sample = ImuSample {
                            time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                            accel_body: enu_to_ned([ax, ay, az]),
                            gyro_body: enu_to_ned([gx, gy, gz]),
                        };
                        *imu_ref.lock().unwrap() = Some(sample);
                    }
                });

                // v0.18.1 — NavSat subscriber: lat/lon/alt deg from
                // the SDF NavSat plugin → local NED via Home.
                let navsat_topic = format!(
                    "/world/{world_for_setup}/model/{model_for_setup}/link/base_link/sensor/navsat_sensor/navsat"
                );
                let mut navsat_sub = node.subscribe::<NavSat>(&navsat_topic).await?;
                tokio::spawn(async move {
                    while let Some((msg, _meta)) = navsat_sub.recv().await {
                        let ned = home_for_setup.project_to_ned_m(
                            msg.latitude_deg,
                            msg.longitude_deg,
                            msg.altitude,
                        );
                        *position_ref.lock().unwrap() = ned;
                    }
                });

                // One publisher per rotor.
                for (n, mut rx) in [rx0, rx1, rx2, rx3].into_iter().enumerate() {
                    let topic = format!(
                        "/world/{world_for_setup}/model/{model_for_setup}/joint/rotor_{n}_joint/cmd_vel"
                    );
                    let publisher = node
                        .advertise::<Double>(&topic, "gz.msgs.Double")
                        .await?;
                    tokio::spawn(async move {
                        while let Some(cmd) = rx.recv().await {
                            let msg = Double {
                                header: None,
                                data: cmd as f64,
                            };
                            // Partition empty by default; gz-sim uses
                            // empty partition for "world" topics.
                            let _ = publisher.publish("", &msg);
                        }
                    });
                }

                Ok::<_, gz_transport_rs::Error>(())
            })?;

            Ok(Self {
                world_name: world,
                model_name: model,
                home,
                latest_imu,
                latest_position_ned_m,
                rotor_tx: [tx0, tx1, tx2, tx3],
                _runtime: runtime,
            })
        }

        /// Map a [0, 1] motor PWM to a Gazebo motor command (rad/s).
        /// MulticopterMotorModel plugin's `cmd_vel` expects rad/s; a
        /// typical scaling for a 5"-class quad is ~1000 rad/s at full
        /// PWM. The exact constant is a property of the SDF model.
        fn pwm_to_rad_per_s(pwm: f32) -> f32 {
            const MAX_MOTOR_RAD_S: f32 = 1000.0;
            pwm.clamp(0.0, 1.0) * MAX_MOTOR_RAD_S
        }
    }

    impl Physics for GazeboPhysics {
        fn name(&self) -> &'static str { "gazebo" }

        fn step(&mut self, motor_pwm: [f32; 4], _dt: f32) {
            for (i, &pwm) in motor_pwm.iter().enumerate() {
                let _ = self.rotor_tx[i].send(Self::pwm_to_rad_per_s(pwm));
            }
        }

        fn measure(&mut self, _noise_std: f32) -> (ImuSample, [f32; 3]) {
            let sample = self.latest_imu.lock().unwrap().clone().unwrap_or(ImuSample {
                time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                accel_body: [0.0; 3],
                gyro_body: [0.0; 3],
            });
            let pos = *self.latest_position_ned_m.lock().unwrap();
            (sample, pos)
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn enu_to_ned_conversion() {
            // Body-frame X-forward = X-forward in both.
            assert_eq!(enu_to_ned([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);
            // Y-left (ENU) = -Y-right (NED).
            assert_eq!(enu_to_ned([0.0, 1.0, 0.0]), [0.0, -1.0, 0.0]);
            // Z-up (ENU) = -Z-down (NED).
            assert_eq!(enu_to_ned([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0]);
            // Gravity in ENU body frame at rest = +Z (up); in NED = -Z (down).
            assert_eq!(enu_to_ned([0.0, 0.0, 9.81]), [0.0, 0.0, -9.81]);
        }

        #[test]
        fn pwm_to_rad_per_s_scales_and_clamps() {
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(0.0), 0.0);
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(1.0), 1000.0);
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(0.5), 500.0);
            // Clamps below 0.
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(-0.5), 0.0);
            // Clamps above 1.
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(2.0), 1000.0);
        }

        // v0.18.1 — NavSat projection tests. Same shape as the
        // `MavlinkBench::Home::project_ned_cm` tests in
        // examples/falcon-hitl-rfspoof/src/mavlink.rs.
        fn budapest_home() -> Home {
            Home { lat_deg: 47.5023456, lon_deg: 19.0401234, alt_m: 120.0 }
        }

        #[test]
        fn home_projects_to_origin() {
            let h = budapest_home();
            let p = h.project_to_ned_m(h.lat_deg, h.lon_deg, h.alt_m);
            assert!(p[0].abs() < 0.01, "north = {}", p[0]);
            assert!(p[1].abs() < 0.01, "east  = {}", p[1]);
            assert!(p[2].abs() < 0.01, "down  = {}", p[2]);
        }

        #[test]
        fn altitude_translates_to_down() {
            let h = budapest_home();
            // 5 m below home → down +5 m.
            let p = h.project_to_ned_m(h.lat_deg, h.lon_deg, h.alt_m - 5.0);
            assert!((p[2] - 5.0).abs() < 0.01, "down = {}", p[2]);
        }

        #[test]
        fn lat_step_translates_to_north() {
            let h = Home { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 };
            // 1° of latitude ≈ 111_195 m on a 6_371_000 m-radius sphere.
            let p = h.project_to_ned_m(1.0, 0.0, 0.0);
            assert!((p[0] - 111_195.0).abs() < 1.0, "north = {}", p[0]);
            assert!(p[1].abs() < 0.01, "east = {}", p[1]);
        }

        #[test]
        fn world_origin_default_is_zero() {
            let h = Home::ORIGIN;
            let p = h.project_to_ned_m(0.0, 0.0, 0.0);
            assert_eq!(p, [0.0, 0.0, 0.0]);
        }
    }
}

#[cfg(feature = "gazebo")]
pub use gz_real::{GazeboPhysics, Home};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_physics_at_rest_stays_quiet() {
        let mut p = MockPhysics::at_rest();
        // No motor input, no gravity-only fall (we step with zero PWM
        // → zero thrust → falls under gravity; check the fall is
        // physically reasonable).
        p.step([0.0; 4], 0.01);
        assert!(p.v_ned[2] > 0.0, "no thrust → should accelerate down (+z NED)");
        assert!(p.v_ned[2] < 1.0, "1 step at dt=0.01 → v ≈ g*dt = 0.098 m/s");
    }

    #[test]
    fn mock_physics_hover_with_full_thrust_climbs() {
        let mut p = MockPhysics::at_rest();
        // Full PWM on all 4 motors: thrust = THRUST_SCALE = 20 N >> gravity.
        // After 1 step at dt=0.01 we expect upward (-z NED) velocity.
        p.step([1.0; 4], 0.01);
        assert!(p.v_ned[2] < 0.0, "max thrust → upward velocity (-z NED)");
    }

    #[test]
    fn mock_physics_measure_returns_sensible_imu() {
        let mut p = MockPhysics::at_rest();
        let (s, pos) = p.measure(0.0);
        assert_eq!(pos, [0.0; 3]);
        // No noise → gyro reads angular velocity (zero at rest).
        assert_eq!(s.gyro_body[0], 0.0);
        // Accel z at rest reads -gravity (NED z-axis points down).
        assert!((s.accel_body[2] - (-GRAVITY)).abs() < 1e-3);
    }

    /// Only meaningful when the `gazebo` feature is OFF — the stub
    /// GazeboPhysics is panic-free + returns zeros. With the feature
    /// ON, `GazeboPhysics::new` actually attempts to connect to
    /// gz-transport and panics if no `gz sim` is running.
    #[cfg(not(feature = "gazebo"))]
    #[test]
    fn gazebo_stub_compiles_and_returns_zeros() {
        let mut g = GazeboPhysics::new("falcon", "quad");
        g.step([0.5; 4], 0.01);
        let (s, pos) = g.measure(0.0);
        assert_eq!(pos, [0.0; 3]);
        assert_eq!(s.accel_body[0], 0.0);
    }
}
