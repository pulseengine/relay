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

    /// Diagnostic counters — `(imu_recv, navsat_recv, motor_send)`.
    /// `None` for backends where the distinction is meaningless
    /// (MockPhysics, the stub). The real gz-transport bridge
    /// overrides this so a bench operator can distinguish
    /// "gz isn't publishing" (`imu_recv == 0`) from "gz publishes
    /// but our subscriber dropped frames" — same diagnostic shape
    /// as `MavlinkBench`'s `frames_recv` / `gpi_recv` from v0.18.2.
    fn counters(&self) -> Option<(u64, u64, u64)> { None }

    /// v0.19.7 — true NED body velocity (m/s), if the backend supplies
    /// one. `None` means "no true velocity source; finite-difference
    /// position yourself". The real gz bridge overrides this with the
    /// OdometryPublisher twist (deterministic, unlike finite-diff
    /// NavSat which left the altitude velocity-cascade marginal).
    fn velocity_ned(&self) -> Option<[f32; 3]> { None }

    /// v0.22 — true NED heading (yaw, rad), if the backend supplies one.
    /// `None` means "no heading reference". The real gz bridge overrides
    /// this from the OdometryPublisher pose orientation — the "compass"
    /// that makes yaw observable for the IEKF (yaw is unobservable from
    /// IMU+GPS alone, the v0.21 ±130° wander).
    fn heading_ned(&self) -> Option<f32> { None }

    /// v0.22 — latest body-frame magnetometer reading (Tesla, NED body
    /// frame), or `None` if no magnetometer. The real heading source.
    fn mag_body_ned(&self) -> Option<[f32; 3]> { None }

    /// v1.113 — per-rotor reported RPM (ESC telemetry), or `None` if the plant
    /// has no rotor feedback. This is the ACHIEVED-per-rotor source the
    /// production `FlightCore` compares against the commanded throttle to form
    /// the single-rotor-out FDI residual (v1.103): a dead rotor reads ~0 RPM
    /// while the controller still commands it → the residual fires the CUSUM.
    /// Without it the FDI is inert, so a SITL rotor-out flight cannot exercise
    /// the recovery. Default `None` (backends without ESC feedback fly without
    /// rotor-fault detection, exactly as the `FlightBackend` default intends).
    fn motor_rpm(&self) -> Option<[i32; 4]> { None }

    /// v1.113 — inject a single-rotor failure: rotor `rotor`'s thrust (and its
    /// reported RPM) drop to zero from now on. The hook the rotor-out scenario
    /// uses to stage the fault a real airframe would suffer. Default no-op (a
    /// plant that cannot model a rotor loss simply ignores it).
    fn fail_rotor(&mut self, _rotor: usize) {}
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
    /// Specific force in the NED frame from the last `step` — the non-gravity
    /// part of the kinematic acceleration (`thrust − drag`). An accelerometer
    /// measures specific force (`a_kinematic − g_gravity`), so `measure` rotates
    /// this into the body frame. At hover it is `[0,0,−g]` (the reaction to
    /// gravity), matching the pre-v1.113 constant; during a climb it carries the
    /// thrust reaction the full IEKF needs to keep vertical velocity observable.
    pub spec_force_ned: [f32; 3],
    /// A single failed rotor (its thrust + RPM forced to zero), or `None`.
    /// Set by `fail_rotor`; defaults off so every existing scenario is
    /// unaffected.
    pub failed_rotor: Option<usize>,
    /// The ACHIEVED per-rotor throttle from the last `step` (post-failure) —
    /// the basis for the reported RPM the production FDI consumes.
    pub achieved_motors: [f32; 4],
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
            spec_force_ned: [0.0, 0.0, -GRAVITY], // at rest: reaction to gravity
            failed_rotor: None,
            achieved_motors: [0.0; 4],
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
        // A failed rotor delivers no thrust regardless of command; record the
        // ACHIEVED per-rotor throttle so the reported RPM reflects the loss (the
        // production FDI keys off commanded-vs-achieved). With no failure this
        // is the command verbatim, so nominal flight is byte-for-byte unchanged.
        let mut achieved = motor_pwm;
        if let Some(f) = self.failed_rotor {
            if f < 4 {
                achieved[f] = 0.0;
            }
        }
        self.achieved_motors = achieved;
        let thrust_normalised =
            ((achieved[0] + achieved[1] + achieved[2] + achieved[3]) / 4.0).clamp(0.0, 1.0);

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
            // Specific force = kinematic accel − gravity = thrust − drag. This
            // is what an accelerometer reads (the full IEKF integrates it, so a
            // constant-gravity fake would starve its vertical-velocity estimate).
            self.spec_force_ned[i] = thrust_ned[i] - drag;
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
        // Body-frame accelerometer: the NED specific force (thrust − drag,
        // computed in `step`) rotated into the body frame via conj(q). At
        // hover/level this reduces to [0, 0, −g] (the reaction to gravity), so
        // the near-level scenarios are unchanged; under a climb it carries the
        // thrust reaction the full IEKF integrates for vertical velocity.
        let f = self.spec_force_ned;
        let qv = [0.0, f[0], f[1], f[2]];
        let qc = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let fb = quat_mul(qc, quat_mul(qv, self.q)); // v_body = conj(q)·v_ned·q
        let accel_body = [
            fb[1] + noise_std * self.next_unit_normal(),
            fb[2] + noise_std * self.next_unit_normal(),
            fb[3] + noise_std * self.next_unit_normal(),
        ];
        let sample = ImuSample {
            time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
            accel_body,
            gyro_body,
        };
        (sample, self.p_ned)
    }

    fn motor_rpm(&self) -> Option<[i32; 4]> {
        // rpm = achieved throttle × full-scale RPM, matching falcon-core's
        // ESC_RPM_FULL (8000) so the FDI residual (rpm/8000 vs command) is
        // scaled correctly. The failed rotor reads 0 → the residual fires.
        const ESC_RPM_FULL: f32 = 8000.0;
        let a = &self.achieved_motors;
        Some([
            (a[0] * ESC_RPM_FULL) as i32,
            (a[1] * ESC_RPM_FULL) as i32,
            (a[2] * ESC_RPM_FULL) as i32,
            (a[3] * ESC_RPM_FULL) as i32,
        ])
    }

    fn fail_rotor(&mut self, rotor: usize) {
        self.failed_rotor = Some(rotor);
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
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
    use crossbeam_channel::Receiver;
    use gz_msgs::actuators::Actuators;
    use gz_msgs::imu::IMU;
    use gz_msgs::magnetometer::Magnetometer;
    use gz_msgs::navsat::NavSat;
    use gz_msgs::odometry::Odometry;
    use gz_msgs::pose_v::Pose_V;
    use gz_transport::{Node, Publisher};

    // The gz.msgs types (Actuators / IMU / NavSat / Pose_V / Magnetometer /
    // Odometry) now come from the `gz-msgs` crate (protobuf), imported above —
    // the pure-Rust bridge's hand-written prost definitions are gone.

    /// gz-sim uses ENU body frame (X forward, Y left, Z up);
    /// falcon uses NED body frame (X forward, Y right, Z down).
    /// Conversion: (x, y, z)_ned = (x, -y, -z)_enu. Same for
    /// accel + gyro since both are body-frame vectors.
    fn enu_to_ned(v: [f32; 3]) -> [f32; 3] {
        [v[0], -v[1], -v[2]]
    }

    /// NED heading (yaw, rad) from a body→ENU orientation quaternion
    /// (Hamilton, w,x,y,z). The ENU heading of body-X is CCW from East;
    /// NED yaw is CW from North, so `ψ_ned = π/2 − ψ_enu`. This is the
    /// v0.22 "compass" reference that makes yaw observable.
    fn enu_quat_to_ned_yaw(w: f64, x: f64, y: f64, z: f64) -> f32 {
        let psi_enu = libm::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        let psi_ned = core::f64::consts::FRAC_PI_2 - psi_enu;
        // wrap to [−π, π]
        libm::remainder(psi_ned, 2.0 * core::f64::consts::PI) as f32
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
        /// The gz-transport node — owns the subscriptions + publisher; kept
        /// alive for the instance's lifetime (dropping it tears down transport).
        _node: Node,
        /// Actuators publisher on `/<model>/command/motor_speed`.
        publisher: Publisher<Actuators>,
        // Inbound channels — filled asynchronously by gz-transport's C++
        // receiver threads; drained to the caches below by `pump()`.
        imu_rx: Receiver<IMU>,
        navsat_rx: Receiver<NavSat>,
        odom_rx: Receiver<Odometry>,
        pose_rx: Receiver<Pose_V>,
        mag_rx: Option<Receiver<Magnetometer>>,
        // Latest-value caches (persist between the sensors' own update rates).
        latest_imu: Mutex<Option<ImuSample>>,
        latest_position_ned_m: Mutex<[f32; 3]>,
        latest_velocity_ned: Mutex<Option<[f32; 3]>>,
        latest_heading_ned: Mutex<Option<f32>>,
        latest_mag_body_ned: Mutex<Option<[f32; 3]>>,
        imu_recv: AtomicU64,
        navsat_recv: AtomicU64,
        motor_send: AtomicU64,
        /// Staged single-rotor failure (0-3, or -1 = none). See the rotor-out slice.
        failed_rotor: AtomicI32,
        /// ACHIEVED per-rotor throttle from the last `step` (post-failure) — the
        /// RPM source the production FDI consumes.
        last_achieved: Mutex<[f32; 4]>,
    }

    impl GazeboPhysics {
        /// Alias for `connect` mirroring the stub signature; panics on failure
        /// (the CLI surface treats a failed connect as a programmer error).
        pub fn new(world: impl Into<String>, model: impl Into<String>) -> Self {
            Self::connect_with_home(world, model, Home::ORIGIN).expect(
                "GazeboPhysics::new: gz-transport Node::new failed; is `gz sim` running \
                 and gz-transport13 on the library path?",
            )
        }

        /// Connect with the world-origin home anchor.
        pub fn connect(world: impl Into<String>, model: impl Into<String>) -> Option<Self> {
            Self::connect_with_home(world, model, Home::ORIGIN)
        }

        /// Connect via the C++-backed `gz-transport`: create a Node, subscribe
        /// the sensor topics (channel-backed), advertise the Actuators
        /// publisher. The C++ transport honours `GZ_IP` / `GZ_PARTITION` from
        /// the environment, so no manual partition handling is needed (unlike
        /// the old pure-Rust bridge). Returns `None` if the node or any
        /// essential subscription/advertise fails.
        pub fn connect_with_home(
            world: impl Into<String>,
            model: impl Into<String>,
            home: Home,
        ) -> Option<Self> {
            let world = world.into();
            let model = model.into();
            let mut node = Node::new()?;

            let imu_topic = format!(
                "/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu"
            );
            let navsat_topic = format!(
                "/world/{world}/model/{model}/link/base_link/sensor/navsat_sensor/navsat"
            );
            let odom_topic = format!("/model/{model}/odometry");
            let pose_topic = format!("/model/{model}/pose");
            let mag_topic = format!(
                "/world/{world}/model/{model}/link/base_link/sensor/mag_sensor/magnetometer"
            );
            // The SDF's MulticopterMotorModel <commandSubTopic> = command/motor_speed.
            let cmd_topic = format!("/{model}/command/motor_speed");

            let imu_rx = node.subscribe_channel::<IMU>(&imu_topic, 200)?;
            let navsat_rx = node.subscribe_channel::<NavSat>(&navsat_topic, 50)?;
            let odom_rx = node.subscribe_channel::<Odometry>(&odom_topic, 50)?;
            let pose_rx = node.subscribe_channel::<Pose_V>(&pose_topic, 50)?;
            // Magnetometer is optional (best-effort heading enhancement).
            let mag_rx = node.subscribe_channel::<Magnetometer>(&mag_topic, 50);
            let publisher = node.advertise::<Actuators>(&cmd_topic)?;

            // Auto-capture the LAUNCH DATUM: if no explicit home was supplied
            // (still ORIGIN), take the first NavSat fix as the home reference so
            // NED position is relative to the launch point. The gz world sits at
            // a non-zero MSL elevation (e.g. falcon-quad.sdf = 488 m Zürich);
            // without this the raw MSL altitude leaks straight into the
            // estimator as a −488 m offset (diagnosed flying the production core:
            // the vehicle looked 488 m "runaway" when it was sitting still).
            let mut home = home;
            if home.lat_deg == 0.0 && home.lon_deg == 0.0 && home.alt_m == 0.0 {
                let start = std::time::Instant::now();
                while start.elapsed().as_secs_f32() < 3.0 {
                    if let Ok(fix) =
                        navsat_rx.recv_timeout(std::time::Duration::from_millis(100))
                    {
                        home = Home {
                            lat_deg: fix.latitude_deg,
                            lon_deg: fix.longitude_deg,
                            alt_m: fix.altitude,
                        };
                        break;
                    }
                }
            }

            Some(Self {
                world_name: world,
                model_name: model,
                home,
                _node: node,
                publisher,
                imu_rx,
                navsat_rx,
                odom_rx,
                pose_rx,
                mag_rx,
                latest_imu: Mutex::new(None),
                latest_position_ned_m: Mutex::new([0.0; 3]),
                latest_velocity_ned: Mutex::new(None),
                latest_heading_ned: Mutex::new(None),
                latest_mag_body_ned: Mutex::new(None),
                imu_recv: AtomicU64::new(0),
                navsat_recv: AtomicU64::new(0),
                motor_send: AtomicU64::new(0),
                failed_rotor: AtomicI32::new(-1),
                last_achieved: Mutex::new([0.0; 4]),
            })
        }

        /// Drain all pending inbound messages into the latest-value caches.
        /// Called at the top of `measure` (the per-tick entry) so the `&self`
        /// read methods see fresh values; keeps only the newest per topic.
        fn pump(&self) {
            // IMU: ENU body → NED body (accel + gyro).
            let mut got_imu = 0u64;
            let mut last_imu = None;
            for msg in self.imu_rx.try_iter() {
                got_imu += 1;
                let accel = msg
                    .linear_acceleration
                    .as_ref()
                    .map(|v| [v.x as f32, v.y as f32, v.z as f32])
                    .unwrap_or([0.0; 3]);
                let gyro = msg
                    .angular_velocity
                    .as_ref()
                    .map(|v| [v.x as f32, v.y as f32, v.z as f32])
                    .unwrap_or([0.0; 3]);
                last_imu = Some(ImuSample {
                    time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                    accel_body: enu_to_ned(accel),
                    gyro_body: enu_to_ned(gyro),
                });
            }
            if let Some(s) = last_imu {
                *self.latest_imu.lock().unwrap() = Some(s);
                self.imu_recv.fetch_add(got_imu, Ordering::Relaxed);
            }
            // NavSat: lat/lon/alt → local NED via Home.
            let mut got_nav = 0u64;
            let mut last_pos = None;
            for msg in self.navsat_rx.try_iter() {
                got_nav += 1;
                last_pos = Some(self.home.project_to_ned_m(
                    msg.latitude_deg,
                    msg.longitude_deg,
                    msg.altitude,
                ));
            }
            if let Some(p) = last_pos {
                *self.latest_position_ned_m.lock().unwrap() = p;
                self.navsat_recv.fetch_add(got_nav, Ordering::Relaxed);
            }
            // Odometry twist: TRUE body velocity ENU → NED.
            let mut last_vel = None;
            for msg in self.odom_rx.try_iter() {
                if let Some(tw) = msg.twist.as_ref() {
                    if let Some(lin) = tw.linear.as_ref() {
                        last_vel =
                            Some(enu_to_ned([lin.x as f32, lin.y as f32, lin.z as f32]));
                    }
                }
            }
            if let Some(v) = last_vel {
                *self.latest_velocity_ned.lock().unwrap() = Some(v);
            }
            // Pose_V: model-root orientation → NED yaw (heading "compass").
            let mut last_yaw = None;
            for msg in self.pose_rx.try_iter() {
                for p in &msg.pose {
                    if let Some(o) = p.orientation.as_ref() {
                        let n2 = o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z;
                        if n2 > 0.5 {
                            let yaw = enu_quat_to_ned_yaw(o.w, o.x, o.y, o.z);
                            if yaw.is_finite() {
                                last_yaw = Some(yaw);
                            }
                            break;
                        }
                    }
                }
            }
            if let Some(y) = last_yaw {
                *self.latest_heading_ned.lock().unwrap() = Some(y);
            }
            // Magnetometer (optional): gz NED-native field → our NED body via Rz(−90°).
            if let Some(rx) = &self.mag_rx {
                let mut last_mag = None;
                for msg in rx.try_iter() {
                    if let Some(f) = msg.field_tesla.as_ref() {
                        let (x, y, z) = (f.x as f32, f.y as f32, f.z as f32);
                        let v = [y, -x, z];
                        if v.iter().all(|c| c.is_finite()) {
                            last_mag = Some(v);
                        }
                    }
                }
                if let Some(m) = last_mag {
                    *self.latest_mag_body_ned.lock().unwrap() = Some(m);
                }
            }
        }

        /// Map a [0,1] mixer output (normalised thrust) to a gz motor command
        /// (rad/s). SQRT map so gz's `thrust ∝ ω²` is LINEAR in the mixer
        /// command (throttle-invariant gain) — the v0.25 stability fix.
        pub fn pwm_to_rad_per_s(pwm: f32) -> f32 {
            const MAX_MOTOR_RAD_S: f32 = 1000.0;
            libm::sqrtf(pwm.clamp(0.0, 1.0)) * MAX_MOTOR_RAD_S
        }
    }

    impl Physics for GazeboPhysics {
        fn name(&self) -> &'static str {
            "gazebo"
        }

        fn step(&mut self, motor_pwm: [f32; 4], _dt: f32) {
            // Apply a staged rotor failure: the failed rotor → 0 rad/s, so the
            // real MulticopterMotorModel loses its thrust + reaction torque.
            let failed = self.failed_rotor.load(Ordering::Relaxed);
            let mut achieved = motor_pwm;
            if (0..4).contains(&failed) {
                achieved[failed as usize] = 0.0;
            }
            *self.last_achieved.lock().unwrap() = achieved;
            let msg = Actuators {
                velocity: vec![
                    Self::pwm_to_rad_per_s(achieved[0]) as f64,
                    Self::pwm_to_rad_per_s(achieved[1]) as f64,
                    Self::pwm_to_rad_per_s(achieved[2]) as f64,
                    Self::pwm_to_rad_per_s(achieved[3]) as f64,
                ],
                ..Default::default()
            };
            let _ = self.publisher.publish(&msg);
            self.motor_send.fetch_add(1, Ordering::Relaxed);
        }

        fn measure(&mut self, _noise_std: f32) -> (ImuSample, [f32; 3]) {
            self.pump();
            let sample = self.latest_imu.lock().unwrap().clone().unwrap_or(ImuSample {
                time: relay_ekf::Timestamp { seconds: 0, fraction: 0 },
                accel_body: [0.0; 3],
                gyro_body: [0.0; 3],
            });
            let pos = *self.latest_position_ned_m.lock().unwrap();
            (sample, pos)
        }

        fn counters(&self) -> Option<(u64, u64, u64)> {
            Some((
                self.imu_recv.load(Ordering::Relaxed),
                self.navsat_recv.load(Ordering::Relaxed),
                self.motor_send.load(Ordering::Relaxed),
            ))
        }

        fn velocity_ned(&self) -> Option<[f32; 3]> {
            *self.latest_velocity_ned.lock().unwrap()
        }

        fn heading_ned(&self) -> Option<f32> {
            *self.latest_heading_ned.lock().unwrap()
        }

        fn mag_body_ned(&self) -> Option<[f32; 3]> {
            // DISABLED: the gz-mag→NED-body frame conversion is UNVALIDATED (its
            // own note: "confirmed at one heading; a yaw-sweep oracle would pin
            // it"). Feeding it drove the IEKF's yaw to ~0.88 rad (50°) at rest,
            // so the geometric controller fought a phantom yaw error → the
            // attitude-priority mixer saturated to [1,0,1,0] (pure yaw) → lost
            // collective → the quad never left the ground (diagnosed v1.113).
            // Without a heading reference the IEKF holds yaw at its initial 0,
            // which is correct for a hover demo. Re-enable once the mag frame is
            // pinned with a yaw-sweep oracle. (`latest_mag_body_ned` still
            // populated for that future validation.)
            None
        }

        fn motor_rpm(&self) -> Option<[i32; 4]> {
            // ACHIEVED throttle × ESC_RPM_FULL (8000, matching falcon-core): the
            // failed rotor reads 0 → the production FDI residual fires.
            const ESC_RPM_FULL: f32 = 8000.0;
            let a = *self.last_achieved.lock().unwrap();
            Some([
                (a[0] * ESC_RPM_FULL) as i32,
                (a[1] * ESC_RPM_FULL) as i32,
                (a[2] * ESC_RPM_FULL) as i32,
                (a[3] * ESC_RPM_FULL) as i32,
            ])
        }

        fn fail_rotor(&mut self, rotor: usize) {
            if rotor < 4 {
                self.failed_rotor.store(rotor as i32, Ordering::Relaxed);
            }
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
            // v0.25 — SQRT map (ω = √pwm·max) so gz thrust ∝ ω² is LINEAR
            // in the mixer's thrust command (throttle-invariant gain).
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(0.0), 0.0);
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(1.0), 1000.0);
            assert!((GazeboPhysics::pwm_to_rad_per_s(0.25) - 500.0).abs() < 1e-3); // √0.25=0.5
            assert!((GazeboPhysics::pwm_to_rad_per_s(0.5) - 707.107).abs() < 1e-2);
            // Clamps below 0 / above 1.
            assert_eq!(GazeboPhysics::pwm_to_rad_per_s(-0.5), 0.0);
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
