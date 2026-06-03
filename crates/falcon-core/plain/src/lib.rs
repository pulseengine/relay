//! # falcon-core — the backend-agnostic flight core (v1.1.0)
//!
//! The verified cascade (IEKF estimator → geometric SE(3) attitude → ADRC
//! inner loop → mixer) factored OUT of the Gazebo bench and behind a
//! hardware-abstraction-layer seam: [`FlightBackend`]. The SAME `no_std`
//! flight code reads IMU/GNSS/mag through the backend and writes motor
//! commands back — nothing in the core knows whether the backend is a
//! simulator or a real flight controller.
//!
//! This is the seam the "build into any drone" claim actually rests on: a
//! drone is "supported" exactly when someone implements `FlightBackend` for
//! its sensors + actuators. The v1.1 deliverable is the seam + the verified
//! inner attitude-stabilization core running through it against a [`SimBackend`];
//! the position/mission outer loop, the gz backend, and a real-hardware
//! backend are the subsequent v1.x releases.

#![no_std]
// 3×3 matrix products read clearest as indexed triple-loops (the same idiom
// the bench's integ_rot uses); the iterator rewrite obscures the math.
#![allow(clippy::needless_range_loop)]

use relay_adrc::{AdrcRate, GyroLpf};
use relay_geo::{GeoAtt, GeoGains};
use relay_iekf::{Iekf, Imu as IekfImu, NavState, Vec3};
use relay_mix_quad::{motors_to_torque_signs, QuadMixer};

/// One inertial-measurement sample in the body frame.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    /// Specific force (accelerometer), m/s².
    pub accel: Vec3,
    /// Angular rate (gyro), rad/s.
    pub gyro: Vec3,
}

/// The hardware-abstraction-layer seam. A flight backend provides the
/// sensors the estimator needs and the actuator the allocator drives, plus a
/// clock. Implement this for a simulator (gz / analytic) OR for a real board
/// (IMU over SPI, GNSS over UART, ESCs over DShot) — the [`FlightCore`] is
/// identical in both cases.
pub trait FlightBackend {
    /// Latest IMU sample (always available).
    fn read_imu(&mut self) -> ImuSample;
    /// Latest position fix in NED metres, or `None` if no fix this tick.
    fn read_position(&mut self) -> Option<Vec3>;
    /// Latest magnetometer field in the body frame (direction only), or
    /// `None` if unavailable.
    fn read_mag(&mut self) -> Option<Vec3>;
    /// Write the per-rotor commands ∈ [0,1] to the actuators.
    fn write_motors(&mut self, motors: &[f32]);
    /// Control period (s) for this tick.
    fn dt(&self) -> f32;
}

/// The verified flight core, generic over the backend. Holds the estimator +
/// controllers; one [`step`](FlightCore::step) reads the backend's sensors,
/// estimates state, computes the stabilizing control, allocates it, and
/// writes the motors back — all on the verified `no_std` crates.
pub struct FlightCore {
    iekf: Iekf,
    geo: GeoAtt,
    adrc: AdrcRate,
    gyro_lpf: GyroLpf,
    mixer: QuadMixer,
    hover_thrust: f32,
    grav_var: f32,
    pos_var: f32,
    mag_var: f32,
    /// Target altitude (NED z, metres; negative = up). v1.2 altitude hold.
    alt_setpoint: f32,
    kp_alt: f32,
    kd_alt: f32,
}

impl FlightCore {
    /// New core, level estimator, falcon-quad gains. `hover_thrust` ∈ [0,1].
    pub fn new(hover_thrust: f32, loop_hz: f32) -> Self {
        FlightCore {
            iekf: Iekf::level(),
            geo: GeoAtt::new(GeoGains::FALCON_QUAD),
            adrc: AdrcRate::falcon_quad(),
            gyro_lpf: GyroLpf::new(60.0, loop_hz),
            mixer: QuadMixer::new(),
            hover_thrust,
            grav_var: 0.5,
            pos_var: 0.01,
            mag_var: 0.1,
            alt_setpoint: 0.0,
            kp_alt: 0.05,
            kd_alt: 0.30,
        }
    }

    /// Command a target altitude (NED z, metres; negative = up). v1.2.
    pub fn set_altitude(&mut self, ned_z: f32) {
        self.alt_setpoint = ned_z;
    }

    /// The estimated nav state (for telemetry / tests).
    pub fn state(&self) -> NavState {
        self.iekf.state()
    }

    /// One control iteration against the backend: sense → estimate → control
    /// (stabilize to level, hold heading) → allocate → actuate.
    pub fn step<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let imu = b.read_imu();

        // ── Estimate ──
        self.iekf.propagate(IekfImu { gyro: imu.gyro, accel: imu.accel }, dt);
        self.iekf.update_gravity(imu.accel, self.grav_var);
        if let Some(p) = b.read_position() {
            self.iekf.update_position(p, self.pos_var);
        }
        if let Some(m) = b.read_mag() {
            self.iekf.update_magnetometer(m, 0.0, self.mag_var);
        }
        let est = self.iekf.state();

        // ── Altitude loop (v1.2) ── thrust = hover − kp·alt_err + kd·v_z,
        // clamped. Vertical is decoupled from the tilt/accel ambiguity, so the
        // estimate's z/vz drive it directly. alt_err = setpoint − estimate.
        let alt_err = self.alt_setpoint - est.p[2];
        let thrust = (self.hover_thrust - self.kp_alt * alt_err + self.kd_alt * est.v[2]).clamp(0.0, 1.0);

        // ── Attitude ── stabilize to level (zero horizontal accel cmd), hold
        // heading 0: geometric desired-rate → ADRC torque on filtered gyro.
        let gyro_f = self.gyro_lpf.filter(imu.gyro);
        let omega_d = self.geo.desired_rate(est.q, [0.0, 0.0, 0.0], 0.0);
        let torque = self.adrc.tick(gyro_f, omega_d, dt);

        // ── Allocate + actuate ──
        let motors = self.mixer.mix(torque, thrust);
        b.write_motors(&motors);
    }
}

/// A deterministic SIMULATION backend (analytic rigid-body attitude plant) —
/// the FIRST backend behind the HAL seam. The motor commands drive the body
/// torque (through the same mixer geometry), the attitude integrates, and the
/// IMU/mag are synthesised from it. Swapping this for a real-hardware
/// `FlightBackend` is the only change needed to fly the same core on a board.
pub struct SimBackend {
    /// Body→NED attitude matrix.
    pub r: [[f32; 3]; 3],
    /// Body rate (rad/s).
    pub omega: Vec3,
    /// Altitude (NED z, m; negative = up) and vertical velocity (NED z, m/s).
    pub alt: f32,
    pub vz: f32,
    j: Vec3,
    dt: f32,
    torque_scale: f32,
    /// Thrust coefficient: total thrust = Σmotors · k_thrust, tuned so that at
    /// the hover collective (4 × 0.5) the thrust equals gravity.
    k_thrust: f32,
    /// A constant body-torque disturbance the verified ESO must reject (v1.2).
    pub disturbance: Vec3,
}

const GRAVITY: f32 = 9.81;

impl SimBackend {
    /// Start at attitude `r0`, at rest, at altitude 0. `dt` = control period.
    pub fn new(r0: [[f32; 3]; 3], dt: f32) -> Self {
        SimBackend {
            r: r0,
            omega: [0.0; 3],
            alt: 0.0,
            vz: 0.0,
            j: GeoGains::FALCON_QUAD.j,
            dt,
            torque_scale: 0.25,
            k_thrust: GRAVITY / (4.0 * 0.5), // hover collective 4×0.5 ⇒ T = g
            disturbance: [0.0; 3],
        }
    }

    /// Body-frame tilt from level (rad): the angle of the body z-axis from NED down.
    pub fn tilt(&self) -> f32 {
        libm::acosf(self.r[2][2].clamp(-1.0, 1.0))
    }

    fn integrate(&mut self, torque: Vec3) {
        let jo = [self.j[0] * self.omega[0], self.j[1] * self.omega[1], self.j[2] * self.omega[2]];
        let gyro = [
            self.omega[1] * jo[2] - self.omega[2] * jo[1],
            self.omega[2] * jo[0] - self.omega[0] * jo[2],
            self.omega[0] * jo[1] - self.omega[1] * jo[0],
        ];
        for i in 0..3 {
            self.omega[i] += self.dt * (torque[i] - gyro[i]) / self.j[i];
        }
        // first-order rotation integration (Rᵢ₊₁ = Rᵢ·(I + [ω]ₓdt))
        let wd = [self.omega[0] * self.dt, self.omega[1] * self.dt, self.omega[2] * self.dt];
        let incr = [[1.0, -wd[2], wd[1]], [wd[2], 1.0, -wd[0]], [-wd[1], wd[0], 1.0]];
        let mut m = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for jj in 0..3 {
                for k in 0..3 {
                    m[i][jj] += self.r[i][k] * incr[k][jj];
                }
            }
        }
        self.r = m;
    }

    /// `Rᵀ·v` — rotate an NED vector into the body frame.
    fn to_body(&self, v: Vec3) -> Vec3 {
        [
            self.r[0][0] * v[0] + self.r[1][0] * v[1] + self.r[2][0] * v[2],
            self.r[0][1] * v[0] + self.r[1][1] * v[1] + self.r[2][1] * v[2],
            self.r[0][2] * v[0] + self.r[1][2] * v[1] + self.r[2][2] * v[2],
        ]
    }
}

impl FlightBackend for SimBackend {
    fn read_imu(&mut self) -> ImuSample {
        // at hover (no translation) the accelerometer reads the gravity
        // reaction (pointing "up" = −z in NED), rotated into the body frame.
        let accel = self.to_body([0.0, 0.0, -GRAVITY]);
        ImuSample { accel, gyro: self.omega }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        Some([0.0, 0.0, self.alt]) // horizontal pinned (vertical+attitude sim); 6-DoF is v1.3
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        Some(self.to_body([1.0, 0.0, 0.0])) // NED north, body frame → yaw observable
    }
    fn write_motors(&mut self, motors: &[f32]) {
        let mut m4 = [0.0f32; 4];
        let mut collective = 0.0f32;
        for (d, &s) in m4.iter_mut().zip(motors.iter()) {
            *d = s;
            collective += s;
        }
        // attitude: allocated torque + the injected disturbance the ESO rejects
        let tq = motors_to_torque_signs(m4);
        let torque = [
            tq[0] * self.torque_scale + self.disturbance[0],
            tq[1] * self.torque_scale + self.disturbance[1],
            tq[2] * self.torque_scale + self.disturbance[2],
        ];
        self.integrate(torque);
        // vertical: thrust (along −body-z) projected onto NED-z, minus gravity.
        let thrust = collective * self.k_thrust;
        let accel_z = (-thrust * self.r[2][2] + GRAVITY) / 1.0; // m = 1
        self.vz += self.dt * accel_z;
        self.alt += self.dt * self.vz;
    }
    fn dt(&self) -> f32 {
        self.dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The SAME verified cascade, run through the HAL seam against the sim
    /// backend, recovers a tilted body to level — demonstrating the flight
    /// core is backend-agnostic (the seam carries the real IEKF + geometric +
    /// ADRC + mixer, not a stub).
    #[test]
    fn flight_core_stabilizes_through_the_hal() {
        let dt = 0.002f32;
        // start tilted ~23° about x
        let th = 0.4f32;
        let (c, s) = (libm::cosf(th), libm::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let mut backend = SimBackend::new(r0, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);

        let tilt0 = backend.tilt();
        assert!(tilt0 > 0.35, "should start tilted, {tilt0}");
        for _ in 0..6000 {
            core.step(&mut backend);
        }
        let tilt = backend.tilt();
        assert!(tilt < 0.1, "core must recover to level through the HAL: {tilt} rad (start {tilt0})");
    }

    /// The backend is a SEAM, not a fixed simulator: a trivial stand-in
    /// backend (constant level IMU, no fixes) drives the core with zero panics
    /// and bounded motors — i.e. any `FlightBackend` impl works.
    #[test]
    fn arbitrary_backend_drives_the_core() {
        struct NullBackend {
            motors: [f32; 4],
        }
        impl FlightBackend for NullBackend {
            fn read_imu(&mut self) -> ImuSample {
                ImuSample { accel: [0.0, 0.0, -GRAVITY], gyro: [0.0; 3] }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                None
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn write_motors(&mut self, motors: &[f32]) {
                for (d, &s) in self.motors.iter_mut().zip(motors.iter()) {
                    *d = s;
                }
            }
            fn dt(&self) -> f32 {
                0.001
            }
        }
        let mut b = NullBackend { motors: [0.0; 4] };
        let mut core = FlightCore::new(0.5, 1000.0);
        for _ in 0..100 {
            core.step(&mut b);
        }
        for &m in &b.motors {
            assert!((0.0..=1.0).contains(&m), "motor out of range: {m}");
        }
    }

    // ── v1.2 altitude hold + disturbance rejection ───────────────────────

    /// The backend-agnostic core climbs to and holds a commanded altitude
    /// through the HAL (the thrust/altitude loop, decoupled from tilt).
    #[test]
    fn altitude_hold_climbs_to_setpoint() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0); // 2 m up (NED z negative)
        for _ in 0..15000 {
            core.step(&mut backend);
        }
        assert!((backend.alt + 2.0).abs() < 0.25, "altitude must reach −2 m: {}", backend.alt);
        assert!(backend.tilt() < 0.1, "should stay level while holding altitude: {}", backend.tilt());
    }

    /// The verified ADRC inner loop REJECTS a sustained body-torque
    /// disturbance and holds the body near level — through the HAL. The ESO
    /// estimates and cancels the disturbance, so the tilt stays bounded.
    #[test]
    fn disturbance_rejected_holds_level() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        backend.disturbance = [0.25, -0.15, 0.0]; // sustained roll+pitch disturbance
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        let mut peak_after = 0.0f32;
        for k in 0..8000 {
            core.step(&mut backend);
            if k > 4000 {
                let t = backend.tilt();
                if t > peak_after {
                    peak_after = t;
                }
            }
        }
        // after the ESO converges, the disturbance is cancelled and the body
        // holds near level (a plain proportional loop would sit at a steady
        // offset; ADRC drives it out).
        assert!(peak_after < 0.12, "ESO must reject the disturbance: steady tilt {peak_after} rad");
    }
}
