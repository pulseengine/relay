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
    /// Battery voltage (V). Default = a healthy pack; real backends read the
    /// ADC. Used by the supervisor's low-battery failsafe (v1.8).
    fn read_battery_v(&mut self) -> f32 {
        16.0
    }
    /// Barometric altitude (NED z, metres; negative = up), or `None` if no
    /// barometer. An INDEPENDENT vertical source the core fuses so altitude
    /// survives GPS-vertical loss (v1.20). Default `None`.
    fn read_baro(&mut self) -> Option<f32> {
        None
    }
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
    /// Target position (NED metres; z negative = up). v1.2 altitude + v1.3
    /// horizontal hold.
    setpoint: Vec3,
    kp_alt: f32,
    kd_alt: f32,
    /// Altitude-error integral (v1.22) — rejects a steady thrust deficit (the
    /// air-density lapse at altitude) the P-D altitude loop alone leaves as an
    /// offset. Anti-windup clamps its thrust contribution.
    ki_alt: f32,
    alt_int: f32,
    alt_int_max: f32,
    kp_pos: f32,
    kd_vel: f32,
    ki_pos: f32,
    a_cmd_max: f32,
    /// Horizontal position-error integral (x,y). The integral term (v1.16) is
    /// what rejects a STEADY force disturbance (wind) the P-D loop alone leaves
    /// as a position offset. Anti-windup clamps its acceleration contribution.
    pos_int: [f32; 2],
    pos_int_max: f32,
    /// Barometric-altitude measurement variance (m²) (v1.20). The baro is fed
    /// into the verified IEKF as a vertical anchor (a position update whose z is
    /// the baro), so altitude AND its rate survive GPS-vertical loss.
    baro_var: f32,
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
            setpoint: [0.0; 3],
            kp_alt: 0.05,
            kd_alt: 0.30,
            ki_alt: 0.0, // opt-in (set_altitude_integral_gain): the integral is
            alt_int: 0.0, // for high-altitude thrust-lapse compensation; default
            alt_int_max: 0.4, // off as it interacts with aggressive alt transients.
            kp_pos: 0.08,
            kd_vel: 0.6,
            ki_pos: 0.02,
            a_cmd_max: 1.0,
            pos_int: [0.0; 2],
            pos_int_max: 1.5,
            baro_var: 0.05, // ≈ (0.2 m)² baro noise; trusted less than a clean GPS-z
        }
    }

    /// Command a target altitude (NED z, metres; negative = up). v1.2.
    pub fn set_altitude(&mut self, ned_z: f32) {
        self.setpoint[2] = ned_z;
    }

    /// Command a full NED position target (metres; z negative = up). v1.3.
    pub fn set_position(&mut self, ned: Vec3) {
        self.setpoint = ned;
    }

    /// Set the horizontal position-loop integral gain (v1.16). `0` disables the
    /// integral (P-D only) — used to show the steady-wind offset the integral
    /// removes. Resets the accumulated integral.
    pub fn set_position_integral_gain(&mut self, ki: f32) {
        self.ki_pos = ki;
        self.pos_int = [0.0; 2];
    }

    /// Set the altitude-loop integral gain (v1.22). `0` disables it (P-D only) —
    /// used to show the altitude offset under thrust lapse the integral removes.
    pub fn set_altitude_integral_gain(&mut self, ki: f32) {
        self.ki_alt = ki;
        self.alt_int = 0.0;
    }

    /// Set the IEKF position-measurement variance (m²) (v1.19). This must match
    /// the ACTUAL GNSS fix noise: the default (0.01 = 1 cm²) over-trusts a
    /// metre-class fix, injecting noise the loop chases into instability. Set it
    /// to ≈ stddev² of the real receiver.
    pub fn set_pos_var(&mut self, var: f32) {
        self.pos_var = var;
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
        // v1.20 — barometer: feed it into the verified IEKF as a vertical anchor
        // (a position update whose horizontal is the current estimate, a no-op,
        // and whose z is the baro). This keeps the IEKF's altitude AND vertical
        // velocity bounded through a GPS-vertical outage, reusing the filter's
        // estimation rather than a hand-rolled complementary filter.
        if let Some(bz) = b.read_baro() {
            let e = self.iekf.state();
            self.iekf.update_position([e.p[0], e.p[1], bz], self.baro_var);
        }
        let est = self.iekf.state();

        // ── Altitude loop (v1.2; v1.20 baro-anchored; v1.22 +I) ── thrust =
        // hover − kp·alt_err − ki·∫alt_err + kd·v_z, clamped. The integral
        // (v1.22) rejects a steady thrust deficit (the air-density lapse at
        // altitude) the P-D loop alone leaves as an altitude offset.
        let alt_err = self.setpoint[2] - est.p[2];
        self.alt_int += alt_err * dt;
        let cap = if self.ki_alt > 0.0 { self.alt_int_max / self.ki_alt } else { 0.0 };
        self.alt_int = self.alt_int.clamp(-cap, cap);
        let thrust = (self.hover_thrust - self.kp_alt * alt_err - self.ki_alt * self.alt_int
            + self.kd_alt * est.v[2])
            .clamp(0.0, 1.0);

        // ── Horizontal position loop (v1.3 P-D, v1.16 +I) ── P-I-D on (pos,
        // vel) error → commanded NED acceleration, magnitude-saturated. The
        // vehicle TILTS to realise a_cmd. The INTEGRAL (v1.16) rejects a steady
        // force disturbance (wind): the P-D terms alone settle at an offset
        // where kp·err balances the wind force; the integral winds up to supply
        // that command at zero error. Anti-windup clamps its accel contribution.
        let perr = [self.setpoint[0] - est.p[0], self.setpoint[1] - est.p[1]];
        for i in 0..2 {
            self.pos_int[i] += perr[i] * dt;
            let cap = if self.ki_pos > 0.0 { self.pos_int_max / self.ki_pos } else { 0.0 };
            self.pos_int[i] = self.pos_int[i].clamp(-cap, cap);
        }
        let mut a_cmd = [
            self.kp_pos * perr[0] - self.kd_vel * est.v[0] + self.ki_pos * self.pos_int[0],
            self.kp_pos * perr[1] - self.kd_vel * est.v[1] + self.ki_pos * self.pos_int[1],
            0.0,
        ];
        let ah = relay_math::sqrtf(a_cmd[0] * a_cmd[0] + a_cmd[1] * a_cmd[1]);
        if ah > self.a_cmd_max {
            let s = self.a_cmd_max / ah;
            a_cmd[0] *= s;
            a_cmd[1] *= s;
        }

        // ── Attitude ── geometric desired-rate from a_cmd (hold heading 0) →
        // ADRC torque on filtered gyro.
        let gyro_f = self.gyro_lpf.filter(imu.gyro);
        let omega_d = self.geo.desired_rate(est.q, a_cmd, 0.0);
        let torque = self.adrc.tick(gyro_f, omega_d, dt);

        // ── Allocate + actuate ──
        let motors = self.mixer.mix(torque, thrust);
        b.write_motors(&motors);
    }
}

/// The autonomy supervisor (v1.8): wraps the verified [`FlightCore`] with the
/// [`relay_fsm`] flight-mode state machine and the failsafe monitors the
/// clean-room audit found missing — a geofence that ACTUATES RTL (not just
/// detects) and a low-battery failsafe. Each step it reads the estimate, fires
/// failsafe + milestone events into the FSM, maps the resulting mode to a
/// position setpoint, and steps the verified core. The same backend-agnostic
/// seam carries it.
pub struct FlightSupervisor {
    core: FlightCore,
    fsm: relay_fsm::FlightFsm,
    home: Vec3,
    fence_radius: f32,
    cruise_alt: f32,
    low_batt_v: f32,
    mission: Vec3,
    rtl_latched: bool,
}

impl FlightSupervisor {
    /// `home` NED, `fence_radius` m (breach ⇒ RTL), `cruise_alt` m AGL,
    /// `low_batt_v` V (below ⇒ failsafe).
    pub fn new(home: Vec3, fence_radius: f32, cruise_alt: f32, low_batt_v: f32) -> Self {
        FlightSupervisor {
            core: FlightCore::new(0.5, 1000.0),
            fsm: relay_fsm::FlightFsm::new(),
            home,
            fence_radius,
            cruise_alt,
            low_batt_v,
            mission: home,
            rtl_latched: false,
        }
    }

    pub fn mode(&self) -> relay_fsm::Mode {
        self.fsm.mode()
    }

    pub fn state(&self) -> NavState {
        self.core.state()
    }

    /// Inject an external command/event (Arm, RequestTakeoff, RequestMission…).
    pub fn command(&mut self, ev: relay_fsm::Event, level: bool, throttle_low: bool) {
        let g = relay_fsm::Gates { level, throttle_low, have_position: true };
        self.fsm.on(ev, g);
    }

    /// Set the mission target (used when in Mission mode).
    pub fn set_mission(&mut self, ned: Vec3) {
        self.mission = ned;
    }

    /// One supervised control step.
    pub fn step<B: FlightBackend>(&mut self, b: &mut B) {
        use relay_fsm::{Event, Gates, Mode};
        let est = self.core.state();
        let dx = est.p[0] - self.home[0];
        let dy = est.p[1] - self.home[1];
        let dist_home = relay_math::sqrtf(dx * dx + dy * dy);
        let alt_agl = -est.p[2]; // NED z negative = up
        let g = Gates { level: true, throttle_low: true, have_position: true };

        // ── FAILSAFE actuation (the audit's gap): geofence breach OR low
        // battery from any flying state ⇒ Failsafe ⇒ the FSM commands RTL. ──
        let batt = b.read_battery_v();
        let breach = dist_home > self.fence_radius || batt < self.low_batt_v;
        if breach && self.fsm.is_airborne() && self.fsm.mode() != Mode::Land {
            self.fsm.on(Event::Failsafe, g);
            self.rtl_latched = true;
        }

        // ── milestone events ──
        match self.fsm.mode() {
            Mode::Takeoff if alt_agl > self.cruise_alt - 0.2 => {
                self.fsm.on(Event::ReachedAltitude, g);
            }
            Mode::Rtl if dist_home < 0.4 => {
                self.fsm.on(Event::ReachedHome, g); // ⇒ Land
            }
            Mode::Land if alt_agl < 0.15 => {
                self.fsm.on(Event::Touchdown, g); // ⇒ Disarmed
            }
            _ => {}
        }

        // ── mode → setpoint ──
        let sp = match self.fsm.mode() {
            Mode::Takeoff | Mode::Loiter => [est.p[0], est.p[1], -self.cruise_alt],
            Mode::Mission => [self.mission[0], self.mission[1], -self.cruise_alt],
            Mode::Rtl => [self.home[0], self.home[1], -self.cruise_alt],
            Mode::Land => [self.home[0], self.home[1], 0.0], // descend to ground
            Mode::Disarmed | Mode::Armed => [est.p[0], est.p[1], est.p[2]], // hold (idle)
        };
        self.core.set_position(sp);
        self.core.step(b);
    }
}

/// Injected sensor pathologies (v1.9) — the four failure modes the clean-room
/// audit found untested because the sim fed the estimator a perfect world:
/// broadband accelerometer **vibration**, a slow **gyro bias drift** the IEKF
/// bias state must track, a **GPS dropout** window that forces dead-reckoning,
/// and **magnetometer interference**. All deterministic (a counter-seeded LCG),
/// so a robustness PASS — or a falsification — is reproducible.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pathology {
    /// Accelerometer vibration amplitude (m/s², broadband zero-mean). Corrupts
    /// the gravity-direction attitude update.
    pub vibration: f32,
    /// Gyro bias drift rate (rad/s per second). Each axis accumulates this ramp;
    /// the IEKF's gyro-bias state must track it or the attitude walks off.
    pub gyro_bias_drift: f32,
    /// GPS dropout: no position fix for control steps in
    /// `[gps_dropout_start, gps_dropout_start + gps_dropout_len)`.
    pub gps_dropout_start: u32,
    pub gps_dropout_len: u32,
    /// Magnetometer interference amplitude (added per-axis, body frame). A
    /// heading perturbation the mag-update variance must tolerate.
    pub mag_interference: f32,
    /// Gyro bias random-walk rate (rad/s per √s) — a STOCHASTIC, wandering bias
    /// (bias-instability), richer than the constant `gyro_bias_drift` ramp. The
    /// IEKF must continuously re-track a moving target (v1.18).
    pub gyro_bias_rw: f32,
    /// Gyro white-noise stddev (rad/s) added to each measured rate (v1.18).
    pub gyro_white: f32,
    /// GNSS position-fix noise stddev (m, per axis) — continuous (v1.19).
    pub gps_noise: f32,
    /// Intermittent GNSS outage period (control steps). If > 0, the fix drops
    /// for `gps_dropout_len` steps every `gps_dropout_period` steps — a
    /// recurring loss the estimator must dead-reckon through and re-acquire
    /// (v1.19, distinct from v1.9's single window).
    pub gps_dropout_period: u32,
}

// ── v1.11 — the real-hardware backend SEAM ───────────────────────────────
//
// This is the honest hardware boundary. Everything ABOVE runs the verified
// flight core against a simulated plant. To fly the SAME `FlightCore` /
// `FlightSupervisor` on a real airframe, you implement the five driver seams
// below (over `embedded-hal` SPI/UART/PWM/ADC for your specific sensors) and
// hand them to a [`HardwareBackend`]. Nothing in the flight code changes — the
// estimator, the geometric controller, the ADRC loop, the mixer, the FSM, the
// failsafes, and the kernel-checked WCET argument are all backend-agnostic.
//
// What this seam DOES give you, here and now (verified by the v1.11 test):
//   • the five driver contracts a board must satisfy, in SI/body-frame units;
//   • a `HardwareBackend` that implements `FlightBackend` purely by delegating
//     to them — so the composition is type-checked end-to-end;
//   • a closed-loop demonstration: the verified core STABILISES through the
//     `HardwareBackend` trait indirection against a shared simulated plant,
//     proving the seam carries a real control loop (not a placeholder).
//
// What it deliberately does NOT claim (the documented GAPS — these need YOUR
// board and must not be faked):
//   • real driver bodies: the register sequences / bus transactions for a
//     specific IMU (e.g. ICM-42688 over SPI), GNSS (UBX over UART), mag, ESC
//     (DShot), and battery ADC — and their on-silicon validation;
//   • sensor calibration (bias/scale/axis alignment) on the real units;
//   • flight-tuning of the control gains on the real airframe;
//   • discharging the v1.10 per-stage WCET leaf budgets with measured
//     Cortex-M7 cycle counts.

/// A 6-axis IMU driver: returns the latest accelerometer + gyro sample already
/// converted to SI body-frame units (m/s², rad/s). Your impl does the SPI/I²C
/// read + unit scaling + axis remap for your specific chip.
pub trait ImuDriver {
    fn read(&mut self) -> ImuSample;
}

/// A position-source driver (GNSS / UWB / VIO): NED metres, or `None` when there
/// is no fresh fix this tick. Your impl parses the receiver and applies the
/// datum/origin transform.
pub trait PositionDriver {
    fn read(&mut self) -> Option<Vec3>;
}

/// A magnetometer driver: body-frame field direction, or `None` when
/// unavailable. Your impl does the read + hard/soft-iron correction.
pub trait MagDriver {
    fn read(&mut self) -> Option<Vec3>;
}

/// The actuator driver: write per-rotor throttles ∈ [0,1] to the ESCs. Your
/// impl maps to DShot / OneShot / PWM.
pub trait MotorDriver {
    fn write(&mut self, motors: &[f32]);
}

/// A battery-voltage driver (ADC), volts. Default keeps a healthy pack so a
/// board without a monitor still flies; override to read the real divider.
pub trait BatteryDriver {
    fn voltage(&mut self) -> f32 {
        16.0
    }
}

/// The real-hardware [`FlightBackend`], generic over the five driver seams. The
/// SAME `FlightCore` / `FlightSupervisor` that flies [`SimBackend`] flies this —
/// the ONLY difference between simulation and a real board is which driver
/// impls sit behind these traits. `dt` is the fixed control period.
pub struct HardwareBackend<I, P, M, O, B> {
    pub imu: I,
    pub gnss: P,
    pub mag: M,
    pub motors: O,
    pub battery: B,
    pub dt: f32,
}

impl<I, P, M, O, B> FlightBackend for HardwareBackend<I, P, M, O, B>
where
    I: ImuDriver,
    P: PositionDriver,
    M: MagDriver,
    O: MotorDriver,
    B: BatteryDriver,
{
    fn read_imu(&mut self) -> ImuSample {
        self.imu.read()
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.gnss.read()
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.mag.read()
    }
    fn write_motors(&mut self, motors: &[f32]) {
        self.motors.write(motors)
    }
    fn dt(&self) -> f32 {
        self.dt
    }
    fn read_battery_v(&mut self) -> f32 {
        self.battery.voltage()
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
    /// Position and velocity in NED (m, m/s). pos[2] is altitude (negative up).
    pub pos: Vec3,
    pub vel: Vec3,
    j: Vec3,
    dt: f32,
    torque_scale: f32,
    /// Thrust coefficient: total thrust = Σmotors · k_thrust, tuned so that at
    /// the hover collective (4 × 0.5) the thrust equals gravity.
    k_thrust: f32,
    /// A constant body-torque disturbance the verified ESO must reject (v1.2).
    pub disturbance: Vec3,
    /// Steady wind velocity in NED (m/s) — a translational FORCE disturbance
    /// (relative-velocity drag, mirroring gz WindEffects F = k·(v_wind−v_body)).
    /// The position loop, not the ESO, must reject this (v1.16).
    pub wind: Vec3,
    /// Gust amplitude (m/s) added to the wind as deterministic broadband noise.
    pub gust_amp: f32,
    /// Quadratic aerodynamic drag coefficient (v1.17): F = −Cd·|v_air|·v_air on
    /// the horizontal relative airspeed (v_body − v_wind). Unlike the linear
    /// wind term it grows with v², so it dominates during fast motion (mission
    /// legs) and caps the drift speed; it is also a stabilising damping force.
    pub drag_quad: f32,
    /// Barometer present? (v1.20) When true, `read_baro` returns the altitude.
    pub baro_enabled: bool,
    /// Barometric altitude noise stddev (m) (v1.20).
    pub baro_noise: f32,
    /// Battery terminal voltage reported to the supervisor's failsafe (v1.8).
    /// When `battery_drain` is on (v1.21) this is COMPUTED each step from the
    /// charge and load, not set: V = 12.6 + 4.2·charge − R·I (open-circuit +
    /// sag), so the failsafe fires on a real endurance limit.
    pub battery_v: f32,
    /// Drain the battery with motor load? (v1.21) Off = a fixed `battery_v`
    /// (v1.8 behaviour). On = the LinearBattery-style draining model below.
    pub battery_drain: bool,
    /// Normalised state of charge (1.0 = full). Drains with motor power (v1.21).
    pub battery_charge: f32,
    /// Thrust-lapse rate per metre of altitude (v1.22): air density falls with
    /// altitude, so thrust = collective·k_thrust·(1 − lapse·alt_m), floored at
    /// 0.4. The altitude loop must raise collective to compensate; if the lapse
    /// exceeds the thrust margin the vehicle hits a service ceiling.
    pub thrust_lapse: f32,
    /// Motor first-order time constant τ (s) (v1.23): the actual rotor speed
    /// lags the command (state += (cmd−state)·dt/τ). 0 = instantaneous. The
    /// actuator lag the ADRC ESO is designed to absorb.
    pub motor_tau: f32,
    /// Per-rotor lagged actual motor state (v1.23).
    motor_state: [f32; 4],
    /// Injected sensor pathologies (v1.9).
    pub path: Pathology,
    /// Control-tick counter (drives the GPS-dropout window).
    step_n: u32,
    /// Accumulated gyro bias (rad/s) injected into the measured rate.
    gyro_bias: Vec3,
    /// LCG state for the deterministic broadband noise.
    rng: u32,
}

const GRAVITY: f32 = 9.81;

impl SimBackend {
    /// Start at attitude `r0`, at rest, at altitude 0. `dt` = control period.
    pub fn new(r0: [[f32; 3]; 3], dt: f32) -> Self {
        SimBackend {
            r: r0,
            omega: [0.0; 3],
            pos: [0.0; 3],
            vel: [0.0; 3],
            j: GeoGains::FALCON_QUAD.j,
            dt,
            torque_scale: 0.25,
            k_thrust: GRAVITY / (4.0 * 0.5), // hover collective 4×0.5 ⇒ T = g
            disturbance: [0.0; 3],
            wind: [0.0; 3],
            gust_amp: 0.0,
            drag_quad: 0.0,
            baro_enabled: false,
            baro_noise: 0.0,
            battery_v: 16.0,
            battery_drain: false,
            battery_charge: 1.0,
            thrust_lapse: 0.0,
            motor_tau: 0.0,
            motor_state: [0.5; 4], // start at hover collective
            path: Pathology::default(),
            step_n: 0,
            gyro_bias: [0.0; 3],
            rng: 0x9E3779B9, // a fixed, non-zero seed (golden-ratio constant)
        }
    }

    /// Inject sensor pathologies (v1.9). Builder form for the robustness tests.
    pub fn with_pathology(mut self, path: Pathology) -> Self {
        self.path = path;
        self
    }

    /// Body-frame tilt from level (rad): the angle of the body z-axis from NED down.
    pub fn tilt(&self) -> f32 {
        relay_math::acosf(self.r[2][2].clamp(-1.0, 1.0))
    }

    /// One deterministic broadband sample in [−1, 1] (LCG; no `rand`, no_std).
    fn noise_unit(&mut self) -> f32 {
        self.rng = self.rng.wrapping_mul(1664525).wrapping_add(1013904223);
        // top 24 bits → [0,1) → [−1,1)
        ((self.rng >> 8) as f32 / (1u32 << 24) as f32) * 2.0 - 1.0
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
        let mut accel = self.to_body([0.0, 0.0, -GRAVITY]);
        // v1.9 — broadband vibration on the specific force.
        if self.path.vibration > 0.0 {
            let v = self.path.vibration;
            accel[0] += v * self.noise_unit();
            accel[1] += v * self.noise_unit();
            accel[2] += v * self.noise_unit();
        }
        // v1.9 — the measured gyro carries the accumulated (drifting) bias the
        // IEKF bias state must estimate out. v1.18 — plus optional white noise.
        let gw = self.path.gyro_white;
        let gyro = [
            self.omega[0] + self.gyro_bias[0] + gw * self.noise_unit(),
            self.omega[1] + self.gyro_bias[1] + gw * self.noise_unit(),
            self.omega[2] + self.gyro_bias[2] + gw * self.noise_unit(),
        ];
        ImuSample { accel, gyro }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        let p = self.path;
        // v1.9 — single GPS dropout window: no fix ⇒ the IEKF dead-reckons.
        if p.gps_dropout_len > 0
            && self.step_n >= p.gps_dropout_start
            && self.step_n < p.gps_dropout_start + p.gps_dropout_len
        {
            return None;
        }
        // v1.19 — INTERMITTENT periodic outage: drop the fix for gps_dropout_len
        // steps every gps_dropout_period steps (recurring loss, not one window).
        if p.gps_dropout_period > 0 && (self.step_n % p.gps_dropout_period) < p.gps_dropout_len {
            return None;
        }
        // v1.19 — continuous GNSS position noise (Gaussian per axis).
        if p.gps_noise > 0.0 {
            return Some([
                self.pos[0] + p.gps_noise * self.noise_unit(),
                self.pos[1] + p.gps_noise * self.noise_unit(),
                self.pos[2] + p.gps_noise * self.noise_unit(),
            ]);
        }
        Some(self.pos) // full 6-DoF NED position (v1.3)
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        let mut m = self.to_body([1.0, 0.0, 0.0]); // NED north → yaw observable
        // v1.9 — magnetometer interference (the update variance must tolerate).
        if self.path.mag_interference > 0.0 {
            let mi = self.path.mag_interference;
            m[0] += mi * self.noise_unit();
            m[1] += mi * self.noise_unit();
            m[2] += mi * self.noise_unit();
        }
        Some(m)
    }
    fn write_motors(&mut self, motors: &[f32]) {
        // v1.23 — first-order MOTOR DYNAMICS: the actual rotor speed lags the
        // command, state += (cmd − state)·dt/τ. The torque/thrust use the lagged
        // ACTUAL, not the command — exactly the actuator lag the ADRC ESO is
        // built to absorb (v0.25). τ = 0 ⇒ instantaneous (prior behaviour).
        let mut m4 = [0.0f32; 4];
        let mut collective = 0.0f32;
        for i in 0..4 {
            let cmd = motors.get(i).copied().unwrap_or(0.0);
            if self.motor_tau > 0.0 {
                self.motor_state[i] += (cmd - self.motor_state[i]) * (self.dt / self.motor_tau);
                m4[i] = self.motor_state[i];
            } else {
                m4[i] = cmd;
            }
            collective += m4[i];
        }
        // attitude: allocated torque + the injected disturbance the ESO rejects
        let tq = motors_to_torque_signs(m4);
        let torque = [
            tq[0] * self.torque_scale + self.disturbance[0],
            tq[1] * self.torque_scale + self.disturbance[1],
            tq[2] * self.torque_scale + self.disturbance[2],
        ];
        self.integrate(torque);
        // translation: thrust acts along −body-z; in NED that is
        // −T·(R·ẑ) = −T·[r02, r12, r22]; add gravity (+g on NED-z). m = 1.
        // v1.22 — air-density thrust lapse: thrust falls with altitude (alt =
        // −pos[2], NED), so the altitude loop must raise collective; beyond the
        // margin the vehicle hits a service ceiling.
        let lapse = if self.thrust_lapse > 0.0 {
            (1.0 - self.thrust_lapse * (-self.pos[2]).max(0.0)).max(0.4)
        } else {
            1.0
        };
        let thrust = collective * self.k_thrust * lapse;
        let mut accel = [
            -thrust * self.r[0][2],
            -thrust * self.r[1][2],
            -thrust * self.r[2][2] + GRAVITY,
        ];
        // v1.16 — wind: a relative-velocity drag force in NED (mass = 1), the
        // gz-WindEffects form F = K_WIND·(v_wind − v_body), with deterministic
        // gusts. A translational disturbance the position loop (not the ESO)
        // must reject. Horizontal only. K_WIND is sized so a strong 5 m/s wind
        // (~0.75 m/s²) stays within the gentle tilt authority (a_cmd_max ≈ 1
        // m/s²); a wind whose force exceeds that authority blows the vehicle
        // away regardless of control — a real, documented limit, not a bug.
        const K_WIND: f32 = 0.15;
        if self.wind[0] != 0.0 || self.wind[1] != 0.0 || self.gust_amp != 0.0 {
            for i in 0..2 {
                let gust = self.gust_amp * self.noise_unit();
                accel[i] += K_WIND * (self.wind[i] + gust - self.vel[i]);
            }
        }
        // v1.17 — quadratic aerodynamic drag on the horizontal relative airspeed
        // (v_body − v_wind): F = −Cd·|v_air|·v_air. Grows with v² (dominates fast
        // motion, caps drift speed) and damps the vehicle. mass = 1.
        if self.drag_quad > 0.0 {
            let va = [self.vel[0] - self.wind[0], self.vel[1] - self.wind[1]];
            let speed = relay_math::sqrtf(va[0] * va[0] + va[1] * va[1]);
            for i in 0..2 {
                accel[i] -= self.drag_quad * speed * va[i];
            }
        }
        for i in 0..3 {
            self.vel[i] += self.dt * accel[i];
            self.pos[i] += self.dt * self.vel[i];
        }
        // v1.9 — advance the tick clock and integrate the gyro bias drift
        // (once per control step; write_motors is the tail of FlightCore::step).
        self.step_n = self.step_n.wrapping_add(1);
        let db = self.path.gyro_bias_drift * self.dt;
        // v1.18 — stochastic random-walk component: bias += σ_rw·√dt·N(0,1). A
        // wandering bias the IEKF must continuously re-track (vs the ramp).
        let rw = self.path.gyro_bias_rw * relay_math::sqrtf(self.dt);
        for i in 0..3 {
            self.gyro_bias[i] += db + rw * self.noise_unit();
        }
        // v1.21 — draining battery (LinearBattery-style): the motor collective is
        // the current draw; charge depletes; terminal voltage = open-circuit +
        // load sag. The supervisor's failsafe then fires on real endurance.
        if self.battery_drain {
            let current = collective; // ∝ total motor power
            self.battery_charge = (self.battery_charge - current * 5.0e-5 * self.dt / 0.002).max(0.0);
            self.battery_v = 12.6 + 4.2 * self.battery_charge - 0.3 * current;
        }
    }
    fn dt(&self) -> f32 {
        self.dt
    }
    fn read_battery_v(&mut self) -> f32 {
        self.battery_v
    }
    fn read_baro(&mut self) -> Option<f32> {
        if self.baro_enabled {
            Some(self.pos[2] + self.baro_noise * self.noise_unit()) // NED z, noisy
        } else {
            None
        }
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
        let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
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
        assert!((backend.pos[2] + 2.0).abs() < 0.25, "altitude must reach −2 m: {}", backend.pos[2]);
        assert!(backend.tilt() < 0.1, "should stay level while holding altitude: {}", backend.tilt());
    }

    /// v1.3 — full 6-DoF: the backend-agnostic core flies to and holds a
    /// horizontal position setpoint through the HAL (tilts to translate, the
    /// estimator + position loop bring it home). Honest scope: the SimBackend
    /// uses the near-hover accelerometer model; the realistic specific-force +
    /// acceleration-compensated path is exercised in the gz bench (v0.30–35).
    #[test]
    fn position_hold_flies_to_setpoint() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([2.0, -1.5, -2.0]); // 2 m N, 1.5 m W, 2 m up
        for _ in 0..30000 {
            core.step(&mut backend);
        }
        let e = [
            backend.pos[0] - 2.0,
            backend.pos[1] + 1.5,
            backend.pos[2] + 2.0,
        ];
        let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        assert!(err < 0.5, "must reach the position setpoint through the HAL: {err} m, pos {:?}", backend.pos);
        assert!(backend.tilt() < 0.15, "settle near level: {} rad", backend.tilt());
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

    // ── v1.8 supervisor: geofence→RTL actuation + battery failsafe ────────

    /// A geofence breach ACTUATES a return-to-launch (the audit's gap: the
    /// geofence detected but never commanded the vehicle home). Commanded to a
    /// mission point OUTSIDE the fence, the drone crosses it, the supervisor
    /// fires Failsafe → the FSM commands RTL → it flies home and lands — never
    /// reaching the out-of-bounds target.
    #[test]
    fn geofence_breach_actuates_rtl_home() {
        use relay_fsm::{Event, Mode};
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.5, 2.0, 14.0);
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut backend);
        }
        assert_eq!(sup.mode(), Mode::Loiter, "should reach Loiter after takeoff");
        sup.set_mission([4.0, 0.0, -2.0]); // OUTSIDE the 1.5 m fence
        sup.command(Event::RequestMission, true, false);
        for _ in 0..40000 {
            sup.step(&mut backend);
            if sup.mode() == Mode::Disarmed {
                break;
            }
        }
        let dh = relay_math::sqrtf(backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1]);
        assert!(dh < 1.0, "RTL must bring it home, not to [4,0]: horiz {dh} m, pos {:?}", backend.pos);
        assert!(
            matches!(sup.mode(), Mode::Land | Mode::Disarmed),
            "RTL should be landing/landed, mode {:?}",
            sup.mode()
        );
    }

    /// A low battery actuates a failsafe (the audit's "no battery failsafe"):
    /// while loitering, the pack sags below threshold → Failsafe → RTL.
    #[test]
    fn low_battery_actuates_failsafe() {
        use relay_fsm::{Event, Mode};
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut backend);
        }
        assert_eq!(sup.mode(), Mode::Loiter);
        backend.battery_v = 13.2; // sag below the 14 V threshold
        for _ in 0..200 {
            sup.step(&mut backend);
        }
        assert!(
            matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed),
            "low battery must trigger a failsafe recovery, mode {:?}",
            sup.mode()
        );
    }

    // ── v1.9 robustness: injected sensor pathologies through the HAL ──────
    //
    // The audit found the estimator was only ever shown against a perfect
    // world. These drive the SAME verified cascade through the HAL with each
    // pathology injected, and assert it HOLDS (bounded tilt / position) or
    // degrades gracefully (recovers after a GPS dropout).

    /// Broadband accelerometer vibration (1.5 m/s² per axis, ≈15 % of g):
    /// altitude-holding, the verified IEKF's gravity update must reject the
    /// zero-mean noise and keep the body near level through the HAL.
    #[test]
    fn holds_through_accelerometer_vibration() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt)
            .with_pathology(Pathology { vibration: 1.5, ..Default::default() });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        let mut peak = 0.0f32;
        for k in 0..15000 {
            core.step(&mut backend);
            if k > 5000 {
                peak = peak.max(backend.tilt());
            }
        }
        assert!(peak < 0.15, "IEKF must reject accel vibration: peak tilt {peak} rad");
        assert!((backend.pos[2] + 2.0).abs() < 0.4, "altitude held under vibration: {}", backend.pos[2]);
    }

    /// A slow gyro bias drift (0.004 rad/s², ≈0.5°/s after 12 s): the IEKF's
    /// gyro-bias state must track it, else the attitude walks off. Compares
    /// against a clean run to show the bias is the only added error.
    #[test]
    fn iekf_tracks_gyro_bias_drift() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt)
            .with_pathology(Pathology { gyro_bias_drift: 0.004, ..Default::default() });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        let mut peak = 0.0f32;
        for k in 0..15000 {
            core.step(&mut backend);
            if k > 8000 {
                peak = peak.max(backend.tilt());
            }
        }
        // injected bias reaches ≈0.004·30 = 0.12 rad/s; without bias estimation
        // the attitude would integrate that into a steady tilt. The IEKF holds.
        assert!(peak < 0.15, "IEKF gyro-bias state must track the drift: peak tilt {peak} rad");
    }

    /// A GPS dropout mid-flight (2 s, steps 6000–7000): while holding position,
    /// the fix vanishes → the IEKF dead-reckons → position drifts, but on the
    /// fix's return the position update RE-CONVERGES. Graceful degradation, the
    /// honest claim (not "no drift" — dead-reckoning on the near-hover accel
    /// model does drift; the test asserts bounded drift + recovery).
    #[test]
    fn survives_gps_dropout_and_recovers() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt).with_pathology(Pathology {
            gps_dropout_start: 6000,
            gps_dropout_len: 1000, // 2 s without a fix
            ..Default::default()
        });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([0.0, 0.0, -2.0]);
        // settle, then fly through the dropout window and well past it
        let mut peak_drift = 0.0f32;
        for k in 0..15000 {
            core.step(&mut backend);
            if (6000..8000).contains(&k) {
                let d = relay_math::sqrtf(backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1]);
                peak_drift = peak_drift.max(d);
            }
        }
        let final_d = relay_math::sqrtf(backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1]);
        assert!(peak_drift < 2.0, "dropout drift must stay bounded: {peak_drift} m");
        assert!(final_d < 0.5, "position must re-converge after the fix returns: {final_d} m");
    }

    /// Magnetometer interference (0.3 of unit field per axis): the heading
    /// update variance must tolerate it — the body holds level + altitude and
    /// does not tumble under the corrupted yaw reference.
    #[test]
    fn tolerates_mag_interference() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt)
            .with_pathology(Pathology { mag_interference: 0.3, ..Default::default() });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        let mut peak = 0.0f32;
        for k in 0..15000 {
            core.step(&mut backend);
            if k > 5000 {
                peak = peak.max(backend.tilt());
            }
        }
        assert!(peak < 0.15, "mag interference must not destabilise attitude: peak tilt {peak} rad");
        assert!((backend.pos[2] + 2.0).abs() < 0.4, "altitude held under mag interference: {}", backend.pos[2]);
    }

    // ── v1.11 — the real-hardware backend SEAM, composed end-to-end ───────

    /// The verified flight core stabilises through the **`HardwareBackend`**
    /// (the real-board seam), not just `SimBackend`. Five mock drivers — the
    /// exact contracts a real board implements (IMU/GNSS/mag/ESC/battery) —
    /// share one simulated plant through a `core::cell::RefCell`; each borrows
    /// it only for its own call, so the motor write steps the physics that the
    /// next IMU read observes. The closed loop runs entirely through the driver
    /// traits, proving the seam carries a real control loop. The ONLY thing
    /// that changes to fly a board is swapping these mocks for silicon drivers.
    #[test]
    fn flight_core_stabilizes_through_the_hardware_seam() {
        use core::cell::RefCell;

        struct MockImu<'a>(&'a RefCell<SimBackend>);
        impl ImuDriver for MockImu<'_> {
            fn read(&mut self) -> ImuSample {
                self.0.borrow_mut().read_imu()
            }
        }
        struct MockPos<'a>(&'a RefCell<SimBackend>);
        impl PositionDriver for MockPos<'_> {
            fn read(&mut self) -> Option<Vec3> {
                self.0.borrow_mut().read_position()
            }
        }
        struct MockMag<'a>(&'a RefCell<SimBackend>);
        impl MagDriver for MockMag<'_> {
            fn read(&mut self) -> Option<Vec3> {
                self.0.borrow_mut().read_mag()
            }
        }
        struct MockEsc<'a>(&'a RefCell<SimBackend>);
        impl MotorDriver for MockEsc<'_> {
            fn write(&mut self, motors: &[f32]) {
                self.0.borrow_mut().write_motors(motors) // steps the shared plant
            }
        }
        struct MockBatt; // uses the default healthy-pack voltage
        impl BatteryDriver for MockBatt {}

        let dt = 0.002f32;
        // start tilted ~23° about x — same plant as the SimBackend HAL test
        let th = 0.4f32;
        let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let plant = RefCell::new(SimBackend::new(r0, dt));

        let mut hw = HardwareBackend {
            imu: MockImu(&plant),
            gnss: MockPos(&plant),
            mag: MockMag(&plant),
            motors: MockEsc(&plant),
            battery: MockBatt,
            dt,
        };
        let mut core = FlightCore::new(0.5, 1.0 / dt);

        let tilt0 = plant.borrow().tilt();
        assert!(tilt0 > 0.35, "should start tilted, {tilt0}");
        for _ in 0..6000 {
            core.step(&mut hw); // the WHOLE loop runs through the driver traits
        }
        // the battery seam answered through the trait too (default healthy pack)
        assert!((hw.read_battery_v() - 16.0).abs() < 1e-6);
        let tilt = plant.borrow().tilt();
        assert!(
            tilt < 0.1,
            "core must recover to level through the HARDWARE seam: {tilt} rad (start {tilt0})"
        );
    }

    // ── v1.16 wind: a translational FORCE disturbance the position loop must
    // reject (the ESO handles torque, not this). The integral term is the fix.

    /// Hold position at the origin under a given wind for N steps; return the
    /// final horizontal distance from home. `ki=Some(0.0)` ⇒ P-D only.
    fn fly_under_wind(wind: Vec3, gust: f32, ki: Option<f32>, steps: usize) -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.wind = wind;
        b.gust_amp = gust;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        if let Some(k) = ki {
            core.set_position_integral_gain(k);
        }
        core.set_position([0.0, 0.0, -2.0]);
        for _ in 0..steps {
            core.step(&mut b);
        }
        relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1])
    }

    /// The P-I-D position loop REJECTS a steady 5 m/s wind — the integral winds
    /// up to counter the constant force, returning near the setpoint.
    #[test]
    fn holds_position_under_steady_wind() {
        let d = fly_under_wind([3.0, 4.0, 0.0], 0.0, None, 30000); // 5 m/s
        assert!(d < 0.6, "P-I-D must reject steady wind: {d} m from home");
    }

    /// FALSIFICATION (the honest motivation): with the integral DISABLED
    /// (ki=0, P-D only) the same wind leaves a much larger steady offset — a
    /// P-D loop cannot reject a constant force. This is why v1.16 adds the
    /// integral, and the integral must cut the offset by more than half.
    #[test]
    fn bare_pd_loop_offsets_under_wind() {
        let d_pd = fly_under_wind([3.0, 4.0, 0.0], 0.0, Some(0.0), 30000);
        let d_pid = fly_under_wind([3.0, 4.0, 0.0], 0.0, None, 30000);
        assert!(d_pd > 1.5, "P-D alone should offset under wind: {d_pd} m");
        assert!(d_pid < d_pd * 0.5, "the integral must cut the offset: PID {d_pid} m vs PD {d_pd} m");
    }

    /// Gusts on top of the steady wind stay bounded — the integral tracks the
    /// mean, the loop rides out the deterministic broadband gust.
    #[test]
    fn rejects_wind_gusts() {
        let d = fly_under_wind([3.0, 0.0, 0.0], 2.0, None, 30000); // 3 m/s + 2 m/s gusts
        assert!(d < 1.0, "P-I-D must keep gusty wind bounded: {d} m from home");
    }

    // ── v1.17 aerodynamic drag (quadratic, ∝ v²) ──────────────────────────

    /// The position loop TRACKS a far setpoint through realistic quadratic
    /// aerodynamic drag — the drag opposes the transit and caps top speed, but
    /// the P-I-D overcomes it and settles on target.
    #[test]
    fn tracks_setpoint_through_aerodynamic_drag() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.drag_quad = 0.05;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([4.0, 0.0, -2.0]);
        for _ in 0..40000 {
            core.step(&mut b);
        }
        let e = [b.pos[0] - 4.0, b.pos[1], b.pos[2] + 2.0];
        let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        assert!(err < 0.6, "must track the setpoint through drag: {err} m");
    }

    /// Quadratic drag DAMPS the vehicle: a horizontally-kicked drone sheds the
    /// kick faster WITH drag than without — the peak excursion is smaller.
    /// Demonstrates the v² damping physics (drag is also a stabilising force).
    #[test]
    fn quadratic_drag_damps_a_kick() {
        let run = |cd: f32| -> f32 {
            let dt = 0.002f32;
            let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            let mut b = SimBackend::new(level, dt);
            b.drag_quad = cd;
            b.vel = [4.0, 0.0, 0.0]; // a 4 m/s horizontal kick
            let mut core = FlightCore::new(0.5, 1.0 / dt);
            core.set_position([0.0, 0.0, -2.0]);
            let mut peak = 0.0f32;
            for _ in 0..15000 {
                core.step(&mut b);
                let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
                peak = peak.max(d);
            }
            peak
        };
        let peak_no_drag = run(0.0);
        let peak_drag = run(0.15);
        assert!(
            peak_drag < peak_no_drag,
            "drag must damp the kick: peak {peak_drag} m vs no-drag {peak_no_drag} m"
        );
    }

    // ── v1.18 IMU bias-instability: a STOCHASTIC random-walk gyro bias + white
    // noise (richer than v1.9's constant ramp). The IEKF must continuously
    // re-track the wandering bias.

    /// The IEKF holds attitude under a random-walk gyro bias-instability +
    /// white gyro noise — it tracks the wandering bias (a moving target) so the
    /// body stays level through a long hover, not just a fixed-ramp bias.
    #[test]
    fn iekf_holds_under_random_walk_gyro_bias() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
            gyro_bias_rw: 0.01, // bias-instability random walk (rad/s/√s)
            gyro_white: 0.01,   // white gyro noise (rad/s)
            ..Default::default()
        });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        let mut peak = 0.0f32;
        for k in 0..20000 {
            core.step(&mut b);
            if k > 6000 {
                peak = peak.max(b.tilt());
            }
        }
        // after ~40 s the injected bias has random-walked to ≈0.01·√40 ≈ 0.06
        // rad/s; the IEKF tracks it, so the steady tilt stays bounded.
        assert!(peak < 0.15, "IEKF must hold under random-walk gyro bias: peak tilt {peak} rad");
    }

    // ── v1.19 GNSS realism: continuous position noise + INTERMITTENT periodic
    // outage (recurring loss), beyond v1.9's single dropout window.

    /// Shared scenario: hold at the origin under 0.3 m noisy GNSS + a recurring
    /// 0.6 s outage every 3 s; `pos_var` is the filter's assumed fix variance.
    /// Returns the peak horizontal drift after settling.
    fn fly_noisy_gps(pos_var: f32) -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
            gps_noise: 0.3,           // 30 cm per-axis fix noise
            gps_dropout_period: 1500, // every 3 s …
            gps_dropout_len: 300,     // … lose the fix for 0.6 s
            ..Default::default()
        });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_pos_var(pos_var);
        core.set_position([0.0, 0.0, -2.0]);
        let mut peak = 0.0f32;
        for k in 0..30000 {
            core.step(&mut b);
            if k > 4000 {
                let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
                peak = peak.max(d);
            }
        }
        peak
    }

    /// Position hold survives noisy + intermittently-dropping GNSS WHEN the
    /// filter's measurement variance matches the receiver: the IEKF smooths the
    /// 0.3 m fix noise and dead-reckons through the recurring outage, bounded.
    #[test]
    fn holds_under_noisy_intermittent_gps() {
        let peak = fly_noisy_gps(0.09); // var = (0.3 m)² — matched to the fix noise
        assert!(peak < 1.5, "variance-matched filter must hold under noisy GNSS: peak {peak} m");
    }

    /// FALSIFICATION: the optimistic default variance (0.01 = 1 cm²) over-trusts
    /// a metre-class fix — the filter injects the noise, the v1.16 integral
    /// winds up on it, and the loop DIVERGES (>10 m). The honest motivation for
    /// matching the measurement variance to the real sensor (set_pos_var).
    #[test]
    fn optimistic_variance_diverges_under_noisy_gps() {
        let peak_optimistic = fly_noisy_gps(0.01); // default 1 cm² — over-trusting
        let peak_matched = fly_noisy_gps(0.09); // matched
        assert!(peak_optimistic > 10.0, "over-trust should diverge: {peak_optimistic} m");
        assert!(peak_matched < peak_optimistic * 0.1, "matched must be far better: {peak_matched} vs {peak_optimistic} m");
    }

    // ── v1.20 barometer fusion: an independent vertical source so altitude
    // survives GPS-vertical loss.

    /// Max altitude error during/after a long GPS outage, with baro on/off.
    fn alt_err_through_gps_loss(baro: bool) -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.baro_enabled = baro;
        b.baro_noise = 0.2;
        b.path.gps_dropout_start = 8000; // GPS drops after settling …
        b.path.gps_dropout_len = 20000; // … for 40 s
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-3.0);
        let mut maxerr = 0.0f32;
        for k in 0..30000 {
            core.step(&mut b);
            if k > 9000 {
                maxerr = maxerr.max((b.pos[2] + 3.0).abs());
            }
        }
        maxerr
    }

    /// Barometer fusion keeps altitude through a 40 s GPS-vertical outage: the
    /// baro (an independent vertical source) anchors the altitude loop, holding
    /// the commanded altitude — and is no worse than the GPS-only dead-reckoning.
    #[test]
    fn baro_holds_altitude_through_gps_loss() {
        let err_baro = alt_err_through_gps_loss(true);
        let err_nobaro = alt_err_through_gps_loss(false);
        assert!(err_baro < 1.5, "baro must hold altitude through GPS loss: {err_baro} m");
        assert!(
            err_baro < err_nobaro,
            "baro must beat GPS-only dead-reckoning: baro {err_baro} vs no-baro {err_nobaro} m"
        );
    }

    // ── v1.21 battery drain: the v1.8 failsafe fires on a REAL endurance limit
    // (a depleting pack whose voltage sags under load), not a set value.

    /// A draining battery actuates the endurance failsafe: the supervisor arms,
    /// takes off and loiters while the pack depletes; the terminal voltage sags
    /// (open-circuit drop + load sag) below the threshold and the supervisor
    /// commands a recovery — fired by genuine energy use, not a set voltage.
    #[test]
    fn battery_drain_actuates_endurance_failsafe() {
        use relay_fsm::Mode;
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut backend = SimBackend::new(level, dt);
        backend.battery_drain = true; // a real depleting pack (starts full)
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.command(relay_fsm::Event::Arm, true, true);
        sup.command(relay_fsm::Event::RequestTakeoff, true, true);
        let mut min_v = 99.0f32;
        let mut fired = false;
        for _ in 0..30000 {
            sup.step(&mut backend);
            min_v = min_v.min(backend.battery_v);
            if matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed) {
                fired = true;
                break;
            }
        }
        assert!(fired, "draining battery must actuate the failsafe");
        assert!(min_v < 14.0, "voltage must have SAGGED below threshold from drain, not set: {min_v} V");
        assert!(
            backend.battery_charge < 0.9,
            "charge must have genuinely depleted: {}",
            backend.battery_charge
        );
    }

    // ── v1.22 air-density thrust lapse: thrust falls with altitude; the
    // altitude integral compensates, and beyond the margin a service ceiling.

    /// Final altitude (m up) after climbing to a target under a thrust lapse.
    fn final_altitude_under_lapse(target_up_m: f32, lapse: f32, ki_alt: Option<f32>) -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.thrust_lapse = lapse;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        if let Some(k) = ki_alt {
            core.set_altitude_integral_gain(k);
        }
        core.set_altitude(-target_up_m);
        for _ in 0..40000 {
            core.step(&mut b);
        }
        -b.pos[2]
    }

    /// The altitude integral rejects the thrust lapse: the vehicle reaches and
    /// holds 20 m even though air-density loss reduces thrust there.
    #[test]
    fn holds_altitude_under_thrust_lapse() {
        let alt = final_altitude_under_lapse(20.0, 0.01, Some(0.02)); // integral ON
        assert!((alt - 20.0).abs() < 1.0, "must hold 20 m despite thrust lapse: {alt} m");
    }

    /// FALSIFICATION: with the altitude integral DISABLED (P-D only) the thrust
    /// lapse leaves the vehicle sagging below the target (a steady thrust
    /// deficit the P-D loop cannot reject); the integral closes the gap.
    #[test]
    fn bare_altitude_pd_sags_under_lapse() {
        let alt_pd = final_altitude_under_lapse(20.0, 0.01, Some(0.0)); // integral OFF
        let alt_pid = final_altitude_under_lapse(20.0, 0.01, Some(0.02)); // integral ON
        assert!(alt_pd < 19.0, "P-D alone should sag below target under lapse: {alt_pd} m");
        assert!(
            (alt_pid - 20.0).abs() < 1.0 && alt_pid > alt_pd,
            "the integral must close the gap: pid {alt_pid} vs pd {alt_pd} m"
        );
    }

    /// The thrust lapse imposes a SERVICE CEILING: commanded to 80 m, the
    /// vehicle cannot climb past where full thrust just hovers (~50 m), even
    /// with the integral — an honest physical limit, not a control failure.
    #[test]
    fn thrust_lapse_imposes_service_ceiling() {
        let alt = final_altitude_under_lapse(80.0, 0.01, Some(0.02)); // integral ON, still ceiling
        assert!(
            (35.0..65.0).contains(&alt),
            "thrust lapse must cap altitude well below the 80 m command: {alt} m"
        );
    }

    // ── v1.23 motor dynamics: first-order spin-up lag — the actuator lag the
    // ADRC ESO is designed to absorb (a robustness confirmation).

    /// The ADRC ESO ABSORBS realistic motor lag: with a 40 ms first-order motor
    /// time constant (the rotor speed lags the command), the verified inner loop
    /// still recovers a tilted body to level — exactly the actuator lag the ESO
    /// was built for (v0.25).
    #[test]
    fn adrc_absorbs_motor_lag() {
        let dt = 0.002f32;
        let th = 0.4f32;
        let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let mut b = SimBackend::new(r0, dt);
        b.motor_tau = 0.04; // 40 ms motor lag — significant
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        for _ in 0..8000 {
            core.step(&mut b);
        }
        assert!(
            b.tilt() < 0.1,
            "ADRC ESO must absorb motor lag and recover to level: {} rad",
            b.tilt()
        );
    }

    /// Position hold stays bounded under motor lag — the lag the v0.25 work
    /// found could destabilise a naive loop is absorbed by the ESO; the vehicle
    /// holds its setpoint.
    #[test]
    fn position_hold_stable_under_motor_lag() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.motor_tau = 0.03; // 30 ms motor lag
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([1.5, -1.0, -2.0]);
        for _ in 0..30000 {
            core.step(&mut b);
        }
        let e = [b.pos[0] - 1.5, b.pos[1] + 1.0, b.pos[2] + 2.0];
        let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        assert!(err < 0.6, "position hold must stay bounded under motor lag: {err} m");
    }
}
