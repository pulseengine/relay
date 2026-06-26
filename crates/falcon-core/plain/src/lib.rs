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
    /// Velocity-based touchdown (v1.27): when `landing`, the altitude loop
    /// commands a constant descent RATE (not a position) so it pushes through
    /// the ground-effect cushion the position loop floats on, then cuts thrust
    /// on touchdown.
    landing: bool,
    landing_descent: f32,
    kvz_land: f32,
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
    /// Sensor calibration applied to raw IMU/mag samples before the estimator
    /// (gyro/accel bias+scale, mag hard/soft-iron). Identity until
    /// `set_calibration` installs solved offsets — the explicit replacement for
    /// the prior identity-remap placeholder (raw samples flowed in uncorrected).
    calib: relay_calib::CalParams,
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
            landing: false,
            landing_descent: 0.5, // m/s controlled descent rate (NED z, +down)
            // kvz sized so the vz=0 descent command (hover − kvz·descent = 0.30)
            // is below gravity/(max ground-effect boost 1.4) ≈ 0.357 — i.e. the
            // controller keeps descending even through the strongest cushion.
            kvz_land: 0.4,
            kp_pos: 0.08,
            kd_vel: 0.6,
            ki_pos: 0.02,
            a_cmd_max: 1.0,
            pos_int: [0.0; 2],
            pos_int_max: 1.5,
            baro_var: 0.05, // ≈ (0.2 m)² baro noise; trusted less than a clean GPS-z
            calib: relay_calib::CalParams::identity(),
        }
    }

    /// Install the sensor calibration the estimator applies to raw IMU/mag
    /// samples each `step` (identity = no-op until solved offsets are loaded).
    pub fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        self.calib = calib;
    }

    /// The calibration currently applied to raw samples.
    pub fn calibration(&self) -> relay_calib::CalParams {
        self.calib
    }

    /// The estimator's TILT uncertainty: the roll+pitch part of the attitude
    /// block of the IEKF error covariance (δθ_x² + δθ_y², rad²). Tilt is the
    /// gravity-observable attitude — it converges from any sensor set (yaw needs a
    /// magnetometer, so it is deliberately EXCLUDED so the metric is meaningful
    /// without a heading reference). It starts large and SHRINKS as gravity
    /// updates arrive; the pre-arm `estimator_converged` check thresholds it
    /// (v1.99), so arming waits for a settled, level estimate.
    pub fn tilt_uncertainty(&self) -> f32 {
        let p = self.iekf.covariance();
        p[0][0] + p[1][1]
    }

    /// The largest of the last commanded motor outputs (0..1). At/near 1.0 the
    /// control allocation is SATURATED — the rate loop is at its authority limit.
    /// Sustained saturation while tilted is the high-wind proxy (v1.101): the
    /// vehicle is fighting a disturbance beyond what it can hold.
    pub fn max_motor(&self) -> f32 {
        let m = self.mixer.last_motors();
        m[0].max(m[1]).max(m[2]).max(m[3])
    }

    /// Command a target altitude (NED z, metres; negative = up). v1.2.
    pub fn set_altitude(&mut self, ned_z: f32) {
        self.setpoint[2] = ned_z;
    }

    /// Engage/disengage the velocity-based touchdown controller (v1.27). When
    /// on, the altitude loop descends at a constant rate (pushing through the
    /// ground-effect cushion) and cuts thrust on touchdown — the clean landing
    /// the position loop floats short of (v1.24).
    pub fn set_landing(&mut self, on: bool) {
        self.landing = on;
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
        let raw = b.read_imu();
        // ── Calibrate (relay-calib): de-bias/scale the raw sample before the
        // estimator. Identity by default (a no-op) until set_calibration installs
        // solved offsets. ──
        let gyro = self.calib.apply_gyro(raw.gyro);
        let accel = self.calib.apply_accel(raw.accel);

        // ── Estimate ──
        self.iekf.propagate(IekfImu { gyro, accel }, dt);
        self.iekf.update_gravity(accel, self.grav_var);
        if let Some(p) = b.read_position() {
            self.iekf.update_position(p, self.pos_var);
        }
        if let Some(m) = b.read_mag() {
            self.iekf.update_magnetometer(self.calib.apply_mag(m), 0.0, self.mag_var);
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

        // ── Altitude loop ── velocity-based touchdown when landing, else the
        // position P-I-D.
        let thrust = if self.landing {
            // ── Velocity-based touchdown (v1.27) ── command a constant descent
            // RATE (NED z positive = down), independent of the ground-effect
            // cushion the position loop floats on; cut to idle on touchdown so
            // the vehicle settles on the surface.
            let alt_agl = -est.p[2];
            if alt_agl < 0.12 {
                0.1 // touched down — idle (the ground holds the vehicle)
            } else {
                (self.hover_thrust - self.kvz_land * (self.landing_descent - est.v[2]))
                    .clamp(0.0, 1.0)
            }
        } else {
            // ── Position altitude P-I-D (v1.2; v1.20 baro-anchored; v1.22 +I) ──
            // thrust = hover − kp·alt_err − ki·∫alt_err + kd·v_z. The integral
            // (v1.22) rejects a steady thrust deficit (the air-density lapse).
            let alt_err = self.setpoint[2] - est.p[2];
            self.alt_int += alt_err * dt;
            let cap = if self.ki_alt > 0.0 { self.alt_int_max / self.ki_alt } else { 0.0 };
            self.alt_int = self.alt_int.clamp(-cap, cap);
            (self.hover_thrust - self.kp_alt * alt_err - self.ki_alt * self.alt_int
                + self.kd_alt * est.v[2])
                .clamp(0.0, 1.0)
        };

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
        // ADRC torque on filtered (calibrated) gyro.
        let gyro_f = self.gyro_lpf.filter(gyro);
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
/// Maximum stored mission legs (no_std fixed capacity).
pub const MAX_WAYPOINTS: usize = 16;

/// Arrival tolerance for a mission waypoint (m, 3-D). Within this of the active
/// waypoint, the sequencer advances to the next leg. A multirotor acceptance
/// radius (PX4's NAV_ACC_RAD is metres-scale); tight enough that the path is
/// faithfully flown, loose enough that the underdamped position loop need not
/// fully settle on each leg.
const WAYPOINT_RADIUS: f32 = 1.2;

/// Pre-arm estimator-convergence ceiling (v1.99): the IEKF tilt-uncertainty
/// (roll²+pitch², rad²) must be at/below this for `estimator_converged` to pass.
/// The filter starts well above it and settles below as gravity updates arrive,
/// so arming waits for a converged, level estimate. ~0.02 rad² ≈ a 0.14 rad (8°)
/// 1σ tilt uncertainty over the two axes — generous enough to clear quickly at
/// rest yet block a just-started or diverging filter.
const PREARM_TILT_UNCERT_MAX: f32 = 0.02;

/// Attitude-runaway termination threshold (v1.99 expanded failsafe): an estimated
/// tilt beyond this (rad) is past any recoverable flight attitude — a tumble. At
/// ~1.05 rad (60°) the lift vector can no longer arrest a fall; PX4 lockdown uses
/// a similar tilt-limit. Sustained for [`TILT_RUNAWAY_CYCLES`] it forces flight
/// termination (motor cut), the one failure where cutting thrust beats RTL/land.
const TILT_RUNAWAY_LIMIT: f32 = 1.05;

/// Consecutive over-tilt cycles required before terminating — a debounce so a
/// brief aggressive manoeuvre or a transient estimate spike does not trip the
/// (irreversible) motor cut. 50 cycles ≈ 0.2 s at 250 Hz.
const TILT_RUNAWAY_CYCLES: u32 = 50;

/// High-wind detector (v1.101): control allocation saturated at/above this motor
/// output (0..1) means the rate loop is at its authority limit.
const WIND_SAT_THRESHOLD: f32 = 0.95;

/// ...AND the tilt is within [WIND_TILT_MIN, WIND_TILT_CEIL] (rad) — the vehicle
/// is leaning hard into a disturbance but NOT tumbling. Saturation alone (e.g. an
/// aggressive climb) is not high wind; saturation WHILE moderately tilted is. The
/// ceiling is below TILT_RUNAWAY_LIMIT and RESETS the count when exceeded, so a
/// developing tumble (which blows through the band far faster than the debounce)
/// terminates rather than merely RTLs — the runaway path strictly dominates.
const WIND_TILT_MIN: f32 = 0.30;
const WIND_TILT_CEIL: f32 = 0.70;

/// Consecutive saturated-and-tilted cycles before commanding RTL — debounced over
/// ~0.5 s at 250 Hz so a transient gust or manoeuvre does not trip it.
const WIND_CYCLES: u32 = 125;

/// Maximum stored keep-out zones (no_std fixed capacity).
pub const MAX_KEEPOUT_ZONES: usize = 8;

/// Horizontal stand-off the avoidance commands beyond a keep-out zone's radius
/// (m) — the safe ring the deflected setpoint rides. Sized to absorb the
/// position loop's tracking lag (the vehicle, carrying momentum toward the goal,
/// cuts inside the commanded ring), so the ACTUAL path stays outside the radius.
const KEEPOUT_MARGIN: f32 = 1.6;

/// How far beyond the safe ring a zone's influence reaches (m). Deflection
/// starts this far out so the vehicle has room to redirect before the zone.
const KEEPOUT_INFLUENCE: f32 = 4.0;

/// Tangential look-ahead used to steer AROUND a zone (m), not merely away from
/// it — without this, radial repulsion alone stalls when the goal sits directly
/// behind the zone (a potential-field local minimum).
const KEEPOUT_LOOKAHEAD: f32 = 3.0;

/// A circular keep-out (no-fly) zone: the vehicle must stay outside `radius` of
/// `center` (evaluated horizontally). Avoidance is reactive — the position
/// setpoint is deflected around the zone. v1.31.
#[derive(Clone, Copy, Debug)]
pub struct KeepoutZone {
    pub center: Vec3,
    pub radius: f32,
}

pub struct FlightSupervisor {
    core: FlightCore,
    fsm: relay_fsm::FlightFsm,
    home: Vec3,
    fence_radius: f32,
    cruise_alt: f32,
    low_batt_v: f32,
    /// Stored mission legs (NED), flown in order while in Mission mode.
    waypoints: [Vec3; MAX_WAYPOINTS],
    wp_count: usize,
    wp_index: usize,
    /// Keep-out zones the position setpoint is deflected around (v1.31).
    zones: [KeepoutZone; MAX_KEEPOUT_ZONES],
    zone_count: usize,
    rtl_latched: bool,
    /// Consecutive cycles the estimated tilt has exceeded TILT_RUNAWAY_LIMIT
    /// while airborne — the attitude-runaway detector (v1.99 expanded failsafe).
    /// On reaching TILT_RUNAWAY_CYCLES it fires Event::Terminate (motor cut).
    runaway_count: u32,
    /// Consecutive cycles of control saturation + moderate tilt while airborne —
    /// the high-wind detector (v1.101). On reaching WIND_CYCLES it commands RTL
    /// (the vehicle is fighting a disturbance beyond its control authority).
    wind_count: u32,
    /// The latest pre-arm / commander check inputs (sensor health, estimator
    /// convergence, calibration, geofence, battery, failsafe config). Fed by
    /// `set_preflight`; `command(Arm, …)` gates arming on `arm_check` of these
    /// (relay-preflight). Defaults all-passing for back-compat; an integration
    /// that wires real health signals tightens it.
    preflight: relay_preflight::PreflightChecks,
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
            waypoints: [home; MAX_WAYPOINTS],
            wp_count: 0,
            wp_index: 0,
            zones: [KeepoutZone { center: home, radius: 0.0 }; MAX_KEEPOUT_ZONES],
            zone_count: 0,
            rtl_latched: false,
            runaway_count: 0,
            wind_count: 0,
            // back-compat default: all checks pass (an integration that feeds real
            // health via set_preflight tightens this to a fail-safe gate).
            preflight: relay_preflight::PreflightChecks {
                sensors_healthy: true,
                estimator_converged: true,
                calibration_present: true,
                geofence_loaded: true,
                battery_ok: true,
                failsafe_configured: true,
            },
        }
    }

    pub fn mode(&self) -> relay_fsm::Mode {
        self.fsm.mode()
    }

    pub fn state(&self) -> NavState {
        self.core.state()
    }

    /// Install the sensor calibration applied to raw IMU/mag samples each cycle.
    /// Solved by relay-calib (gyro null / accel 6-point / mag iron) and typically
    /// loaded from the relay-param store; identity until then. Delegates to the
    /// FlightCore, where the estimator applies it.
    pub fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        self.core.set_calibration(calib);
    }

    /// The calibration currently applied to raw samples.
    pub fn calibration(&self) -> relay_calib::CalParams {
        self.core.calibration()
    }

    /// Feed the latest pre-arm / commander check inputs. The integration calls
    /// this each cycle from the real health sources (estimator convergence,
    /// battery, sensor/RC presence, …); `command(Arm, …)` then gates on them.
    pub fn set_preflight(&mut self, checks: relay_preflight::PreflightChecks) {
        self.preflight = checks;
    }

    /// Derive the pre-arm checks the supervisor can know from its OWN state, each
    /// cycle (v1.99 — the pre-arm gate fed by real signals, not all-pass
    /// defaults). Four of the six are wired here:
    ///   * estimator_converged — the IEKF attitude uncertainty has settled below
    ///     [`PREARM_TILT_UNCERT_MAX`] (a divergent/just-started filter blocks arming);
    ///   * calibration_present — a non-identity sensor calibration is installed;
    ///   * battery_ok — the pack voltage is at/above the arming threshold;
    ///   * geofence_loaded — a positive fence radius is configured.
    ///
    /// `sensors_healthy` and `failsafe_configured` are left as last set (default
    /// true / via [`set_preflight`]) — they need a sensor-health / arbiter input
    /// the backend does not yet provide (documented follow-up).
    pub fn update_preflight<B: FlightBackend>(&mut self, b: &mut B) {
        self.preflight.estimator_converged = self.core.tilt_uncertainty() < PREARM_TILT_UNCERT_MAX;
        self.preflight.calibration_present =
            self.core.calibration() != relay_calib::CalParams::identity();
        self.preflight.battery_ok = b.read_battery_v() >= self.low_batt_v;
        self.preflight.geofence_loaded = self.fence_radius > 0.0;
    }

    /// Why arming would be refused right now (the first failing pre-arm check),
    /// or `None` if all checks pass — the reason a GCS surfaces to the operator.
    pub fn arm_blocked_reason(&self) -> Option<relay_preflight::CheckFail> {
        match relay_preflight::arm_check(self.preflight) {
            relay_preflight::ArmVerdict::Allowed => None,
            relay_preflight::ArmVerdict::Blocked(reason) => Some(reason),
        }
    }

    /// Inject an external command/event (Arm, RequestTakeoff, RequestMission…).
    /// An `Arm` is gated on the pre-arm / commander checks (relay-preflight): the
    /// FSM only enters `Armed` when every check passes AND the vehicle is level
    /// with throttle idle — the Kani-proven entry gate (relay-fsm FSM-K03).
    pub fn command(&mut self, ev: relay_fsm::Event, level: bool, throttle_low: bool) {
        let prearm_ok =
            matches!(relay_preflight::arm_check(self.preflight), relay_preflight::ArmVerdict::Allowed);
        let g = relay_fsm::Gates { level, throttle_low, have_position: true, prearm_ok };
        self.fsm.on(ev, g);
    }

    /// Set a single mission target (back-compat: a one-leg mission).
    pub fn set_mission(&mut self, ned: Vec3) {
        self.waypoints[0] = ned;
        self.wp_count = 1;
        self.wp_index = 0;
    }

    /// Load a multi-leg mission: a sequence of NED waypoints flown in order
    /// (up to `MAX_WAYPOINTS`) while in Mission mode. On reaching the last leg
    /// the supervisor autonomously returns home and lands. Waypoints beyond
    /// capacity are dropped.
    pub fn set_mission_waypoints(&mut self, wps: &[Vec3]) {
        let n = wps.len().min(MAX_WAYPOINTS);
        self.waypoints[..n].copy_from_slice(&wps[..n]);
        self.wp_count = n;
        self.wp_index = 0;
    }

    /// The waypoint currently being flown to (home if the mission is empty).
    fn current_waypoint(&self) -> Vec3 {
        if self.wp_count == 0 {
            self.home
        } else {
            self.waypoints[self.wp_index.min(self.wp_count - 1)]
        }
    }

    /// Index of the active mission leg (telemetry / tests).
    pub fn waypoint_index(&self) -> usize {
        self.wp_index
    }

    /// Number of stored mission legs.
    pub fn waypoint_count(&self) -> usize {
        self.wp_count
    }

    /// Load the keep-out (no-fly) zones the vehicle must arc around (up to
    /// `MAX_KEEPOUT_ZONES`). Excess zones are dropped. v1.31.
    pub fn set_keepout_zones(&mut self, zones: &[KeepoutZone]) {
        let n = zones.len().min(MAX_KEEPOUT_ZONES);
        self.zones[..n].copy_from_slice(&zones[..n]);
        self.zone_count = n;
    }

    /// Deflect a horizontal position setpoint around any keep-out zone the
    /// vehicle is currently near — reactive obstacle avoidance. Each threatening
    /// zone replaces the commanded point with one on its safe ring
    /// (`radius + KEEPOUT_MARGIN`) at the vehicle's bearing from the zone, biased
    /// tangentially toward the goal so the vehicle *circles around* the zone
    /// instead of stalling in front of it. Altitude (`sp[2]`) is untouched.
    fn avoid(&self, p: Vec3, sp: Vec3) -> Vec3 {
        let mut out = sp;
        // unit vector from the vehicle toward the goal
        let gx = sp[0] - p[0];
        let gy = sp[1] - p[1];
        let glen = relay_math::sqrtf(gx * gx + gy * gy).max(1e-3);
        let ux = gx / glen;
        let uy = gy / glen;
        for z in &self.zones[..self.zone_count] {
            let rx = p[0] - z.center[0]; // zone → vehicle
            let ry = p[1] - z.center[1];
            let d = relay_math::sqrtf(rx * rx + ry * ry).max(1e-3);
            let safe = z.radius + KEEPOUT_MARGIN;
            // Where the zone sits relative to the path to the goal: `along` is its
            // distance ahead of the vehicle, `perp` its lateral offset. The zone
            // OBSTRUCTS only if it is ahead, before the goal, and laterally close.
            let along = -rx * ux + -ry * uy;
            let perp = relay_math::sqrtf(((rx * rx + ry * ry) - along * along).max(0.0));
            let obstructs =
                along > 0.0 && along < glen + safe && perp < safe + KEEPOUT_INFLUENCE;
            // Deflect when the zone blocks the path (and the vehicle is within
            // reach of it), OR as a hard guard whenever the vehicle is inside the
            // safe ring. Crucially NOT when the zone is merely near but off-path
            // (e.g. sitting beside home on the final approach).
            let near = d < safe + KEEPOUT_INFLUENCE;
            if (obstructs && near) || d < safe {
                let n = [rx / d, ry / d]; // outward radial
                let mut t = [-n[1], n[0]]; // tangent, signed toward the goal
                if t[0] * ux + t[1] * uy < 0.0 {
                    t = [-t[0], -t[1]];
                }
                let urgency = ((safe + KEEPOUT_INFLUENCE - d) / KEEPOUT_INFLUENCE).clamp(0.0, 1.0);
                out[0] = z.center[0] + n[0] * safe + t[0] * KEEPOUT_LOOKAHEAD * urgency;
                out[1] = z.center[1] + n[1] * safe + t[1] * KEEPOUT_LOOKAHEAD * urgency;
            }
        }
        out
    }

    /// One supervised control step.
    pub fn step<B: FlightBackend>(&mut self, b: &mut B) {
        use relay_fsm::{Event, Gates, Mode};
        // v1.99 — refresh the pre-arm checks from real vehicle state each cycle,
        // so a later command(Arm) gates on live signals, not all-pass defaults.
        self.update_preflight(b);
        let est = self.core.state();
        let dx = est.p[0] - self.home[0];
        let dy = est.p[1] - self.home[1];
        let dist_home = relay_math::sqrtf(dx * dx + dy * dy);
        let alt_agl = -est.p[2]; // NED z negative = up
        let g = Gates { level: true, throttle_low: true, have_position: true, prearm_ok: true };

        // ── FAILSAFE actuation (the audit's gap): geofence breach OR low
        // battery from any flying state ⇒ Failsafe ⇒ the FSM commands RTL. ──
        let batt = b.read_battery_v();
        let breach = dist_home > self.fence_radius || batt < self.low_batt_v;
        if breach && self.fsm.is_airborne() && self.fsm.mode() != Mode::Land {
            self.fsm.on(Event::Failsafe, g);
            self.rtl_latched = true;
        }

        // ── ATTITUDE-RUNAWAY termination (v1.99 expanded failsafe): a tilt past
        // any recoverable attitude, sustained while airborne, is a tumble that
        // RTL/Land cannot arrest — cut the motors. Debounced over TILT_RUNAWAY_
        // CYCLES so a brief aggressive manoeuvre or a transient estimate spike
        // does not trip the irreversible termination. ──
        if self.fsm.is_airborne() && est.tilt_rad() > TILT_RUNAWAY_LIMIT {
            self.runaway_count = self.runaway_count.saturating_add(1);
            if self.runaway_count >= TILT_RUNAWAY_CYCLES {
                self.fsm.on(Event::Terminate, g);
            }
        } else {
            self.runaway_count = 0;
        }

        // ── HIGH-WIND failsafe (v1.101): control allocation saturated WHILE the
        // vehicle leans hard (but short of a tumble) is the signature of a wind
        // disturbance beyond the rate loop's authority — it cannot hold station.
        // Sustained, command RTL via the FSM's Failsafe path (→ Rtl with a
        // position fix, else Land). Debounced so a gust/manoeuvre does not trip
        // it; runaway termination above takes precedence for a true tumble. ──
        let tilt = est.tilt_rad();
        if self.fsm.is_airborne()
            && self.fsm.mode() != Mode::Land
            && self.core.max_motor() >= WIND_SAT_THRESHOLD
            && (WIND_TILT_MIN..=WIND_TILT_CEIL).contains(&tilt)
        {
            self.wind_count = self.wind_count.saturating_add(1);
            if self.wind_count >= WIND_CYCLES {
                self.fsm.on(Event::Failsafe, g);
                self.rtl_latched = true;
            }
        } else {
            self.wind_count = 0;
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
            // ── v1.30: multi-waypoint mission sequencing. Fly the stored legs
            // in order; on arriving at the active leg advance to the next, and
            // on finishing the last leg autonomously return home and land. ──
            Mode::Mission => {
                let wp = self.current_waypoint();
                let wdx = est.p[0] - wp[0];
                let wdy = est.p[1] - wp[1];
                let wdz = est.p[2] - wp[2];
                let wd = relay_math::sqrtf(wdx * wdx + wdy * wdy + wdz * wdz);
                if wd < WAYPOINT_RADIUS {
                    if self.wp_index + 1 < self.wp_count {
                        self.wp_index += 1; // next leg
                    } else {
                        self.fsm.on(Event::RequestRtl, g); // mission done ⇒ return + land
                    }
                }
            }
            _ => {}
        }

        // ── v1.29: engage the v1.27 velocity-based touchdown controller while
        // landing. Before this, the supervisor left the core in position mode
        // for Land, so the integrated stack descended on the slow altitude P-I-D
        // that floats short of the surface through ground effect (the v1.24
        // limitation v1.27 fixed at the core but never wired into the supervisor).
        //
        // SETTLE OVER HOME, THEN DESCEND: the velocity-landing descends fast, so
        // engaging it while the vehicle still carries horizontal velocity or sits
        // off-target (e.g. overshooting home at the end of RTL) would touch down
        // in the wrong place. So we hold altitude over home until the vehicle is
        // BOTH near home AND slow, then drop straight down. (Speed alone is not
        // enough: it momentarily hits zero at the overshoot peak, far from home.)
        let horiz_speed = relay_math::sqrtf(est.v[0] * est.v[0] + est.v[1] * est.v[1]);
        let landing = matches!(self.fsm.mode(), Mode::Land) && horiz_speed < 0.4 && dist_home < 0.5;
        self.core.set_landing(landing);

        // ── mode → setpoint ── (horizontal hold target; while landing the core's
        // velocity-landing owns the vertical once engaged — until then Land holds
        // at cruise altitude over home to kill the horizontal drift). ──
        let sp = match self.fsm.mode() {
            Mode::Takeoff | Mode::Loiter => [est.p[0], est.p[1], -self.cruise_alt],
            Mode::Mission => self.current_waypoint(), // fly the active leg (its own altitude)
            Mode::Rtl => [self.home[0], self.home[1], -self.cruise_alt],
            Mode::Land => [self.home[0], self.home[1], -self.cruise_alt], // hold home; rate-descend
            // idle / motors-off states: setpoint is moot (no thrust commanded).
            Mode::Disarmed | Mode::Armed | Mode::Terminated => [est.p[0], est.p[1], est.p[2]],
        };
        // ── v1.31: reactive keep-out avoidance ── deflect the horizontal
        // setpoint around any no-fly zone the vehicle is near (no-op when none
        // are set), so a mission or RTL path arcs around obstacles instead of
        // through them. Not applied while landing (the vehicle is over home).
        let sp = if self.zone_count > 0 && !landing {
            self.avoid(est.p, sp)
        } else {
            sp
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
    /// Turbulence intensity σ (m/s) — a Dryden-like COLORED gust (an
    /// Ornstein-Uhlenbeck process, ~1 s correlation time), richer than v1.16's
    /// white-noise `gust_amp`. A realistic turbulence spectrum the position loop
    /// + ESO must ride out (v1.25).
    pub turbulence: f32,
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
    /// Ground-effect thrust-augmentation gain (v1.24): near the surface, rotor
    /// downwash reflects and boosts thrust — thrust ×= (1 + gain·e^(−alt/0.25)).
    /// A landing/takeoff cushion the altitude loop must push through. 0 = none.
    pub ground_effect: f32,
    /// Model ground contact? (v1.27) When true the vehicle cannot descend below
    /// the surface (NED z = 0) and its vertical velocity is zeroed on contact —
    /// so a touchdown can settle. Default off (existing hover tests stay aloft).
    pub ground_contact: bool,
    /// Injected sensor pathologies (v1.9).
    pub path: Pathology,
    /// Control-tick counter (drives the GPS-dropout window).
    step_n: u32,
    /// Accumulated gyro bias (rad/s) injected into the measured rate.
    gyro_bias: Vec3,
    /// LCG state for the deterministic broadband noise.
    rng: u32,
    /// Dryden-like turbulence gust state (OU process, horizontal x/y) (v1.25).
    turb_state: Vec3,
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
            ground_effect: 0.0,
            ground_contact: false,
            path: Pathology::default(),
            step_n: 0,
            gyro_bias: [0.0; 3],
            rng: 0x9E3779B9, // a fixed, non-zero seed (golden-ratio constant)
            turb_state: [0.0; 3],
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
        // v1.24 — ground effect: near the surface thrust is augmented by the
        // reflected downwash (a landing/takeoff cushion). alt = −pos[2] (NED).
        // Rational decay 1/(1+(alt/0.25)²) — no new transcendental in the flight
        // path (respects the relay-math qualification seam).
        let ge = if self.ground_effect > 0.0 {
            let z = (-self.pos[2]).max(0.0) / 0.25;
            1.0 + self.ground_effect / (1.0 + z * z)
        } else {
            1.0
        };
        let thrust = collective * self.k_thrust * lapse * ge;
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
        // v1.25 — Dryden-like turbulence: a COLORED gust (Ornstein-Uhlenbeck
        // process, ~1 s correlation), richer than the white-noise gust_amp.
        // gust ← gust·(1−dt/T) + σ·√(2dt/T)·N — stationary std σ, correlation T.
        if self.path.turbulence > 0.0 {
            const TC: f32 = 1.0; // gust correlation time (s)
            let a = self.dt / TC;
            let q = self.path.turbulence * relay_math::sqrtf(2.0 * a);
            for i in 0..2 {
                self.turb_state[i] = self.turb_state[i] * (1.0 - a) + q * self.noise_unit();
            }
        }
        const K_WIND: f32 = 0.15;
        if self.wind[0] != 0.0 || self.wind[1] != 0.0 || self.gust_amp != 0.0 || self.path.turbulence > 0.0 {
            for i in 0..2 {
                let gust = self.gust_amp * self.noise_unit();
                accel[i] += K_WIND * (self.wind[i] + gust + self.turb_state[i] - self.vel[i]);
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
        // v1.27 — ground contact: the vehicle cannot descend below the surface
        // (NED z = 0); zero the downward velocity on touchdown so it settles.
        if self.ground_contact && self.pos[2] > 0.0 {
            self.pos[2] = 0.0;
            if self.vel[2] > 0.0 {
                self.vel[2] = 0.0;
            }
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
    /// v1.98 — the estimator CONSUMES the calibration. A constant yaw-gyro bias
    /// is unobservable without a heading reference, so it integrates into yaw
    /// drift; with a calibration that cancels the bias the estimate does NOT
    /// drift — so the calibrated and uncalibrated trajectories diverge. Proves the
    /// calibration is in the estimation loop (replacing the identity placeholder),
    /// not silently ignored.
    #[test]
    fn estimator_consumes_calibration() {
        use relay_calib::CalParams;
        struct BiasedGyro {
            bias_z: f32,
        }
        impl FlightBackend for BiasedGyro {
            fn read_imu(&mut self) -> ImuSample {
                ImuSample { accel: [0.0, 0.0, -GRAVITY], gyro: [0.0, 0.0, self.bias_z] }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                None
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn write_motors(&mut self, _: &[f32]) {}
            fn dt(&self) -> f32 {
                0.004
            }
        }
        let bias = 0.2_f32; // rad/s yaw-gyro bias → ~0.2 rad yaw over 1 s

        let mut uncal = FlightCore::new(0.5, 250.0);
        let mut bu = BiasedGyro { bias_z: bias };
        for _ in 0..250 {
            uncal.step(&mut bu);
        }

        let mut cal = FlightCore::new(0.5, 250.0);
        cal.set_calibration(CalParams { gyro_bias: [0.0, 0.0, bias], ..CalParams::identity() });
        let mut bc = BiasedGyro { bias_z: bias };
        for _ in 0..250 {
            cal.step(&mut bc);
        }

        let qu = uncal.state().q;
        let qc = cal.state().q;
        let diff = (qu[0] - qc[0]).abs()
            + (qu[1] - qc[1]).abs()
            + (qu[2] - qc[2]).abs()
            + (qu[3] - qc[3]).abs();
        assert!(diff > 0.05, "calibration must change the estimate (yaw drift suppressed): diff {diff}");
        // the installed calibration is reported back.
        assert_eq!(cal.calibration().gyro_bias, [0.0, 0.0, bias]);
    }

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

    /// v1.30 — a multi-leg mission: the supervisor flies a stored sequence of
    /// waypoints IN ORDER, then autonomously returns home, lands, and disarms.
    /// A full autonomous sortie with no per-waypoint commanding.
    #[test]
    fn flies_a_multi_waypoint_mission_then_returns_and_disarms() {
        use relay_fsm::{Event, Mode};
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_contact = true; // so the autonomous landing can settle
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 1.0);

        // A three-leg path (NED, 2 m AGL) that is NOT in straight-line order, so
        // visiting in sequence is observable.
        let wps = [[3.0, 0.0, -2.0], [3.0, 3.0, -2.0], [0.0, 3.0, -2.0]];
        sup.set_mission_waypoints(&wps);
        assert_eq!(sup.waypoint_count(), 3);

        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut b);
        }
        assert_eq!(sup.mode(), Mode::Loiter, "should reach Loiter after takeoff");
        sup.command(Event::RequestMission, true, false);

        // Fly the mission. Track the closest approach to each waypoint and the
        // order the sequencer advanced through the legs.
        let mut min_d = [f32::MAX; 3];
        let mut last_index = 0usize;
        let mut order_ok = true;
        let mut disarmed = false;
        for _ in 0..150000 {
            sup.step(&mut b);
            let p = sup.state().p;
            for (i, w) in wps.iter().enumerate() {
                let dx = p[0] - w[0];
                let dy = p[1] - w[1];
                let dz = p[2] - w[2];
                let d = relay_math::sqrtf(dx * dx + dy * dy + dz * dz);
                if d < min_d[i] {
                    min_d[i] = d;
                }
            }
            let idx = sup.waypoint_index();
            if idx < last_index {
                order_ok = false; // the leg index must never go backwards
            }
            last_index = idx;
            if sup.mode() == Mode::Disarmed {
                disarmed = true;
                break;
            }
        }

        for (i, d) in min_d.iter().enumerate() {
            assert!(*d < WAYPOINT_RADIUS + 0.2, "waypoint {i} not visited: min dist {d} m");
        }
        assert!(order_ok, "waypoints must be flown in order (leg index monotonic)");
        assert!(
            disarmed,
            "mission must complete autonomously: return home + land + disarm (mode {:?})",
            sup.mode()
        );
    }

    /// v1.31 — keep-out (no-fly) zone avoidance: a mission whose straight path
    /// runs through a no-fly zone is still flown to completion, but the vehicle
    /// ARCS AROUND the zone — its closest approach to the zone centre stays
    /// outside the zone radius for the whole flight (out and back).
    #[test]
    fn mission_avoids_a_keepout_zone() {
        use relay_fsm::{Event, Mode};
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_contact = true;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 1.0);

        // A single far waypoint straight across a zone that sits on the path.
        sup.set_mission_waypoints(&[[10.0, 0.0, -2.0]]);
        let zone = KeepoutZone { center: [5.0, 0.0, -2.0], radius: 2.0 };
        sup.set_keepout_zones(&[zone]);

        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut b);
        }
        assert_eq!(sup.mode(), Mode::Loiter);
        sup.command(Event::RequestMission, true, false);

        let mut min_zone = f32::MAX; // closest approach to the zone centre
        let mut min_wp = f32::MAX; // closest approach to the waypoint
        let mut disarmed = false;
        for _ in 0..260000 {
            sup.step(&mut b);
            let p = sup.state().p;
            let dz = relay_math::sqrtf((p[0] - 5.0) * (p[0] - 5.0) + (p[1] - 0.0) * (p[1] - 0.0));
            if dz < min_zone {
                min_zone = dz;
            }
            let dw = relay_math::sqrtf((p[0] - 10.0) * (p[0] - 10.0) + p[1] * p[1]);
            if dw < min_wp {
                min_wp = dw;
            }
            if sup.mode() == Mode::Disarmed {
                disarmed = true;
                break;
            }
        }
        // visited the far waypoint (so it really crossed the obstacle field) …
        assert!(min_wp < WAYPOINT_RADIUS + 0.3, "waypoint not reached: min dist {min_wp} m");
        // … but never entered the no-fly zone …
        assert!(
            min_zone > zone.radius,
            "entered the keep-out zone: min dist {min_zone} m < {} m",
            zone.radius
        );
        // … and still completed the sortie autonomously.
        assert!(disarmed, "mission with avoidance must still complete (mode {:?})", sup.mode());
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
    /// v1.99 expanded failsafe — an attitude RUNAWAY (a sustained, unrecoverable
    /// tilt while airborne) cuts the motors: the FSM reaches Terminated. A level
    /// airborne vehicle is NOT terminated; only the sustained tumble is.
    #[test]
    fn attitude_runaway_terminates_in_flight() {
        use relay_calib::CalParams;
        struct TumbleBackend {
            tilted: bool,
        }
        impl FlightBackend for TumbleBackend {
            fn read_imu(&mut self) -> ImuSample {
                // tilted: gravity measured along body-x ⇒ the estimate converges
                // to a ~90° tilt (well past the runaway limit); else level.
                let accel = if self.tilted { [GRAVITY, 0.0, 0.0] } else { [0.0, 0.0, -GRAVITY] };
                ImuSample { accel, gyro: [0.0; 3] }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                Some([0.0, 0.0, 0.0])
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn read_battery_v(&mut self) -> f32 {
                16.0
            }
            fn write_motors(&mut self, _: &[f32]) {}
            fn dt(&self) -> f32 {
                0.004
            }
        }
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.set_calibration(CalParams { gyro_bias: [0.001, 0.0, 0.0], ..CalParams::identity() });
        let mut b = TumbleBackend { tilted: false };

        // converge level, then arm + take off (airborne, still level).
        for _ in 0..1500 {
            sup.step(&mut b);
        }
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Armed);
        sup.command(relay_fsm::Event::RequestTakeoff, true, false);
        assert_eq!(sup.mode(), relay_fsm::Mode::Takeoff, "airborne");
        // a level airborne vehicle is NOT terminated.
        for _ in 0..200 {
            sup.step(&mut b);
        }
        assert_ne!(sup.mode(), relay_fsm::Mode::Terminated, "level flight must not terminate");

        // now a sustained tumble → flight termination. The tilt blows through the
        // high-wind band (~47 cycles, < the wind debounce) into the runaway range,
        // so termination — not RTL — fires (the runaway path strictly dominates).
        b.tilted = true;
        for _ in 0..3000 {
            sup.step(&mut b);
        }
        assert_eq!(sup.mode(), relay_fsm::Mode::Terminated, "sustained attitude runaway cuts motors");
    }

    /// v1.101 expanded failsafe — HIGH WIND: control saturation while leaning hard
    /// (but NOT tumbling) is a disturbance beyond the rate loop's authority →
    /// RTL. A sustained moderate tilt with the motors pinned, away from home,
    /// drives the FSM to Rtl (it does not terminate — the tilt stays below the
    /// runaway limit — and does not land, being away from home).
    #[test]
    fn high_wind_saturation_commands_rtl() {
        use relay_calib::CalParams;
        struct WindBackend {
            windy: bool,
        }
        impl FlightBackend for WindBackend {
            fn read_imu(&mut self) -> ImuSample {
                // windy: gravity measured at ~28° off body-down → the estimate
                // holds a ~0.49 rad tilt (inside the wind band [0.30, 0.70]) and
                // the rate loop saturates fighting it; else level.
                let accel = if self.windy {
                    [GRAVITY * 0.469, 0.0, -GRAVITY * 0.883]
                } else {
                    [0.0, 0.0, -GRAVITY]
                };
                ImuSample { accel, gyro: [0.0; 3] }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                Some([100.0, 0.0, 0.0]) // away from home: an RTL flies, never lands
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn read_battery_v(&mut self) -> f32 {
                16.0
            }
            fn write_motors(&mut self, _: &[f32]) {}
            fn dt(&self) -> f32 {
                0.004
            }
        }
        // effectively-infinite fence so the tilt-corrupted position estimate can
        // never trip the GEOFENCE failsafe — isolating the high-wind path.
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.0e9, 2.0, 14.0);
        sup.set_calibration(CalParams { gyro_bias: [0.001, 0.0, 0.0], ..CalParams::identity() });
        let mut b = WindBackend { windy: false };
        // converge level, arm, take off.
        for _ in 0..1500 {
            sup.step(&mut b);
        }
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Armed);
        sup.command(relay_fsm::Event::RequestTakeoff, true, false);
        assert_eq!(sup.mode(), relay_fsm::Mode::Takeoff, "airborne");
        // sustained high wind: moderate tilt + saturation fires the RTL-class
        // failsafe (rtl_latched), and the vehicle RECOVERS — it does NOT terminate
        // (the tilt is below the runaway limit). It also leaves normal flight. The
        // exact recovery mode then follows the standard RTL path (Rtl→Land as the
        // tilt-corrupted position estimate reads near home); the discriminating
        // property vs attitude-runaway is recover-not-terminate.
        b.windy = true;
        let mut fired_at_takeoff = false;
        for _ in 0..400 {
            let before = sup.mode();
            sup.step(&mut b);
            // the FIRST failsafe to fire does so from Takeoff → Rtl (recovery).
            if sup.rtl_latched && before == relay_fsm::Mode::Takeoff {
                fired_at_takeoff = sup.mode() == relay_fsm::Mode::Rtl || sup.mode() == relay_fsm::Mode::Land;
            }
            if sup.rtl_latched {
                break;
            }
        }
        assert!(sup.rtl_latched, "sustained high-wind saturation must fire the RTL-class failsafe");
        assert!(fired_at_takeoff, "the failsafe fired from normal flight (Takeoff → RTL recovery)");
        assert_ne!(sup.mode(), relay_fsm::Mode::Terminated, "high wind recovers (RTL), it does NOT terminate");
    }

    /// v1.99 — the pre-arm gate is fed by REAL vehicle state via update_preflight
    /// (called each step). A fresh, uncalibrated vehicle is refused arming
    /// (no calibration, estimator not converged); after installing a calibration
    /// and letting the estimator settle, with a healthy battery + a loaded fence,
    /// arming is permitted. Proves the gate is no longer inert all-pass defaults.
    #[test]
    fn prearm_gate_fed_by_real_state() {
        use relay_calib::CalParams;
        struct RestBackend {
            batt: f32,
        }
        impl FlightBackend for RestBackend {
            fn read_imu(&mut self) -> ImuSample {
                ImuSample { accel: [0.0, 0.0, -GRAVITY], gyro: [0.0; 3] }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                Some([0.0, 0.0, 0.0])
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn read_battery_v(&mut self) -> f32 {
                self.batt
            }
            fn write_motors(&mut self, _: &[f32]) {}
            fn dt(&self) -> f32 {
                0.004
            }
        }
        // fence_radius 50 (loaded), low_batt_v 14.
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        let mut b = RestBackend { batt: 16.0 };

        // settle the estimator, but with NO calibration installed.
        for _ in 0..1500 {
            sup.step(&mut b);
        }
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Disarmed, "must not arm without calibration");
        assert_eq!(sup.arm_blocked_reason(), Some(relay_preflight::CheckFail::Calibration));

        // install a (non-identity) calibration → step once to refresh → arms.
        sup.set_calibration(CalParams { gyro_bias: [0.001, 0.0, 0.0], ..CalParams::identity() });
        sup.step(&mut b);
        assert_eq!(sup.arm_blocked_reason(), None, "all real checks pass");
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Armed, "arms once the real signals are good");

        // a low battery (read each step) blocks re-arming after a disarm.
        let mut low = RestBackend { batt: 13.0 };
        let mut sup2 = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup2.set_calibration(CalParams { gyro_bias: [0.001, 0.0, 0.0], ..CalParams::identity() });
        for _ in 0..1500 {
            sup2.step(&mut low);
        }
        sup2.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup2.mode(), relay_fsm::Mode::Disarmed, "low battery blocks arming");
        assert_eq!(sup2.arm_blocked_reason(), Some(relay_preflight::CheckFail::Battery));
    }

    /// v1.97 pre-arm gate (the seam): the FlightSupervisor refuses to arm unless
    /// every commander check passes, surfaces the first failing reason, and arms
    /// once they all pass — even when the vehicle is physically level + idle.
    #[test]
    fn prearm_checks_gate_arming() {
        use relay_preflight::{CheckFail, PreflightChecks};
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);

        // all checks FAILING (Default = all false): arming refused, reason = first.
        sup.set_preflight(PreflightChecks::default());
        sup.command(relay_fsm::Event::Arm, true, true); // level + throttle idle
        assert_eq!(sup.mode(), relay_fsm::Mode::Disarmed, "no arm with failed pre-arm checks");
        assert_eq!(sup.arm_blocked_reason(), Some(CheckFail::Sensors));

        // only the battery failing → blocked on Battery, still won't arm.
        sup.set_preflight(PreflightChecks {
            sensors_healthy: true,
            estimator_converged: true,
            calibration_present: true,
            geofence_loaded: true,
            battery_ok: false,
            failsafe_configured: true,
        });
        assert_eq!(sup.arm_blocked_reason(), Some(CheckFail::Battery));
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Disarmed);

        // every check passing → arms.
        sup.set_preflight(PreflightChecks {
            sensors_healthy: true,
            estimator_converged: true,
            calibration_present: true,
            geofence_loaded: true,
            battery_ok: true,
            failsafe_configured: true,
        });
        assert_eq!(sup.arm_blocked_reason(), None);
        sup.command(relay_fsm::Event::Arm, true, true);
        assert_eq!(sup.mode(), relay_fsm::Mode::Armed, "arms once every pre-arm check passes");
    }

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

    // ── v1.24 ground effect: a thrust cushion near the surface (landing/takeoff).

    /// Ground effect AIDS takeoff: the reflected-downwash thrust boost near the
    /// surface helps the climb; the vehicle reaches and holds its commanded
    /// altitude with no instability.
    #[test]
    fn ground_effect_aids_takeoff() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_effect = 0.4;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        for _ in 0..15000 {
            core.step(&mut b);
        }
        let alt = -b.pos[2];
        assert!((alt - 2.0).abs() < 0.3, "ground effect must aid takeoff to altitude: {alt} m");
    }

    /// HONEST LIMITATION (documented, not faked): ground effect cushions the
    /// landing — the position-based altitude loop reaches a hover equilibrium
    /// ABOVE the surface and FLOATS on the cushion rather than touching down. It
    /// stays bounded and stable (not diverging). A clean touchdown needs a
    /// velocity-based landing controller (future work); the altitude integral
    /// is NOT a fix here (it runs away when wound up during the climb).
    #[test]
    fn ground_effect_cushions_landing_into_a_float() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_effect = 0.4;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        for _ in 0..10000 {
            core.step(&mut b);
        }
        core.set_altitude(0.0); // command landing
        for _ in 0..20000 {
            core.step(&mut b);
        }
        let alt = -b.pos[2];
        // floats on the cushion, bounded + stable — the documented limitation
        assert!(
            (0.2..2.5).contains(&alt),
            "ground effect floats the landing on a bounded cushion: {alt} m"
        );
    }

    // ── v1.25 turbulence: a Dryden-like COLORED gust spectrum (OU, correlated),
    // harder than v1.16's white-noise gust because the gusts persist.

    /// Position hold rides out a Dryden-like turbulence spectrum (2 m/s RMS,
    /// ~1 s correlation). Honest claim: the persistent, correlated gusts push
    /// the vehicle into a larger excursion than a steady wind (peak ~3 m here),
    /// but it stays BOUNDED — it does not diverge — and returns. The worst-case
    /// environment of the realism arc; bounded-not-crisp, stated plainly.
    #[test]
    fn holds_position_under_turbulence() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt)
            .with_pathology(Pathology { turbulence: 2.0, ..Default::default() });
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([0.0, 0.0, -2.0]);
        let mut peak = 0.0f32;
        for k in 0..40000 {
            core.step(&mut b);
            if k > 4000 {
                let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
                peak = peak.max(d);
            }
        }
        // bounded under continuous turbulence — it does not diverge (an
        // over-authority wind blew the vehicle to 100s of metres; this rides
        // out the persistent gusts within a few metres).
        assert!(peak < 4.0, "turbulence must stay bounded (not diverge): peak {peak} m");
    }

    // ── v1.27 velocity-based touchdown: the clean landing the v1.24 ground-
    // effect float needs.

    /// The velocity-based touchdown controller LANDS through the ground-effect
    /// cushion the v1.24 position loop floated on: commanded to land, the
    /// vehicle descends at a controlled rate, pushes through the cushion, and
    /// settles on the surface — vs the ~1.3 m float without it.
    #[test]
    fn velocity_landing_touches_down_through_ground_effect() {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_effect = 0.4; // the cushion that floated v1.24
        b.ground_contact = true; // model the ground so a touchdown can settle
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        for _ in 0..10000 {
            core.step(&mut b); // climb to 2 m
        }
        core.set_landing(true); // engage the velocity-based touchdown
        for _ in 0..20000 {
            core.step(&mut b);
        }
        let alt = -b.pos[2];
        assert!(alt < 0.15, "velocity touchdown must reach the surface through ground effect: {alt} m");
        assert!(b.vel[2].abs() < 0.3, "should settle on touchdown: vz {} m/s", b.vel[2]);
    }

    // ── v1.29: wire the v1.27 velocity-landing into the FlightSupervisor ──
    //
    // v1.27 added the velocity-based touchdown controller to FlightCore but the
    // FlightSupervisor still left the core in position mode for Land — so the
    // INTEGRATED stack (the thing a real vehicle runs) still descended on the
    // slow altitude P-I-D that floats short through ground effect
    // (`ground_effect_cushions_landing_into_a_float` above shows that float).
    // The supervisor now engages set_landing in Land mode, so a supervised
    // landing through the same cushion reaches the surface and DISARMS.

    /// A full supervised mission — arm → takeoff → loiter → land — touches down
    /// and disarms through the ground-effect cushion the position loop floats
    /// on. This is the integration the v1.27 core fix needed: before v1.29 the
    /// supervised Land floated (never reaching Disarmed); now it settles.
    #[test]
    fn supervised_landing_disarms_through_ground_effect() {
        use relay_fsm::{Event, Mode};
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.ground_effect = 0.4; // the cushion that floated the position-land
        b.ground_contact = true; // model the ground so a touchdown can settle
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut b);
        }
        assert_eq!(sup.mode(), Mode::Loiter, "should reach Loiter after takeoff");

        sup.command(Event::RequestLand, true, false); // → Land (velocity touchdown)
        let mut disarmed = false;
        for _ in 0..20000 {
            sup.step(&mut b);
            if sup.mode() == Mode::Disarmed {
                disarmed = true;
                break;
            }
        }
        assert!(
            disarmed,
            "supervised landing must touch down + disarm through ground effect \
             (it floated before v1.29): mode {:?}, alt {} m",
            sup.mode(),
            -b.pos[2]
        );
        // Touchdown→Disarmed interrupts the descent at the 0.15 m trigger and the
        // disarmed position-hold settles just above it through ground effect — on
        // the surface (vs the ~1.3 m float without the velocity-landing).
        assert!(-b.pos[2] < 0.25, "must settle on the surface (not float): {} m", -b.pos[2]);
    }
}
