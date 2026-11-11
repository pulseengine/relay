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

// RE-EXPORTED, because this crate's PUBLIC API already speaks these types and a
// consumer cannot name them otherwise. `FlightSupervisor::mode()` returns
// `relay_fsm::Mode`, `command()` takes a `relay_fsm::Event`, and `set_check()`
// takes a `relay_preflight::CheckId` — so any caller must already have those
// types, yet without these lines it has to declare a SECOND path dependency on
// the same crates and hope the versions agree. The published wasm component is
// the case that found this: it depends on `falcon-core` alone, deliberately, so
// that what ships is what is verified (#388 / scripts/audit-component-deps.rs).
pub use relay_calib;
pub use relay_fsm;
pub use relay_preflight;

use relay_adrc::{AdrcRate, GyroLpf};
use relay_geo::{GeoAtt, GeoGains, quat_to_rotmat, thrust_axis_ned};
/// The estimator partition's sole output (16×f32), re-exported so a component
/// that carries it across a seam (the `cascade-split` orchestrator, #575) can
/// name the type the supervisor reads without a second `relay-iekf` dependency.
pub use relay_iekf::NavState;
use relay_iekf::{Iekf, Imu as IekfImu, RotorFaultDetector, Vec3};
use relay_mix_quad::{QuadMixer, motors_to_torque_signs};

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
    /// Latest absolute heading (yaw, rad, NED — CW from North), or `None` if no
    /// heading source. A DIRECT yaw reference (a fused compass, a dual-antenna
    /// GNSS heading, or a sim's truth heading) fed straight to the IEKF's
    /// `update_yaw` — the alternative to inferring yaw from the raw
    /// magnetometer field via `read_mag`, for backends that already resolve a
    /// clean heading. Default `None` (v1.113). A backend supplies at most one of
    /// `read_mag`/`read_heading`; both feed the same unobservable yaw state.
    fn read_heading(&mut self) -> Option<f32> {
        None
    }
    /// Write the per-rotor commands ∈ [0,1] to the actuators.
    fn write_motors(&mut self, motors: &[f32]);
    /// Control period (s) for this tick.
    fn dt(&self) -> f32;
    /// Battery voltage (V), or `None` when this backend has no battery sense.
    ///
    /// `None` MEANS ABSENT, NOT HEALTHY. The default used to be a hardcoded
    /// 16.0 V — a healthy 4S pack — so a backend that supplied nothing asserted
    /// the pack was fine, forever. The low-battery failsafe and the BatteryOk
    /// pre-arm row both consume this, so on the published wasm component, in gz
    /// SITL, and on any HardwareBackend whose driver did not override it, both
    /// reported healthy no matter how flat the pack was (#413).
    ///
    /// Every OTHER absent channel on this trait already returns `None` and the
    /// core skips that fusion — `read_battery_i` included. Voltage was the one
    /// that fabricated a plausible number instead.
    fn read_battery_v(&mut self) -> Option<f32> {
        None
    }
    /// Battery discharge current (A) from the power module's current sense
    /// (PM02D), or `None` when no current sensing exists. With current the
    /// supervisor's battery estimate is coulomb-counted and sag-compensated
    /// (BATTERY-P02); without it the estimator runs a FLAGGED voltage-only
    /// fallback with wider margins. Default `None`.
    fn read_battery_i(&mut self) -> Option<f32> {
        None
    }
    /// BOTH GNSS receivers' fixes for this cycle (GNSS-P02, v1.121: the
    /// first vehicle carries 2× M9N). `None` = the backend has no dual-GNSS
    /// concept — the core falls back to the single [`read_position`] lane.
    /// With `Some`, the core's dual-receiver selector (health gating,
    /// inverse-variance blending, one-interval failover, latched divergence
    /// flag) feeds the estimator instead.
    fn read_gnss_dual(
        &mut self,
    ) -> Option<(
        Option<falcon_gnss_ubx::dual::NedFix>,
        Option<falcon_gnss_ubx::dual::NedFix>,
    )> {
        None
    }
    /// Downward rangefinder distance (m), already wire-decoded and
    /// quality-gated at the driver layer (TF02-Pro: relay-flowrange tf02,
    /// RANGEDRV-P01). The core tilt-compensates it, innovation-gates it
    /// against the estimated altitude, and fuses it as a tight vertical
    /// anchor for the landing phase. Default `None`.
    fn read_range(&mut self) -> Option<f32> {
        None
    }
    /// Barometric altitude (NED z, metres; negative = up), or `None` if no
    /// barometer. An INDEPENDENT vertical source the core fuses so altitude
    /// survives GPS-vertical loss (v1.20). Default `None`.
    fn read_baro(&mut self) -> Option<f32> {
        None
    }
    /// Per-rotor reported RPM (from ESC telemetry, e.g. DroneCAN esc.Status), or
    /// `None` if the ESCs report no RPM. The ACHIEVED-per-rotor source for the
    /// single-rotor-out FDI: the core compares it against the commanded throttle
    /// to form the effectiveness residual (v1.103). Default `None` = no telemetry,
    /// so a backend without ESC feedback simply runs without rotor-fault detection.
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        None
    }
}

/// The verified flight core, generic over the backend. Holds the estimator +
/// controllers; one [`step`](FlightCore::step) reads the backend's sensors,
/// estimates state, computes the stabilizing control, allocates it, and
/// writes the motors back — all on the verified `no_std` crates.
/// The ESTIMATOR partition (PART-P01, v1.124): IEKF propagate + every
/// measurement update (gravity, dual-GNSS position, magnetometer, yaw,
/// baro) + calibration application — the M4 half of the agreed RT1176
/// core mapping (jess#144 / DD-023). Publishes [`NavState`] (16×f32,
/// 64 bytes) as its SOLE output; the only other data crossing the
/// partition boundary is the sensor fan-out the hardware bus provides to
/// both cores (the raw IMU sample + heading), passed in by the caller so
/// a sim backend's one sample is never drawn twice.
pub struct EstimatorPartition {
    iekf: Iekf,
    grav_var: f32,
    pos_var: f32,
    mag_var: f32,
    /// Barometric-altitude measurement variance (m²) (v1.20). The baro is fed
    /// into the verified IEKF as a vertical anchor (a position update whose z is
    /// the baro), so altitude AND its rate survive GPS-vertical loss.
    baro_var: f32,
    /// Dual-receiver GNSS selector (GNSS-P02, v1.121): health gating,
    /// inverse-variance blending, one-interval failover, latched divergence.
    dual_gnss: falcon_gnss_ubx::dual::DualGnss,
    /// GNSS anti-spoof CUSUM (relay-iekf, previously an ORPHANED verified
    /// leaf): fed the position innovation each GNSS update; a sustained
    /// directional walk-off latches. OR-ed with the selector divergence into
    /// [`nav_compromised`](Self::nav_compromised).
    /// Ticks since the last ACCEPTED position fix, and the window over which
    /// that aiding counts as fresh (#452/#434). While position aiding is fresh
    /// the gravity update is SKIPPED: a multirotor's accelerometer senses
    /// specific force along the thrust axis, so taking it as gravity pulls the
    /// tilt estimate level exactly when the vehicle tilts to accelerate, and
    /// the hold then diverges at every fix rate. Tilt stays observable through
    /// the position innovations instead. With no aiding — on the ground, or
    /// through a GNSS outage — the accelerometer is the only attitude
    /// Control ticks since the last ACCEPTED position fix. Kept as an aiding
    /// freshness signal (diagnostics / future policies); it no longer gates the
    /// gravity update, because resuming that update on staleness was measured
    /// to make attitude 24x WORSE during an outage (#483).
    ticks_since_aiding: u32,
    /// Set by [`FlightCore::step`] from the cascade's own collective: the
    /// gravity update is only suppressed while the vehicle is FLYING. On the
    /// ground the accelerometer is the tilt reference that lets pre-arm see a
    /// settled, level estimate, and there the reading really is gravity.
    in_flight: bool,
    spoof: relay_iekf::SpoofMonitor,
    /// Sensor calibration applied to raw IMU/mag samples before the estimator
    /// (gyro/accel bias+scale, mag hard/soft-iron). Identity until
    /// `set_calibration` installs solved offsets — the explicit replacement for
    /// the prior identity-remap placeholder (raw samples flowed in uncorrected).
    calib: relay_calib::CalParams,
    /// Control-step counter, for the FDI spin-up guard. At arm/spin-up the ESC
    /// RPM lags the commanded throttle (real actuators, or a sim reporting the
    /// achieved rotor state), so the commanded-vs-achieved effectiveness
    /// residual spikes on EVERY rotor for the first fraction of a second — which
    /// would false-trip the CUSUM and drop a healthy vehicle into the degraded
    /// 3-rotor law. The FDI is held off until the actuators have spun up
    /// (`fdi_warmup_steps`), then runs normally (v1.113 — caught flying the
    /// production core against the gz plant, where achieved starts at 0).
    step_count: u32,
    fdi_warmup_steps: u32,
    /// The loop rate `fdi_warmup_steps` is counted at. Re-synced from the
    /// backend's own `dt` exactly as the cascade's (see [`resynced_rate`]),
    /// so the innovation-gate warm-up is a fixed TIME (#609 finding 7).
    designed_hz: f32,
}

impl EstimatorPartition {
    /// `pub` for the split deployment (#575): a `cascade-estimator` component
    /// builds this half on its own. `loop_hz` is the NOMINAL rate the fused
    /// core would have been built with; both halves re-derive their warm-up
    /// from the rate the backend is actually stepped at, by the same rule
    /// ([`resynced_rate`], [`FlightCore::warmup_steps`]), so they engage
    /// their gates on the same tick as the fused core.
    pub fn new(loop_hz: f32) -> Self {
        EstimatorPartition {
            iekf: Iekf::level(),
            grav_var: 0.5,
            pos_var: 0.01,
            mag_var: 0.1,
            baro_var: 0.05, // ≈ (0.2 m)² baro noise; trusted less than a clean GPS-z
            dual_gnss: falcon_gnss_ubx::dual::DualGnss::new(),
            // threshold 60 m·steps, drift slack 0.15 m/step: rejects zero-mean
            // GNSS noise, latches on a sustained directional walk-off.
            spoof: relay_iekf::SpoofMonitor::new(60.0, 0.15),
            // `warmup` is 0.2 s of steps, so 5x it is one second of staleness:
            // long enough to ride out a 5 Hz fix interval (0.2 s) and a missed
            // fix, short enough that a real outage falls back to the
            // accelerometer within a second.
            ticks_since_aiding: u32::MAX,
            in_flight: false,
            step_count: 0,
            fdi_warmup_steps: FlightCore::warmup_steps(loop_hz),
            designed_hz: loop_hz,
            calib: relay_calib::CalParams::identity(),
        }
    }

    /// Re-count the warm-up at the rate the backend really steps at (#609
    /// finding 7). The same tolerance and the same count as the cascade's
    /// `resync_loop_rate`, through the same two functions. The running
    /// `step_count` is rescaled to the new rate with it (review of v1.141,
    /// finding 3; see [`FlightCore::rescale_ticks`]).
    fn resync_loop_rate(&mut self, dt: f32) {
        if let Some(actual) = resynced_rate(self.designed_hz, dt) {
            self.fdi_warmup_steps = FlightCore::warmup_steps(actual);
            self.step_count = FlightCore::rescale_ticks(self.step_count, self.designed_hz, actual);
            self.designed_hz = actual;
        }
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
    /// Covariance-diagonal SUMMARY [attitude, velocity, position] — the
    /// per-block diagonal sums of the 15x15 error covariance. The blackbox
    /// logs these (LOG-P02) so a post-hoc analysis can see estimator health
    /// alongside the state, and the replay oracle covers them bit-exactly.
    pub fn cov_summary(&self) -> [f32; 3] {
        let p = self.iekf.covariance();
        [
            p[0][0] + p[1][1] + p[2][2],
            p[3][3] + p[4][4] + p[5][5],
            p[6][6] + p[7][7] + p[8][8],
        ]
    }
    /// Navigation-integrity flag (GNSS-P02): the anti-spoof CUSUM latched a
    /// sustained innovation walk-off, OR the dual-receiver selector latched
    /// receiver divergence. Consumed by pre-arm (and any failsafe policy).
    pub fn nav_compromised(&self) -> bool {
        self.spoof.spoofed() || self.dual_gnss.diverged()
    }
    /// The dual-GNSS selector's latched divergence flag alone (GNSS-P02).
    pub fn gnss_diverged(&self) -> bool {
        self.dual_gnss.diverged()
    }
    /// Set the IEKF position-measurement variance (m²) (v1.19). This must match
    /// the ACTUAL GNSS fix noise: the default (0.01 = 1 cm²) over-trusts a
    /// metre-class fix, injecting noise the loop chases into instability. Set it
    /// to ≈ stddev² of the real receiver.
    pub fn set_pos_var(&mut self, var: f32) {
        self.pos_var = var;
    }
    /// Set the IEKF velocity/position covariance-diagonal floor (m²/s², m²)
    /// (v1.113). 0 = off (default). Keeps the estimator from going deaf on a
    /// long static hover (the covariance would otherwise collapse and the NIS
    /// gate reject the correct fixes). See `Iekf::set_process_floor`.
    pub fn set_process_floor(&mut self, vel: f32, pos: f32) {
        self.iekf.set_process_floor(vel, pos);
    }
    /// Tell the estimator whether the vehicle is flying (#452). `FlightCore`
    /// derives it from the cascade's own collective; a standalone estimator
    /// keeps the ground behaviour, which is the pre-#452 behaviour.
    pub fn set_in_flight(&mut self, flying: bool) {
        self.in_flight = flying;
    }
    /// The estimated nav state (for telemetry / tests).
    pub fn state(&self) -> NavState {
        self.iekf.state()
    }

    /// GNSS estimator input, shared by [`step`](Self::step) and
    /// [`step_estimate_only`](Self::step_estimate_only): the dual-receiver
    /// selector (GNSS-P02) when the backend has one, else the legacy single
    /// lane. Feeds the anti-spoof CUSUM with the position innovation.
    fn fuse_gnss<B: FlightBackend>(&mut self, b: &mut B) {
        if let Some((ga, gb)) = b.read_gnss_dual() {
            // The innovation-gate reference engages after warm-up (a cold
            // estimate would disqualify honest receivers).
            let est_ref = if self.step_count >= self.fdi_warmup_steps * 5 {
                Some(self.iekf.state().p)
            } else {
                None
            };
            let dec = self.dual_gnss.update(ga, gb, est_ref);
            if let Some(p) = dec.pos {
                if let Some(e) = est_ref {
                    self.spoof.update([p[0] - e[0], p[1] - e[1], p[2] - e[2]]);
                }
                if self.iekf.update_position(p, self.pos_var) {
                    self.ticks_since_aiding = 0;
                }
            }
        } else if let Some(p) = b.read_position() {
            if self.iekf.update_position(p, self.pos_var) {
                self.ticks_since_aiding = 0;
            }
        }
    }

    /// One estimator tick: fanned-out IMU sample + heading, own reads of
    /// GNSS/mag/baro, publish the estimate. The partition's ONLY output.
    pub fn step<B: FlightBackend>(
        &mut self,
        b: &mut B,
        raw: ImuSample,
        heading: Option<f32>,
        dt: f32,
    ) -> NavState {
        // ── Calibrate (relay-calib): de-bias/scale the raw sample before the
        // estimator. Identity by default (a no-op) until set_calibration installs
        // solved offsets. ──
        self.resync_loop_rate(dt);
        let gyro = self.calib.apply_gyro(raw.gyro);
        let accel = self.calib.apply_accel(raw.accel);

        // ── Estimate ──
        self.iekf.propagate(IekfImu { gyro, accel }, dt);
        // GRAVITY UPDATE ONLY WHILE NOT FLYING (#452/#434/#483).
        //
        // The reading is SPECIFIC FORCE f = a_inertial - g. It equals -g, and
        // so gives the vertical, only while the vehicle is not accelerating.
        // Under thrust it points along the THRUST AXIS and shows almost none of
        // the tilt. Suppressing it while position aiding is fresh was #452.
        //
        // v1.140 then RESUMED it on aiding staleness alone, and that is worse
        // than no guard: during a GNSS outage the position loop chases a
        // drifting estimate, so the vehicle is MANEUVERING — exactly when the
        // reading is least valid. Measured (#483), peak |cos(tilt)_est - true|
        // through a 15 s outage on a plant with a gyro-bias ramp:
        //
        //     staleness fallback (v1.140)                    0.3787
        //     + |f| within 25% of g                          0.3787
        //     + |f| within 0.5% of g                         0.3787
        //     + |f| within 0.5% of g AND rate < 0.1 rad/s    0.3457
        //     NO in-flight gravity update (this)             0.0155
        //
        // A VALIDITY GATE ON |f| DOES NOT WORK, and the 50x tolerance sweep
        // above shows it is not a tuning problem. A quad holding altitude at
        // tilt θ has |f| = g/cos θ — inside 0.5% of g for θ < 5.7° — while the
        // direction is along body -z. So the update asserts "you are level"
        // regardless of the actual tilt: it does not merely fail to inform, it
        // ERASES tilt. Magnitude cannot distinguish the case, because the
        // thrust axis IS what the accelerometer reports.
        //
        // In flight the attitude reference is the gyro (with its bias state
        // observed through GNSS/mag/heading). On the ground, at rest, the
        // accelerometer genuinely reads -g and is the reference that lets
        // pre-arm see a settled, level estimate.
        if !self.in_flight {
            self.iekf.update_gravity(accel, self.grav_var);
        }
        self.ticks_since_aiding = self.ticks_since_aiding.saturating_add(1);
        self.fuse_gnss(b);
        if let Some(m) = b.read_mag() {
            self.iekf
                .update_magnetometer(self.calib.apply_mag(m), 0.0, self.mag_var);
        }
        // v1.113 — direct heading update: a backend that resolves a clean
        // absolute yaw (fused compass / GNSS heading / sim truth) feeds it
        // straight to the IEKF, making the otherwise-unobservable yaw observable
        // without inferring it from a raw magnetometer field.
        if let Some(yaw) = heading {
            self.iekf.update_yaw(yaw, self.mag_var);
        }
        // v1.20 — barometer: feed it into the verified IEKF as a vertical anchor
        // (a position update whose horizontal is the current estimate, a no-op,
        // and whose z is the baro). This keeps the IEKF's altitude AND vertical
        // velocity bounded through a GPS-vertical outage, reusing the filter's
        // estimation rather than a hand-rolled complementary filter.
        if let Some(bz) = b.read_baro() {
            let e = self.iekf.state();
            self.iekf
                .update_position([e.p[0], e.p[1], bz], self.baro_var);
        }
        self.step_count = self.step_count.saturating_add(1);
        self.iekf.state()
    }
}

/// The rate a partition should RE-DERIVE its rate-dependent state for, if
/// the backend's `dt` says it is being stepped more than 2% away from
/// `designed_hz` (jitter must not rebuild filters every tick); `None` to keep
/// it, or for a `dt` that is not a positive finite number. ONE rule for both
/// partitions (#609 finding 7), so a split's halves resync on the same tick.
fn resynced_rate(designed_hz: f32, dt: f32) -> Option<f32> {
    if !(dt.is_finite() && dt > 0.0) {
        return None;
    }
    let actual = 1.0 / dt;
    let drift = relay_math::fabsf(actual - designed_hz);
    (drift > 0.02 * designed_hz).then_some(actual)
}

/// The CASCADE partition (PART-P01, v1.124): geometric attitude, ADRC,
/// mixer + rotor-out FDI/reconfiguration, position/altitude loops,
/// RPM-notch gyro path and terrain-relative landing reference — the M7
/// half. Consumes the estimator partition's [`NavState`] (in the fused
/// core: same tick; across the MU/RPMsg hop: delayed — the delay
/// campaign is the requirement's oracle) plus the same fanned-out IMU
/// sample (the rate loop reads the gyro directly, as on hardware).
pub struct CascadePartition {
    geo: GeoAtt,
    adrc: AdrcRate,
    gyro_lpf: GyroLpf,
    mixer: QuadMixer,
    hover_thrust: f32,
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
    /// Latest gated terrain-relative AGL from the rangefinder (m), or `None`
    /// (v1.121, RANGEDRV-P01): tilt-compensated, innovation-gated. The
    /// landing law's altitude reference when present — centimetre truth
    /// exactly where the baro/GNSS metres of error matter most.
    range_agl: Option<f32>,
    /// RPM-tracked harmonic notch bank on the CONTROL-path gyro (NOTCH-P01,
    /// v1.122): per-motor fundamental + 2nd harmonic, driven by the same
    /// read_motor_rpm seam the FDI consumes; clean unity bypass without RPM
    /// telemetry. The estimator keeps the raw (calibrated) gyro — the IEKF's
    /// gravity/mag updates already reject zero-mean vibration, and filtering
    /// the estimator input would add lag where it is not needed.
    notch: relay_notch::HarmonicNotchBank<4>,
    /// Single-rotor-out fault detection & isolation (relay-iekf CUSUM on the
    /// per-rotor commanded-vs-achieved effectiveness residual). On isolation,
    /// `failed_motor` latches and the control law switches to reduced-attitude +
    /// the reconfigured allocator (v1.103 — the SITL-verified recovery, promoted
    /// into the production loop).
    fdi: RotorFaultDetector,
    /// The isolated failed rotor (latched), or `None`. Drives the degraded path.
    failed_motor: Option<usize>,
    /// Previous control tick's per-rotor command (v1.115, FAULT-P03). The ESC
    /// RPM telemetry read at the top of a step reflects the command from the
    /// PREVIOUS step (one-tick actuation/telemetry lag), so the effectiveness
    /// residual must compare achieved against THIS, not the current command —
    /// otherwise an abrupt collective step spikes every rotor's residual at once
    /// and the FDI false-isolates a healthy rotor.
    last_motor_cmd: [f32; 4],
    /// FDI observability (v1.115): last `(rate2, tilt_cos, gate_open, resid)`
    /// evaluated in the single-rotor-out detector — surfaced via `fdi_diag()`
    /// so the dispersed campaign can see WHY detection stalls under noise (the
    /// rate gate closing vs the residual), not just that it did.
    dbg_fdi: (f32, f32, bool, [f32; 4]),
    /// Commanded heading (yaw, rad, NED). A quad HOLDS its launch heading, not
    /// North — so when a heading reference first arrives (`read_heading`), the
    /// initial heading is captured here and the geometric controller drives yaw
    /// to it (not to 0, which would demand an unachievable snap-to-North on
    /// takeoff and saturate the mixer). v1.113. Stays 0 for backends with no
    /// heading source (unchanged behaviour).
    yaw_setpoint: f32,
    yaw_captured: bool,
    /// Diagnostics: the last geometric desired body rate + ADRC torque (for the
    /// gz yaw-loop investigation). Not part of the control state.
    last_omega_d: Vec3,
    last_torque: Vec3,
    /// Control-step counter, for the FDI spin-up guard. At arm/spin-up the ESC
    /// RPM lags the commanded throttle (real actuators, or a sim reporting the
    /// achieved rotor state), so the commanded-vs-achieved effectiveness
    /// residual spikes on EVERY rotor for the first fraction of a second — which
    /// would false-trip the CUSUM and drop a healthy vehicle into the degraded
    /// 3-rotor law. The FDI is held off until the actuators have spun up
    /// (now `fdi_guard_steps`, 0.3 s, #609 finding 18), then runs normally
    /// (v1.113 — caught flying the production core against the gz plant,
    /// where achieved starts at 0). `fdi_warmup_steps` (0.2 s) is no longer
    /// the FDI guard: it times the yaw-capture freeze (x5).
    step_count: u32,
    fdi_warmup_steps: u32,
    /// Control steps since the FDI was last (re-)armed: the spin-up guard's
    /// OWN counter (#609 finding 16). `step_count` above counted from
    /// construction only, so a second flight in the same process spun up with
    /// no guard at all. This one restarts at every spin-up
    /// ([`Self::rearm_fdi`]); `step_count` keeps its other job (the one-time
    /// launch-heading capture) unchanged.
    fdi_spinup: u32,
    /// The spin-up guard in ticks: [`FDI_SPINUP_GUARD_S`] at the resynced
    /// rate (#609 finding 18). The FDI runs once `fdi_spinup` reaches it.
    /// Separate from `fdi_warmup_steps` (0.2 s), which keeps driving the
    /// yaw-capture freeze unchanged.
    fdi_guard_steps: u32,
    /// `dt / FDI_ROTOR_LAG_TAU_S` at the resynced rate, capped at 1: the
    /// per-tick gain of the rotor lag model (#609 finding 18).
    fdi_lag_alpha: f32,
    /// The rotor lag model's state per rotor: the effectiveness a healthy
    /// rotor lagging [`FDI_ROTOR_LAG_TAU_S`] behind `last_motor_cmd` would
    /// report now. The FDI residual's reference. Zeroed by `rearm_fdi`.
    fdi_lag_model: [f32; 4],
    /// The stall check's window in ticks ([`FDI_STALL_WINDOW_S`] at the
    /// resynced rate, #609 finding 18).
    fdi_stall_steps: u32,
    /// Consecutive ticks each rotor has read stalled during this spin-up's
    /// guard. Zeroed by `rearm_fdi` and by any tick without RPM telemetry,
    /// and on a co-stalled tick once a shared window has filled (see
    /// `fdi_stall_common`). Rescaled to the new rate by `resync_loop_rate`.
    fdi_stall_run: [u32; 4],
    /// A stall window filled while two or more rotors were stalled at once
    /// (review of v1.141, finding 2): a COMMON-MODE stall, not a rotor
    /// fault. While set, every co-stalled tick resets the runs; cleared on
    /// the first tick with at most one rotor stalled, by a tick without RPM
    /// and by `rearm_fdi`.
    fdi_stall_common: bool,
    /// The cascade's own copy of the sensor calibration (the M7 sees the
    /// same IMU stream; `CalParams` is `Copy`, `set_calibration` syncs both).
    calib: relay_calib::CalParams,
    /// The loop rate `gyro_lpf` and `notch` were DESIGNED for. Re-synced from
    /// the backend's own `dt()` on the first tick — see `resync_loop_rate`.
    designed_hz: f32,
}

impl CascadePartition {
    /// `pub` for the split deployment (#575): a `cascade-control` component
    /// builds this half on its own. Same `loop_hz` contract as
    /// [`EstimatorPartition::new`].
    pub fn new(hover_thrust: f32, loop_hz: f32) -> Self {
        CascadePartition {
            designed_hz: loop_hz,
            geo: GeoAtt::new(GeoGains::FALCON_QUAD),
            adrc: AdrcRate::falcon_quad(),
            gyro_lpf: GyroLpf::new(60.0, loop_hz),
            mixer: QuadMixer::new(),
            hover_thrust,
            setpoint: [0.0; 3],
            kp_alt: 0.05,
            kd_alt: 0.30,
            ki_alt: 0.0,      // opt-in (set_altitude_integral_gain): the integral is
            alt_int: 0.0,     // for high-altitude thrust-lapse compensation; default
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
            range_agl: None,
            notch: relay_notch::HarmonicNotchBank::new(loop_hz),
            // CUSUM thresholds matching the SITL-verified fault-tolerance chain.
            fdi: RotorFaultDetector::new(FDI_CUSUM_THRESHOLD, FDI_CUSUM_DRIFT),
            failed_motor: None,
            yaw_setpoint: 0.0,
            yaw_captured: false,
            last_omega_d: [0.0; 3],
            last_torque: [0.0; 3],
            last_motor_cmd: [0.0; 4],
            dbg_fdi: (0.0, 1.0, false, [0.0; 4]),
            step_count: 0,
            fdi_warmup_steps: FlightCore::warmup_steps(loop_hz),
            fdi_spinup: 0,
            fdi_guard_steps: FlightCore::fdi_guard_steps(loop_hz),
            fdi_lag_alpha: FlightCore::fdi_lag_alpha(loop_hz),
            fdi_lag_model: [0.0; 4],
            fdi_stall_steps: FlightCore::fdi_stall_steps(loop_hz),
            fdi_stall_run: [0; 4],
            fdi_stall_common: false,
            calib: relay_calib::CalParams::identity(),
        }
    }

    /// The largest of the last commanded motor outputs (0..1). At/near 1.0 the
    /// control allocation is SATURATED — the rate loop is at its authority limit.
    /// Sustained saturation while tilted is the high-wind proxy (v1.101): the
    /// vehicle is fighting a disturbance beyond what it can hold.
    pub fn max_motor(&self) -> f32 {
        let m = self.mixer.last_motors();
        m[0].max(m[1]).max(m[2]).max(m[3])
    }
    /// FLYING, from the cascade's own last command (#452): the mean motor
    /// output has passed 70% of the hover collective. A vehicle sitting on the
    /// ground with its rotors idle is below it; one holding altitude is at
    /// ~100% of it. Used to decide whether the accelerometer is a usable
    /// gravity reference (it is, at rest; it is not, under thrust).
    pub fn flying(&self) -> bool {
        let m = self.mixer.last_motors();
        let mean = (m[0] + m[1] + m[2] + m[3]) * 0.25;
        // THE ERRORS HERE ARE NOT SYMMETRIC (#481).
        //
        // This decides `in_flight`, which is now the ONLY thing admitting the
        // gravity update (#483). Getting it wrong in the two directions costs
        // very different amounts:
        //
        //   says NOT flying while airborne -> the gravity update runs IN
        //     FLIGHT, which is the full #452 divergence: specific force points
        //     along the thrust axis, the tilt estimate is pulled level, and the
        //     hold diverges at every fix rate. Catastrophic, and silent.
        //   says flying while on the ground -> no gravity update on the ground,
        //     so pre-arm loses its tilt reference. Visible at pre-arm, and the
        //     vehicle does not take off.
        //
        // So bias toward "flying". A threshold of `0.7 * hover_thrust` alone
        // fails the dangerous way whenever hover_thrust is OVER-estimated, and
        // it is hardcoded to 0.5 in every shipped entry point --
        // `wasm/cm/flight`, `embedded/falcon-cortex-m`, `falcon-param`,
        // `falcon-hitl`, `FlightSupervisor::new` -- while
        // `wasm/cm/cascade` falls back to 0.5 when `configure` was never
        // called. An airframe hovering at 0.35 mean would then read NOT flying
        // in a steady hover.
        //
        // The absolute floor breaks that dependency: any vehicle holding a
        // mean collective above it is flying, whatever the configuration
        // claims. Idle-on-the-ground is well below it.
        const AIRBORNE_FLOOR: f32 = 0.25;
        let threshold = if 0.7 * self.hover_thrust < AIRBORNE_FLOOR {
            0.7 * self.hover_thrust
        } else {
            AIRBORNE_FLOOR
        };
        mean > threshold
    }
    /// The isolated failed rotor (latched), or `None` — the supervisor lands the
    /// vehicle when this is set (a 3-rotor quad cannot navigate; v1.103).
    /// The latest gated terrain-relative AGL (m) from the rangefinder
    /// (RANGEDRV-P01) — the land detector's ground-proximity input.
    pub fn range_agl(&self) -> Option<f32> {
        self.range_agl
    }
    pub fn failed_motor(&self) -> Option<usize> {
        self.failed_motor
    }
    /// The captured heading-hold setpoint (yaw, rad, NED). For telemetry/tests.
    pub fn yaw_setpoint(&self) -> f32 {
        self.yaw_setpoint
    }
    /// Set the hover-thrust feedforward (per-airframe; clamped to [0,1]).
    pub fn set_hover_thrust_core(&mut self, t: f32) {
        self.hover_thrust = if t.is_finite() {
            t.clamp(0.0, 1.0)
        } else {
            self.hover_thrust
        };
    }
    /// Set the landing descent rate (m/s, NED +down; clamped to [0.1, 2]).
    pub fn set_landing_descent(&mut self, vz: f32) {
        self.landing_descent = if vz.is_finite() {
            vz.clamp(0.1, 2.0)
        } else {
            self.landing_descent
        };
    }
    /// Current altitude P/D gains (tuning observability, v1.119).
    pub fn altitude_gains(&self) -> (f32, f32) {
        (self.kp_alt, self.kd_alt)
    }
    /// Current altitude integral gain (tuning observability, v1.119).
    /// The altitude loop's accumulated integral (m·s). Read-only, for
    /// diagnosis: an integral that keeps growing through a hold is the
    /// signature of transient windup rather than steady-bias rejection.
    pub fn altitude_integral(&self) -> f32 {
        self.alt_int
    }

    pub fn altitude_integral_gain(&self) -> f32 {
        self.ki_alt
    }
    /// Current hover-thrust feedforward (tuning observability, v1.119).
    pub fn hover_thrust(&self) -> f32 {
        self.hover_thrust
    }
    /// Current landing descent rate (tuning observability, v1.119).
    pub fn landing_descent(&self) -> f32 {
        self.landing_descent
    }
    /// The loop rate the rate-dependent filters are CURRENTLY designed for.
    ///
    /// Exposed so the resync can be OBSERVED rather than grepped. Independent
    /// review found every `steps:` entry of FV-FALCON-RATE-002 and
    /// FV-FALCON-FAULT-005 passing rc=0 in a tree where `resync_loop_rate` had
    /// been reverted to an early `return` — the tokens they grep for are all
    /// still present, so presence-greps cannot catch the regression they were
    /// written to catch (#499). A test that reads this value can.
    pub fn designed_loop_hz(&self) -> f32 {
        self.designed_hz
    }

    /// Diagnostics: last geometric desired body rate + last ADRC torque.
    pub fn last_omega_d(&self) -> Vec3 {
        self.last_omega_d
    }
    pub fn last_torque(&self) -> Vec3 {
        self.last_torque
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
        self.ki_alt = ki.clamp(0.0, 0.2);
        self.alt_int = 0.0;
    }
    /// Set the altitude P and D gains (v1.113). The defaults (0.05, 0.30) are
    /// gentle, tuned for the analytic plant; a stiffer plant (real gz) wants a
    /// higher kp to hold altitude firmly and a matched kd to damp the slow
    /// climb/overshoot oscillation the soft loop leaves.
    pub fn set_altitude_gains(&mut self, kp: f32, kd: f32) {
        self.kp_alt = kp.clamp(0.0, 1.0);
        self.kd_alt = kd.clamp(0.0, 3.0);
    }

    /// Keep stateful filters warm while idle (estimate-only mode): the
    /// gyro low-pass tracks so the first live tick has no filter jump.
    ///
    /// `pub` for the split deployment (#575): the control half of
    /// [`FlightCore::step_estimate_only`] runs in another component there.
    pub fn keep_warm(&mut self, raw: ImuSample) {
        let gyro = self.calib.apply_gyro(raw.gyro);
        let _ = self.gyro_lpf.filter(gyro);
    }

    /// One cascade tick: NavState + fanned IMU in, motor commands out.
    /// `heading_present` gates the launch-heading capture exactly as the
    /// fused core did (the capture reads the ESTIMATE, not the heading —
    /// moved here from mid-fusion, a seam change active only during the
    /// ~1 s pre-freeze window on heading-equipped vehicles; the analytic
    /// plant provides no heading, so the golden replay is bit-exact).
    /// Rebuild the rate-dependent filters if the caller's declared loop rate
    /// disagrees with the rate the backend is ACTUALLY being stepped at.
    ///
    /// WHY THIS EXISTS (#270/#398). `gyro_lpf` and the harmonic `notch` are
    /// DESIGNS: their coefficients are computed from a sample rate. Feed a
    /// filter designed for 1 kHz at 250 Hz and its real corner lands at a
    /// quarter of the intended frequency, with all the extra phase lag that
    /// implies — which is how a stable inner loop becomes a limit cycle.
    ///
    /// `FlightSupervisor::new` hardcoded `FlightCore::new(0.5, 1000.0)` while
    /// the gz bench steps it at 250 Hz. `run_flightcore` passes `1.0 / dt` and
    /// was correct. That single difference is the whole of the attitude ring
    /// that #270 has tracked since July and #398 sits downstream of. Measured
    /// on the gz falcon-quad, supervised hover:
    ///
    /// ```text
    /// declared 1000 Hz, stepped at 250      declared 250, stepped at 250
    ///   roll/pitch rate rms  2.47 rad/s       0.0015 rad/s
    ///   motors               0.12 <-> 1.0000  0.5815 - 0.5857
    ///   FDI gate shut        33.8%            0.0%
    ///   rotor-out verdict    FAIL 11/12       PASS 3/3
    /// ```
    ///
    /// A setter would have been forgettable in exactly the way the hardcoded
    /// constructor already was, so this takes the rate from the backend and
    /// costs one comparison per tick.
    ///
    /// THE WARM-UP TOO (#609 finding 7, #587 maintainer decision 2026-10-21):
    /// the FDI spin-up guard (and the yaw-capture freeze at 5x the 0.2 s
    /// warm-up) is a fixed TIME, counted at the resynced rate. It used to
    /// stay at the count the nominal 1 kHz gave (200 ticks), so the 250 Hz gz
    /// bench held the FDI off for 0.8 s and the yaw capture for 4 s, and a
    /// 2 kHz frame shrank the guard to 0.1 s. Since #609 finding 18 the guard
    /// is its own time, [`FDI_SPINUP_GUARD_S`] (0.3 s), and the rotor lag
    /// model's per-tick gain is re-derived here too.
    ///
    /// THE RUNNING COUNTERS TOO (review of v1.141, finding 3): re-deriving
    /// only the thresholds left the counters measured against them
    /// (`fdi_spinup`, `fdi_stall_run`, `step_count`) in OLD-rate ticks, so
    /// one late frame (dt 2 ms at 1 kHz, or 8 ms at 250 Hz) halved the stall
    /// window under a run already counted at the full rate, and a healthy
    /// rotor 60 ms into a 100 ms tolerance was isolated permanently; the
    /// guard could likewise release early. They are rescaled to the new
    /// rate here ([`FlightCore::rescale_ticks`]), so every one of them stays
    /// a measure of elapsed TIME.
    fn resync_loop_rate(&mut self, dt: f32) {
        if let Some(actual) = resynced_rate(self.designed_hz, dt) {
            let from = self.designed_hz;
            self.fdi_spinup = FlightCore::rescale_ticks(self.fdi_spinup, from, actual);
            self.step_count = FlightCore::rescale_ticks(self.step_count, from, actual);
            let mut i = 0;
            while i < 4 {
                self.fdi_stall_run[i] =
                    FlightCore::rescale_ticks(self.fdi_stall_run[i], from, actual);
                i += 1;
            }
            self.gyro_lpf = GyroLpf::new(60.0, actual);
            self.notch = relay_notch::HarmonicNotchBank::new(actual);
            self.fdi_warmup_steps = FlightCore::warmup_steps(actual);
            self.fdi_guard_steps = FlightCore::fdi_guard_steps(actual);
            self.fdi_lag_alpha = FlightCore::fdi_lag_alpha(actual);
            self.fdi_stall_steps = FlightCore::fdi_stall_steps(actual);
            self.designed_hz = actual;
        }
    }

    pub fn step<B: FlightBackend>(
        &mut self,
        b: &mut B,
        est: &NavState,
        raw: ImuSample,
        heading_present: bool,
        dt: f32,
    ) {
        self.resync_loop_rate(dt);
        let gyro = self.calib.apply_gyro(raw.gyro);
        if heading_present && !self.yaw_captured {
            // Capture the launch heading as the hold setpoint — the vehicle holds
            // THIS heading, not North. Captured AFTER the estimator+heading have
            // settled (past the warmup), NOT on the first raw sample, which can
            // be an init transient (gz's first Pose_V gave 1.57 rad while the
            // true heading was 0.94 — a frozen 0.6 rad error saturated the mixer).
            if !self.yaw_captured {
                // TRACK the ESTIMATE's yaw (not the raw heading) as the setpoint
                // while it settles, then FREEZE. Tracking the estimate keeps the
                // yaw error ≈ 0 during startup, so NO yaw torque is commanded —
                // tracking the raw heading instead left a transient mismatch
                // (heading 1.57 vs a lagging est 0.14) that commanded an initial
                // yaw torque and spun the airframe up. Freeze at 5× the warmup
                // (~1 s), by when the estimate has converged to the true heading.
                let e = *est;
                self.yaw_setpoint = relay_math::atan2f(
                    2.0 * (e.q[0] * e.q[3] + e.q[1] * e.q[2]),
                    1.0 - 2.0 * (e.q[2] * e.q[2] + e.q[3] * e.q[3]),
                );
                if self.step_count >= self.fdi_warmup_steps * 5 {
                    self.yaw_captured = true;
                }
            }
        }
        // v1.121 — terrain-relative AGL (RANGEDRV-P01): the decoded,
        // strength-gated TF02 return, tilt-compensated (relay-flowrange, the
        // previously-orphaned verified leaf). NOT fused into the z state —
        // three absolute references (biased GNSS-z, biased baro, true range)
        // would fight and the estimate would settle metres off. Instead the
        // range is the LANDING-PHASE altitude reference directly (the PX4
        // terrain-estimator role): innovation-gated against the estimator's
        // AGL so an obstacle overflight (sudden short return) is REJECTED —
        // it can never fold the touchdown logic — while a biased-baro/GNSS
        // descent still flares off the true ground.
        self.range_agl = b.read_range().and_then(|r| {
            let e = *est;
            // min 0.10 = the TF02 device floor: the touchdown threshold
            // (0.12 m) must sit INSIDE the valid band, not in the blind
            // zone below the gate (0.15 would go dark 3 cm too early).
            relay_flowrange::range_to_altitude(r, e.tilt_rad(), 0.10, 35.0).filter(|agl| {
                let est_agl = -e.p[2];
                // gate 3 m: wider than credible baro/GNSS vertical bias (the
                // assist must ENGAGE against lying references), narrower
                // than an obstacle step worth rejecting.
                (agl - est_agl) > -3.0 && (agl - est_agl) < 3.0
            })
        });

        // ── Altitude loop ── velocity-based touchdown when landing, else the
        // position P-I-D.
        let thrust = if self.landing {
            // ── Velocity-based touchdown (v1.27) ── command a constant descent
            // RATE (NED z positive = down), independent of the ground-effect
            // cushion the position loop floats on; cut to idle on touchdown so
            // the vehicle settles on the surface.
            // v1.121: terrain-relative AGL from the rangefinder when present
            // (centimetre truth exactly where the flare needs it); estimator
            // AGL otherwise. The gate above guarantees the two never differ
            // by an obstacle-sized step.
            let alt_agl = self.range_agl.unwrap_or(-est.p[2]);
            if alt_agl < 0.12 {
                0.1 // touched down — idle (the ground holds the vehicle)
            } else {
                // ROTOR-OUT feed-forward (v1.117, FAULT-P04): the law is
                // P-only around the hover baseline, so a 25% lift deficit
                // (3 rotors carrying 4 rotors' weight) becomes a steady-state
                // descent error of ~kvz⁻¹·Δhover ≈ +0.4 m/s — a hard landing.
                // Compensate by raising the baseline to the 3-rotor per-rotor
                // hover (×4/3); the rank-3 allocator's collective is 3·t, so
                // 3·(4/3·hover) restores the 4-rotor hover collective exactly.
                let hover_eff = if self.failed_motor.is_some() {
                    self.hover_thrust * (4.0 / 3.0)
                } else {
                    self.hover_thrust
                };
                (hover_eff - self.kvz_land * (self.landing_descent - est.v[2])).clamp(0.0, 1.0)
            }
        } else {
            // ── Position altitude P-I-D (v1.2; v1.20 baro-anchored; v1.22 +I) ──
            // thrust = hover − kp·alt_err − ki·∫alt_err + kd·v_z. The integral
            // (v1.22) rejects a steady thrust deficit (the air-density lapse).
            let alt_err = self.setpoint[2] - est.p[2];
            // ANTI-WINDUP (v1.139): integrate ONLY when the vehicle is not
            // already converging on the setpoint.
            //
            // The integral exists to reject a STEADY thrust deficit (air-density
            // lapse). Charging it during the APPROACH makes it an oscillator
            // instead. Measured on the gz falcon-quad holding 2 m, before this
            // gate — the integral peaks exactly as the target is reached, and
            // the vehicle then limit-cycles through it:
            //
            //   t=7.9s  true_z -2.14 (at target)  alt_int -6.26  <- peak charge
            //   t=13.9  true_z -2.64 (overshoot)  alt_int -3.45
            //   t=25.9  true_z -2.07              alt_int +1.52
            //   t=39.9  true_z -0.08 (near ground) alt_int -5.72
            //
            // At ki 0.03 a +-6 integral is a +-0.19 thrust swing on a 0.585
            // hover. Reported independently from a Windows/WSL2 bench as "it
            // starts oscillating after about 15 seconds".
            //
            // WHY THIS CONDITION and not the two obvious ones. A velocity gate
            // cannot separate the cases: the climb runs at only 0.1-0.29 m/s,
            // so any threshold loose enough to be meaningful stays open through
            // the whole transient. An error band cannot either: the thrust-lapse
            // sag this integral MUST reject is ~1.5 m, larger than the band that
            // would block the climb. Their product does separate them —
            // converging (error and vertical velocity share a sign) is the
            // transient; stuck (velocity ~0 with error present) is the steady
            // deficit. NED sign convention: below the target means alt_err < 0
            // and closing it means v[2] < 0, so converging is a POSITIVE
            // product either way up.
            //
            // A MAGNITUDE FLOOR, added with #452. The product test alone reads
            // a vehicle that has STOPPED short as "still converging": parked
            // 2.15 m below the setpoint under thrust lapse, the residual climb
            // is 0.0005 m/s, which shares its sign with the error and holds the
            // integral off forever. The old estimator hid this — its vertical
            // velocity lagged and dithered in sign, so the integral charged by
            // accident. With the accelerometer reporting true specific force
            // the estimate is exact (velocity error ~0.000 m/s) and the gate
            // never opens. 0.05 m/s sits below the 0.1-0.29 m/s climb #404
            // measured (so the approach is still gated) and above the creep of
            // a stalled hold.
            const VZ_CONVERGING_MIN: f32 = 0.05;
            let converging =
                alt_err * est.v[2] > 0.0 && relay_math::fabsf(est.v[2]) > VZ_CONVERGING_MIN;
            if !converging {
                self.alt_int += alt_err * dt;
            }
            let cap = if self.ki_alt > 0.0 {
                self.alt_int_max / self.ki_alt
            } else {
                0.0
            };
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
            let cap = if self.ki_pos > 0.0 {
                self.pos_int_max / self.ki_pos
            } else {
                0.0
            };
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

        // v1.122 — RPM-tracked harmonic notches on the CONTROL path, ahead
        // of the low-pass (NOTCH-P01): read the achieved RPM once per step
        // (shared with the FDI below, `step_fdi`), track or bypass cleanly.
        let rpm_now = b.read_motor_rpm();
        self.notch.update_rpm(rpm_now);
        let gyro_ctrl = self.notch.apply(gyro);
        // advance the (stateful) gyro low-pass every cycle so it stays warm.
        let gyro_f = self.gyro_lpf.filter(gyro_ctrl);

        // ── Attitude + allocate ──
        let motors = if let Some(failed) = self.failed_motor {
            // DEGRADED (single-rotor-out): reduced-attitude S² drives the thrust
            // axis and RELINQUISHES yaw, and the reconfigured allocator pins the
            // failed rotor to 0 (MIX-P08) — the SITL-verified recovery path,
            // promoted into the production loop (v1.103).
            let r = quat_to_rotmat(est.q);
            let b3_d = thrust_axis_ned(a_cmd).unwrap_or([0.0, 0.0, 1.0]);
            let torque = self.geo.moment_reduced(&r, gyro_ctrl, b3_d);
            self.mixer
                .mix_rotor_out(failed, torque, thrust, ROTOR_OUT_FLOOR)
        } else {
            // NORMAL: full-attitude geometric desired-rate → ADRC torque → mix.
            let omega_d = self.geo.desired_rate(est.q, a_cmd, self.yaw_setpoint);
            let torque = self.adrc.tick(gyro_f, omega_d, dt);
            self.last_omega_d = omega_d;
            self.last_torque = torque;
            // THRUST-PRIORITY mix (MIX-P05): scale torque down to keep every
            // motor in [floor,1] rather than sacrificing collective — so a large
            // transient torque (e.g. the yaw slew to the launch heading) cannot
            // steal lift and pin the vehicle to the ground. The gz bench needs
            // this: attitude-priority `mix` collapsed collective near saturation
            // → never lifted. Zero-sum torque columns keep the mean (collective)
            // exactly `thrust` (v1.113).
            self.mixer.mix_thrust_floor(torque, thrust, 0.0)
        };

        // ── ESO ANTI-WINDUP (#270) ── tell the observer what the ALLOCATOR
        // actually delivered. The mix above is thrust-priority, so under
        // saturation it scales the TORQUE down and the vehicle receives
        // `s * torque`. An ESO driven by the COMMAND integrates a torque that
        // was never applied: the residual grows, z2 winds up toward
        // beta2 = omega_o^2, the control answers with MORE torque, and the loop
        // sustains its own saturation. Measured on the gz plant as a
        // rail-to-rail 3.5 Hz limit cycle latching within one second of takeoff
        // and never recovering.
        //
        // Tick N uses tick N-1's fraction, and that is correct rather than an
        // off-by-one: the observer can only be told what was APPLIED, which is
        // not knowable until after the allocator has run.
        //
        // Both arms above set the scale — `mix_thrust_floor` to the uniform `s`
        // it applied, `mix_rotor_out` to 1.0 meaning not-modelled — so this read
        // can never pick up a stale value from an earlier tick.
        let delivered = self.mixer.last_torque_scale();
        self.adrc.set_delivered_fraction([delivered; 3]);

        // ── Single-rotor-out FDI ── form the per-rotor effectiveness residual
        // |commanded − achieved| from ESC RPM telemetry and feed the CUSUM; on
        // isolation, latch the failed rotor (next step runs the degraded path).
        // Held off for `fdi_guard_steps` (spin-up guard, FDI_SPINUP_GUARD_S)
        // — see the field docs.
        self.step_count = self.step_count.saturating_add(1);
        self.step_fdi(est, gyro_f, rpm_now);
        // Record this tick's command for next tick's residual alignment.
        self.last_motor_cmd = motors;

        b.write_motors(&motors);
    }
}

/// The verified flight core — now the FUSED composition of the two
/// partitions (PART-P01): one struct, the same public API, and the type
/// system proves the only estimator→cascade crossing is [`NavState`].
pub struct FlightCore {
    /// The estimator partition (M4 half) — public for the partitioned
    /// deployment shape and the delay-robustness harness.
    pub est: EstimatorPartition,
    /// The cascade partition (M7 half).
    pub casc: CascadePartition,
}

impl FlightCore {
    pub fn new(hover_thrust: f32, loop_hz: f32) -> Self {
        FlightCore {
            est: EstimatorPartition::new(loop_hz),
            casc: CascadePartition::new(hover_thrust, loop_hz),
        }
    }

    /// The warm-up step count at `loop_hz`: 0.2 s of ticks, ROUNDED.
    ///
    /// It seeds the estimator's innovation-gate warm-up and the cascade's
    /// yaw-capture freeze (both ×5, ≥10 steps floor). It USED to be the FDI
    /// spin-up guard as well; since #609 finding 18 that is its own, longer
    /// time ([`Self::fdi_guard_steps`], [`FDI_SPINUP_GUARD_S`]).
    ///
    /// ONE definition, `pub` (#575): the fused core, the partitioned core and
    /// a split deployment that builds each half in its own component must all
    /// derive it identically, or the halves engage their gates on different
    /// ticks than the fused core does. Both halves call it at construction
    /// AND on every rate resync ([`resynced_rate`], #609 finding 7).
    ///
    /// ROUNDED, NOT TRUNCATED: the rate a resync derives is `1/dt` in f32,
    /// and 1/(1/1000) is 999.99994, whose 0.2 s truncates to 199 ticks. A
    /// 1 kHz frame must count the 200 the nominal construction counts.
    pub fn warmup_steps(loop_hz: f32) -> u32 {
        ((loop_hz * 0.2 + 0.5) as u32).max(10)
    }

    /// A running tick count, counted at `from_hz`, re-expressed in ticks of
    /// `to_hz`: the same elapsed time, ROUNDED like [`Self::warmup_steps`]
    /// (review of v1.141, finding 3). Both partitions call it on a rate
    /// resync for every counter compared against a rate-derived threshold,
    /// so a fixed-TIME threshold is never met by ticks of another rate. The
    /// f32 -> u32 cast saturates, so a saturated counter stays saturated; a
    /// non-positive `from_hz` (never produced by [`resynced_rate`]) leaves
    /// the count unchanged. One definition for the fused core and both
    /// halves of a split.
    pub fn rescale_ticks(count: u32, from_hz: f32, to_hz: f32) -> u32 {
        if !(from_hz > 0.0 && to_hz > 0.0) {
            return count;
        }
        (count as f32 * (to_hz / from_hz) + 0.5) as u32
    }

    /// Install solved sensor offsets — synced to BOTH partitions (each
    /// core applies the same calibration to its copy of the IMU stream).
    pub fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        self.est.calib = calib;
        self.casc.calib = calib;
    }

    pub fn calibration(&self) -> relay_calib::CalParams {
        self.est.calib
    }

    /// One MOTORS-OFF tick (the supervisor's Disarmed / Armed / Terminated
    /// path): estimator warm, cascade filters warm, zero motors commanded.
    ///
    /// NOT FLYING, BY CONSTRUCTION (#609 finding 8). This tick commands zero
    /// thrust, so the estimator is told the vehicle is not flying — the same
    /// rule [`step`](Self::step) applies from the cascade's previous command,
    /// read here from the command this tick is known to write. Until #609 only
    /// `step` wrote the flag, and a supervised touchdown fires (AGL < 0.15 m)
    /// while the landing law still commands near-hover collective: the flag
    /// stayed TRUE on the ground, the accelerometer gravity update stayed
    /// suppressed for as long as the vehicle sat there, and with no GNSS and a
    /// small gyro-bias ramp the published tilt of a level, grounded airframe
    /// walked off by ~1 rad in 60 s
    /// (`tests/grounded_estimator_after_touchdown.rs`). After a termination in
    /// the air the vehicle is unpowered too; its specific force is then drag,
    /// not thrust, and `update_gravity` inflates the variance with |a| − g.
    pub fn step_estimate_only<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let raw = b.read_imu();
        let heading = b.read_heading();
        self.est.set_in_flight(false);
        let _ = self.est.step(b, raw, heading, dt);
        self.casc.keep_warm(raw);
        b.write_motors(&[0.0, 0.0, 0.0, 0.0]);
    }

    /// One fused control tick: sensor fan-out → estimator partition →
    /// NavState → cascade partition (zero transport delay). The
    /// partitioned deployment inserts the MU/RPMsg hop exactly at the
    /// NavState hand-off (the delay-robustness campaign is its oracle).
    pub fn step<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let raw = b.read_imu();
        let heading = b.read_heading();
        // The cascade's PREVIOUS command says whether we are flying (#452).
        self.est.set_in_flight(self.casc.flying());
        let ns = self.est.step(b, raw, heading, dt);
        self.casc.step(b, &ns, raw, heading.is_some(), dt);
    }

    pub fn tilt_uncertainty(&self) -> f32 {
        self.est.tilt_uncertainty()
    }

    pub fn cov_summary(&self) -> [f32; 3] {
        self.est.cov_summary()
    }

    pub fn nav_compromised(&self) -> bool {
        self.est.nav_compromised()
    }

    pub fn gnss_diverged(&self) -> bool {
        self.est.gnss_diverged()
    }

    pub fn set_pos_var(&mut self, var: f32) {
        self.est.set_pos_var(var)
    }

    pub fn set_process_floor(&mut self, vel: f32, pos: f32) {
        self.est.set_process_floor(vel, pos)
    }

    pub fn state(&self) -> NavState {
        self.est.state()
    }

    pub fn max_motor(&self) -> f32 {
        self.casc.max_motor()
    }

    pub fn range_agl(&self) -> Option<f32> {
        self.casc.range_agl()
    }

    pub fn failed_motor(&self) -> Option<usize> {
        self.casc.failed_motor()
    }

    pub fn yaw_setpoint(&self) -> f32 {
        self.casc.yaw_setpoint()
    }

    pub fn set_hover_thrust_core(&mut self, t: f32) {
        self.casc.set_hover_thrust_core(t)
    }

    pub fn set_landing_descent(&mut self, vz: f32) {
        self.casc.set_landing_descent(vz)
    }

    pub fn altitude_gains(&self) -> (f32, f32) {
        self.casc.altitude_gains()
    }

    pub fn altitude_integral_gain(&self) -> f32 {
        self.casc.altitude_integral_gain()
    }

    /// Forwards `CascadePartition::altitude_integral` — read-only diagnosis.
    pub fn altitude_integral(&self) -> f32 {
        self.casc.altitude_integral()
    }

    pub fn hover_thrust(&self) -> f32 {
        self.casc.hover_thrust()
    }

    pub fn landing_descent(&self) -> f32 {
        self.casc.landing_descent()
    }

    pub fn fdi_diag(&self) -> (f32, f32, bool, [f32; 4]) {
        self.casc.fdi_diag()
    }
    /// See [`CascadePartition::designed_loop_hz`].
    pub fn designed_loop_hz(&self) -> f32 {
        self.casc.designed_loop_hz()
    }

    pub fn last_omega_d(&self) -> Vec3 {
        self.casc.last_omega_d()
    }

    pub fn last_torque(&self) -> Vec3 {
        self.casc.last_torque()
    }

    pub fn set_altitude(&mut self, ned_z: f32) {
        self.casc.set_altitude(ned_z)
    }

    pub fn set_landing(&mut self, on: bool) {
        self.casc.set_landing(on)
    }

    pub fn set_position(&mut self, ned: Vec3) {
        self.casc.set_position(ned)
    }

    pub fn set_position_integral_gain(&mut self, ki: f32) {
        self.casc.set_position_integral_gain(ki)
    }

    /// Forwards `CascadePartition::rearm_fdi` (#609 finding 16).
    pub fn rearm_fdi(&mut self) {
        self.casc.rearm_fdi()
    }

    pub fn set_altitude_integral_gain(&mut self, ki: f32) {
        self.casc.set_altitude_integral_gain(ki)
    }

    pub fn set_altitude_gains(&mut self, kp: f32, kd: f32) {
        self.casc.set_altitude_gains(kp, kd)
    }
}

/// The PARTITIONED deployment shape (PART-P01, v1.124): the two halves of
/// [`FlightCore`] with the inter-core transport hop modeled between them —
/// the estimator's published [`NavState`] reaches the cascade DELAYED (and
/// optionally jittered), as it will across the RT1176's MU/RPMsg seam.
/// `delay_ticks = 0` reproduces the fused core exactly (same code path);
/// the delay-robustness campaign runs 1–2 ticks ± 1 tick of jitter.
pub struct PartitionedCore {
    pub est: EstimatorPartition,
    pub casc: CascadePartition,
    /// NavState transport buffer, newest at index 0 (bounded: the hop is
    /// at most a few ticks; 8 is headroom, not a target).
    buf: [NavState; 8],
    buf_len: usize,
    /// Base transport delay in ticks (0 = fused-equivalent).
    delay_ticks: usize,
    /// Per-tick jitter schedule: an LCG draws −1/0/+1 around the base
    /// delay when enabled (deterministic per seed — campaigns replicate).
    jitter: bool,
    lcg: u32,
}

impl PartitionedCore {
    pub fn new(
        hover_thrust: f32,
        loop_hz: f32,
        delay_ticks: usize,
        jitter: bool,
        seed: u32,
    ) -> Self {
        PartitionedCore {
            est: EstimatorPartition::new(loop_hz),
            casc: CascadePartition::new(hover_thrust, loop_hz),
            buf: [NavState::identity(); 8],
            buf_len: 0,
            delay_ticks: delay_ticks.min(6),
            jitter,
            lcg: seed | 1,
        }
    }

    pub fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        self.est.calib = calib;
        self.casc.calib = calib;
    }

    pub fn set_altitude(&mut self, ned_z: f32) {
        self.casc.set_altitude(ned_z);
    }

    pub fn state(&self) -> NavState {
        self.est.state()
    }

    fn draw_jitter(&mut self) -> i32 {
        self.lcg = self.lcg.wrapping_mul(1664525).wrapping_add(1013904223);
        ((self.lcg >> 8) % 3) as i32 - 1 // −1, 0, +1
    }

    /// One control tick with the transport hop: the estimator publishes,
    /// the cascade consumes the estimate from `delay ± jitter` ticks ago
    /// (clamped to what has been published — start-up serves the oldest
    /// available rather than inventing a state).
    pub fn step<B: FlightBackend>(&mut self, b: &mut B) {
        let dt = b.dt();
        let raw = b.read_imu();
        let heading = b.read_heading();
        self.est.set_in_flight(self.casc.flying());
        let ns = self.est.step(b, raw, heading, dt);
        // push newest at 0
        for i in (1..self.buf.len()).rev() {
            self.buf[i] = self.buf[i - 1];
        }
        self.buf[0] = ns;
        self.buf_len = (self.buf_len + 1).min(self.buf.len());
        let mut d = self.delay_ticks as i32;
        if self.jitter {
            d += self.draw_jitter();
        }
        let idx = d.clamp(0, self.buf_len as i32 - 1) as usize;
        let delayed = self.buf[idx];
        self.casc.step(b, &delayed, raw, heading.is_some(), dt);
    }
}

/// Nominal ESC RPM at full throttle — normalises reported RPM to a per-rotor
/// EFFECTIVENESS (≈ commanded throttle when healthy) for the single-rotor-out FDI
/// residual (v1.103). Only the ratio matters; a per-airframe value scales it.
const ESC_RPM_FULL: f32 = 8000.0;

/// Thrust floor for the reconfigured (single-rotor-out) allocator. With the
/// rank-3 allocation (v1.114) the diagonal-opposite of the failed rotor
/// legitimately rests at 0 — that couple-cancelling zero is what gives the
/// pair a PARASITIC-FREE pure-lift axis — so the floor is 0: a positive floor
/// would clamp that rotor up and reinject the very moment the allocation
/// removes (and, being below the desaturation base, would collapse all
/// roll/pitch authority to zero). Tilt authority now comes from the solved
/// allocation, not a floor.
const ROTOR_OUT_FLOOR: f32 = 0.0;

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

/// A battery-voltage driver (ADC), volts, or `None` when the board has no
/// monitor.
///
/// The default used to return 16.0 V with the rationale "so a board without a
/// monitor still flies". A board without a battery monitor should NOT fly on a
/// fabricated reading — it should fail the BatteryOk pre-arm row and say so.
/// Override this to read the real divider (#413).
pub trait BatteryDriver {
    fn voltage(&mut self) -> Option<f32> {
        None
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
    fn read_battery_v(&mut self) -> Option<f32> {
        self.battery.voltage()
    }
}

/// A deterministic SIMULATION backend (analytic rigid-body attitude plant) —
/// the FIRST backend behind the HAL seam. The motor commands drive the body
/// torque (through the same mixer geometry), the attitude integrates, and the
/// IMU/mag are synthesised from it. Swapping this for a real-hardware
/// `FlightBackend` is the only change needed to fly the same core on a board.
pub struct SimBackend {
    /// The voltage sense has FAILED (harness knob): `read_battery_v` returns
    /// `None` from here on, as an ADC or harness fault does mid-flight. Absent
    /// from boot is the pre-arm case; LOSING it while airborne is the failsafe
    /// case, and they are not the same test.
    pub battery_lost: bool,
    /// SPECIFIC FORCE in NED (m/s²), what an accelerometer actually senses:
    /// the non-gravitational acceleration the plant produced last step
    /// (thrust + wind + drag, and the ground reaction while in contact).
    /// A real IMU, and gz's, reports this; the gravity reaction is what it
    /// reads at rest, and under thrust it points along the thrust axis. #452.
    specific_force_ned: Vec3,
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
    /// Quadratic ROTATIONAL (aerodynamic) drag coefficient (v1.114): a body-rate
    /// damping torque τ = −c·ω·|ω| per axis. Quadratic so it is negligible at
    /// normal attitude rates (≲1 rad/s) yet bounds a runaway spin — the effect
    /// that caps a real single-rotor-out quad's yaw spin (both surviving lift
    /// rotors share a spin sense, so the reaction torque is otherwise
    /// unopposed). Off (0) by default so existing tests are unchanged; the
    /// rotor-out recovery opts in, as it does for the other aero pathologies.
    pub rot_drag: f32,
    /// Barometer present? (v1.20) When true, `read_baro` returns the altitude.
    pub baro_enabled: bool,
    /// Barometric altitude noise stddev (m) (v1.20).
    pub baro_noise: f32,
    /// Battery terminal voltage reported to the supervisor's failsafe (v1.8).
    /// When `battery_drain` is on (v1.21) this is COMPUTED each step from the
    /// charge and load, not set: V = 12.6 + 4.2·charge − R·I (open-circuit +
    /// sag), so the failsafe fires on a real endurance limit.
    pub battery_v: f32,
    /// The collective thrust command from the last write_motors — the sim's
    /// battery LOAD (v1.120): drives the current sense read_battery_i exposes.
    last_collective: f32,
    /// Rotor-harmonic gyro vibration amplitude (rad/s) — NARROWBAND lines at
    /// each motor's rotation frequency (v1.122, NOTCH-P01's contaminant; the
    /// broadband `vibration` pathology cannot exercise a notch).
    pub rotor_vib: f32,
    /// Per-motor vibration phase accumulators (rad).
    rotor_vib_phase: [f32; 4],
    /// RPM telemetry available? (bidir-DShot present). false = read_motor_rpm
    /// returns None: the notch bank must bypass and the FDI loses its source.
    pub rpm_telemetry: bool,
    /// GNSS position bias (m, NED) — real receivers carry a slowly-varying
    /// meters-class VERTICAL offset (multipath/atmosphere) that zero-mean
    /// noise does not model: it does not average out. The biased-vertical
    /// landing scenario (RANGEDRV-P01) needs it.
    pub gps_pos_bias: [f32; 3],
    /// Baro static bias (m, NED z) — the lying-baro landing scenario
    /// (RANGEDRV-P01): baro altitude is off by metres exactly at the flare.
    pub baro_bias: f32,
    /// Rangefinder model (v1.121): when true, read_range returns the TRUE
    /// AGL + noise (a decoded, gated TF02 return), overridden by
    /// `range_fault` when set (obstacle overflight / false short return).
    pub range_enabled: bool,
    pub range_noise: f32,
    /// Scripted rangefinder override (m) — obstacle injection.
    pub range_fault: Option<f32>,
    /// Dual-GNSS model (v1.121): when true, read_gnss_dual serves two
    /// receivers derived from truth + per-receiver noise/offset/health
    /// scripts; read_position is then unused by the core.
    pub gnss_dual_enabled: bool,
    /// Per-receiver scripted fault: None = healthy; Some(fix) overrides.
    pub gnss_a_down: bool,
    pub gnss_b_down: bool,
    /// Additive NED offset on receiver B (divergence/spoof injection).
    pub gnss_b_offset: [f32; 2],
    /// Reported accuracy per receiver (m).
    pub gnss_a_acc: f32,
    pub gnss_b_acc: f32,
    /// Scripted current sense for tests (v1.120): returned by read_battery_i
    /// when the drain model is off. `None` = no current sense (fallback mode).
    pub battery_i: Option<f32>,
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
    /// Injected SINGLE-ROTOR FAILURE (v1.114): the named rotor produces no
    /// thrust or torque (its lagged actual is pinned to 0) and its ESC reports
    /// RPM 0, so the same rigid-body plant exercises the FDI + reduced-attitude
    /// recovery that only the gz bench used to see. `None` = all healthy.
    failed_rotor: Option<usize>,
}

const GRAVITY: f32 = 9.81;

impl SimBackend {
    /// Start at attitude `r0`, at rest, at altitude 0. `dt` = control period.
    pub fn new(r0: [[f32; 3]; 3], dt: f32) -> Self {
        SimBackend {
            battery_lost: false,
            // At rest on the ground the accelerometer reads the gravity
            // reaction: −g on NED z (the vehicle is held up, not falling).
            specific_force_ned: [0.0, 0.0, -GRAVITY],
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
            rot_drag: 0.0,
            baro_enabled: false,
            baro_noise: 0.0,
            battery_v: 16.0,
            battery_i: None,
            rotor_vib: 0.0,
            rotor_vib_phase: [0.0; 4],
            rpm_telemetry: true,
            gps_pos_bias: [0.0; 3],
            baro_bias: 0.0,
            range_enabled: false,
            range_noise: 0.0,
            range_fault: None,
            gnss_dual_enabled: false,
            gnss_a_down: false,
            gnss_b_down: false,
            gnss_b_offset: [0.0, 0.0],
            gnss_a_acc: 1.0,
            gnss_b_acc: 1.0,
            battery_drain: false,
            battery_charge: 1.0,
            thrust_lapse: 0.0,
            motor_tau: 0.0,
            motor_state: [0.5; 4], // start at hover collective
            last_collective: 2.0,
            ground_effect: 0.0,
            ground_contact: false,
            path: Pathology::default(),
            step_n: 0,
            gyro_bias: [0.0; 3],
            rng: 0x9E3779B9, // a fixed, non-zero seed (golden-ratio constant)
            turb_state: [0.0; 3],
            failed_rotor: None,
        }
    }

    /// Inject sensor pathologies (v1.9). Builder form for the robustness tests.
    pub fn with_pathology(mut self, path: Pathology) -> Self {
        self.path = path;
        self
    }

    /// Kill rotor `i` (v1.114): from now on it produces no thrust/torque and its
    /// ESC reports RPM 0, driving the core's FDI + reduced-attitude recovery on
    /// the analytic plant (the offline twin of the gz rotor-out bench).
    pub fn fail_rotor(&mut self, i: usize) {
        self.failed_rotor = Some(i);
    }

    /// Reseed the deterministic noise LCG (v1.114): lets a Monte-Carlo campaign
    /// give each trial an independent noise realisation (otherwise every fresh
    /// backend replays the same sequence). A zero seed is nudged off 0.
    pub fn reseed(&mut self, seed: u32) {
        self.rng = if seed == 0 { 0x9E37_79B9 } else { seed };
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
        let jo = [
            self.j[0] * self.omega[0],
            self.j[1] * self.omega[1],
            self.j[2] * self.omega[2],
        ];
        let gyro = [
            self.omega[1] * jo[2] - self.omega[2] * jo[1],
            self.omega[2] * jo[0] - self.omega[0] * jo[2],
            self.omega[0] * jo[1] - self.omega[1] * jo[0],
        ];
        for i in 0..3 {
            // Quadratic rotational aero drag τ_d = −c·ω·|ω| (v1.114): bounds a
            // rotor-out spin, negligible at normal rates. Uses the pre-update ω.
            let drag = self.rot_drag * self.omega[i] * relay_math::fabsf(self.omega[i]);
            self.omega[i] += self.dt * (torque[i] - gyro[i] - drag) / self.j[i];
        }
        // first-order rotation integration (Rᵢ₊₁ = Rᵢ·(I + [ω]ₓdt))
        let wd = [
            self.omega[0] * self.dt,
            self.omega[1] * self.dt,
            self.omega[2] * self.dt,
        ];
        let incr = [
            [1.0, -wd[2], wd[1]],
            [wd[2], 1.0, -wd[0]],
            [-wd[1], wd[0], 1.0],
        ];
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
        // The accelerometer senses SPECIFIC FORCE, rotated into the body
        // frame — thrust included. At rest that is the gravity reaction
        // (−z in NED); under thrust it points along the thrust axis, which
        // is why tilt is nearly invisible to it in flight (#452, #434).
        let mut accel = self.to_body(self.specific_force_ned);
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
        // v1.122 — per-motor rotational vibration: a LINE at each rotor's
        // rotation frequency (phase advanced from the lagged motor state),
        // split across roll/pitch like a real airframe transmits it.
        let mut vib = [0.0f32; 3];
        if self.rotor_vib > 0.0 {
            for m in 0..4 {
                let f = self.motor_state[m] * ESC_RPM_FULL / 60.0;
                self.rotor_vib_phase[m] += 2.0 * core::f32::consts::PI * f * self.dt;
                if self.rotor_vib_phase[m] > 2.0 * core::f32::consts::PI {
                    self.rotor_vib_phase[m] -= 2.0 * core::f32::consts::PI;
                }
                vib[0] += self.rotor_vib * relay_math::sinf(self.rotor_vib_phase[m]);
                vib[1] += self.rotor_vib * relay_math::cosf(self.rotor_vib_phase[m]);
            }
        }
        let gyro = [
            self.omega[0] + self.gyro_bias[0] + vib[0] + gw * self.noise_unit(),
            self.omega[1] + self.gyro_bias[1] + vib[1] + gw * self.noise_unit(),
            self.omega[2] + self.gyro_bias[2] + vib[2] + gw * self.noise_unit(),
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
        // v1.121 — plus the receiver's slowly-varying bias (multipath /
        // atmosphere): a metres-class offset zero-mean noise cannot model.
        let bias = self.gps_pos_bias;
        if p.gps_noise > 0.0 {
            return Some([
                self.pos[0] + bias[0] + p.gps_noise * self.noise_unit(),
                self.pos[1] + bias[1] + p.gps_noise * self.noise_unit(),
                self.pos[2] + bias[2] + p.gps_noise * self.noise_unit(),
            ]);
        }
        Some([
            self.pos[0] + bias[0],
            self.pos[1] + bias[1],
            self.pos[2] + bias[2],
        ]) // full 6-DoF NED position (v1.3) + receiver bias (v1.121)
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
        for i in 0..4 {
            let cmd = motors.get(i).copied().unwrap_or(0.0);
            if self.motor_tau > 0.0 {
                self.motor_state[i] += (cmd - self.motor_state[i]) * (self.dt / self.motor_tau);
                m4[i] = self.motor_state[i];
            } else {
                m4[i] = cmd;
            }
        }
        // v1.114 — a FAILED rotor produces no thrust/torque regardless of the
        // (lagged) command. Pin it to 0 BEFORE the collective/torque sums so
        // the plant experiences the true rotor-out upset the recovery fights.
        if let Some(f) = self.failed_rotor {
            m4[f] = 0.0;
        }
        let mut collective = 0.0f32;
        for &v in m4.iter() {
            collective += v;
        }
        self.motor_state = m4; // achieved state (what the ESCs report back)
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
        if self.wind[0] != 0.0
            || self.wind[1] != 0.0
            || self.gust_amp != 0.0
            || self.path.turbulence > 0.0
        {
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
        let vel_before = self.vel;
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
        // The specific force is the ACHIEVED acceleration minus gravity —
        // taken after the ground clamp, so a vehicle resting on the surface
        // senses the normal reaction (−g) rather than a free fall. Exact for
        // this Euler plant: no finite-difference lag. #452.
        for i in 0..3 {
            self.specific_force_ned[i] = (self.vel[i] - vel_before[i]) / self.dt;
        }
        self.specific_force_ned[2] -= GRAVITY;
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
        self.last_collective = collective;
        if self.battery_drain {
            let current = collective; // ∝ total motor power
            self.battery_charge =
                (self.battery_charge - current * 5.0e-5 * self.dt / 0.002).max(0.0);
            self.battery_v = 12.6 + 4.2 * self.battery_charge - 0.3 * current;
        }
    }
    fn dt(&self) -> f32 {
        self.dt
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        if self.battery_lost {
            return None;
        }
        Some(self.battery_v)
    }
    fn read_range(&mut self) -> Option<f32> {
        if !self.range_enabled {
            return None;
        }
        if let Some(f) = self.range_fault {
            return Some(f);
        }
        let agl = -self.pos[2]; // flat terrain at z = 0
        if !(0.1..=40.0).contains(&agl) {
            return None; // outside the TF02 envelope (driver-gated)
        }
        Some(agl + self.range_noise * self.noise_unit())
    }
    fn read_gnss_dual(
        &mut self,
    ) -> Option<(
        Option<falcon_gnss_ubx::dual::NedFix>,
        Option<falcon_gnss_ubx::dual::NedFix>,
    )> {
        use falcon_gnss_ubx::dual::NedFix;
        if !self.gnss_dual_enabled {
            return None;
        }
        let mut base = self.pos;
        let n = self.path.gps_noise;
        base[0] += n * self.noise_unit();
        base[1] += n * self.noise_unit();
        let a = if self.gnss_a_down {
            None
        } else {
            Some(NedFix {
                pos: base,
                acc_m: self.gnss_a_acc,
                sats: 14,
                fix_ok: true,
            })
        };
        let b = if self.gnss_b_down {
            None
        } else {
            let mut pb = base;
            pb[0] += self.gnss_b_offset[0];
            pb[1] += self.gnss_b_offset[1];
            Some(NedFix {
                pos: pb,
                acc_m: self.gnss_b_acc,
                sats: 14,
                fix_ok: true,
            })
        };
        Some((a, b))
    }
    fn read_battery_i(&mut self) -> Option<f32> {
        // Physically consistent with the drain model's sag (0.3·collective V
        // at R = 0.024 Ω): I = 12.5·collective A (25 A at hover). No current
        // sense when the drain model is off — those tests exercise the
        // flagged voltage-only fallback.
        if self.battery_drain {
            Some(12.5 * self.last_collective)
        } else {
            self.battery_i
        }
    }
    fn read_baro(&mut self) -> Option<f32> {
        if self.baro_enabled {
            Some(self.pos[2] + self.baro_bias + self.baro_noise * self.noise_unit()) // NED z, noisy + bias
        } else {
            None
        }
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        if !self.rpm_telemetry {
            return None;
        }
        // ACHIEVED per-rotor RPM from the lagged actual motor state — a failed
        // rotor reads 0 (it was pinned in `write_motors`). This is the FDI's
        // commanded-vs-achieved source; with no failure it tracks the command.
        let mut rpm = [0i32; 4];
        for (r, &s) in rpm.iter_mut().zip(self.motor_state.iter()) {
            *r = (s * ESC_RPM_FULL) as i32;
        }
        Some(rpm)
    }
}

pub mod blackbox_backend;
/// The published seam's supervisor lifecycle rules, written ONCE (#609
/// single-source refactor): `SupervisorHost`.
pub mod host;
/// The cascade-split forwarding (#575): `SplitCore` + what each partition
/// component runs behind the seam.
pub mod split;
pub mod tuning;

// The supervisor and the rotor-out FDI live in their own files; the modules
// stay private and everything is re-exported here, so no public path changed.
mod fdi;
#[cfg(kani)]
mod kani_proofs;
mod supervisor;
pub use fdi::*;
pub use supervisor::*;

#[cfg(test)]
mod blackbox_replay_tests;

#[cfg(test)]
fn falcon_core_tuning_register(store: &mut relay_param::ParamStore<8>) -> bool {
    crate::tuning::register_tuning(store)
}

#[cfg(test)]
mod tests;

/// v1.110 — Monte-Carlo THREE-WAY fail-safe campaigns. Where the point-tests
/// prove ONE runaway terminates and ONE wind gust RTLs, these sweep the
/// dispersion and validate the SAFETY NET itself: inside the recoverable set no
/// fail-safe fires (no false positive), and outside it the CORRECT fail-safe
/// (terminate vs RTL) fires and latches. Each trial drives the full production
/// FlightSupervisor (real IEKF + controller + mixer) via a tilt-injecting
/// backend — the same mechanism as the point-tests, dispersed and reduced to a
/// pass-rate + per-regime counts. Reproducible from (seed, index); companion to
/// the nominal-recovery campaigns in examples/falcon-sitl-gz.
#[cfg(test)]
mod failsafe_campaign;

/// #609 finding 7 / #587 (maintainer decision 2026-10-21): the FDI spin-up
/// guard and the estimator's innovation-gate warm-up are a fixed TIME
/// (the guard 0.3 s since #609 finding 18; the warm-up 0.2 s, and 5x it), counted in ticks of the rate the core RESYNCS to from
/// the backend's own `dt`, not of the nominal 1 kHz it was constructed at.
#[cfg(test)]
mod warmup_time_tests;
