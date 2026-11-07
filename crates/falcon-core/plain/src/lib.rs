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

    /// FDI observability (v1.115): `(rate2, tilt_cos, gate_open, resid[4])` from
    /// the last single-rotor-out detector evaluation. `gate_open` is the
    /// near-level/low-rate gate; when it stays false under sensor noise the
    /// detector never sees the residual — the FAULT-P03 failure mode.
    pub fn fdi_diag(&self) -> (f32, f32, bool, [f32; 4]) {
        self.dbg_fdi
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
    /// RE-ARM the rotor-out FDI for a new spin-up (#609 finding 16): restart the
    /// spin-up guard, forget the previous flight's last motor command and the
    /// rotor lag model's state (the residual's reference, which nothing else
    /// resets on the ground) and clear
    /// the CUSUM accumulators. ONLY while no rotor is isolated: a rotor the FDI
    /// HAS isolated stays isolated, so the `MotorsHealthy` pre-arm row keeps
    /// refusing to arm a vehicle with a dead motor (#565) and the degraded law
    /// stays engaged. Called by the supervisor on every motors-off -> flying
    /// edge.
    pub fn rearm_fdi(&mut self) {
        if self.failed_motor.is_none() {
            self.fdi_spinup = 0;
            self.last_motor_cmd = [0.0; 4];
            self.fdi_lag_model = [0.0; 4];
            self.fdi_stall_run = [0; 4];
            self.fdi_stall_common = false;
            self.fdi = RotorFaultDetector::new(FDI_CUSUM_THRESHOLD, FDI_CUSUM_DRIFT);
        }
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
        // (shared with the FDI below), track or bypass cleanly.
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
        self.fdi_spinup = self.fdi_spinup.saturating_add(1);
        // ROTOR LAG MODEL (#609 finding 18): advance the first-order model of
        // what a healthy rotor reports now, driven by the command that
        // produced this tick's telemetry (`last_motor_cmd`, the v1.115
        // alignment). Every tick, gate open or shut, so it never goes stale.
        let mut k = 0;
        while k < 4 {
            self.fdi_lag_model[k] +=
                (self.last_motor_cmd[k] - self.fdi_lag_model[k]) * self.fdi_lag_alpha;
            k += 1;
        }
        // ATTITUDE/RATE GATE: a rotor-out residual is only DISTINGUISHABLE in
        // near-level, low-rate flight. During an attitude transient the
        // controller commands large asymmetric motors, so |commanded − achieved|
        // spikes on the throttled/saturated rotors and would false-trip the CUSUM
        // — you cannot diagnose a dead rotor while the airframe is tumbling. Only
        // run the detector when the vehicle is roughly level and not spinning
        // (v1.113 — caught a phantom rotor-out during an attitude transient on gz).
        let tilt_cos = 1.0 - 2.0 * (est.q[1] * est.q[1] + est.q[2] * est.q[2]); // R[2][2]
        // ROLL/PITCH rate only — NOT yaw (v1.115, FAULT-P03). The gate exists to
        // avoid diagnosing a dead rotor mid-TUMBLE (a roll/pitch upset spikes
        // the residual on saturated rotors). But a single-rotor-out RELINQUISHES
        // yaw: the body then spins freely about its near-vertical axis at several
        // rad/s — normal, not tumbling. Including yaw here slammed the gate shut
        // for the rest of the flight, so any fault not caught in the brief window
        // before the spin built was NEVER caught (26/300 misses + 43 mis-isolations
        // under heavy sensor noise). Roll/pitch rate stays low through the spin,
        // keeping detection available while still blocking a real tumble.
        let rp_rate2 = gyro_f[0] * gyro_f[0] + gyro_f[1] * gyro_f[1];
        // Rate gate at 2 rad/s (was 1): the REAL gz plant hovers with an
        // attitude limit-cycle near 1 rad/s roll/pitch, which held this gate
        // chronically shut — the FDI then lost the race against the runaway
        // detector on a genuine rotor-out (gz, v1.117: Terminate fired at
        // kill+0.6 s before isolation ever happened). A true tumble is
        // > 3 rad/s, so 2 rad/s keeps the you-cannot-diagnose-mid-tumble
        // purpose; the false-trip protection is carried by the v1.115
        // command-ALIGNED residual, not this gate.
        let fdi_steady = tilt_cos > 0.90 && rp_rate2 < 4.0; // ≲26° tilt, ≲2 rad/s roll+pitch
        let mut dbg_resid = [0.0f32; 4];
        // ── SPIN-UP STALL CHECK (#609 finding 18) ── inside the guard only,
        // and deliberately NOT behind `fdi_steady`: an airborne start shuts
        // that gate before the guard releases. A rotor reading near-zero RPM
        // with its (previous, aligned) command above FDI_STALL_CMD_MIN for
        // FDI_STALL_WINDOW_S unbroken is isolated now. A tick WITHOUT RPM is
        // not evidence: it resets every run (see FDI_STALL_RPM_FRAC).
        //
        // COMMON-MODE REJECTION (review of v1.141, finding 2), the stall
        // path's analogue of the residual path's min-subtraction (#609
        // finding 17). All four runs are updated FIRST, then judged: the
        // loop used to isolate inside the update, so when every rotor read
        // 0 RPM (a telemetry bus up but not yet populated, bidirectional
        // DShot before the ESCs commutate) rotor 0, checked first, was
        // isolated at the window, permanently, and the `MotorsHealthy`
        // pre-arm row then refused every re-arm.
        //
        // THE RULE: a rotor is isolated only while it is the ONLY rotor
        // stalled on this tick. Two or more stalled at once is not a
        // single-rotor fault, and why ">= 2" rather than "all four":
        //  - a multirotor with two dead rotors is not recoverable by the
        //    single-rotor reallocation this isolation engages, so refusing
        //    to isolate one of them loses nothing, while isolating a rotor
        //    on a telemetry fault latches a healthy vehicle as dead;
        //  - "stalled NOW", not "has filled the window": runs start only
        //    once a rotor's command passes FDI_STALL_CMD_MIN, so a shared
        //    stall can be staggered by a few ticks, and the first rotor
        //    would then fill its window alone (and three ESCs commutating
        //    while one is still silent is the same pattern).
        // A real dead rotor's healthy siblings read near zero only for the
        // first few ms of the spin-up (the rotor lag; a 100 ms soft-start
        // for longer), so its own run keeps counting through that co-stall
        // and it is isolated at the window IF every sibling has left the
        // stall band before the window fills
        // (tests/fdi_spinup_guard.rs
        // `a_stalled_rotor_beside_a_slow_healthy_rotor_is_still_isolated_in_the_window`).
        // But a window that FILLS while shared is discarded: from then on
        // every co-stalled tick resets the runs (`fdi_stall_common`), so a
        // rotor left silent when the others recover must earn a fresh
        // window on its own.
        //
        // KNOWN LIMIT (c), a DOUBLE fault (review of v1.140, defect C): a
        // dead rotor whose healthy SIBLING's RPM telemetry stays at 0 past
        // the window (an unpopulated bus slot, a dead telemetry wire) is
        // co-stalled on every tick, so this check never isolates it. Started
        // AIRBORNE it is never isolated at all: the residual path is shut by
        // the rate gate once the guard releases, and the vehicle falls (every
        // dead/silent pair, 250 Hz and 1 kHz; pinned by tests/fdi_spinup_guard.rs
        // `known_limit_c_dead_rotor_beside_a_silent_sibling_is_not_isolated_airborne`).
        // On a ground spin-up the isolation is deferred to the guard's
        // release and the residual path (measured, SimBackend, 50 ms lag:
        // isolated on the guard's own tick, 0.3 s, at 0.26 rad of tilt,
        // peak 1.02 rad, the supervisor already in Land). v1.142: disambiguate co-stalled
        // rotors by the body-rate / attitude response, which a dead rotor
        // produces and a dead telemetry channel does not.
        if self.failed_motor.is_none() && self.fdi_spinup < self.fdi_guard_steps {
            match rpm_now {
                Some(rpm) => {
                    let mut stalled = 0u32;
                    let mut longest = 0u32;
                    let mut i = 0;
                    while i < 4 {
                        let achieved = (rpm[i] as f32 / ESC_RPM_FULL).clamp(0.0, 2.0);
                        if achieved < FDI_STALL_RPM_FRAC
                            && self.last_motor_cmd[i] > FDI_STALL_CMD_MIN
                        {
                            self.fdi_stall_run[i] = self.fdi_stall_run[i].saturating_add(1);
                            stalled += 1;
                            if self.fdi_stall_run[i] > longest {
                                longest = self.fdi_stall_run[i];
                            }
                        } else {
                            self.fdi_stall_run[i] = 0;
                        }
                        i += 1;
                    }
                    if stalled >= 2 {
                        if longest >= self.fdi_stall_steps {
                            self.fdi_stall_common = true;
                        }
                        if self.fdi_stall_common {
                            self.fdi_stall_run = [0; 4];
                        }
                    } else {
                        self.fdi_stall_common = false;
                        if stalled == 1 && longest >= self.fdi_stall_steps {
                            let mut j = 0;
                            while j < 4 {
                                if self.fdi_stall_run[j] > 0 {
                                    self.failed_motor = Some(j);
                                }
                                j += 1;
                            }
                        }
                    }
                }
                None => {
                    self.fdi_stall_run = [0; 4];
                    self.fdi_stall_common = false;
                }
            }
        }
        if self.failed_motor.is_none() && self.fdi_spinup >= self.fdi_guard_steps && fdi_steady {
            if let Some(rpm) = rpm_now {
                let mut resid = [0.0f32; 4];
                let mut i = 0;
                while i < 4 {
                    let achieved = (rpm[i] as f32 / ESC_RPM_FULL).clamp(0.0, 2.0);
                    // Compare achieved against the LAG-MODELLED command (#609
                    // finding 18): the previous command that produced it
                    // (v1.115 — the telemetry lags one tick, so the current
                    // command spikes every residual on an abrupt step), passed
                    // through FDI_ROTOR_LAG_TAU_S. A healthy rotor still
                    // catching up reads ~0; a dead one reads the model.
                    resid[i] = (self.fdi_lag_model[i] - achieved).abs();
                    i += 1;
                }
                // COMMON-MODE REJECTION (#609 finding 17): subtract what all
                // four rotors share. The command-aligned residual above assumes
                // ONE tick of telemetry lag; a real ESC/rotor lags by a
                // first-order time constant of tens of ms, so any abrupt
                // COLLECTIVE step leaves all four behind their commands by the
                // same amount. The supervised landing makes that step when
                // the velocity law engages (hover ~0.5 -> ~0.3): at 50 ms lag
                // every residual read 0.19, equal to 1e-6, the CUSUM crossed
                // on tick 7 and rotor 0 was isolated only because it is
                // checked first. A single rotor out is DIFFERENTIAL (one
                // residual rises, three stay near zero), so removing the
                // per-tick minimum leaves it intact: Loiter isolation stays
                // at 3 ticks; isolation at the landing step goes 4 -> 8-12
                // ticks (tests/fdi_common_mode.rs).
                //
                // KEPT with the lag model (#609 finding 18): the model is
                // one assumed time constant, so a fleet of ESCs lagging
                // faster or slower than it, all alike, still leaves a shared
                // residual on every collective step, and this removes it.
                let common = resid[0].min(resid[1]).min(resid[2]).min(resid[3]);
                for r in resid.iter_mut() {
                    *r -= common;
                }
                dbg_resid = resid;
                self.failed_motor = self.fdi.update(resid);
            }
        }
        self.dbg_fdi = (rp_rate2, tilt_cos, fdi_steady, dbg_resid);
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

    /// The rotor-out FDI spin-up guard at `loop_hz`: [`FDI_SPINUP_GUARD_S`]
    /// of ticks, ROUNDED like [`Self::warmup_steps`] (1/(1/1000) is
    /// 999.99994 in f32), same >= 10 floor (#609 finding 18). ONE definition
    /// for the fused core and both halves of a split.
    pub fn fdi_guard_steps(loop_hz: f32) -> u32 {
        ((loop_hz * FDI_SPINUP_GUARD_S + 0.5) as u32).max(10)
    }

    /// The stall check's window at `loop_hz`: [`FDI_STALL_WINDOW_S`] of
    /// ticks, ROUNDED, at least 1 (#609 finding 18). One definition for the
    /// fused core and both halves of a split.
    pub fn fdi_stall_steps(loop_hz: f32) -> u32 {
        ((loop_hz * FDI_STALL_WINDOW_S + 0.5) as u32).max(1)
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

    /// The rotor lag model's per-tick gain at `loop_hz`:
    /// `(1 / loop_hz) / FDI_ROTOR_LAG_TAU_S`, capped at 1 (#609 finding 18).
    /// Forward Euler, no libm on the flight path.
    pub fn fdi_lag_alpha(loop_hz: f32) -> f32 {
        ((1.0 / loop_hz) / FDI_ROTOR_LAG_TAU_S).min(1.0)
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

/// How long a FOLLOW target (see [`MissionSource::Follow`]) stays in force
/// without a fresh one before the stream is STALE (#609 finding 4, maintainer
/// decision 2026-10-28: the published component's frame target is a live
/// setpoint). 1.0 s: ten times the longest frame period the component accepts
/// (its dt clamp tops out at 100 ms), so at least ten consecutive frames must
/// all carry an unusable target; and short enough that a dead stream is not
/// flown on. The last good target is a fixed POINT, so in the worst case the
/// vehicle spends one more second closing on where the host last asked.
/// Enforced by the verified `relay_offboard::OffboardReceiver` (OFFB-P01,
/// Kani OFFB-K02: strictly past the timeout the stream is ALWAYS stale).
pub const FOLLOW_TARGET_TIMEOUT_US: u64 = 1_000_000;

/// BRAKE, THEN LATCH (#609 finding 4, Decision B as re-decided by the maintainer
/// 2026-10-28; the position integral is left untouched).
///
/// A Loiter entered while the vehicle is moving faster than
/// [`HOLD_BRAKE_ENTRY_MS`] (Mission -> Loiter mid-leg, or the stale-follow HOLD)
/// does not latch on its first tick. It BRAKES: the horizontal setpoint each
/// tick is `p - v * HOLD_BRAKE_LEAD_S`, a point behind the vehicle along its own
/// velocity, so the position term adds `kp * LEAD * v` of braking to the
/// damping term (0.08 * 8 = 0.64 on top of kd 0.6: roughly double), and the
/// NEGATIVE error that point presents unwinds, through the loop's own integral
/// law, the integral the leg charged toward its waypoint. The hold point is
/// latched only once the estimated horizontal speed has stayed under
/// [`HOLD_REST_MS`] for [`HOLD_REST_S`]: at sustained rest there is neither
/// error nor damping force, so whatever integral remains is the steady-wind
/// compensation (WIND-P01), and latching there leaves nothing to overshoot with.
/// [`HOLD_BRAKE_MAX_S`] bounds the braking phase: past it the point is latched
/// wherever the vehicle is.
///
/// WHY THIS AND NOT A PREDICTED STOPPING POINT, measured on `SimBackend`
/// (#616's (3c) leg, 3.4 m/s at the command, 2 and 5 m/s wind): latching at
/// the kinematic stop `p + v|v| / (2 a_cmd_max)` still overshot 11.6-11.9 m and
/// left 2.5-2.7 m in HOLD-P01's window, because the leg-charged integral (up to
/// ~1.4 m/s^2 of push against a 1.0 m/s^2 command limit) keeps driving the
/// vehicle on past ANY point latched while it is charged. A pure-damping brake
/// (setpoint = the current estimate) never stops for the same reason, and
/// drifted into the 50 m fence in 17 s. The rest criterion is what makes this
/// robust: a lead of 6-16 s all passed HOLD-P01 with it, while a 0.5 s rest
/// window latched at oscillation turning points and failed at some leads.
pub const HOLD_BRAKE_LEAD_S: f32 = 8.0;
/// Entering Loiter faster than this (m/s, estimated, horizontal) brakes first.
pub const HOLD_BRAKE_ENTRY_MS: f32 = 0.3;
/// "At rest" for the brake-then-latch rule (m/s, estimated, horizontal).
pub const HOLD_REST_MS: f32 = 0.1;
/// How long the vehicle must stay at rest before the hold point is latched (s).
pub const HOLD_REST_S: f32 = 3.0;
/// Longest braking phase before the hold point is latched regardless (s).
pub const HOLD_BRAKE_MAX_S: f32 = 30.0;

/// Where Mission mode takes its setpoint from (#609 finding 4).
///
/// The two are DIFFERENT CONTRACTS and deliberately do not share a code path:
///
///   * `Waypoints` (the default, every native user): the stored mission
///     (`set_mission` / `set_mission_waypoints`, MISSION-P01). Arriving at a leg
///     advances to the next; arriving at the LAST leg ends the sortie with an
///     autonomous RTL + land. Unchanged.
///   * `Follow` (the published component): an offboard-style LIVE setpoint
///     ([`FlightSupervisor::set_follow_target`]). The latest fresh target is
///     the setpoint every tick; REACHING IT ENDS NOTHING (the vehicle holds it,
///     in Mission, for as long as the host keeps sending it). The host ends
///     Mission explicitly with RequestRtl / RequestLand / RequestLoiter. A
///     stream with no fresh target for [`FOLLOW_TARGET_TIMEOUT_US`] (never
///     started, stopped, or only non-finite targets) is STALE, and the
///     supervisor takes `relay_fsafe`'s policy action for a stale offboard
///     stream, HOLD, which it flies as RequestLoiter: a Loiter at the point the
///     vehicle had reached, latched on entry like every Loiter. A stream that
///     resumes does NOT re-enter Mission by itself; the host sends
///     RequestMission again.
///
/// Why not one path with a flag: the component used to feed its target
/// through `set_mission`, which RESET a one-leg mission every tick, so arrival
/// fired the mission-complete RTL and the vehicle landed while the host was
/// still streaming the target. Bending the waypoint sequencer into a "never
/// finish" mode would leave the follow path with no freshness contract at all;
/// a separate source gets the verified timeout instead, and MISSION-P01's
/// sequencer stays exactly as it was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionSource {
    Waypoints,
    Follow,
}

/// Nominal ESC RPM at full throttle — normalises reported RPM to a per-rotor
/// EFFECTIVENESS (≈ commanded throttle when healthy) for the single-rotor-out FDI
/// residual (v1.103). Only the ratio matters; a per-airframe value scales it.
const ESC_RPM_FULL: f32 = 8000.0;

/// Rotor-out FDI CUSUM level and per-step drift, matching the SITL-verified
/// fault-tolerance chain. Named so a re-armed detector is built identically
/// to the original (#609 finding 16).
const FDI_CUSUM_THRESHOLD: f32 = 0.5;
const FDI_CUSUM_DRIFT: f32 = 0.1;

/// The rotor-out FDI SPIN-UP GUARD, in seconds after every motors-off ->
/// flying edge (#609 finding 18, maintainer decision 2026-11-04 "T300+M50").
/// A fixed TIME, counted in ticks of the RESYNCED loop rate and rounded
/// ([`FlightCore::fdi_guard_steps`]), exactly like finding 7's warm-up.
///
/// It was 0.2 s (the shared [`FlightCore::warmup_steps`]). One ESC whose
/// output ramps up more slowly than the other three (a soft-start) leaves a
/// DIFFERENTIAL residual that finding 17's common-mode subtraction cannot
/// remove, and a 0.2 s guard released while it was still large: measured on
/// SimBackend, 48/60/76 of 192 (ramp x lag x rotor) spin-ups at
/// 250/500/1000 Hz isolated the slow, HEALTHY rotor. With this guard and the
/// lag model below: 4/8/12, every one of them a 300 ms ramp (the named
/// known limit in `tests/fdi_spinup_guard.rs`).
///
/// THE CEILING IS CLOSE. A rotor dead from the first tick is caught when
/// the guard releases, so the guard IS its detection latency: at 0.3 s it is
/// isolated in <= 0.3 s with tilt <= 0.33 rad; at 0.4 s the vehicle flips
/// before isolation. The margin to that ceiling is ~0.1 s on the GROUND
/// (SimBackend's ground contact holds the airframe; not checked on gz or
/// hardware); do not lengthen this without re-measuring the dead-at-spin-up
/// case. AIRBORNE the margin is NEGATIVE: a core started hovering with a
/// rotor dead from tick 1 shuts the 2 rad/s roll/pitch rate gate at
/// ~0.26 s, before 0.3 s releases the guard. For a FULLY stalled rotor
/// that is covered by the spin-up stall check ([`FDI_STALL_RPM_FRAC`]),
/// which isolates it in ~0.1 s inside the guard; a rotor that spins with
/// reduced effectiveness from an airborne start is not, and still waits
/// for this guard.
pub const FDI_SPINUP_GUARD_S: f32 = 0.3;

/// First-order time constant (s) of the ROTOR LAG MODEL the FDI residual is
/// computed against (#609 finding 18). The residual compares the achieved
/// effectiveness (ESC RPM / [`ESC_RPM_FULL`]) with the command passed
/// through `y += (u - y) * dt / tau` (forward Euler, `dt / tau` capped at 1),
/// instead of with the raw previous command: a healthy rotor tracking its
/// command with this lag then reads ~0 even while it is still catching up.
///
/// ASSUMPTION, NOT A MEASUREMENT. 50 ms is the lag the SimBackend sweep was
/// run at; it is not derived from the first vehicle's ESCs. Replace it from
/// real ESC data for the BOM's ESCs: the spin-UP and spin-DOWN time
/// constants (they differ; this model uses one), the ESC soft-start profile,
/// the unit-to-unit spread, and the RPM telemetry latency. A plant slower
/// than the model leaves a residual on every transient (removed by the
/// common-mode subtraction only when all four lag alike); a faster one
/// leaves the model behind instead. Not a configuration field on purpose:
/// it would be another integrator-facing seam change.
pub const FDI_ROTOR_LAG_TAU_S: f32 = 0.05;

/// SPIN-UP STALL CHECK (#609 finding 18, maintainer decision 2026-11-04
/// "(c)"): DURING the spin-up guard only, a rotor whose achieved
/// effectiveness (ESC RPM / [`ESC_RPM_FULL`]) stays below this fraction
/// while its command is above [`FDI_STALL_CMD_MIN`] for
/// [`FDI_STALL_WINDOW_S`] of consecutive ticks is isolated at once, without
/// waiting for the guard. A rotor dead from the first tick is then caught
/// in ~0.1 s (tilt ~0.05 rad) instead of 0.3 s, and an AIRBORNE start, whose
/// roll/pitch rate gate shuts at ~0.26 s, is caught before that (it never
/// was with the 0.3 s guard alone).
///
/// ASSUMPTION, NOT A MEASUREMENT, like [`FDI_ROTOR_LAG_TAU_S`]. 5 % / 20 % /
/// 100 ms were chosen from the SimBackend sweep: the longest run any
/// HEALTHY rotor spent below 5 % with its command above 20 % was 68 ms (a
/// 300 ms ESC soft-start behind an 80 ms lag), so 100 ms leaves 32 ms.
/// Replace them from BOM ESC startup data. THE RISK: a SENSORLESS ESC can
/// report zero RPM, or no RPM at all, while it aligns and starts
/// commutating, and that phase can outlast 100 ms at low throttle; a real
/// ESC doing so would be isolated as dead on a healthy spin-up.
///
/// WHAT IT CATCHES: only a FULLY stalled rotor (RPM ~ 0). A rotor that
/// spins with reduced effectiveness at spin-up is left to the residual and
/// still waits for the guard ([`FDI_SPINUP_GUARD_S`]).
///
/// MISSING RPM DOES NOT COUNT: a tick with no RPM telemetry (the seam's
/// `motor-rpm: none`) is not evidence of a stall. It RESETS every rotor's run
/// to zero, so only an unbroken window of present, near-zero readings can
/// isolate; a backend without RPM never trips this.
///
/// A SHARED STALL DOES NOT COUNT EITHER (review of v1.141, finding 2): a
/// rotor is isolated only while it is the only rotor stalled on that tick.
/// Two or more stalled at once (zero RPM on every rotor from a telemetry bus
/// that is up but not populated, or ESCs that have not commutated yet) is
/// not a single-rotor fault and isolates nothing; see the stall check in
/// `CascadePartition::step` for the rule and its justification.
pub const FDI_STALL_RPM_FRAC: f32 = 0.05;
/// The command a rotor must be above for its near-zero RPM to count as a
/// stall (#609 finding 18; see [`FDI_STALL_RPM_FRAC`]). Below it, a low RPM
/// is just a low command. Assumption pending BOM ESC startup data.
pub const FDI_STALL_CMD_MIN: f32 = 0.20;
/// The unbroken time a rotor must read stalled to be isolated (#609 finding
/// 18; see [`FDI_STALL_RPM_FRAC`]), rounded to ticks of the resynced rate.
/// Assumption pending BOM ESC startup data.
pub const FDI_STALL_WINDOW_S: f32 = 0.1;

/// Thrust floor for the reconfigured (single-rotor-out) allocator. With the
/// rank-3 allocation (v1.114) the diagonal-opposite of the failed rotor
/// legitimately rests at 0 — that couple-cancelling zero is what gives the
/// pair a PARASITIC-FREE pure-lift axis — so the floor is 0: a positive floor
/// would clamp that rotor up and reinject the very moment the allocation
/// removes (and, being below the desaturation base, would collapse all
/// roll/pitch authority to zero). Tilt authority now comes from the solved
/// allocation, not a floor.
const ROTOR_OUT_FLOOR: f32 = 0.0;

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

/// The setpoint regimes whose horizontal target is LATCHED on entry rather than
/// re-derived each tick (#609 finding 4, maintainer decision 2026-10-28:
/// "latch on entry", PX4-like). The latch is captured from the estimate on the
/// first tick a regime is derived, and held until the regime changes.
///
///   * `Hold` = Takeoff or Loiter. Armed -> Takeoff captures the launch point,
///     so the climb is flown over it. Takeoff -> Loiter is the SAME regime and
///     keeps that point: re-latching at the top of the climb would bake in
///     whatever the wind did during it. Mission -> Loiter (RequestLoiter, or
///     ReachedHome) enters the regime afresh: BRAKE, THEN LATCH (maintainer
///     re-decision 2026-10-28) -- see [`HOLD_BRAKE_LEAD_S`]. A vehicle entering
///     Loiter while moving brakes first and latches where it actually comes to
///     rest ("stop here"), not the point it was passing when the command
///     arrived, not the waypoint and not the takeoff point. One already at rest
///     latches at once.
///     Rtl -> Loiter and Land -> Loiter do not exist in `relay_fsm`; every
///     failsafe goes to Rtl / Land / Terminated, which leave only through
///     Touchdown -> Disarmed, so Loiter after a failsafe is always reached
///     through a fresh Armed -> Takeoff, which latches its own launch point.
///   * `None` = every other mode: Mission flies its leg, Rtl and the healthy
///     Land fly home, the motors-off states command nothing, and the ROTOR-OUT
///     Land keeps commanding the CURRENT estimate (zero horizontal demand).
///
/// WHY ROTOR-OUT LAND IS NOT LATCHED, measured rather than argued (2026-10-28,
/// `SimBackend`, `rot_drag` 0.02, each rotor 0-3 killed from a settled Loiter
/// under 5 m/s wind). From 2 m the descent takes ~3.7 s and the two policies
/// are indistinguishable (touchdown 0.54-0.72 m from the isolation point
/// either way, peak tilt within 0.004 rad). From 10 m (~17-18 s of descent)
/// latching is WORSE on every rotor: touchdown 9.9-39.2 m from the isolation
/// point against 3.5-26.1 m, peak tilt 0.21-0.29 rad against 0.13-0.25 rad. A
/// spinning, yaw-less 3-rotor airframe cannot fly a position correction
/// cleanly; asking it to fight the wind back to a point makes it tilt harder
/// and land further away. So rotor-out Land stays "level up and come down
/// here", and the latch is confined to the healthy hold modes.
///
/// The frame's `target` in the published component is read only in Mission
/// (a live follow setpoint, #609 finding 4); it never moves this point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HoldRegime {
    None,
    Hold,
}

/// WHAT A [`FlightSupervisor`] NEEDS FROM THE CORE IT SUPERVISES (#575).
///
/// The supervisor used to own a concrete [`FlightCore`] and call it directly,
/// so it could only ever supervise a fused core in its own address space. The
/// split deployment (`cascade-split`, #575) runs the two partitions in two
/// separate components joined by the `nav-state` seam, and the supervisor in
/// a third. It still has to drive them, with the mode machine, the failsafe
/// arbiter and the pre-arm rows intact, or the published split component is
/// the stabiliser #414 was filed about.
///
/// So this trait is EXACTLY the set of calls `FlightSupervisor` makes on its
/// core. Nothing was added for convenience: every method maps to a `self.core.*`
/// call site that existed before the trait did. [`FlightCore`] implements it by
/// pure delegation to its own inherent methods, so a `FlightSupervisor` (the
/// default `C = FlightCore`) runs the same code it always ran.
///
/// An implementation over remote halves must preserve what
/// [`FlightCore::step`] and [`FlightCore::step_estimate_only`] do, including
/// the ONE-TICK-DELAYED `flying` edge from the control half back into the
/// estimator (#452) and its ABSENCE on estimate-only ticks. Losing either
/// breaks flight, not the build.
pub trait SupervisedCore {
    /// The estimator's current navigation state.
    fn state(&self) -> NavState;
    /// See [`EstimatorPartition::tilt_uncertainty`].
    fn tilt_uncertainty(&self) -> f32;
    /// The calibration the estimator applies to raw samples.
    fn calibration(&self) -> relay_calib::CalParams;
    /// Install a calibration on EVERY half that applies one — the estimator
    /// AND the control partition, which de-biases its own copy of the gyro
    /// stream (see [`FlightCore::set_calibration`]). Forwarding it to only one
    /// half makes the two disagree about what the gyro read.
    fn set_calibration(&mut self, calib: relay_calib::CalParams);
    /// See [`EstimatorPartition::nav_compromised`].
    fn nav_compromised(&self) -> bool;
    /// See [`EstimatorPartition::gnss_diverged`].
    fn gnss_diverged(&self) -> bool;
    /// See [`CascadePartition::failed_motor`].
    fn failed_motor(&self) -> Option<usize>;
    /// See [`CascadePartition::max_motor`].
    fn max_motor(&self) -> f32;
    /// See [`CascadePartition::set_landing`].
    fn set_landing(&mut self, on: bool);
    /// See [`CascadePartition::set_position`].
    fn set_position(&mut self, ned: Vec3);
    /// See [`CascadePartition::rearm_fdi`] (#609 finding 16). No default body
    /// on purpose: a core that silently did not re-arm would fly its second
    /// spin-up with no FDI guard, the defect this hook fixes.
    fn rearm_fdi(&mut self);
    /// One control tick — the semantics of [`FlightCore::step`].
    fn step<B: FlightBackend>(&mut self, b: &mut B);
    /// One estimate-only tick, motors commanded zero — the semantics of
    /// [`FlightCore::step_estimate_only`].
    fn step_estimate_only<B: FlightBackend>(&mut self, b: &mut B);
}

/// The fused core: pure delegation to the inherent methods, so the default
/// `FlightSupervisor` (= `FlightSupervisor<FlightCore>`) is unchanged.
impl SupervisedCore for FlightCore {
    fn state(&self) -> NavState {
        FlightCore::state(self)
    }
    fn tilt_uncertainty(&self) -> f32 {
        FlightCore::tilt_uncertainty(self)
    }
    fn calibration(&self) -> relay_calib::CalParams {
        FlightCore::calibration(self)
    }
    fn set_calibration(&mut self, calib: relay_calib::CalParams) {
        FlightCore::set_calibration(self, calib)
    }
    fn nav_compromised(&self) -> bool {
        FlightCore::nav_compromised(self)
    }
    fn gnss_diverged(&self) -> bool {
        FlightCore::gnss_diverged(self)
    }
    fn failed_motor(&self) -> Option<usize> {
        FlightCore::failed_motor(self)
    }
    fn max_motor(&self) -> f32 {
        FlightCore::max_motor(self)
    }
    fn set_landing(&mut self, on: bool) {
        FlightCore::set_landing(self, on)
    }
    fn set_position(&mut self, ned: Vec3) {
        FlightCore::set_position(self, ned)
    }
    fn rearm_fdi(&mut self) {
        FlightCore::rearm_fdi(self)
    }
    fn step<B: FlightBackend>(&mut self, b: &mut B) {
        FlightCore::step(self, b)
    }
    fn step_estimate_only<B: FlightBackend>(&mut self, b: &mut B) {
        FlightCore::step_estimate_only(self, b)
    }
}

/// The construction parameters `FlightSupervisor::new` gives its core:
/// `FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ)`.
///
/// `pub` so a split deployment (#575) builds its two halves from the SAME
/// numbers rather than a copy of them. The loop rate is a nominal design rate:
/// both partitions' `resync_loop_rate` re-derive the filters AND the warm-up
/// counts from the backend's real `dt` on the first tick (#609 finding 7), by
/// one shared rule, so the halves engage their FDI and innovation gates on the
/// same tick as the fused core whatever rate the frames run at.
pub const SUPERVISOR_CORE_HOVER_THRUST: f32 = 0.5;
/// See [`SUPERVISOR_CORE_HOVER_THRUST`].
pub const SUPERVISOR_CORE_LOOP_HZ: f32 = 1000.0;

pub struct FlightSupervisor<C = FlightCore> {
    core: C,
    fsm: relay_fsm::FlightFsm,
    home: Vec3,
    fence_radius: f32,
    cruise_alt: f32,
    /// The verified battery estimator (BATTERY-P02, v1.120): coulomb count +
    /// sag compensation with current sense, flagged voltage-only fallback
    /// without. Its debounced, latched flags — not raw voltage — drive the
    /// low-battery failsafe and the pre-arm battery check.
    batt_est: relay_batt::BatteryEstimator,
    /// The latest battery state, refreshed every step (telemetry seam:
    /// SYS_STATUS battery fields + degraded flag come from here).
    batt_state: relay_batt::BattState,
    /// Did this backend supply a battery voltage on the last tick? `false`
    /// means ABSENT, and the BatteryOk pre-arm row fails accordingly (#413).
    batt_present: bool,
    /// Seconds since the last battery reading ARRIVED. A pack that stops
    /// reporting in flight is not a healthy pack: the estimator stops being
    /// updated, so `batt_state`'s low/critical latches freeze at their last
    /// value and the failsafe below could never fire (#413's in-flight half,
    /// found in the v1.139 candidate review).
    batt_missing_s: f32,
    /// Whether a battery reading has EVER arrived. Silence only counts as a
    /// failsafe once the vehicle has shown it has a sense to lose — the same
    /// declare-on-first-sight rule the ESC-telemetry pre-arm row uses. A rig
    /// that never had battery monitoring (the SITL backends report `None`)
    /// must not be failsafed a second after takeoff; refusing to ARM without
    /// a pack is #413's half and belongs to pre-arm, not to the air.
    batt_seen: bool,
    /// Stored mission legs (NED), flown in order while in Mission mode.
    waypoints: [Vec3; MAX_WAYPOINTS],
    wp_count: usize,
    wp_index: usize,
    /// Keep-out zones the position setpoint is deflected around (v1.31).
    zones: [KeepoutZone; MAX_KEEPOUT_ZONES],
    zone_count: usize,
    rtl_latched: bool,
    /// The horizontal hold point (NED north, east) Takeoff and Loiter fly to
    /// (#609 finding 4). LATCHED ON ENTRY from the estimate, never recomputed
    /// from it: a setpoint re-derived from the current estimate every tick has
    /// zero horizontal error by construction, so nothing held position against
    /// wind. See [`HoldRegime`] for exactly when it is (re-)captured.
    hold_xy: [f32; 2],
    /// Which latched setpoint regime the previous tick's setpoint was derived
    /// for. A change of regime is how entry is detected, whichever path the
    /// mode changed by (a `command` between ticks, or a failsafe / milestone
    /// inside `step`).
    hold_regime: HoldRegime,
    /// BRAKE, THEN LATCH: true while a moving Loiter entry is still braking and
    /// `hold_xy` is the brake point, not yet the latched hold point.
    hold_braking: bool,
    /// The previous tick commanded motors OFF (Disarmed / Armed / Terminated).
    /// The next flying tick is a spin-up and re-arms the FDI (#609 finding 16).
    motors_were_off: bool,
    /// Time spent braking, and time continuously at rest within it (s).
    brake_s: f32,
    brake_rest_s: f32,
    /// Where Mission takes its setpoint from. See [`MissionSource`].
    mission_source: MissionSource,
    /// Freshness of the FOLLOW target stream (OFFB-P01's verified receiver).
    follow: relay_offboard::OffboardReceiver,
    /// Monotonic counter handed to `follow` with each accepted target. The
    /// receiver rejects a non-increasing counter (replay); here every accepted
    /// call is a new sample, so it simply increments.
    follow_counter: u64,
    /// Supervisor time (µs), advanced by the backend's `dt` at the top of every
    /// `step`. The follow-stream timeout is measured on it.
    clock_us: u64,
    /// Consecutive cycles the estimated tilt has exceeded TILT_RUNAWAY_LIMIT
    /// while airborne — the attitude-runaway detector (v1.99 expanded failsafe).
    /// On reaching TILT_RUNAWAY_CYCLES it fires Event::Terminate (motor cut).
    runaway_count: u32,
    /// Consecutive cycles of control saturation + moderate tilt while airborne —
    /// the high-wind detector (v1.101). On reaching WIND_CYCLES it commands RTL
    /// (the vehicle is fighting a disturbance beyond its control authority).
    wind_count: u32,
    /// The PREARM-P03 check TABLE (v1.122): the legacy six checks mirrored
    /// as rows 0..=5 plus the breadth rows (estimator/nav integrity, config
    /// consistency, hardware consistency, safety state). Self-populatable
    /// rows are refreshed every step; integration-fed rows (RC/GCS link,
    /// params-from-NVM, sensor freshness, …) enter via [`set_check`] —
    /// setting one DECLARES it required from then on.
    ///
    /// THE ONLY PRE-ARM VERDICT (#609, single-source refactor). Until then a
    /// legacy six-field `relay_preflight::PreflightChecks` lived beside this
    /// table (since fe29d37, 2026-07-16) with its own `arm_check` verdict, and
    /// the two disagreed in both directions (#609 finding 6). Arming, failsafe
    /// bit 7 and [`FlightSupervisor::arm_blocked_check`] all read this table.
    check_table: relay_preflight::CheckTable,
    /// A VALID calibration has been explicitly installed through
    /// [`set_calibration`](Self::set_calibration) and not since revoked by an
    /// invalid one. THIS, not "the calibration differs from the identity", is
    /// what `update_preflight` reports as `CalibrationPresent` (jess#167,
    /// #611): per-sensor calibration is applied below this seam, so an
    /// explicitly configured IDENTITY is a valid calibration meaning
    /// "calibrated upstream, no residual". Never configured => false.
    calibration_configured: bool,
}

/// How long a battery may say NOTHING before it counts as lost, in seconds.
///
/// HOISTED OUT OF `FlightSupervisor::step` (was a local `const` there) so
/// `failsafe_flags` reports the SAME threshold the failsafe acts on. A second
/// copy of `1.0` would have been a drift hazard of exactly the kind this crate
/// keeps finding: an observer that disagrees with the thing it observes is worse
/// than no observer, because it is believed.
pub const BATT_SILENCE_S: f32 = 1.0;

/// Bit positions for [`FlightSupervisor::failsafe_flags`].
///
/// WHY A BITMASK AND NOT AN ENUM: several causes are active at once — a flat pack
/// inside a geofence breach is two, and the supervisor's own `breach` term
/// deliberately ORs geofence with battery, so an enum would have to pick a winner
/// and discard the rest. It is also the shape the published seam needs
/// (SUPERVISOR-P01): fixed width, no `option<>`, safe to read through a seqlock.
pub mod failsafe_flags {
    /// The supervisor has latched an RTL. Absorbing until disarm.
    pub const RTL_LATCHED: u32 = 1 << 0;
    /// Pack voltage/SoC is below the LOW threshold.
    pub const BATT_LOW: u32 = 1 << 1;
    /// Pack is below the CRITICAL threshold.
    pub const BATT_CRITICAL: u32 = 1 << 2;
    /// A pack that HAD been reporting has gone silent for longer than
    /// [`super::BATT_SILENCE_S`]. Distinct from "never had a battery sense",
    /// which is NOT a failsafe (#413/#475: declare on first sight).
    pub const BATT_SILENT: u32 = 1 << 3;
    /// Estimated position is outside the geofence radius.
    pub const GEOFENCE_BREACH: u32 = 1 << 4;
    /// Tilt is over the runaway limit and the terminate counter is ARMING.
    /// Reaching the threshold produces `Mode::Terminated`, which the mode field
    /// reports; this bit is the approach to it.
    pub const RUNAWAY_ARMING: u32 = 1 << 5;
    /// Motors saturated at a sustained tilt — the high-wind counter is ARMING.
    pub const WIND_ARMING: u32 = 1 << 6;
    /// A pre-arm row is failing, so arming is refused.
    pub const ARM_BLOCKED: u32 = 1 << 7;
    /// Mission takes a FOLLOW target ([`super::MissionSource::Follow`], the
    /// published component) and no fresh one is in force: never started, or
    /// none accepted for [`super::FOLLOW_TARGET_TIMEOUT_US`]. In Mission this is
    /// the cause of the HOLD (Loiter) the supervisor takes (#609 finding 4).
    pub const TARGET_STALE: u32 = 1 << 8;
}

impl FlightSupervisor<FlightCore> {
    /// `home` NED, `fence_radius` m (breach ⇒ RTL), `cruise_alt` m AGL,
    /// `low_batt_v` V (below ⇒ failsafe). Supervises a fused [`FlightCore`].
    pub fn new(home: Vec3, fence_radius: f32, cruise_alt: f32, low_batt_v: f32) -> Self {
        Self::with_core(
            FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ),
            home,
            fence_radius,
            cruise_alt,
            low_batt_v,
        )
    }

    /// Set the hover-thrust feedforward on the wrapped core (per-plant: the
    /// analytic sim hovers at ~0.49, the real gz falcon-quad at ~0.585).
    pub fn set_hover_thrust(&mut self, t: f32) {
        self.core.casc.hover_thrust = if t.is_finite() {
            t.clamp(0.0, 1.0)
        } else {
            0.5
        };
    }
}

impl<C: SupervisedCore> FlightSupervisor<C> {
    /// Severity rank of a mode, for failsafe arbitration (#562).
    ///
    /// A `Failsafe` event must only ESCALATE. The FSM's `(Rtl, Failsafe) =>
    /// Land` is right for a WORSE condition and wrong for the SAME one, and
    /// every trigger here is LEVEL-triggered: a vehicle flying home is, by
    /// definition, still outside the fence, so the breach re-sent `Failsafe`
    /// every tick and RTL was demoted to land-where-you-are after a single
    /// 4 ms tick -- landing OUTSIDE the fence it was recovering from. The
    /// battery failsafe shares that expression and `relay-batt`'s flags latch,
    /// so it was equally persistent.
    ///
    /// Ranks let each send ask "would this escalate?" instead of "is the
    /// condition true?". This is `relay-fsafe`'s most-severe-wins + monotone
    /// latch semantics applied at the send; that crate is not a drop-in
    /// replacement because its `Triggers` has no runaway, wind, motor-out or
    /// battery-silence field and its policy cannot reach `Terminate`.
    fn mode_rank(m: relay_fsm::Mode) -> u8 {
        match m {
            relay_fsm::Mode::Terminated => 4,
            relay_fsm::Mode::Land => 3,
            relay_fsm::Mode::Rtl => 2,
            _ => 1,
        }
    }

    /// Supervise an arbitrary [`SupervisedCore`] — the split deployment's
    /// remote estimator/control pair (#575). [`FlightSupervisor::new`] is this
    /// with a fused [`FlightCore`]; every other field is initialised here, once,
    /// so the two constructors cannot drift.
    ///
    /// `inline(always)` because the published component links an 8 KiB shadow
    /// stack and a by-value `FlightCore` already trapped there once; inlining
    /// lets `new`'s core be built straight into the supervisor.
    #[inline(always)]
    pub fn with_core(
        core: C,
        home: Vec3,
        fence_radius: f32,
        cruise_alt: f32,
        low_batt_v: f32,
    ) -> Self {
        FlightSupervisor {
            core,
            fsm: relay_fsm::FlightFsm::new(),
            home,
            fence_radius,
            cruise_alt,
            // Fallback thresholds preserve the constructor's contract: the
            // configured low_batt_v is the (4S) voltage-only LOW threshold.
            batt_est: relay_batt::BatteryEstimator::new(relay_batt::BattConfig {
                low_v_cell_fallback: low_batt_v / 4.0,
                crit_v_cell_fallback: low_batt_v / 4.0 - 0.15,
                ..relay_batt::BattConfig::default()
            }),
            batt_state: relay_batt::BattState::default(),
            // Starts false: nothing has been read yet, so nothing is known.
            batt_present: false,
            batt_missing_s: 0.0,
            batt_seen: false,
            waypoints: [home; MAX_WAYPOINTS],
            wp_count: 0,
            wp_index: 0,
            zones: [KeepoutZone {
                center: home,
                radius: 0.0,
            }; MAX_KEEPOUT_ZONES],
            zone_count: 0,
            rtl_latched: false,
            hold_xy: [home[0], home[1]],
            hold_regime: HoldRegime::None,
            hold_braking: false,
            motors_were_off: true,
            brake_s: 0.0,
            brake_rest_s: 0.0,
            mission_source: MissionSource::Waypoints,
            follow: relay_offboard::OffboardReceiver::new(FOLLOW_TARGET_TIMEOUT_US),
            follow_counter: 0,
            clock_us: 0,
            runaway_count: 0,
            wind_count: 0,
            check_table: {
                // back-compat: the always-required six rows start PASSING —
                // a supervisor that has never stepped can still be armed by
                // the bring-up tests; the first update_preflight refreshes
                // every row it can measure with real signals. A host that
                // must not arm before a measured step declares those rows
                // false itself (#609 finding 2).
                let mut t = relay_preflight::CheckTable::new();
                use relay_preflight::CheckId;
                for id in CheckId::ALL.iter().take(6) {
                    t.set(*id, true);
                }
                t
            },
            calibration_configured: false,
        }
    }

    /// Mutable access to the wrapped [`FlightCore`] — the BENCH-TUNING seam
    /// (v1.117): a plant-specific bench (gz: hover thrust, GNSS variance,
    /// altitude gains, estimator process floor) tunes the same core the
    /// supervisor flies. Flight logic must not use this to bypass the
    /// supervisor's mode/setpoint ownership.
    pub fn core_mut(&mut self) -> &mut C {
        &mut self.core
    }

    pub fn mode(&self) -> relay_fsm::Mode {
        self.fsm.mode()
    }

    /// Every failsafe cause the supervisor currently holds, as a bitmask over
    /// [`failsafe_flags`].
    ///
    /// WHY THIS EXISTS (SUPERVISOR-P01, #414). The published component wraps
    /// `FlightCore`, so a host embedding it gets no mode machine and no
    /// failsafes at all. Wiring the supervisor in is the fix, but the supervisor
    /// COULD NOT REPORT ITS OWN FAILSAFE STATE: `rtl_latched`, `runaway_count`
    /// and `wind_count` were private with no accessor, so there was nothing for
    /// a seam — or a GCS — to read. `mode()` shows the OUTCOME (Rtl, Land,
    /// Terminated); this shows the CAUSE.
    ///
    /// EVERY BIT MIRRORS A CONDITION `step` ALREADY ACTS ON, deliberately. None
    /// of these thresholds are new, and `BATT_SILENCE_S` is now shared with
    /// `step` rather than copied. An observer that disagrees with the thing it
    /// observes is worse than none, because it is believed.
    pub fn failsafe_flags(&self) -> u32 {
        use failsafe_flags as f;
        let mut bits = 0u32;
        if self.rtl_latched {
            bits |= f::RTL_LATCHED;
        }
        if self.batt_state.low {
            bits |= f::BATT_LOW;
        }
        if self.batt_state.critical {
            bits |= f::BATT_CRITICAL;
        }
        // `batt_seen &&` is load-bearing: a vehicle that NEVER had a battery
        // sense must not read as a silent pack (#413 fail-unsafe, #475
        // declare-on-first-sight). Same guard `step` uses.
        if self.batt_seen && self.batt_missing_s > BATT_SILENCE_S {
            bits |= f::BATT_SILENT;
        }
        if self.outside_geofence() {
            bits |= f::GEOFENCE_BREACH;
        }
        if self.runaway_count > 0 {
            bits |= f::RUNAWAY_ARMING;
        }
        if self.wind_count > 0 {
            bits |= f::WIND_ARMING;
        }
        // ARM_BLOCKED (bit 7) = "`command(Arm)` would be refused RIGHT NOW",
        // judged by the SAME 21-row `check_table` that `command` gates on
        // (#609 finding 6). It used to read `arm_blocked_reason()`, the legacy
        // six-field struct, so it disagreed with the gate both ways: a
        // MotorsHealthy / FailsafeConfigured refusal left it clear, and the
        // component's identity calibration (which fails only the legacy
        // struct) held it set on every tick, armed or not.
        //
        // Scope: a refusal only means something while DISARMED; arming is the
        // thing being refused. In any other mode (Armed, Takeoff, Loiter, ...,
        // Terminated) the vehicle is already past the gate, so the bit is
        // CLEAR rather than reporting a verdict nobody is asking for. The
        // table verdict is still available via `arm_blocked_check()`.
        if self.fsm.mode() == relay_fsm::Mode::Disarmed && self.arm_blocked_check().is_some() {
            bits |= f::ARM_BLOCKED;
        }
        if self.mission_source == MissionSource::Follow && self.follow_target().is_none() {
            bits |= f::TARGET_STALE;
        }
        bits
    }

    pub fn state(&self) -> NavState {
        self.core.state()
    }

    /// Install the sensor calibration applied to raw IMU/mag samples each cycle,
    /// and return whether it was installed.
    ///
    /// A VALID calibration ([`relay_calib::CalParams::is_valid`]: finite, every
    /// scale positive, soft-iron positive definite) is installed and marks the calibration
    /// CONFIGURED, so `CalibrationPresent` passes AT ONCE — the IDENTITY
    /// included: it means "calibrated upstream, no residual", because
    /// per-sensor calibration is applied below this layer (jess#167). An
    /// INVALID one is NOT installed: the identity is installed in its place
    /// and the calibration is marked NOT configured, revoking any earlier
    /// valid one, so arming is refused at once (#611). A supervisor that is
    /// never given one reports `CalibrationPresent` false from construction.
    ///
    /// ONE SOURCE OF TRUTH (#609 single-source refactor): the row is only ever
    /// a projection of [`calibration_configured`](Self::calibration_configured),
    /// written here at once and re-projected by `update_preflight`. The one
    /// window where they differ is a NEVER-STEPPED supervisor that was never
    /// given a calibration: it keeps the constructor's bring-up default (row
    /// true) until its first step, which ~27 native call sites (the gz
    /// harness among them) arm on.
    ///
    /// Delegates to the FlightCore, where the estimator applies it.
    /// (`FlightCore::set_calibration` itself does not validate; it has no
    /// pre-arm gate to report to.)
    pub fn set_calibration(&mut self, calib: relay_calib::CalParams) -> bool {
        let ok = calib.is_valid();
        self.core.set_calibration(if ok {
            calib
        } else {
            relay_calib::CalParams::identity()
        });
        self.calibration_configured = ok;
        // Project the flag into the row NOW, so a revocation refuses arming at
        // once rather than from the next step. `update_preflight` re-projects
        // the SAME flag each step; nothing writes the row from anything else.
        self.check_table
            .set(relay_preflight::CheckId::CalibrationPresent, ok);
        ok
    }

    /// Whether a valid calibration is configured — the value of the
    /// `CalibrationPresent` pre-arm row, always.
    pub fn calibration_configured(&self) -> bool {
        self.calibration_configured
    }

    /// The calibration currently applied to raw samples.
    pub fn calibration(&self) -> relay_calib::CalParams {
        self.core.calibration()
    }

    /// Derive the pre-arm rows the supervisor can know from its OWN state, each
    /// cycle (v1.99 — the pre-arm gate fed by real signals, not all-pass
    /// defaults), straight into the PREARM-P03 table:
    ///   * `SensorsHealthy` — no navigation-integrity latch (GNSS-P02). Both
    ///     latches it reads (`SpoofMonitor::spoofed`, `DualGnss::diverged`)
    ///     are monotone for the life of the core — nothing in falcon-core
    ///     calls a ground reset — so this is the value the legacy sticky
    ///     `sensors_healthy = false` produced;
    ///   * `EstimatorConverged` — the IEKF attitude uncertainty has settled
    ///     below [`PREARM_TILT_UNCERT_MAX`] (a divergent/just-started filter
    ///     blocks arming);
    ///   * `GeofenceLoaded` — a positive fence radius is configured;
    ///   * `BatteryOk` — a reading arrived AND the estimated battery state
    ///     (BATTERY-P02: debounced, latched low/critical flags) is clear;
    ///   * the breadth rows below.
    ///
    /// Two of the always-required six are CONFIGURED, not measured
    /// (#609 single-source refactor):
    ///
    /// `CalibrationPresent` is re-projected here from the supervisor's own
    /// `calibration_configured` flag (a VALID calibration was explicitly
    /// configured; the identity counts as "calibrated upstream") — the same
    /// flag [`set_calibration`](Self::set_calibration) projects at once. It is
    /// not a measurement and has no other source.
    ///
    /// `FailsafeConfigured` is NOT written here. The constructor sets it true
    /// (it is handed the failsafe configuration), and an integration that
    /// knows better re-declares it via [`set_check`]. Until #609's
    /// single-source refactor this rewrote it every cycle from a default-true
    /// legacy field nothing refreshed, which CLOBBERED such a re-declaration.
    pub fn update_preflight<B: FlightBackend>(&mut self, _b: &mut B) {
        use relay_preflight::CheckId;
        let outside = self.outside_geofence();
        let nav_compromised = self.core.nav_compromised();
        let tilt_converged = self.core.tilt_uncertainty() < PREARM_TILT_UNCERT_MAX;
        let t = &mut self.check_table;
        // GNSS-P02: a latched receiver divergence or spoof walk-off blocks
        // arming (the flag only clears via a ground reset).
        t.set(CheckId::SensorsHealthy, !nav_compromised);
        t.set(CheckId::EstimatorConverged, tilt_converged);
        // A projection of the configured flag, not a measurement (see above).
        t.set(CheckId::CalibrationPresent, self.calibration_configured);
        t.set(CheckId::GeofenceLoaded, self.fence_radius > 0.0);
        // No battery data => NOT ok. A vehicle that cannot see its pack must not
        // report that the pack is fine.
        t.set(
            CheckId::BatteryOk,
            self.batt_present && !(self.batt_state.low || self.batt_state.critical),
        );
        t.set(CheckId::EstimatorInnovation, !nav_compromised);
        t.set(CheckId::GnssAgreement, !self.core.gnss_diverged());
        t.set(
            CheckId::GeofenceSane,
            self.fence_radius.is_finite()
                && self.fence_radius > 0.0
                && self.home.iter().all(|v| v.is_finite()),
        );
        // Battery must clear the low threshold WITH margin for the takeoff
        // draw — arming at low+ε means an RTL seconds after liftoff.
        t.set(
            CheckId::BatteryTakeoffMargin,
            self.batt_state.degraded || self.batt_state.soc >= 0.30,
        );
        // PREARM-P03 propulsion row. A rotor the FDI has ISOLATED stays
        // isolated — `RotorFaultDetector::update` returns early forever once
        // `failed` is set — so the hazard OUTLIVES the flight it appeared in,
        // and the next arm must see it. Nothing else in the table does:
        // `EscTelemetry` only tracks eRPM arriving, and the `rtl_latched`
        // term below is guarded by `mode != Mode::Land`, so a fault first
        // isolated during the landing descent never reached it (#564 — the
        // vehicle re-armed with a dead motor and the table named no reason
        // at all). Derived live, exactly as the battery terms are.
        t.set(CheckId::MotorsHealthy, self.core.failed_motor().is_none());
        // The geofence as a LIVE hazard (#609 finding 5): the complement of
        // the in-flight breach term, the same quantity (`outside_geofence`).
        // SUPV-P02 clears the RTL latch on disarm because the live hazard is
        // what must block re-arming; until this row nothing did for the
        // fence, so a vehicle disarmed outside it re-armed. A FIRST arm
        // outside the fence is refused too (maintainer decision 2026-11-04).
        t.set(CheckId::InsideGeofence, !outside);
        t.set(
            CheckId::NoFailsafeLatched,
            !self.rtl_latched && !self.batt_state.low && !self.batt_state.critical,
        );
    }

    /// Is the ESTIMATED position outside the geofence (horizontal distance
    /// from home greater than the fence radius)? THE one definition: the
    /// in-flight breach failsafe, failsafe bit 4 and the `InsideGeofence`
    /// pre-arm row all read it (#609 finding 5), so the row cannot disagree
    /// with the failsafe it stands in for on the ground. A NaN radius or
    /// position compares false here, i.e. NOT outside; a NaN radius is
    /// refused separately by `GeofenceSane`, and the seam never installs
    /// one (`SupervisorConfig::is_valid`).
    fn outside_geofence(&self) -> bool {
        let est = self.core.state();
        let dx = est.p[0] - self.home[0];
        let dy = est.p[1] - self.home[1];
        relay_math::sqrtf(dx * dx + dy * dy) > self.fence_radius
    }

    /// Shared view of the wrapped [`FlightCore`] (estimator state, FDI,
    /// GNSS-integrity flags) — the telemetry/pre-arm read seam.
    pub fn core(&self) -> &C {
        &self.core
    }

    /// The latest estimated battery state (BATTERY-P02): sag-compensated SoC,
    /// coulomb count, failsafe flags, and the degraded (voltage-only) flag —
    /// the SYS_STATUS battery seam (map via `relay_batt::sys_status_fields`).
    pub fn battery(&self) -> relay_batt::BattState {
        self.batt_state
    }

    /// PREARM-P03 integration seam: feed an integration-owned check row
    /// (RC/GCS link, params-from-NVM, calibration age, sensor freshness,
    /// rangefinder plausibility). Setting a row DECLARES it — the arm gate
    /// is thereafter blocked whenever it fails (monotone, PREFLIGHT-K04).
    pub fn set_check(&mut self, id: relay_preflight::CheckId, pass: bool) {
        self.check_table.set(id, pass);
    }

    /// Why arming would be refused by the PREARM-P03 table (the first
    /// failing required row), with the operator STATUSTEXT via
    /// `CheckId::reason_text`. `None` = the table allows.
    /// Read-only view of the PREARM-P03 table: which rows are declared
    /// (required) and which pass. The verdict itself is
    /// [`arm_blocked_check`](Self::arm_blocked_check); this exists so a host
    /// and its tests can check a declaration row by row.
    pub fn check_table(&self) -> &relay_preflight::CheckTable {
        &self.check_table
    }

    pub fn arm_blocked_check(&self) -> Option<relay_preflight::CheckId> {
        match relay_preflight::arm_check_table(&self.check_table) {
            relay_preflight::TableVerdict::Allowed => None,
            relay_preflight::TableVerdict::Blocked(id) => Some(id),
        }
    }

    /// Inject an external command/event (Arm, RequestTakeoff, RequestMission…).
    /// An `Arm` is gated on the pre-arm / commander checks (relay-preflight): the
    /// FSM only enters `Armed` when every check passes AND the vehicle is level
    /// with throttle idle — the Kani-proven entry gate (relay-fsm FSM-K03).
    pub fn command(&mut self, ev: relay_fsm::Event, level: bool, throttle_low: bool) {
        // v1.122: the PREARM-P03 table is the gate — the always-required six
        // rows 0..=5 plus the declared breadth rows. It is the ONLY verdict:
        // the legacy six-field struct beside it is gone (#609 single-source).
        let prearm_ok = matches!(
            relay_preflight::arm_check_table(&self.check_table),
            relay_preflight::TableVerdict::Allowed
        );
        let g = relay_fsm::Gates {
            level,
            throttle_low,
            have_position: true,
            prearm_ok,
        };
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

    /// Choose where Mission takes its setpoint from (#609 finding 4). The
    /// default is [`MissionSource::Waypoints`]; [`host::SupervisorHost`] (the
    /// published seam, its mirror, the HITL endpoint and the split) selects
    /// [`MissionSource::Follow`] on every supervisor it builds. The
    /// selector is explicit rather than inferred from which setter was called
    /// last, so a stray `set_mission` cannot silently turn a live follow into a
    /// mission that RTLs on arrival, nor the reverse.
    pub fn set_mission_source(&mut self, src: MissionSource) {
        self.mission_source = src;
    }

    /// The latched Takeoff/Loiter hold point (NED north, east), or `None` when
    /// not in Takeoff/Loiter or while a moving Loiter entry is still braking
    /// (see [`HOLD_BRAKE_LEAD_S`]). Telemetry / tests.
    pub fn hold_point(&self) -> Option<[f32; 2]> {
        let holding = matches!(
            self.fsm.mode(),
            relay_fsm::Mode::Takeoff | relay_fsm::Mode::Loiter
        );
        if holding && self.hold_regime == HoldRegime::Hold && !self.hold_braking {
            Some(self.hold_xy)
        } else {
            None
        }
    }

    /// The active [`MissionSource`].
    pub fn mission_source(&self) -> MissionSource {
        self.mission_source
    }

    /// Deliver one FOLLOW target (NED metres; z negative = up), the live
    /// Mission setpoint under [`MissionSource::Follow`]. Returns whether it was
    /// accepted.
    ///
    /// FRESHNESS. A call that delivers a target with all three components
    /// FINITE is one fresh sample, time-stamped on the supervisor clock; the
    /// published component makes one call per `step` frame, so per-tick delivery
    /// is what keeps the stream alive. A target with ANY non-finite component
    /// is REJECTED: it is not followed, it does not refresh the stream, and the
    /// last accepted target stays in force until [`FOLLOW_TARGET_TIMEOUT_US`]
    /// expires, after which the stream is stale (see [`MissionSource`]). The
    /// WIT frame has no "absent" flag, so a non-finite target is also how a host
    /// says "no target this frame".
    ///
    /// What this CANNOT see: a host that stops calling `step` altogether. No
    /// code inside the component runs then, so no timeout can fire; that
    /// failure belongs to the host's own watchdog. And a host whose transport
    /// re-sends a stale target every frame is, to this receiver, a fresh one.
    ///
    /// Accepted in every mode (it is only CONSUMED in Mission), so a host that
    /// streams before RequestMission enters Mission with a fresh target.
    pub fn set_follow_target(&mut self, ned: Vec3) -> bool {
        if !(ned[0].is_finite() && ned[1].is_finite() && ned[2].is_finite()) {
            return false;
        }
        self.follow_counter = self.follow_counter.saturating_add(1);
        self.follow.accept(
            relay_offboard::OffboardSetpoint::position(ned),
            self.follow_counter,
            self.clock_us,
        )
    }

    /// The follow target in force now, or `None` when the stream is stale
    /// (never started, timed out). Telemetry / tests.
    pub fn follow_target(&self) -> Option<Vec3> {
        match self.follow.status(self.clock_us) {
            relay_offboard::OffboardStatus::Active(sp) => Some(sp.position_ned),
            relay_offboard::OffboardStatus::Stale => None,
        }
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
            let obstructs = along > 0.0 && along < glen + safe && perp < safe + KEEPOUT_INFLUENCE;
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
        // The RTL latch is an IN-FLIGHT monotonicity device: it stops a
        // level-triggered breach re-firing Failsafe and demoting the Rtl it
        // just commanded (#562). It is NOT an arming interlock, and the field's
        // own doc has always said "Absorbing until disarm". Clearing it here is
        // what that contract promised — before this, ANY failsafe, including a
        // gust long since passed, refused every later arm for the life of the
        // process (#564).
        //
        // LEVEL-triggered, not edge-triggered, so there is no transition to
        // miss; sound because all three set sites are airborne-guarded, so
        // nothing re-sets it while Disarmed. Placed BEFORE update_preflight so
        // the NoFailsafeLatched row refreshes on the SAME tick instead of
        // leaving one cycle of stale refusal.
        //
        // What still blocks re-arming is the hazard itself, re-derived every
        // cycle: MotorsHealthy for a rotor the FDI has isolated (#565),
        // InsideGeofence for a vehicle outside its fence (#609 finding 5 —
        // until then nothing did, so a vehicle disarmed outside the fence
        // re-armed), and the two battery terms. THE BATTERY TERMS ARE NOT
        // LIVE (#609 finding 5): relay-batt's low/critical flags latch until
        // `BatteryEstimator::reset()`, which nothing calls, and the seam's
        // `rebuild_permitted` refuses a rebuild over a latched pack. A battery
        // failsafe is therefore TERMINAL until the process restarts; a fresh
        // pack does not clear it (tests/battery_failsafe_terminal.rs). An
        // explicit pack-replaced reset is planned for v1.142, not built here.
        // High wind is deliberately NOT blocked — it is
        // the one trigger with no ground-observable signal (the detector needs
        // airborne saturation and wind_count is zeroed when !is_airborne), so a
        // transient gust must not brick the airframe (maintainer call, #564).
        if self.fsm.mode() == Mode::Disarmed {
            self.rtl_latched = false;
        }
        // Supervisor clock for the follow-stream timeout. A backend dt that is
        // not a positive finite number still advances it (by 1 ms), so a broken
        // clock cannot freeze a stale stream into a fresh one.
        let dt_s = b.dt();
        let dt_us = if dt_s.is_finite() && dt_s > 0.0 {
            ((dt_s * 1.0e6 + 0.5) as u64).max(1)
        } else {
            1_000
        };
        self.clock_us = self.clock_us.saturating_add(dt_us);
        // v1.99 — refresh the pre-arm checks from real vehicle state each cycle,
        // so a later command(Arm) gates on live signals, not all-pass defaults.
        self.update_preflight(b);
        let est = self.core.state();
        let dx = est.p[0] - self.home[0];
        let dy = est.p[1] - self.home[1];
        let dist_home = relay_math::sqrtf(dx * dx + dy * dy);
        let alt_agl = -est.p[2]; // NED z negative = up
        let g = Gates {
            level: true,
            throttle_low: true,
            have_position: true,
            prearm_ok: true,
        };

        // ── FAILSAFE actuation (the audit's gap): geofence breach OR low
        // battery from any flying state ⇒ Failsafe ⇒ the FSM commands RTL. ──
        // BATTERY-P02: the failsafe acts on the ESTIMATED battery state (sag-
        // compensated + coulomb-counted; flagged voltage-only fallback when no
        // current sense) — a throttle-punch sag does not false-trigger, and a
        // rebounding spent pack cannot hide.
        // ABSENCE IS NOT HEALTH (#413). A backend with no battery sense returns
        // `None`; feeding the estimator a fabricated voltage would make the
        // low-battery failsafe and the BatteryOk pre-arm row assert a healthy
        // pack forever. Record the absence instead and let the pre-arm gate
        // refuse, which is the fail-safe direction.
        match b.read_battery_v() {
            Some(v) => {
                self.batt_present = true;
                self.batt_seen = true;
                self.batt_missing_s = 0.0;
                self.batt_state = self.batt_est.update(b.dt(), v, b.read_battery_i());
            }
            None => {
                self.batt_present = false;
                self.batt_missing_s += b.dt();
            }
        }
        // PREARM-P03 hardware row: ESC/eRPM telemetry alive (the notch's and
        // FDI's source). DECLARE-ON-FIRST-SIGHT: a vehicle that has never
        // shown eRPM (no bidir-DShot) is not gated on it, but once seen,
        // its LOSS blocks arming (one-way, like every declared row).
        match b.read_motor_rpm() {
            Some(_) => self
                .check_table
                .set(relay_preflight::CheckId::EscTelemetry, true),
            None => {
                if self
                    .check_table
                    .is_required(relay_preflight::CheckId::EscTelemetry)
                {
                    self.check_table
                        .set(relay_preflight::CheckId::EscTelemetry, false);
                }
            }
        }
        // A pack that stopped REPORTING counts as a battery failsafe once the
        // silence is longer than a dropped sample. Without this the latches
        // above simply freeze at their last-known-healthy value and the
        // vehicle flies until the pack is flat: `batt_present` alone is only
        // read by the pre-arm gate, which is long past by then. One second is
        // far longer than any sane sense interval and far shorter than the
        // reserve an RTL needs.
        let batt_lost = self.batt_seen && self.batt_missing_s > BATT_SILENCE_S;
        let batt_fail = self.batt_state.low || self.batt_state.critical || batt_lost;
        let breach = self.outside_geofence() || batt_fail;
        // A critical pack must reach Land even from Rtl; a fence breach or a
        // low pack must not re-demote the Rtl it just commanded (#562).
        let want = if self.batt_state.critical { 3 } else { 2 };
        if breach && self.fsm.is_airborne() && want > Self::mode_rank(self.fsm.mode()) {
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
            if self.wind_count >= WIND_CYCLES && 2 > Self::mode_rank(self.fsm.mode()) {
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
            // ── #609 finding 4: a FOLLOW target is a live setpoint. Reaching it
            // ends nothing; only a STALE stream does, and then with the
            // verified arbiter's policy for a stale offboard stream (HOLD,
            // flown as Loiter). Otherwise the host ends Mission itself. ──
            Mode::Mission if self.mission_source == MissionSource::Follow => {
                if self.follow_target().is_none() {
                    let policy = relay_fsafe::Triggers {
                        offboard_stale: true,
                        ..relay_fsafe::Triggers::default()
                    }
                    .policy_action();
                    match policy {
                        relay_fsafe::FailsafeAction::None => {}
                        relay_fsafe::FailsafeAction::Hold => {
                            self.fsm.on(Event::RequestLoiter, g);
                        }
                        relay_fsafe::FailsafeAction::Rtl => {
                            self.fsm.on(Event::RequestRtl, g);
                        }
                        relay_fsafe::FailsafeAction::Land => {
                            self.fsm.on(Event::RequestLand, g);
                        }
                        relay_fsafe::FailsafeAction::Terminate => {
                            self.fsm.on(Event::Terminate, g);
                        }
                    }
                }
            }
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
        // ROTOR-OUT exception (v1.117, FAULT-P04): a 3-rotor quad lands WHERE
        // IT IS, immediately — it must not commute to home first (no yaw
        // authority to navigate, and the settle-over-home wait leaves the
        // vertical on the position P-D, which cannot hold a 3-rotor hover:
        // per-rotor demand rises 0.5 → ~0.67 and the loop sags to the ground
        // at an uncontrolled ~3.4 m/s. The velocity-landing law arrests that
        // and rides down at its commanded descent rate instead.)
        let landing = matches!(self.fsm.mode(), Mode::Land)
            && (self.core.failed_motor().is_some() || (horiz_speed < 0.4 && dist_home < 0.5));
        self.core.set_landing(landing);

        // ── HOLD-POINT LATCH (#609 finding 4) ── capture the horizontal hold
        // point on ENTRY to a latched regime, from the estimate this tick
        // started with; hold it until the regime changes. Updated every tick,
        // ahead of the motors-off early return, so a Disarmed tick ends the
        // regime and the next Takeoff latches afresh. Rules: [`HoldRegime`].
        let regime = match self.fsm.mode() {
            Mode::Takeoff | Mode::Loiter => HoldRegime::Hold,
            _ => HoldRegime::None,
        };
        if regime == HoldRegime::Hold && self.hold_regime != HoldRegime::Hold {
            self.hold_xy = [est.p[0], est.p[1]];
            // BRAKE, THEN LATCH: a Loiter entered while moving brakes first.
            // Takeoff is entered only from Armed, on the ground and at rest, so
            // it always latches here, exactly as #616 defines it.
            self.hold_braking =
                self.fsm.mode() == Mode::Loiter && horiz_speed > HOLD_BRAKE_ENTRY_MS;
            self.brake_s = 0.0;
            self.brake_rest_s = 0.0;
        }
        if regime != HoldRegime::Hold {
            self.hold_braking = false;
        }
        if self.hold_braking {
            self.brake_s += dt_s;
            if horiz_speed < HOLD_REST_MS {
                self.brake_rest_s += dt_s;
            } else {
                self.brake_rest_s = 0.0;
            }
            if self.brake_rest_s >= HOLD_REST_S || self.brake_s >= HOLD_BRAKE_MAX_S {
                // At rest (or out of time): latch where it is.
                self.hold_braking = false;
                self.hold_xy = [est.p[0], est.p[1]];
            } else {
                // The brake point: behind the vehicle along its own velocity.
                self.hold_xy = [
                    est.p[0] - est.v[0] * HOLD_BRAKE_LEAD_S,
                    est.p[1] - est.v[1] * HOLD_BRAKE_LEAD_S,
                ];
            }
        }
        self.hold_regime = regime;

        // ── mode → setpoint ── (horizontal hold target; while landing the core's
        // velocity-landing owns the vertical once engaged — until then Land holds
        // at cruise altitude over home to kill the horizontal drift). ──
        let sp = match self.fsm.mode() {
            // Latched on entry, NOT the current estimate: that had zero
            // horizontal error by construction and drifted with the wind.
            Mode::Takeoff | Mode::Loiter => [self.hold_xy[0], self.hold_xy[1], -self.cruise_alt],
            Mode::Mission => match self.mission_source {
                MissionSource::Waypoints => self.current_waypoint(), // the active leg (its own altitude)
                // The LATEST fresh follow target. Stale is unreachable here in
                // practice (the milestone above already left Mission this
                // tick); if it ever is, command zero demand where the vehicle
                // is rather than any stale target.
                MissionSource::Follow => self
                    .follow_target()
                    .unwrap_or([est.p[0], est.p[1], est.p[2]]),
            },
            Mode::Rtl => [self.home[0], self.home[1], -self.cruise_alt],
            // Land holds HOME (rate-descend once settled) — except rotor-out,
            // which holds the CURRENT position: land-where-you-are (v1.117).
            // Deliberately NOT latched like Loiter (#609 finding 4): measured
            // worse on a 3-rotor airframe, see [`HoldRegime`].
            Mode::Land if self.core.failed_motor().is_some() => {
                [est.p[0], est.p[1], -self.cruise_alt]
            }
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
        // ── MOTORS OFF in idle states (v1.117, found on the gz plant) ──
        // Disarmed/Armed/Terminated must COMMAND ZEROS, not keep stepping the
        // flight loop: the analytic plant's ground quietly held the vehicle so
        // the always-stepping core was invisible, but on gz a post-touchdown
        // Disarmed kept the altitude loop live and flew the vehicle away
        // (and "Terminate = motors cut" was only ever an FSM claim, never an
        // actuator command — the software half of PART-P02's backstop).
        if matches!(
            self.fsm.mode(),
            Mode::Disarmed | Mode::Armed | Mode::Terminated
        ) {
            self.core.step_estimate_only(b); // estimator warm, motors OFF
            self.motors_were_off = true;
            return;
        }
        // SPIN-UP (#609 finding 16): the first flying tick after motors-off
        // re-arms the rotor-out FDI, so every flight gets its spin-up guard
        // and a residual reference from THIS flight. A rotor already isolated
        // stays isolated (see `CascadePartition::rearm_fdi`).
        if self.motors_were_off {
            self.core.rearm_fdi();
            self.motors_were_off = false;
        }
        self.core.set_position(sp);
        self.core.step(b);

        // ── MOTOR-FAILURE failsafe (v1.103): once the core's FDI isolates a
        // rotor, LAND ASAP — a 3-rotor quad has no yaw authority to navigate home,
        // so RTL is unsafe. Fire Failsafe with have_position=false, which the FSM
        // maps to Land (not Rtl) from any flying state. The core has already
        // switched to the reduced-attitude + reconfigured allocator. ──
        if self.core.failed_motor().is_some()
            && self.fsm.is_airborne()
            && self.fsm.mode() != Mode::Land
        {
            let land = Gates {
                level: true,
                throttle_low: true,
                have_position: false,
                prearm_ok: true,
            };
            // Land-level: this MUST still escalate out of Rtl (rank 2).
            if 3 > Self::mode_rank(self.fsm.mode()) {
                self.fsm.on(Event::Failsafe, land);
            }
            self.rtl_latched = true;
        }
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
