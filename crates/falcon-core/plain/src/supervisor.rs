//! The autonomy supervisor: [`FlightSupervisor`], its failsafe flags, the
//! mission / keep-out / hold types it alone uses, and the [`SupervisedCore`]
//! seam it drives a core through. Moved out of `lib.rs`; every public item is
//! re-exported at the crate root, so `falcon_core::FlightSupervisor` and the
//! other existing paths are unchanged.

use super::*;

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
pub(crate) const WAYPOINT_RADIUS: f32 = 1.2;

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
    pub(crate) rtl_latched: bool,
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

    /// EVERY pre-arm row that currently FAILS, as a bitmask (TRANSPORT-P02):
    /// bit `id as usize` is set iff row `id` is REQUIRED (always for the
    /// legacy six, once declared for a breadth row) and does not pass. That is
    /// exactly the predicate [`relay_preflight::arm_check_table`] gates on, read
    /// off the SAME `check_table` that [`command`](Self::command) judges an
    /// `Arm` against, so
    ///
    ///   `prearm_failing_mask() == 0`  <=>  `arm_blocked_check().is_none()`
    ///
    /// and the row `arm_blocked_check()` names is the FIRST set bit in
    /// [`relay_preflight::CheckId::ALL`] (gate) order — NOT necessarily the
    /// lowest bit: `InsideGeofence` is bit 20 but gate position 10.
    ///
    /// An undeclared breadth row is NOT failing (it is not required), so a
    /// vehicle that never fed, say, an RC link reads 0 here and arms.
    ///
    /// Unlike `failsafe_flags()` bit 7 (`ARM_BLOCKED`) this is NOT scoped to
    /// `Disarmed`: it is the table verdict in every mode, which is what a
    /// host needs to name the row behind a refusal.
    pub fn prearm_failing_mask(&self) -> u32 {
        const _: () = assert!(relay_preflight::CHECK_COUNT <= 32);
        let mut mask = 0u32;
        for id in relay_preflight::CheckId::ALL {
            if self.check_table.is_required(id) && !self.check_table.passed(id) {
                mask |= 1u32 << (id as usize);
            }
        }
        mask
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
        // explicit pack-replaced reset is planned for v1.143 (#650), not built here.
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
