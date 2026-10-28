//! `SupervisorHost` — THE ONE implementation of the published seam's
//! supervisor lifecycle rules (#609, single-source refactor).
//!
//! WHY THIS EXISTS. The `@0.12.0` seam (`wasm/cm/cascade`) wraps a
//! [`FlightSupervisor`] and has to decide things a bare supervisor does not:
//! when a host-supplied configuration is usable, when a reconfigure may rebuild
//! the supervisor, which pre-arm rows a fresh supervisor must declare failing,
//! and how a calibration survives a rebuild. #611 wrote those rules into the
//! component, and the wasm/native differential needed a second copy in
//! `tests/cascade-sitl-wasm/src/native_mirror.rs`, and the #575 split
//! orchestrator a third. Three copies of safety rules is three places to fix
//! the next finding — #609 itself found five defects at lines earlier fixes
//! had just touched. Here they are written once, in falcon-core, and unit-tested
//! at the source; the component and the mirror are field-for-field adapters,
//! so the differential tests the ABI rather than two copies of the logic.
//!
//! WHAT IS FAIL-CLOSED AND WHAT IS NOT, deliberately:
//!   * [`SupervisorConfig`] and the calibration ([`validate_calibration`]) are
//!     SAFETY configuration. An invalid one is never clamped: it counts as
//!     absent, which refuses arming through a named pre-arm row.
//!   * [`VehicleConfig`] is TUNING. Each invalid field falls back to the
//!     built-in default ([`VehicleConfig::tuning`]), which is wrong for the
//!     airframe but not divergent. That is the behaviour the seam has shipped
//!     since v0.9, kept bit-for-bit here; making it refuse instead would change
//!     the exported behaviour and is a separate decision.
//!
//! STACK. The published component links an 8 KiB shadow stack and a by-value
//! `FlightCore` already trapped there once. So the host never returns a
//! [`FlightSupervisor`] by value: it builds one straight into its own slot
//! (`Option::get_or_insert_with`) and hands out references. A host lives in a
//! `static`, hence [`SupervisorHost::new`] is `const`.
//!
//! FOR THE #575 SPLIT. Everything here except [`SupervisorHost::build`] reads
//! the supervisor only through calls the split's generic supervisor also has
//! (`mode`, `battery`, `core().failed_motor()`, `core().nav_compromised()`,
//! `set_check`, `set_calibration`, `command`, `step`, `set_mission`). The one
//! core-specific step is how the tuning reaches a freshly built core, so on
//! rebase `SupervisorHost` becomes generic over the core and `build` the one
//! per-core hook (the fused core: `set_hover_thrust` + the setters below; the
//! split: send the same [`VehicleTuning`] to both partitions). The ORDER inside
//! `build` is an invariant either way: construct, apply tuning, install the
//! calibration, declare the rows. A partition reset after `set_calibration`
//! would wipe the calibration.

use crate::{FlightBackend, FlightCore, FlightSupervisor, Vec3};
use relay_calib::CalParams;
use relay_fsm::{Event, Mode};
use relay_preflight::CheckId;

/// The seam's `vehicle-config` record, as sent: plant tuning, not safety
/// configuration. See [`VehicleConfig::tuning`] for how invalid fields are
/// treated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleConfig {
    pub hover_thrust: f32,
    /// Documented as authoritative by the seam, and NOT READ: the supervisor's
    /// core is built at the nominal 1 kHz and `resync_loop_rate` re-derives
    /// the filters from the frame's `dt` (#609 finding 7, open). Carried so
    /// the record stays field-for-field with the WIT.
    pub loop_rate_hz: f32,
    pub pos_var: f32,
    pub process_floor_vel: f32,
    pub process_floor_pos: f32,
    pub altitude_kp: f32,
    pub altitude_kd: f32,
    pub altitude_ki: f32,
    pub position_ki: f32,
}

/// [`VehicleConfig`] after the per-field fallbacks: exactly the values the
/// core's setters receive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleTuning {
    pub hover_thrust: f32,
    pub pos_var: f32,
    pub process_floor_vel: f32,
    pub process_floor_pos: f32,
    pub altitude_kp: f32,
    pub altitude_kd: f32,
    pub altitude_ki: f32,
    pub position_ki: f32,
}

/// Sanitise one tuning number. A calibration arrives across an ABI from code
/// this crate does not control, and `pos-var = 0` or a NaN gain does not fail
/// loudly — it quietly destroys the filter several seconds later. A rejected
/// value falls back to the built-in default.
fn sane(v: f32, default: f32, positive: bool) -> f32 {
    if v.is_finite() && (!positive || v > 0.0) {
        v
    } else {
        default
    }
}

impl VehicleConfig {
    /// The tuning a core is built with. Each field that is non-finite (or,
    /// where it must be, not > 0) falls back to falcon-core's own default —
    /// note `position_ki`'s is 0.02, NOT zero.
    pub fn tuning(&self) -> VehicleTuning {
        VehicleTuning {
            hover_thrust: sane(self.hover_thrust, 0.5, true),
            pos_var: sane(self.pos_var, 0.01, true),
            process_floor_vel: sane(self.process_floor_vel, 0.0, false),
            process_floor_pos: sane(self.process_floor_pos, 0.0, false),
            altitude_kp: sane(self.altitude_kp, 0.05, false),
            altitude_kd: sane(self.altitude_kd, 0.30, false),
            altitude_ki: sane(self.altitude_ki, 0.0, false),
            position_ki: sane(self.position_ki, 0.02, false),
        }
    }
}

impl VehicleTuning {
    /// Apply to a fused core, IN PLACE (never a by-value builder: see the
    /// module's STACK note).
    pub fn apply(&self, core: &mut FlightCore) {
        core.set_pos_var(self.pos_var);
        core.set_process_floor(self.process_floor_vel, self.process_floor_pos);
        core.set_altitude_gains(self.altitude_kp, self.altitude_kd);
        core.set_altitude_integral_gain(self.altitude_ki);
        core.set_position_integral_gain(self.position_ki);
    }
}

/// The seam's `supervisor-config` record: SAFETY configuration.
///
/// `cruise_alt` is metres AGL and POSITIVE — the one altitude in the seam that
/// is not NED-signed; the supervisor negates it when deriving setpoints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupervisorConfig {
    pub home: Vec3,
    pub fence_radius: f32,
    pub cruise_alt: f32,
    pub low_batt_v: f32,
}

impl SupervisorConfig {
    /// Is this configuration usable AS GIVEN? (#609 finding 3)
    ///
    /// Each bound is the field's OWN definition, not an invented range:
    ///   * `home` — NED metres, any finite point;
    ///   * `fence_radius` — metres about home, finite and > 0 (0 or negative is
    ///     no fence; +inf is no fence);
    ///   * `cruise_alt` — metres AGL, POSITIVE (the supervisor negates it; 0 or
    ///     negative commands a descent into the ground), finite;
    ///   * `low_batt_v` — whole-pack volts, finite and > 0 (0 or negative puts
    ///     both derived per-cell thresholds at or below zero, so the battery
    ///     failsafe could never fire).
    pub fn is_valid(&self) -> bool {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        self.home.iter().all(|v| v.is_finite())
            && pos(self.fence_radius)
            && pos(self.cruise_alt)
            && pos(self.low_batt_v)
    }
}

/// GEOFENCE RADIUS WHEN UNCONFIGURED — large enough that the breach term
/// cannot fire, i.e. the geofence is INACTIVE rather than wrong. 0.0 would
/// latch an RTL on tick one ("a supervisor on defaults is worse than no
/// supervisor", SUPERVISOR-P01); a finite value avoids relying on inf
/// comparisons surviving every arithmetic path the breach term takes.
pub const SUP_FENCE_UNSET: f32 = 1.0e9;
/// LOW-BATTERY THRESHOLD WHEN UNCONFIGURED — 0.0, so both derived per-cell
/// fallbacks sit at or below zero and no measured voltage can cross them.
/// Guessing a cell count is not the safer choice: 14 V fires far too late on
/// a 6S pack and reads permanently low on a 3S one.
pub const SUP_LOW_BATT_UNSET: f32 = 0.0;
/// CRUISE ALTITUDE WHEN UNCONFIGURED, metres AGL and positive.
///
/// The three `*_UNSET` values are reached ONLY while arming is refused
/// (`FailsafeConfigured` false): they keep an unconfigured or mis-configured
/// supervisor from latching spurious failsafes on the ground. They are never
/// what an ARMED vehicle flies on.
pub const SUP_CRUISE_UNSET: f32 = 2.0;

/// The calibration as given, or `None` if it is INVALID. Never clamped.
///
/// NOT A SECOND COPY OF THE RULE: validity is
/// [`CalParams::is_valid`](relay_calib::CalParams::is_valid) (finite, accel
/// scale > 0, soft-iron positive definite), the same function
/// `FlightSupervisor::set_calibration` applies. This only lifts it into an
/// `Option` the host can store for rebuilds.
pub fn validate_calibration(c: CalParams) -> Option<CalParams> {
    c.is_valid().then_some(c)
}

/// The pre-arm rows [`FlightSupervisor::update_preflight`] (re)writes from real
/// input on EVERY `step`. A freshly built supervisor has measured none of them,
/// so the host declares every one FALSE at construction (#609 finding 2).
///
/// WHY. `FlightSupervisor::new` forces the always-required six rows TRUE for
/// bring-up back-compat and leaves the breadth rows undeclared, i.e. not
/// required. A host whose first act was `arm` — the first call, or any call
/// after a configure rebuilt the supervisor — was judged against that, and
/// armed past battery, estimator convergence, calibration and `MotorsHealthy`
/// without any of them ever having been measured.
///
/// EXACTLY the rows `update_preflight` refreshes, and no others. Declaring an
/// integration-fed row (RC/GCS link, params-from-NVM, sensor freshness, ...)
/// would block arming forever, because nothing in the seam ever writes it
/// again. `EscTelemetry` is excluded for the same reason: `step` declares it
/// only on first sight of eRPM. The two CONFIGURED rows are NOT step-fed
/// (since #609's single-source refactor `update_preflight` writes neither):
///   * `FailsafeConfigured` — the host declares it from the stored supervisor
///     configuration at build, see [`SupervisorHost::build`];
///   * `CalibrationPresent` — a projection of the supervisor's own
///     `calibration_configured` flag, the ONE source of truth
///     (`FlightSupervisor::set_calibration` projects it at once and
///     `update_preflight` re-projects the same flag). The host re-installs
///     the stored calibration at build, so a rebuild keeps it, and projects
///     the flag there, so a host that sent none is refused from the first
///     `arm` rather than on the constructor's bring-up default.
pub const ROWS_FED_BY_STEP: [CheckId; 10] = [
    CheckId::SensorsHealthy,
    CheckId::EstimatorConverged,
    CheckId::GeofenceLoaded,
    CheckId::BatteryOk,
    CheckId::EstimatorInnovation,
    CheckId::GnssAgreement,
    CheckId::GeofenceSane,
    CheckId::BatteryTakeoffMargin,
    CheckId::MotorsHealthy,
    CheckId::NoFailsafeLatched,
];

/// What a `configure_*` call did. The seam's calls return unit, so a host
/// learns this only by re-reading state; native callers and tests get it here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Configured {
    /// Stored and in effect (or in effect from the next build).
    Accepted,
    /// The value was INVALID. It was not installed, and it REVOKED any earlier
    /// valid one: the latest thing the host sent was broken, and flying on an
    /// older value it has just tried to replace is not the fail-safe reading.
    /// Arming is refused until a valid value arrives.
    Invalid,
    /// Discarded, not queued: the supervisor is in a state the call may not
    /// change (see [`SupervisorHost::rebuild_permitted`] and
    /// [`SupervisorHost::configure_calibration`]).
    Rejected,
}

/// Owns the seam's supervisor and the configuration it is (re)built from.
///
/// Lazily built: the supervisor exists from the first `command` or `step`.
/// Dropping it is how an accepted `configure_vehicle` / `configure_supervisor`
/// takes effect; the next call rebuilds it.
pub struct SupervisorHost {
    sup: Option<FlightSupervisor>,
    /// The last vehicle tuning the host sent, as sent. `None` keeps the core's
    /// built-in defaults.
    vehicle: Option<VehicleConfig>,
    /// The last VALID supervisor configuration. `None` (never sent, or the
    /// last one sent was invalid) is NOT a neutral state: the
    /// configuration-dependent failsafes run on the inactive `SUP_*_UNSET`
    /// values and `FailsafeConfigured` is declared FALSE, so arming is refused.
    supcfg: Option<SupervisorConfig>,
    /// The last VALID calibration the host configured, kept ONLY so a rebuild
    /// can re-install it. `None` = never sent, or revoked by an invalid one.
    /// Whether a calibration is configured is the SUPERVISOR's
    /// `calibration_configured` flag (and the `CalibrationPresent` row it
    /// writes); this mirrors it across rebuilds and is never read as the
    /// verdict.
    calib: Option<CalParams>,
}

impl Default for SupervisorHost {
    fn default() -> Self {
        Self::new()
    }
}

impl SupervisorHost {
    /// Nothing configured, nothing built. `const` so a component can hold the
    /// host in a `static`.
    pub const fn new() -> Self {
        SupervisorHost {
            sup: None,
            vehicle: None,
            supcfg: None,
            calib: None,
        }
    }

    /// The supervisor, if one has been built.
    pub fn supervisor(&self) -> Option<&FlightSupervisor> {
        self.sup.as_ref()
    }

    /// Is a usable supervisor configuration in effect? The value the
    /// `FailsafeConfigured` row is declared from.
    pub fn failsafe_configured(&self) -> bool {
        self.supcfg.is_some()
    }

    /// The calibration a (re)built supervisor is given, if any.
    pub fn calibration(&self) -> Option<CalParams> {
        self.calib
    }

    /// May a `configure_vehicle` / `configure_supervisor` DROP the live
    /// supervisor? (#609 finding 1)
    ///
    /// Both used to drop it unconditionally, and the next `step` rebuilt a
    /// supervisor in `Mode::Disarmed`, which commands ZERO motors — a
    /// re-configure in Loiter cut the motors in flight, and a configure after
    /// `terminate` escaped the absorbing `Terminated` state.
    ///
    /// A rebuild is therefore permitted ONLY when nothing is lost by it:
    ///   * no supervisor exists yet, or
    ///   * it is `Disarmed` (on the ground, by the FSM's own
    ///     never-disarm-airborne invariant) AND holds none of the latches a
    ///     rebuild would silently reset:
    ///       - an ISOLATED ROTOR (`failed_motor`, the #565 dead-motor
    ///         interlock — the FDI never un-isolates, a fresh core has `None`),
    ///       - a NAVIGATION-INTEGRITY latch (`nav_compromised`, GNSS-P02: "only
    ///         clears via a ground reset" — a configure is not that reset),
    ///       - a LATCHED low/critical BATTERY flag (relay-batt latches these;
    ///         a fresh estimator would start clear).
    ///
    /// Everything else — Armed, any airborne mode, `Terminated`, a latched
    /// Disarmed supervisor — REJECTS the call: the configuration is discarded,
    /// not queued. A pending configuration would be applied at a moment the
    /// host did not pick (the next touchdown), and applying it is itself a
    /// rebuild, so it would reset exactly the latches listed above.
    pub fn rebuild_permitted(&self) -> bool {
        match &self.sup {
            None => true,
            Some(sup) => {
                let batt = sup.battery();
                sup.mode() == Mode::Disarmed
                    && sup.core().failed_motor().is_none()
                    && !sup.core().nav_compromised()
                    && !batt.low
                    && !batt.critical
            }
        }
    }

    /// Install the vehicle tuning, applied by rebuilding the supervisor — only
    /// while [`rebuild_permitted`](Self::rebuild_permitted). A rebuild rather
    /// than an in-place update: applying a late change in place would honour
    /// some fields and not others.
    pub fn configure_vehicle(&mut self, cfg: VehicleConfig) -> Configured {
        if !self.rebuild_permitted() {
            return Configured::Rejected;
        }
        self.vehicle = Some(cfg);
        self.sup = None;
        Configured::Accepted
    }

    /// Install the supervisor's safety configuration, applied by rebuilding the
    /// supervisor — only while [`rebuild_permitted`](Self::rebuild_permitted).
    ///
    /// A rebuild because `home` seeds the waypoints and keep-out zone centres
    /// at construction, and `low_batt_v` is folded into the battery
    /// estimator's per-cell fallbacks there too: an in-place change would be a
    /// partially-applied safety configuration. An INVALID configuration is
    /// treated exactly like none (never clamped), and still rebuilds, so the
    /// supervisor runs on the inactive constants with arming refused.
    pub fn configure_supervisor(&mut self, cfg: SupervisorConfig) -> Configured {
        if !self.rebuild_permitted() {
            return Configured::Rejected;
        }
        let valid = cfg.is_valid();
        self.supcfg = valid.then_some(cfg);
        self.sup = None;
        if valid {
            Configured::Accepted
        } else {
            Configured::Invalid
        }
    }

    /// Install the IMU / magnetometer calibration (relay#586, #609).
    ///
    /// IN PLACE, NOT BY REBUILD: `set_calibration` is a pure setter on what the
    /// estimator subtracts from the raw sample, so the latches
    /// `rebuild_permitted` protects are not at stake. What IS at stake is the
    /// attitude estimate stepping under the controller, so it is accepted ONLY
    /// while the supervisor is `Disarmed` (or not yet built); every other mode
    /// REJECTS it — the same reject-not-queue rule as the configure calls.
    ///
    /// A VALID calibration (the identity included — "calibrated upstream",
    /// jess#167) is CONFIGURED. An INVALID one ([`validate_calibration`]) is
    /// not installed: it REVOKES any earlier one, so arming is refused by
    /// `CalibrationPresent`. The supervisor's `set_calibration` decides both
    /// and writes the row; the host passes it the call AS SENT.
    pub fn configure_calibration(&mut self, cal: CalParams) -> Configured {
        if let Some(sup) = &self.sup {
            if sup.mode() != Mode::Disarmed {
                return Configured::Rejected;
            }
        }
        self.calib = validate_calibration(cal);
        if let Some(sup) = self.sup.as_mut() {
            // AS SENT, never a substitute: an invalid one must reach the
            // supervisor so it revokes (installs the identity, clears its
            // flag and the CalibrationPresent row).
            sup.set_calibration(cal);
        }
        if self.calib.is_some() {
            Configured::Accepted
        } else {
            Configured::Invalid
        }
    }

    /// Deliver one pilot command to the mode machine.
    ///
    /// The supervisor is built here if it does not exist yet, rather than the
    /// command being dropped: a host whose first act is `arm` must get a real
    /// verdict from the pre-arm table, not silence. That verdict is a REFUSAL
    /// until a `step` has run ([`ROWS_FED_BY_STEP`], #609 finding 2).
    pub fn command(&mut self, ev: Event, level: bool, throttle_low: bool) {
        self.ensure().command(ev, level, throttle_low);
    }

    /// One control tick. `mission_leg` is the frame's target, which is a
    /// MISSION LEG at `@0.12.0`, not a position command: the supervisor owns
    /// the position setpoint and derives one per mode, so outside Mission it is
    /// ignored (#609 finding 4).
    pub fn step<B: FlightBackend>(&mut self, mission_leg: Vec3, b: &mut B) -> &FlightSupervisor {
        let sup = self.ensure();
        sup.set_mission(mission_leg);
        sup.step(b);
        sup
    }

    /// The supervisor, built into its slot if absent.
    fn ensure(&mut self) -> &mut FlightSupervisor {
        if self.sup.is_none() {
            self.build();
        }
        self.sup.as_mut().expect("built above")
    }

    /// Build the supervisor INTO ITS SLOT, in the invariant order: construct,
    /// apply tuning, install the calibration, declare the rows.
    ///
    /// The only core-specific step in this file (see the module docs, FOR THE
    /// #575 SPLIT).
    fn build(&mut self) {
        let (home, fence, cruise, low_batt) = match &self.supcfg {
            Some(c) => (c.home, c.fence_radius, c.cruise_alt, c.low_batt_v),
            None => (
                [0.0, 0.0, 0.0],
                SUP_FENCE_UNSET,
                SUP_CRUISE_UNSET,
                SUP_LOW_BATT_UNSET,
            ),
        };
        let tuning = self.vehicle.map(|v| v.tuning());
        let calib = self.calib;
        let configured = self.failsafe_configured();
        // Constructed straight into the slot — the by-value form of this
        // construction is what overflowed the component's 8 KiB shadow stack.
        let sup = self
            .sup
            .get_or_insert_with(|| FlightSupervisor::new(home, fence, cruise, low_batt));
        // `FlightSupervisor::new` hardcodes `FlightCore::new(0.5, 1000.0)`, so
        // hover thrust must be re-applied; the loop RATE resyncs from the
        // backend's `dt` on the first tick, the warm-up count does not (#609
        // finding 7, open).
        if let Some(t) = tuning {
            sup.set_hover_thrust(t.hover_thrust);
            t.apply(sup.core_mut());
        }
        // A configured calibration survives a rebuild; none configured stays
        // none (the fresh supervisor's CalibrationPresent row starts false).
        if let Some(c) = calib {
            sup.set_calibration(c);
        }
        // The row is the supervisor's own flag, projected now so it holds
        // from the first `arm` — the constructor's bring-up default (true)
        // must not stand for "configured" on a host that sent none.
        sup.set_check(CheckId::CalibrationPresent, sup.calibration_configured());
        // UNDECLARED IS FAIL, NOT PASS (#609 finding 2): until a `step` has
        // run, nothing a step feeds has been measured.
        for id in ROWS_FED_BY_STEP {
            sup.set_check(id, false);
        }
        // Declared once, here: the configuration only changes through a
        // rebuild, and since #609's single-source refactor a `step` no longer
        // overwrites this row.
        sup.set_check(CheckId::FailsafeConfigured, configured);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GRAVITY, ImuSample, SimBackend};

    const CAL: CalParams = CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        accel_bias: [0.0; 3],
        accel_scale: [1.0; 3],
        mag_offset: [0.0; 3],
        mag_scale: [1.0; 3],
        mag_offdiag: [0.0; 3],
    };
    const GOOD: SupervisorConfig = SupervisorConfig {
        home: [0.0, 0.0, 0.0],
        fence_radius: 50.0,
        cruise_alt: 2.0,
        low_batt_v: 14.0,
    };

    /// Level, at rest, GNSS at the origin, a configurable pack voltage.
    struct Rest {
        batt: Option<f32>,
    }
    impl FlightBackend for Rest {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, 0.0])
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn read_heading(&mut self) -> Option<f32> {
            Some(0.0)
        }
        fn read_battery_v(&mut self) -> Option<f32> {
            self.batt
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
    }

    /// A host configured with `cfg` + a valid calibration, stepped until the
    /// estimator has settled, then asked to arm. Returns the resulting mode.
    fn arm_after_settling(cfg: SupervisorConfig) -> (Mode, Option<CheckId>) {
        let mut h = SupervisorHost::new();
        h.configure_supervisor(cfg);
        h.configure_calibration(CAL);
        let mut b = Rest { batt: Some(16.8) };
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        let s = h.supervisor().unwrap();
        (s.mode(), s.arm_blocked_check())
    }

    #[test]
    fn control_a_valid_configuration_arms() {
        assert_eq!(arm_after_settling(GOOD), (Mode::Armed, None));
    }

    /// #609 finding 3: every field, every way to be invalid, refuses arming by
    /// `FailsafeConfigured` — never clamped into something permissive.
    #[test]
    fn an_invalid_supervisor_config_refuses_arm_per_field() {
        let bad = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];
        let mut cases: [Option<SupervisorConfig>; 32] = [None; 32];
        let mut n = 0;
        let mut push = |c: SupervisorConfig| {
            cases[n] = Some(c);
            n += 1;
        };
        for v in bad {
            for i in 0..3 {
                let mut c = GOOD;
                c.home[i] = v;
                push(c);
            }
        }
        for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.0, -1.0, -30.0] {
            push(SupervisorConfig {
                fence_radius: v,
                ..GOOD
            });
            push(SupervisorConfig {
                cruise_alt: v,
                ..GOOD
            });
            push(SupervisorConfig {
                low_batt_v: v,
                ..GOOD
            });
        }
        assert_eq!(n, 27);
        for c in cases.iter().flatten() {
            assert!(!c.is_valid(), "{c:?} must be invalid");
            let mut h = SupervisorHost::new();
            assert_eq!(h.configure_supervisor(*c), Configured::Invalid);
            assert!(!h.failsafe_configured());
            let (mode, why) = arm_after_settling(*c);
            assert_eq!(mode, Mode::Disarmed, "{c:?} armed");
            assert_eq!(why, Some(CheckId::FailsafeConfigured), "{c:?}");
        }
    }

    /// An invalid configuration REVOKES an earlier valid one.
    #[test]
    fn an_invalid_supervisor_config_revokes_the_valid_one() {
        let mut h = SupervisorHost::new();
        assert_eq!(h.configure_supervisor(GOOD), Configured::Accepted);
        assert!(h.failsafe_configured());
        let bad = SupervisorConfig {
            fence_radius: f32::NAN,
            ..GOOD
        };
        assert_eq!(h.configure_supervisor(bad), Configured::Invalid);
        assert!(!h.failsafe_configured());
    }

    /// An unconfigured supervisor runs on the INACTIVE constants (no spurious
    /// RTL on the ground) and refuses to arm by `FailsafeConfigured`.
    #[test]
    fn an_unconfigured_host_runs_inert_and_refuses_arm() {
        let mut h = SupervisorHost::new();
        h.configure_calibration(CAL);
        let mut b = Rest { batt: Some(16.8) };
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        let s = h.supervisor().unwrap();
        assert_eq!(s.mode(), Mode::Disarmed);
        assert_eq!(s.arm_blocked_check(), Some(CheckId::FailsafeConfigured));
        assert_eq!(s.failsafe_flags() & !crate::failsafe_flags::ARM_BLOCKED, 0);
    }

    /// #609 finding 2: a never-stepped supervisor declares every step-fed row
    /// FAILING, and refuses `arm` — whatever the configuration.
    #[test]
    fn a_fresh_supervisor_declares_step_fed_rows_false() {
        let mut h = SupervisorHost::new();
        h.configure_supervisor(GOOD);
        h.configure_calibration(CAL);
        h.command(Event::Arm, true, true);
        let s = h.supervisor().unwrap();
        assert_eq!(s.mode(), Mode::Disarmed, "a never-stepped host armed");
        let t = s.check_table();
        for id in ROWS_FED_BY_STEP {
            assert!(
                t.is_required(id) && !t.passed(id),
                "{id:?} not declared false"
            );
        }
        assert!(t.passed(CheckId::FailsafeConfigured), "configured");
        // The CONFIGURED calibration row is not step-fed: it reads the
        // supervisor's flag at once, before any step.
        assert!(s.calibration_configured());
        assert!(
            t.passed(CheckId::CalibrationPresent),
            "calibration configured"
        );
        // A host that sent NO calibration: the row is the supervisor's flag
        // (false) from build, not the constructor's bring-up default (true).
        let mut bare = SupervisorHost::new();
        bare.configure_supervisor(GOOD);
        bare.command(Event::Arm, true, true);
        let s = bare.supervisor().unwrap();
        assert!(!s.calibration_configured());
        assert!(
            !s.check_table().passed(CheckId::CalibrationPresent),
            "an unconfigured calibration passed on the bring-up default"
        );
        // And a rebuild re-declares them: configure again, then arm.
        let mut b = Rest { batt: Some(16.8) };
        h.step([0.0, 0.0, -2.0], &mut b);
        h.configure_supervisor(GOOD);
        h.command(Event::Arm, true, true);
        assert_eq!(
            h.supervisor().unwrap().mode(),
            Mode::Disarmed,
            "rebuilt host armed"
        );
    }

    /// The step-fed list is EXACTLY what a step writes: after steps from the
    /// fresh declaration, every listed row has been rewritten by
    /// `update_preflight` (here, all pass), so none is stuck at the
    /// declaration. And the two CONFIGURED rows are NOT step-fed: a step never
    /// overwrites `FailsafeConfigured` (unconfigured here, so false) nor
    /// `CalibrationPresent` (which always equals the supervisor's flag — true
    /// when one is configured, false when none is, step or no step).
    #[test]
    fn every_step_fed_row_is_rewritten_by_a_step() {
        // The IDENTITY case discriminates the retired "!= identity" rule: it is
        // configured (true) while differing from nothing.
        for cal in [Some(CAL), Some(CalParams::identity()), None] {
            let calibrated = cal.is_some();
            let mut h = SupervisorHost::new();
            if let Some(c) = cal {
                h.configure_calibration(c);
            }
            let mut b = Rest { batt: Some(16.8) };
            for _ in 0..1500 {
                h.step([0.0, 0.0, -2.0], &mut b);
            }
            let s = h.supervisor().unwrap();
            let t = s.check_table();
            for id in ROWS_FED_BY_STEP {
                assert!(t.passed(id), "{id:?} never refreshed by a step");
            }
            assert!(
                !t.passed(CheckId::FailsafeConfigured),
                "a step must not clobber the declaration"
            );
            assert_eq!(s.calibration_configured(), calibrated);
            assert_eq!(
                t.passed(CheckId::CalibrationPresent),
                calibrated,
                "CalibrationPresent must equal the supervisor's flag"
            );
            assert!(!ROWS_FED_BY_STEP.contains(&CheckId::CalibrationPresent));
            assert!(!ROWS_FED_BY_STEP.contains(&CheckId::FailsafeConfigured));
        }
    }

    #[test]
    fn calibration_validation_per_field() {
        assert_eq!(validate_calibration(CAL), Some(CAL));
        assert_eq!(
            validate_calibration(CalParams::identity()),
            Some(CalParams::identity())
        );
        for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for i in 0..3 {
                for f in 0..6 {
                    let mut c = CAL;
                    match f {
                        0 => c.gyro_bias[i] = v,
                        1 => c.accel_bias[i] = v,
                        2 => c.accel_scale[i] = v,
                        3 => c.mag_offset[i] = v,
                        4 => c.mag_scale[i] = v,
                        _ => c.mag_offdiag[i] = v,
                    }
                    assert_eq!(validate_calibration(c), None, "field {f}[{i}] = {v}");
                }
            }
        }
        for v in [0.0, -1.0] {
            for i in 0..3 {
                let mut c = CAL;
                c.accel_scale[i] = v;
                assert_eq!(validate_calibration(c), None, "accel_scale[{i}] = {v}");
                let mut c = CAL;
                c.mag_scale[i] = v;
                assert_eq!(validate_calibration(c), None, "mag_scale[{i}] = {v}");
                // A negative BIAS is a real bias.
                let mut c = CAL;
                c.gyro_bias[i] = v;
                assert!(validate_calibration(c).is_some());
            }
        }
        // Soft-iron: a symmetric, positive-definite off-diagonal is valid; one
        // that makes the matrix indefinite is not (xy = 2 with unit scales).
        let pd = CalParams {
            mag_offdiag: [0.1, 0.05, -0.1],
            ..CAL
        };
        assert!(validate_calibration(pd).is_some());
        let indefinite = CalParams {
            mag_offdiag: [2.0, 0.0, 0.0],
            ..CAL
        };
        assert_eq!(validate_calibration(indefinite), None);
        // NOT a second copy of the rule: exactly CalParams::is_valid.
        for c in [CAL, pd, indefinite, CalParams::identity()] {
            assert_eq!(validate_calibration(c).is_some(), c.is_valid());
        }
    }

    /// Valid → installed (also survives a rebuild); invalid → identity
    /// installed, earlier one revoked, arm refused by `CalibrationPresent`.
    #[test]
    fn calibration_install_revoke_and_survive_rebuild() {
        let mut h = SupervisorHost::new();
        assert_eq!(h.configure_calibration(CAL), Configured::Accepted);
        h.configure_supervisor(GOOD);
        let mut b = Rest { batt: Some(16.8) };
        h.step([0.0, 0.0, -2.0], &mut b);
        assert_eq!(h.supervisor().unwrap().calibration(), CAL);
        // Rebuild: the calibration is re-installed.
        assert_eq!(h.configure_supervisor(GOOD), Configured::Accepted);
        h.step([0.0, 0.0, -2.0], &mut b);
        assert_eq!(h.supervisor().unwrap().calibration(), CAL);
        // Invalid: revokes, installs identity in place.
        let bad = CalParams {
            mag_scale: [1.0, 0.0, 1.0],
            ..CAL
        };
        assert_eq!(h.configure_calibration(bad), Configured::Invalid);
        assert_eq!(h.calibration(), None);
        let s = h.supervisor().unwrap();
        assert_eq!(s.calibration(), CalParams::identity());
        // Revoked AT ONCE, before any step.
        assert!(!s.calibration_configured());
        assert!(!s.check_table().passed(CheckId::CalibrationPresent));
        // And a rebuild does not resurrect it.
        assert_eq!(h.configure_supervisor(GOOD), Configured::Accepted);
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        let s = h.supervisor().unwrap();
        assert_eq!(s.mode(), Mode::Disarmed);
        assert_eq!(s.arm_blocked_check(), Some(CheckId::CalibrationPresent));
    }

    /// An explicitly configured IDENTITY is a valid calibration ("calibrated
    /// upstream", jess#167): it arms. Never configured: refused.
    #[test]
    fn identity_calibration_arms_and_none_refuses() {
        let mut h = SupervisorHost::new();
        h.configure_supervisor(GOOD);
        assert_eq!(
            h.configure_calibration(CalParams::identity()),
            Configured::Accepted
        );
        let mut b = Rest { batt: Some(16.8) };
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Armed);

        let mut h = SupervisorHost::new();
        h.configure_supervisor(GOOD);
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        let s = h.supervisor().unwrap();
        assert_eq!(s.mode(), Mode::Disarmed);
        assert_eq!(s.arm_blocked_check(), Some(CheckId::CalibrationPresent));
    }

    /// Calibration is accepted only while Disarmed (or unbuilt).
    #[test]
    fn calibration_is_rejected_unless_disarmed() {
        let (mode, _) = arm_after_settling(GOOD);
        assert_eq!(mode, Mode::Armed);
        let mut h = SupervisorHost::new();
        h.configure_supervisor(GOOD);
        h.configure_calibration(CAL);
        let mut b = Rest { batt: Some(16.8) };
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Armed);
        let other = CalParams {
            gyro_bias: [0.002, 0.0, 0.0],
            ..CAL
        };
        assert_eq!(h.configure_calibration(other), Configured::Rejected);
        assert_eq!(h.supervisor().unwrap().calibration(), CAL);
        assert_eq!(h.calibration(), Some(CAL));
        // An invalid one is rejected too, NOT allowed to revoke in flight.
        let bad = CalParams {
            gyro_bias: [f32::NAN, 0.0, 0.0],
            ..CAL
        };
        assert_eq!(h.configure_calibration(bad), Configured::Rejected);
        assert_eq!(h.supervisor().unwrap().calibration(), CAL);
    }

    fn vcfg() -> VehicleConfig {
        VehicleConfig {
            hover_thrust: 0.55,
            loop_rate_hz: 250.0,
            pos_var: 0.02,
            process_floor_vel: 0.0,
            process_floor_pos: 0.0,
            altitude_kp: 0.05,
            altitude_kd: 0.3,
            altitude_ki: 0.0,
            position_ki: 0.02,
        }
    }

    /// Both configure calls in every rebuild-guard state.
    fn assert_rejected(h: &mut SupervisorHost, why: &str) {
        let before = h.supervisor().unwrap().mode();
        assert!(!h.rebuild_permitted(), "{why}: rebuild permitted");
        assert_eq!(h.configure_vehicle(vcfg()), Configured::Rejected, "{why}");
        assert_eq!(h.configure_supervisor(GOOD), Configured::Rejected, "{why}");
        // Rejected means the live supervisor is untouched — not dropped.
        assert_eq!(h.supervisor().map(|s| s.mode()), Some(before), "{why}");
    }

    #[test]
    fn rebuild_guard_permits_unbuilt_and_clean_disarmed() {
        let mut h = SupervisorHost::new();
        assert!(h.rebuild_permitted());
        assert_eq!(h.configure_vehicle(vcfg()), Configured::Accepted);
        let mut b = Rest { batt: Some(16.8) };
        h.step([0.0, 0.0, -2.0], &mut b);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Disarmed);
        assert!(h.rebuild_permitted());
        assert_eq!(h.configure_supervisor(GOOD), Configured::Accepted);
        assert!(h.supervisor().is_none(), "an accepted configure drops it");
    }

    /// #609 finding 1: Armed, airborne and Terminated all reject.
    #[test]
    fn rebuild_guard_rejects_armed_airborne_and_terminated() {
        let mut h = SupervisorHost::new();
        h.configure_supervisor(GOOD);
        h.configure_calibration(CAL);
        let mut b = Rest { batt: Some(16.8) };
        for _ in 0..1500 {
            h.step([0.0, 0.0, -2.0], &mut b);
        }
        h.command(Event::Arm, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Armed);
        assert_rejected(&mut h, "Armed");
        h.command(Event::RequestTakeoff, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Takeoff);
        assert_rejected(&mut h, "Takeoff");
        h.command(Event::Terminate, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Terminated);
        assert_rejected(&mut h, "Terminated");
        // Terminated stays absorbing: nothing re-arms it.
        h.command(Event::Arm, true, true);
        assert_eq!(h.supervisor().unwrap().mode(), Mode::Terminated);
    }

    /// A latched low battery on a Disarmed supervisor rejects.
    #[test]
    fn rebuild_guard_rejects_a_latched_battery() {
        // Each latch ALONE, so neither clause of the guard can hide behind the
        // other (mutation-tested: a pack below CRITICAL is also LOW, so a
        // critical-only case let a dropped `!batt.low` clause survive).
        // 4S at a 14 V low threshold: low = 3.50 V/cell, critical = 3.35.
        for (v, low, critical) in [(13.6, true, false), (13.0, true, true)] {
            let mut h = SupervisorHost::new();
            h.configure_supervisor(GOOD);
            let mut b = Rest { batt: Some(v) };
            for _ in 0..1500 {
                h.step([0.0, 0.0, -2.0], &mut b);
            }
            let s = h.supervisor().unwrap();
            assert_eq!(s.mode(), Mode::Disarmed);
            let batt = s.battery();
            assert_eq!(
                (batt.low, batt.critical),
                (low, critical),
                "precondition at {v} V"
            );
            assert_rejected(&mut h, "battery latch");
        }
    }

    /// A Disarmed supervisor with an ISOLATED ROTOR rejects (#565 interlock).
    /// The latched state is produced by a real flight (the falcon-core
    /// rotor-out path), then handed to the host.
    #[test]
    fn rebuild_guard_rejects_an_isolated_rotor() {
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, 0.002);
        b.rot_drag = 0.02;
        b.ground_contact = true;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        sup.set_calibration(CAL);
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        for _ in 0..8000 {
            sup.step(&mut b);
        }
        b.fail_rotor(1);
        for _ in 0..45000 {
            sup.step(&mut b);
            if sup.mode() == Mode::Disarmed && sup.core().failed_motor().is_some() {
                break;
            }
        }
        assert_eq!(sup.mode(), Mode::Disarmed);
        assert!(
            sup.core().failed_motor().is_some(),
            "precondition: rotor isolated"
        );
        let mut h = SupervisorHost::new();
        h.sup = Some(sup);
        assert_rejected(&mut h, "isolated rotor");
    }

    /// A Disarmed supervisor with a NAVIGATION-INTEGRITY latch rejects.
    #[test]
    fn rebuild_guard_rejects_a_nav_integrity_latch() {
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, 0.004);
        b.gnss_dual_enabled = true;
        b.ground_contact = true;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        for _ in 0..2000 {
            sup.step(&mut b);
        }
        b.gnss_b_offset = [12.0, 0.0];
        for _ in 0..2000 {
            sup.step(&mut b);
        }
        assert_eq!(sup.mode(), Mode::Disarmed);
        assert!(sup.core().nav_compromised(), "precondition: nav latched");
        let mut h = SupervisorHost::new();
        h.sup = Some(sup);
        assert_rejected(&mut h, "nav integrity");
    }

    /// The tuning fallbacks: valid values pass through, each invalid field
    /// falls back to its own default (`position_ki` to 0.02, not 0).
    #[test]
    fn vehicle_tuning_fallbacks() {
        let t = vcfg().tuning();
        assert_eq!(t.hover_thrust, 0.55);
        assert_eq!(t.pos_var, 0.02);
        let nan = VehicleConfig {
            hover_thrust: f32::NAN,
            loop_rate_hz: f32::NAN,
            pos_var: 0.0,
            process_floor_vel: f32::INFINITY,
            process_floor_pos: f32::NAN,
            altitude_kp: f32::NAN,
            altitude_kd: f32::NAN,
            altitude_ki: f32::NAN,
            position_ki: f32::NAN,
        }
        .tuning();
        assert_eq!(
            nan,
            VehicleTuning {
                hover_thrust: 0.5,
                pos_var: 0.01,
                process_floor_vel: 0.0,
                process_floor_pos: 0.0,
                altitude_kp: 0.05,
                altitude_kd: 0.30,
                altitude_ki: 0.0,
                position_ki: 0.02,
            }
        );
        // Sign: hover thrust and pos_var must be > 0; gains may be 0 / negative.
        let neg = VehicleConfig {
            hover_thrust: -0.1,
            altitude_ki: -0.1,
            ..vcfg()
        }
        .tuning();
        assert_eq!(neg.hover_thrust, 0.5);
        assert_eq!(neg.altitude_ki, -0.1);
    }
}
