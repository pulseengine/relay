//! Relay Preflight — built-in-test (BIT) + arming-check gate.
//!
//! PX4 refuses to arm until a battery of pre-flight checks pass; falcon's arm
//! gate (relay-fsm) only checked level + airborne-confirm. This adds the BIT
//! arbiter: a set of checkable pre-flight predicates (sensor health, estimator
//! convergence, calibration present, geofence loaded, battery above the arming
//! threshold, failsafe configured), with the verified property:
//!
//!   **arming is BLOCKED unless EVERY required check passes** — and when blocked,
//!   the FIRST failing check is reported (so the pilot/GCS knows why).
//!
//! This is the highest-safety-value gate before motors spin, so it is proven
//! EXHAUSTIVELY (the check set is small — Kani enumerates every combination).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// The individual pre-flight checks (each `true` = passing).
#[derive(Clone, Copy, Debug, Default)]
pub struct PreflightChecks {
    /// All required sensors (IMU/GNSS/mag/baro) present and reporting sane data.
    pub sensors_healthy: bool,
    /// The estimator has converged (covariance settled, NEES in band).
    pub estimator_converged: bool,
    /// Sensor calibration (accel/gyro/mag) is present.
    pub calibration_present: bool,
    /// A geofence boundary is loaded.
    pub geofence_loaded: bool,
    /// Battery is above the arming threshold.
    pub battery_ok: bool,
    /// A failsafe action set is configured.
    pub failsafe_configured: bool,
}

/// Which check failed (the first failing one, in priority order).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckFail {
    /// `sensors_healthy` failed.
    Sensors,
    /// `estimator_converged` failed.
    Estimator,
    /// `calibration_present` failed.
    Calibration,
    /// `geofence_loaded` failed.
    Geofence,
    /// `battery_ok` failed.
    Battery,
    /// `failsafe_configured` failed.
    Failsafe,
}

/// The arming verdict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmVerdict {
    /// Every required check passed — arming is permitted.
    Allowed,
    /// At least one check failed — arming is BLOCKED; the first failure is named.
    Blocked(CheckFail),
}

impl PreflightChecks {
    /// Are ALL checks passing?
    pub fn all_pass(&self) -> bool {
        self.sensors_healthy
            && self.estimator_converged
            && self.calibration_present
            && self.geofence_loaded
            && self.battery_ok
            && self.failsafe_configured
    }
}

/// The arming gate: `Allowed` iff every required check passes; otherwise
/// `Blocked` with the FIRST failing check (priority order: sensors → estimator →
/// calibration → geofence → battery → failsafe).
pub fn arm_check(c: PreflightChecks) -> ArmVerdict {
    if !c.sensors_healthy {
        ArmVerdict::Blocked(CheckFail::Sensors)
    } else if !c.estimator_converged {
        ArmVerdict::Blocked(CheckFail::Estimator)
    } else if !c.calibration_present {
        ArmVerdict::Blocked(CheckFail::Calibration)
    } else if !c.geofence_loaded {
        ArmVerdict::Blocked(CheckFail::Geofence)
    } else if !c.battery_ok {
        ArmVerdict::Blocked(CheckFail::Battery)
    } else if !c.failsafe_configured {
        ArmVerdict::Blocked(CheckFail::Failsafe)
    } else {
        ArmVerdict::Allowed
    }
}

/// The v1.122 check TABLE (PREARM-P03): the pre-arm breadth as DATA, not
/// scattered conditionals — coverage is enumerable (`CheckId::ALL`) and
/// testable row by row. The legacy six checks are rows 0..=5; the gate
/// blocks on the first failing REQUIRED row in table order. Rows a given
/// integration cannot feed yet stay optional (not required) until it
/// declares them via [`CheckTable::set`] — a bench SITL without an RC
/// receiver must not be hard-blocked by a link check it cannot satisfy,
/// but once declared, a failing row ALWAYS blocks (monotone, Kani-proven).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum CheckId {
    // — the legacy six (always required) —
    SensorsHealthy = 0,
    EstimatorConverged = 1,
    CalibrationPresent = 2,
    GeofenceLoaded = 3,
    BatteryOk = 4,
    FailsafeConfigured = 5,
    // — estimator / navigation integrity —
    EstimatorInnovation = 6,
    GnssAgreement = 7,
    // — configuration consistency —
    ParamsFromNvm = 8,
    GeofenceSane = 9,
    ThresholdsOrdered = 10,
    // — hardware consistency —
    SensorsFresh = 11,
    EscTelemetry = 12,
    BatteryTakeoffMargin = 13,
    RangefinderPlausible = 14,
    // — calibration / safety state —
    CalibrationFresh = 15,
    RcLink = 16,
    GcsLink = 17,
    NoFailsafeLatched = 18,
}

/// Number of table rows.
pub const CHECK_COUNT: usize = 19;

impl CheckId {
    /// Every row, in gate (priority) order — coverage equals this length
    /// by construction.
    pub const ALL: [CheckId; CHECK_COUNT] = [
        CheckId::SensorsHealthy,
        CheckId::EstimatorConverged,
        CheckId::CalibrationPresent,
        CheckId::GeofenceLoaded,
        CheckId::BatteryOk,
        CheckId::FailsafeConfigured,
        CheckId::EstimatorInnovation,
        CheckId::GnssAgreement,
        CheckId::ParamsFromNvm,
        CheckId::GeofenceSane,
        CheckId::ThresholdsOrdered,
        CheckId::SensorsFresh,
        CheckId::EscTelemetry,
        CheckId::BatteryTakeoffMargin,
        CheckId::RangefinderPlausible,
        CheckId::CalibrationFresh,
        CheckId::RcLink,
        CheckId::GcsLink,
        CheckId::NoFailsafeLatched,
    ];

    /// Distinct operator-readable reason, sized for a STATUSTEXT payload
    /// (MAVLINK-P06 carries these to the GCS on a blocked arm).
    pub fn reason_text(self) -> &'static str {
        match self {
            CheckId::SensorsHealthy => "PREARM: sensors unhealthy",
            CheckId::EstimatorConverged => "PREARM: estimator not converged",
            CheckId::CalibrationPresent => "PREARM: no calibration",
            CheckId::GeofenceLoaded => "PREARM: no geofence",
            CheckId::BatteryOk => "PREARM: battery low/critical",
            CheckId::FailsafeConfigured => "PREARM: failsafe unconfigured",
            CheckId::EstimatorInnovation => "PREARM: innovation out of band",
            CheckId::GnssAgreement => "PREARM: GNSS receivers disagree",
            CheckId::ParamsFromNvm => "PREARM: params are defaults (no NVM)",
            CheckId::GeofenceSane => "PREARM: geofence insane",
            CheckId::ThresholdsOrdered => "PREARM: failsafe thresholds unordered",
            CheckId::SensorsFresh => "PREARM: sensor data stale",
            CheckId::EscTelemetry => "PREARM: ESC telemetry missing",
            CheckId::BatteryTakeoffMargin => "PREARM: battery below takeoff margin",
            CheckId::RangefinderPlausible => "PREARM: rangefinder implausible",
            CheckId::CalibrationFresh => "PREARM: calibration stale",
            CheckId::RcLink => "PREARM: RC link down",
            CheckId::GcsLink => "PREARM: GCS link down",
            CheckId::NoFailsafeLatched => "PREARM: failsafe latched",
        }
    }
}

/// The table: per-row (required, passed). The legacy six are ALWAYS
/// required; the breadth rows become required when the integration first
/// sets them (declaring "this vehicle has this signal").
#[derive(Clone, Copy, Debug)]
pub struct CheckTable {
    required: [bool; CHECK_COUNT],
    passed: [bool; CHECK_COUNT],
}

impl Default for CheckTable {
    fn default() -> Self {
        Self::new()
    }
}

impl CheckTable {
    /// Legacy six required (and failing — a fresh table must not arm);
    /// breadth rows undeclared.
    pub fn new() -> Self {
        let mut required = [false; CHECK_COUNT];
        for r in required.iter_mut().take(6) {
            *r = true;
        }
        CheckTable { required, passed: [false; CHECK_COUNT] }
    }

    /// Set a row's pass state. Setting ANY row marks it required — an
    /// integration that reports a signal is thereafter gated on it
    /// (declaring is a one-way door; see PREFLIGHT-K04 monotonicity).
    pub fn set(&mut self, id: CheckId, pass: bool) {
        let i = id as usize;
        self.required[i] = true;
        self.passed[i] = pass;
    }

    pub fn is_required(&self, id: CheckId) -> bool {
        self.required[id as usize]
    }

    pub fn passed(&self, id: CheckId) -> bool {
        self.passed[id as usize]
    }
}

/// The table verdict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableVerdict {
    Allowed,
    /// Blocked by the FIRST failing required row (gate order); carry its
    /// id — `reason_text` turns it into the operator STATUSTEXT.
    Blocked(CheckId),
}

/// The table gate: `Allowed` iff every REQUIRED row passes; otherwise the
/// first failing required row in table order. Total; monotone (proven:
/// PREFLIGHT-K03/K04).
pub fn arm_check_table(t: &CheckTable) -> TableVerdict {
    for id in CheckId::ALL {
        if t.is_required(id) && !t.passed(id) {
            return TableVerdict::Blocked(id);
        }
    }
    TableVerdict::Allowed
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    fn all_ok() -> PreflightChecks {
        PreflightChecks {
            sensors_healthy: true,
            estimator_converged: true,
            calibration_present: true,
            geofence_loaded: true,
            battery_ok: true,
            failsafe_configured: true,
        }
    }

    #[test]
    fn all_pass_arms() {
        assert_eq!(arm_check(all_ok()), ArmVerdict::Allowed);
    }

    #[test]
    fn any_fail_blocks() {
        let mut c = all_ok();
        c.battery_ok = false;
        assert_eq!(arm_check(c), ArmVerdict::Blocked(CheckFail::Battery));
    }

    #[test]
    fn first_failure_is_reported_in_priority() {
        let mut c = all_ok();
        c.sensors_healthy = false;
        c.battery_ok = false; // two fail; sensors has priority
        assert_eq!(arm_check(c), ArmVerdict::Blocked(CheckFail::Sensors));
    }

    /// PREARM-P03 per-row pair tests, TABLE-DRIVEN: for every row, (a) an
    /// all-pass table with that row failed blocks with EXACTLY that id and
    /// a distinct reason text; (b) clearing it re-allows. Coverage equals
    /// the table length by construction — a row added to CheckId::ALL is
    /// automatically covered.
    #[test]
    fn every_row_blocks_alone_and_clears() {
        use std::collections::HashSet;
        let mut texts = HashSet::new();
        for id in CheckId::ALL {
            let mut t = CheckTable::new();
            for other in CheckId::ALL {
                t.set(other, true);
            }
            assert_eq!(arm_check_table(&t), TableVerdict::Allowed);
            t.set(id, false);
            assert_eq!(
                arm_check_table(&t),
                TableVerdict::Blocked(id),
                "row {id:?} must block with its own id"
            );
            assert!(
                texts.insert(id.reason_text()),
                "reason text for {id:?} must be DISTINCT"
            );
            t.set(id, true);
            assert_eq!(arm_check_table(&t), TableVerdict::Allowed, "{id:?} clears");
        }
        assert_eq!(texts.len(), CHECK_COUNT);
    }

    /// Undeclared breadth rows do not gate (a bench without RC must not be
    /// hard-blocked by a link check it cannot satisfy) — but the legacy six
    /// are ALWAYS required.
    #[test]
    fn undeclared_rows_do_not_gate_legacy_always_do() {
        let mut t = CheckTable::new();
        for id in CheckId::ALL.iter().take(6) {
            t.set(*id, true);
        }
        assert_eq!(arm_check_table(&t), TableVerdict::Allowed, "six pass, rest undeclared");
        let fresh = CheckTable::new();
        assert_eq!(
            arm_check_table(&fresh),
            TableVerdict::Blocked(CheckId::SensorsHealthy),
            "a fresh table must not arm"
        );
    }

    #[test]
    fn default_is_all_failing_blocked() {
        // a fresh checks struct (all false) must NOT arm.
        assert_eq!(arm_check(PreflightChecks::default()), ArmVerdict::Blocked(CheckFail::Sensors));
    }
}
