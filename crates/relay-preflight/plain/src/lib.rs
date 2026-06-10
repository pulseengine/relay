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

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
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

    #[test]
    fn default_is_all_failing_blocked() {
        // a fresh checks struct (all false) must NOT arm.
        assert_eq!(arm_check(PreflightChecks::default()), ArmVerdict::Blocked(CheckFail::Sensors));
    }
}
