//! Relay EKF stub — v0.1 placeholder.
//!
//! Emits a constant identity-attitude, zero-velocity, zero-position
//! `VehicleState` on every tick. Real complementary-filter EKF lands
//! in v0.2 (`relay-ekf`); this stub exists so the falcon-quad world
//! has a state-estimator component to wire up in v0.1.
//!
//! What's tested:
//!   - `EkfStub::tick()` always returns a unit quaternion (the
//!     v0.1 surrogate for SWREQ-FALCON-EKF-P01 — see proofs/ when
//!     the real EKF lands).
//!   - Output is deterministic (same tick → same state).
//!   - `innovation` is below the warning threshold (the stub never
//!     reports sensor disagreement because it doesn't read sensors).

#![no_std]
#![forbid(unsafe_code)]

/// Timestamp: seconds + 2^32-fraction. Mirrors
/// `pulseengine:relay-common-types/types.{timestamp}` for v0.1
/// without requiring the wit-bindgen integration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timestamp {
    pub seconds: u64,
    pub fraction: u32,
}

/// Vehicle state — the EKF's output. Mirrors
/// `pulseengine:relay-control/dynamics.{vehicle-state}`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleState {
    pub time: Timestamp,
    /// Unit quaternion (w, x, y, z), body-to-NED.
    pub quaternion: [f32; 4],
    /// Position in NED frame, meters.
    pub position_ned: [f32; 3],
    /// Velocity in NED frame, m/s.
    pub velocity_ned: [f32; 3],
    /// Innovation health metric (0.0 = perfect agreement).
    pub innovation: f32,
}

/// The stub EKF — holds the last-tick time so output is "fresh".
#[derive(Clone, Copy, Debug, Default)]
pub struct EkfStub {
    last_time: Timestamp,
}

impl EkfStub {
    pub const fn new() -> Self {
        Self { last_time: Timestamp { seconds: 0, fraction: 0 } }
    }

    /// Advance the stub by one tick. The real EKF (v0.2+) consumes
    /// IMU/GPS/mag/baro streams here; the stub just stamps the
    /// current time and emits identity attitude.
    pub fn tick(&mut self, time: Timestamp) -> VehicleState {
        self.last_time = time;
        VehicleState {
            time,
            quaternion: [1.0, 0.0, 0.0, 0.0],
            position_ned: [0.0, 0.0, 0.0],
            velocity_ned: [0.0, 0.0, 0.0],
            innovation: 0.0,
        }
    }

    /// Time of the last tick.
    pub const fn last_time(&self) -> Timestamp {
        self.last_time
    }
}

/// Tolerance used by `is_unit_quaternion` checks in tests.
pub const QUATERNION_NORM_TOLERANCE: f32 = 1.0e-6;

/// Check that a quaternion is unit-norm within tolerance.
/// Mirrors the contract that the real EKF will satisfy under Verus.
pub fn is_unit_quaternion(q: [f32; 4]) -> bool {
    let norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    let diff = if norm_sq >= 1.0 { norm_sq - 1.0 } else { 1.0 - norm_sq };
    diff <= QUATERNION_NORM_TOLERANCE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_stub_has_zero_time() {
        let s = EkfStub::new();
        assert_eq!(s.last_time(), Timestamp { seconds: 0, fraction: 0 });
    }

    #[test]
    fn tick_returns_identity_quaternion() {
        let mut s = EkfStub::new();
        let state = s.tick(Timestamp { seconds: 1, fraction: 0 });
        assert_eq!(state.quaternion, [1.0, 0.0, 0.0, 0.0]);
        assert!(is_unit_quaternion(state.quaternion));
    }

    #[test]
    fn tick_returns_zero_position_and_velocity() {
        let mut s = EkfStub::new();
        let state = s.tick(Timestamp { seconds: 1, fraction: 0 });
        assert_eq!(state.position_ned, [0.0, 0.0, 0.0]);
        assert_eq!(state.velocity_ned, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn tick_passes_time_through() {
        let mut s = EkfStub::new();
        let t = Timestamp { seconds: 42, fraction: 12345 };
        let state = s.tick(t);
        assert_eq!(state.time, t);
        assert_eq!(s.last_time(), t);
    }

    #[test]
    fn innovation_is_quiet_in_stub() {
        // Real EKF reports innovation as residual magnitude; stub
        // never sees sensor disagreement, so it stays at 0.0.
        let mut s = EkfStub::new();
        let state = s.tick(Timestamp { seconds: 0, fraction: 0 });
        assert_eq!(state.innovation, 0.0);
    }

    #[test]
    fn deterministic_across_ticks_with_same_time() {
        let mut a = EkfStub::new();
        let mut b = EkfStub::new();
        let t = Timestamp { seconds: 7, fraction: 0 };
        let state_a = a.tick(t);
        let state_b = b.tick(t);
        assert_eq!(state_a, state_b);
    }

    #[test]
    fn unit_quaternion_check_rejects_clearly_non_unit() {
        assert!(!is_unit_quaternion([2.0, 0.0, 0.0, 0.0]));
        assert!(!is_unit_quaternion([0.0, 0.0, 0.0, 0.0]));
        assert!(is_unit_quaternion([1.0, 0.0, 0.0, 0.0]));
        assert!(is_unit_quaternion([0.0, 1.0, 0.0, 0.0]));
    }

    use proptest::prelude::*;

    proptest! {
        #[test]
        fn tick_always_emits_unit_quaternion(
            seconds in any::<u64>(),
            fraction in any::<u32>(),
        ) {
            let mut s = EkfStub::new();
            let state = s.tick(Timestamp { seconds, fraction });
            prop_assert!(is_unit_quaternion(state.quaternion));
        }

        #[test]
        fn tick_innovation_within_healthy_range(
            seconds in any::<u64>(),
        ) {
            let mut s = EkfStub::new();
            let state = s.tick(Timestamp { seconds, fraction: 0 });
            // Healthy innovation < 1.0 sigma; stub is at 0.
            prop_assert!(state.innovation < 1.0);
        }
    }
}
