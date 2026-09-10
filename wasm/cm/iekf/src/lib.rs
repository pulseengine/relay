//! falcon-iekf — the VERIFIED Invariant-EKF (SE₂(3)) as a Component Model
//! component, exporting the same `pulseengine:falcon-cascade/ekf` interface as the v0.6
//! Mahony `falcon-ekf` component. This is the v1.4 step of replacing the old
//! cascade with the verified stack: the IEKF is now a real WIT component, so
//! it can be composed (wac) and fused (meld) like any other.
//!
//! Stateful: the `Iekf` (SE₂(3) state + 15×15 covariance) persists across
//! `estimate` calls in a component-instance-local cell. Each call propagates
//! the filter with the IMU and runs the gravity (tilt) update — the
//! attitude-estimate role the cascade's `ekf` slot fills.

#![cfg_attr(not(feature = "std"), no_std)]

// Bounded work-memory arena + the canonical-ABI `cabi_realloc` export + a panic
// handler, from the shared crate so the `unsafe` lives in one audited place.
// no_std keeps the component free of WASI and of `memory.grow`, which is what
// blocks `meld fuse --memory shared --address-rebase` (gale#89, meld#299).
#[cfg(not(feature = "std"))]
falcon_cm_rt::export_cm_rt!();

#[allow(warnings)]
#[cfg(feature = "bazel-bindings")]
use falcon_iekf_bindings as bindings;
#[cfg(not(feature = "bazel-bindings"))]
mod bindings;

use core::cell::RefCell;

/// A component instance is single-threaded and never re-entered concurrently by
/// the runtime, so a `Sync` cell is sound. Replaces `thread_local!`, whose
/// `std` dependency was what pulled the WASI imports in.
struct SingleThreaded<T>(RefCell<T>);
// SAFETY: the component model guarantees single-threaded, non-reentrant access.
unsafe impl<T> Sync for SingleThreaded<T> {}

use bindings::exports::pulseengine::falcon_cascade::ekf::Guest;
use bindings::pulseengine::falcon_cascade::types::{SensorFrame, VehicleState};

use relay_iekf::{Iekf, Imu as RImu};

/// Lazily initialised: `Iekf::level()` is not a `const fn` (it builds a
/// NavState), and unlike `thread_local!` a plain `static` needs a const
/// initialiser. Making the constructor const would mean editing relay-iekf —
/// a Kani-verified flight crate — for the sake of a static, so the laziness
/// lives here in the component instead.
static IEKF: SingleThreaded<Option<Iekf>> = SingleThreaded(RefCell::new(None));

struct Component;

impl Guest for Component {
    fn estimate(sensors: SensorFrame) -> VehicleState {
        let imu = sensors.imu;
        let accel = [imu.ax, imu.ay, imu.az];
        let gyro = [imu.gx, imu.gy, imu.gz];

        // v0.8: the host's ACTUAL period, not a hardcoded 1 kHz. v0.7 assumed
        // 0.001 unconditionally, so a 250 Hz host integrated 1 ms of motion
        // per 4 ms of real time and accumulated 27 m of altitude error with no
        // error signal of any kind. Clamped to [0.1 ms, 100 ms] (10 kHz..10 Hz)
        // so a garbage frame cannot wind the filter; a non-finite dt falls back
        // to the v0.7 constant, which is the only value that was ever safe to
        // assume.
        let dt = if sensors.dt_s.is_finite() {
            sensors.dt_s.clamp(0.0001, 0.1)
        } else {
            0.001
        };

        let st = {
            let mut guard = IEKF.0.borrow_mut();
            let f = guard.get_or_insert_with(|| {
                let mut f = Iekf::level();
                // CONFIG PARITY with falcon-core (v1.113): without a
                // covariance floor the filter goes deaf on a static hover —
                // P collapses, the NIS gate starts rejecting CORRECT position
                // fixes, and the estimate locks onto a wrong state while
                // reporting high confidence.
                //
                // Measured, 250 Hz, 0.2 m/s^2 accel noise, 5 Hz fixes supplied
                // and accepted by the interface: WITHOUT the floor the vehicle
                // fell to -123 m whether or not position was offered (-123.18
                // with fixes vs -123.32 without — i.e. the fixes were being
                // thrown away). The native bench sets exactly these values in
                // both of its FlightCore scenarios.
                f.set_process_floor(0.30, 0.05);
                f
            });
            // Propagate the SE_2(3) state, then the gravity (tilt) update —
            // the verified attitude estimate.
            f.propagate(RImu { gyro, accel }, dt);
            f.update_gravity(accel, 0.5);

            // v0.8 aiding. Variances match falcon-core's defaults exactly
            // (grav 0.5 / pos 0.01 / mag 0.1) so the composed cascade and the
            // native FlightCore fuse identically — a different constant here
            // would make the two paths quietly disagree.
            //
            // Every field is optional and `none` reproduces v0.7 behaviour, so
            // an IMU-only host stays valid. It also stays divergent: on the
            // SITL plant at 0.2 m/s^2 accelerometer noise, IMU-only inverted to
            // -121 m where the native cascade held 0.39 m.
            if let Some(p) = sensors.position_ned {
                f.update_position([p.x, p.y, p.z], 0.01);
            }
            if let Some(m) = sensors.mag_body {
                f.update_magnetometer([m.x, m.y, m.z], 0.0, 0.1);
            }
            if let Some(yaw) = sensors.heading_rad {
                f.update_yaw(yaw, 0.1);
            }
            f.state()
        };
        VehicleState {
            qw: st.q[0],
            qx: st.q[1],
            qy: st.q[2],
            qz: st.q[3],
            pos_n: st.p[0],
            pos_e: st.p[1],
            pos_d: st.p[2],
            vel_n: st.v[0],
            vel_e: st.v[1],
            vel_d: st.v[2],
            // Body rates are the gyro reading, passed through for the rate loop.
            wx: imu.gx,
            wy: imu.gy,
            wz: imu.gz,
            innovation: 0.0,
        }
    }
}

bindings::export!(Component with_types_in bindings);
