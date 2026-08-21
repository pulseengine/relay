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
use bindings::pulseengine::falcon_cascade::types::{ImuSample, VehicleState};

use relay_iekf::{Iekf, Imu as RImu};

/// Lazily initialised: `Iekf::level()` is not a `const fn` (it builds a
/// NavState), and unlike `thread_local!` a plain `static` needs a const
/// initialiser. Making the constructor const would mean editing relay-iekf —
/// a Kani-verified flight crate — for the sake of a static, so the laziness
/// lives here in the component instead.
static IEKF: SingleThreaded<Option<Iekf>> = SingleThreaded(RefCell::new(None));

struct Component;

impl Guest for Component {
    fn estimate(imu: ImuSample) -> VehicleState {
        let accel = [imu.ax, imu.ay, imu.az];
        let gyro = [imu.gx, imu.gy, imu.gz];
        let st = {
            let mut guard = IEKF.0.borrow_mut();
            let f = guard.get_or_insert_with(Iekf::level);
            // 1 kHz inner step: propagate the SE₂(3) state, then the gravity
            // (tilt) update — the verified attitude estimate.
            f.propagate(RImu { gyro, accel }, 0.001);
            f.update_gravity(accel, 0.5);
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
