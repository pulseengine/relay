//! falcon-core as a WebAssembly Component Model component (v1.26).
//!
//! The SAME verified IEKF → geometric SE(3) → ADRC → mixer cascade that runs
//! native (and bare-metal on Cortex-M) here runs the closed control loop
//! against the analytic SimBackend INSIDE a Component-Model component, so
//! wasmtime — or any CM runtime — can execute the verified core and observe it
//! stabilise. This is the keystone artifact the separate hardware-integration
//! project (meld → loom → synth → gale) consumes; this crate only has to make
//! the verified core a portable, runnable component.

#![cfg_attr(not(feature = "std"), no_std)]

// Bounded work-memory arena + the canonical-ABI `cabi_realloc` export + a panic
// handler, from the shared crate so the `unsafe` lives in one audited place.
// no_std keeps the component free of WASI and of `memory.grow`, which is what
// blocks `meld fuse --memory shared --address-rebase` (gale#89, meld#299).
#[cfg(not(feature = "std"))]
falcon_cm_rt::export_cm_rt!();

#[allow(warnings)]
mod bindings;

use bindings::Guest;
use falcon_core::{FlightCore, SimBackend};

struct Component;

impl Guest for Component {
    /// Recover a body tilted ~0.4 rad to level; return the final tilt (rad).
    fn run_stabilization() -> f32 {
        let dt = 0.002f32;
        let th = 0.4f32;
        let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let mut b = SimBackend::new(r0, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-2.0);
        for _ in 0..6000 {
            core.step(&mut b);
        }
        b.tilt()
    }

    /// Fly to a [2,-1.5,-2] m setpoint; return the final position error (m).
    fn run_position_hold() -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([2.0, -1.5, -2.0]);
        for _ in 0..30000 {
            core.step(&mut b);
        }
        let e = [b.pos[0] - 2.0, b.pos[1] + 1.5, b.pos[2] + 2.0];
        relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
    }
}

bindings::export!(Component with_types_in bindings);
