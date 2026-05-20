//! falcon-rate-component — `relay-rate` exposed as a WASM export
//! surface for the v0.6 meld → wasm-opt → synth pipeline.
//!
//! Same scalar-ABI discipline as `falcon-mix-component`: no linear-
//! memory pointers, no allocation, so the canonical ABI is the flat
//! core-wasm signature and `synth` lowers it to ARM directly.
//!
//! The rate controller is stateful, so the export runs a fixed
//! closed-loop digest: spin a `RatePid` against a trivial
//! integrator plant for a bounded number of ticks and return a
//! scalar summary of the response. The same code path is unit-
//! tested natively against `relay-rate` so the wasm export is a
//! faithful proxy.

#![cfg_attr(target_arch = "wasm32", no_std)]

use relay_rate::{RatePid, Timestamp};

/// Closed-loop step-response digest.
///
/// Commands a constant body-rate setpoint about the x-axis and runs
/// the PID against a pure-integrator plant (`ω̇ = τ / I`) for one
/// simulated second at 1 kHz. Returns the final tracked rate — for a
/// well-tuned loop this converges to `setpoint_x`.
///
/// This exercises the real `RatePid::tick` arithmetic (PID, anti-
/// windup, clamp) so the pipeline compiles genuine controller code,
/// not a toy.
#[unsafe(export_name = "falcon-rate-step-digest")]
pub extern "C" fn falcon_rate_step_digest(setpoint_x: f32) -> f32 {
    let mut pid = RatePid::new();
    let dt = 1.0_f32 / 1000.0;
    let inertia = 0.005_f32;
    let mut omega = [0.0_f32; 3];
    let setpoint = [setpoint_x, 0.0, 0.0];
    // Integer timestamp math — `f32::fract` is std-only and this
    // crate is no_std on the wasm32 target.
    for ms in 1..=1000u64 {
        let frac = ((ms % 1000) * (1u64 << 32) / 1000) as u32;
        let torque = pid.tick(
            Timestamp { seconds: ms / 1000, fraction: frac },
            omega,
            setpoint,
        );
        for k in 0..3 {
            omega[k] += (torque[k] / inertia) * dt;
        }
    }
    omega[0]
}

/// Single-tick torque digest: one `RatePid::tick` from rest with the
/// given setpoint, returns the x-axis torque. A cheap scalar check
/// that needs no loop — fastest cross-pipeline reference number.
#[unsafe(export_name = "falcon-rate-torque")]
pub extern "C" fn falcon_rate_torque(setpoint_x: f32) -> f32 {
    let mut pid = RatePid::new();
    let torque = pid.tick(
        Timestamp { seconds: 0, fraction: 1 },
        [0.0; 3],
        [setpoint_x, 0.0, 0.0],
    );
    torque[0]
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn step_digest_converges_toward_setpoint() {
        let final_rate = falcon_rate_step_digest(1.0);
        // One second of 1 kHz control should bring the rate close to
        // the 1.0 rad/s setpoint on this plant.
        assert!((final_rate - 1.0).abs() < 0.2,
            "step digest {} not near setpoint 1.0", final_rate);
    }

    #[test]
    fn torque_digest_sign_follows_setpoint() {
        assert!(falcon_rate_torque(1.0) > 0.0);
        assert!(falcon_rate_torque(-1.0) < 0.0);
        assert_eq!(falcon_rate_torque(0.0), 0.0);
    }
}
