//! falcon-mix-component — `relay-mix-quad` exposed as a WASM export
//! surface for the v0.6 meld → wasm-opt → synth pipeline.
//!
//! The control crates (`relay-mix-quad` et al.) are pure no_std
//! libraries. This thin wrapper gives them a flat `extern "C"` export
//! surface so the crate compiles to a `wasm32-unknown-unknown` core
//! module that:
//!
//!   - runs in `wasmtime` as the reference oracle,
//!   - fuses with sibling components via `meld fuse`,
//!   - optimises through `wasm-opt`,
//!   - AOT-compiles to ARM Cortex-M via `synth compile --cortex-m`.
//!
//! The export surface is deliberately scalar-in / scalar-out: no
//! linear-memory pointers, no allocation. That keeps the wasm trivial
//! for `synth` to lower to ARM and trivial for `wasmtime` / Renode to
//! drive. The structured WIT-component surface (records, lists) is a
//! later step once the synth backend handles the canonical ABI.
//!
//! When built as `rlib` (the default `cargo test` path) the same
//! functions are plain Rust and unit-tested against `relay-mix-quad`
//! directly.

#![cfg_attr(target_arch = "wasm32", no_std)]

use relay_mix_quad::QuadMixer;

/// Mix one motor command for the falcon-quad X-config airframe.
///
/// `idx` selects the motor (0–3, wraps mod 4). `roll`/`pitch`/`yaw`
/// are the body-frame torque commands, `thrust` the collective.
/// Returns that motor's PWM value in `[0, 1]`.
///
/// Exported with the kebab-case name the WIT world declares
/// (`falcon-mix-motor`) so `wasm-tools component embed` lifts it
/// into the component's interface and `meld` can fuse it. The WIT
/// world (`wit/falcon-control.wit`) is the spar-codegen shape.
#[unsafe(export_name = "falcon-mix-motor")]
pub extern "C" fn falcon_mix_motor(
    idx: u32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    thrust: f32,
) -> f32 {
    let mut mixer = QuadMixer::new();
    let motors = mixer.mix([roll, pitch, yaw], thrust);
    motors[(idx % 4) as usize]
}

/// Sum of all four motor commands — a cheap scalar digest of a full
/// mix, handy as a single-number reference check across the pipeline
/// (wasmtime vs native vs synth-on-Renode all compare this).
#[unsafe(export_name = "falcon-mix-total")]
pub extern "C" fn falcon_mix_total(roll: f32, pitch: f32, yaw: f32, thrust: f32) -> f32 {
    let mut mixer = QuadMixer::new();
    let m = mixer.mix([roll, pitch, yaw], thrust);
    m[0] + m[1] + m[2] + m[3]
}

// On the wasm32 target a cdylib needs a panic handler (no std).
#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn motor_export_matches_native_mixer() {
        let mut native = QuadMixer::new();
        let expected = native.mix([0.3, -0.2, 0.1], 0.5);
        for idx in 0..4u32 {
            let got = falcon_mix_motor(idx, 0.3, -0.2, 0.1, 0.5);
            assert!((got - expected[idx as usize]).abs() < 1.0e-6,
                "motor {} export {} != native {}", idx, got, expected[idx as usize]);
        }
    }

    #[test]
    fn total_export_is_sum_of_motors() {
        let mut native = QuadMixer::new();
        let m = native.mix([0.0, 0.0, 0.0], 0.5);
        let expected = m[0] + m[1] + m[2] + m[3];
        let got = falcon_mix_total(0.0, 0.0, 0.0, 0.5);
        assert!((got - expected).abs() < 1.0e-6);
        // Zero torque, 0.5 thrust → all four motors at 0.5 → total 2.0.
        assert!((got - 2.0).abs() < 1.0e-6);
    }

    #[test]
    fn idx_wraps_mod_four() {
        let a = falcon_mix_motor(0, 0.1, 0.1, 0.1, 0.5);
        let b = falcon_mix_motor(4, 0.1, 0.1, 0.1, 0.5);
        assert_eq!(a, b);
    }
}
