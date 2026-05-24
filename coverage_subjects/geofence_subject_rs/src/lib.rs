//! Witness MC/DC coverage subject — exercises real relay-lc
//! `Geofence::check` from a WASI-free wasm32-unknown-unknown core
//! module, so witness's embedded runtime can drive it without the
//! WASI gap that blocks the full cascade target.
//!
//! Strategy: compile this crate with `panic = "abort"` + `opt-level
//! = "z"` + `lto`; the verified `Geofence::check` has no panic
//! paths (Verus LC-P09/P10 + Kani LC-K01..K05), so the link step
//! produces a module with **zero** WASI imports.
//!
//! Three exports drive distinct branches of the latch state machine:
//!
//!   run_inside    -> in-fence sample, latch stays off (the
//!                    `else` arm + the `else` of the outside check)
//!   run_outside   -> out-of-fence sample on a fresh fence (the
//!                    `else` arm + the `if` of the outside check)
//!   run_latched   -> already-latched fence, any sample (the early
//!                    `if self.violation_latched` return)
//!
//! Together they exercise every decision branch in `check()`, so
//! the LCOV report from witness has every line of the verified
//! function visited.

// no_std + custom panic handler are only needed for the wasm32
// target; on the host the std panic handler is already supplied
// and the wasm32::unreachable intrinsic doesn't exist.
#![cfg_attr(target_arch = "wasm32", no_std)]
// Witness needs unmangled exports to discover the entry points; the
// alternative would be a wit-bindgen component, which would defeat
// the WASI-free goal. We allow the warning here because every
// `no_mangle` symbol is a const string we control, and the crate is
// only ever loaded as a witness subject (no library use).
#![allow(unsafe_code)]

use relay_lc::engine::Geofence;

/// Panic-handler: aborts. Verus + Kani prove `Geofence::check`
/// never panics, so this should never be reached — but Rust still
/// requires a `#[panic_handler]` for the wasm32-unknown-unknown
/// target since the subject is `no_std` there.
#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

/// Build a fresh 100 m × 100 m × 100 m fence centred on origin.
/// `#[inline(never)]` keeps the fn body in the emitted module so
/// witness can attribute branch counters to it; black_box on the
/// inputs prevents the optimizer from constant-folding the call.
#[inline(never)]
fn fence(min: i32, max: i32) -> Geofence {
    let min = core::hint::black_box(min);
    let max = core::hint::black_box(max);
    Geofence::new(min, max, min, max, min, max)
}

/// Drive Geofence::check through a non-elidable path. `#[inline(never)]`
/// + `black_box` on the inputs prevents the optimizer from merging
/// functionally-identical call sites (run_latched & run_inside both
/// return 0 but exercise distinct branches in check()).
#[inline(never)]
fn drive(mut g: Geofence, n: i32, e: i32, d: i32, latched_pre: bool) -> i32 {
    let n = core::hint::black_box(n);
    let e = core::hint::black_box(e);
    let d = core::hint::black_box(d);
    if latched_pre {
        g.violation_latched = true;
    }
    if g.check(n, e, d) { 1 } else { 0 }
}

/// Inside-the-fence: latch stays off (the `else` of the outside check).
#[unsafe(no_mangle)]
pub extern "C" fn run_inside() -> i32 {
    drive(fence(-10_000, 10_000), 0, 0, 0, false)
}

/// Outside-the-fence: latch trips on first call (the `if` of the outside check).
#[unsafe(no_mangle)]
pub extern "C" fn run_outside() -> i32 {
    drive(fence(-10_000, 10_000), 0, 20_000, 0, false)
}

/// Already-latched: early-return path (the `if self.violation_latched`).
#[unsafe(no_mangle)]
pub extern "C" fn run_latched() -> i32 {
    drive(fence(-10_000, 10_000), 0, 100_000, 0, true)
}
