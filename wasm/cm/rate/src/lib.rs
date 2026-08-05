//! falcon-rate — the body-rate PID as a Component Model component.
//! Wraps `relay-rate::RatePid`.
//!
//! Stateful: the PID integrator + derivative state persist across `tick`
//! calls. Timestamps synthesised as a 1 kHz counter. The measured body rate
//! comes from `vehicle-state.{wx,wy,wz}` (the gyro reading the EKF component
//! passes through).
//!
//! `no_std` unless the `std` feature is on, so the component imports NO WASI — a
//! pure `falcon:cascade/types` -> `rate` transformer, which is what lowers to
//! bare metal (synth -> gale on M4/M7/F100) and what a WASI-less wasmtime host
//! can drive directly. (The single-threaded statics replace `thread_local!`,
//! whose `std` dependency was the sole reason WASI was linked.) The Bazel
//! bindings path keeps its existing environment.
//!
//! FEATURE SPLIT (v1.130): `bazel-bindings` selects the BINDINGS SOURCE only;
//! `std` selects std-vs-no_std. These were briefly conflated on one flag, which
//! made the Bazel build (which sets bazel-bindings) silently produce a
//! std/WASI-carrying artifact while the cargo-component build produced the
//! WASI-free one — two build systems, same source, materially different output.
//! rules_rust does not apply Cargo's default-features, so a Bazel build enables
//! ONLY the features it lists: `bazel-bindings` without `std` => no_std.

#![cfg_attr(not(feature = "std"), no_std)]

// no_std (cargo-component path) allocator — a BOUNDED BUMP ARENA, deliberately
// not a growing allocator.
//
// WHY NOT lol_alloc (what this used to be): a growing allocator emits
// `memory.grow`, and `memory.grow` blocks the single-address-space / no-grow
// MCU lowering that meld (`--memory shared`) and gale need — confirmed by
// disassembling the shipped falcon-rate:1.130.0, where the only `memory.grow`
// in the module sat inside lol_alloc. So the fix that made the component
// encode correctly also made it harder to lower. This is the same conclusion
// pulseengine/wit-bindgen reached in `cabi-realloc-extern` (back cabi_realloc
// with a bounded embedder arena instead of the global growing allocator —
// gale#89, meld#299); this is that idea applied locally, because that runtime
// feature is scoped to non-p2 targets and the release builds wasm32-wasip2.
//
// Sizing: the canonical ABI only allocates here for the SPILLED PARAMETER AREA
// of `rate.tick` — vehicle-state (14 f32) + rate-setpoint (4 f32) = 18 flat
// params exceeds the canonical MAX_FLAT_PARAMS (16), so the host writes the
// arguments into our memory and we return a pointer for the result. That is a
// few hundred bytes per call at most; 8 KiB is ample. Exhaustion TRAPS rather
// than growing — a bounded, analyzable failure mode, which is what the
// embedded target wants.
//
// A component instance is never re-entered concurrently by the runtime, so
// single-threaded access is sound.
#[cfg(not(feature = "std"))]
mod arena {
    use core::alloc::{GlobalAlloc, Layout};
    use core::cell::UnsafeCell;

    /// Bounded backing store. Sized for the spilled-parameter area (see above).
    const ARENA_BYTES: usize = 8 * 1024;

    #[repr(align(16))]
    struct Store(UnsafeCell<[u8; ARENA_BYTES]>);
    // SAFETY: the component model guarantees single-threaded, non-reentrant access.
    unsafe impl Sync for Store {}

    static STORE: Store = Store(UnsafeCell::new([0; ARENA_BYTES]));
    static NEXT: BumpCursor = BumpCursor(UnsafeCell::new(0));

    struct BumpCursor(UnsafeCell<usize>);
    // SAFETY: as above — single-threaded, non-reentrant.
    unsafe impl Sync for BumpCursor {}

    pub struct BumpArena;

    impl BumpArena {
        /// Reset the cursor. The canonical ABI hands us a fresh parameter area
        /// per call and never asks us to free, so reclaiming at the top of each
        /// export keeps a long-running component from exhausting the arena.
        #[inline]
        pub fn reset() {
            unsafe { *NEXT.0.get() = 0 };
        }
    }

    unsafe impl GlobalAlloc for BumpArena {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            unsafe {
                let cur = *NEXT.0.get();
                let base = STORE.0.get() as *mut u8;
                let aligned = (cur + layout.align() - 1) & !(layout.align() - 1);
                let end = match aligned.checked_add(layout.size()) {
                    Some(e) => e,
                    None => return core::ptr::null_mut(),
                };
                if end > ARENA_BYTES {
                    // Bounded failure: null propagates to a trap in the ABI
                    // shim rather than silently growing linear memory.
                    return core::ptr::null_mut();
                }
                *NEXT.0.get() = end;
                base.add(aligned)
            }
        }

        /// Bump arena: individual frees are no-ops; `reset()` reclaims in bulk.
        unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {}
    }
}

#[cfg(not(feature = "std"))]
#[global_allocator]
static ALLOC: arena::BumpArena = arena::BumpArena;

// The Component-Model canonical ABI requires the guest to export
// `cabi_realloc`. wit-bindgen-rt provides it ONLY when it can lean on std —
// its implementation is `#[cfg(not(target_env = "p2"))]`, because on
// wasm32-wasip2 it expects the std toolchain to supply the symbol. This crate
// is `no_std` on the cargo-component path, so nothing exported it and
// `wasm-component-ld` failed with "module does not export a function named
// `cabi_realloc`" — leaving a raw CORE MODULE where a component was expected.
// Export it here, backed by the same single-threaded allocator.
#[cfg(not(feature = "std"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn cabi_realloc(
    old_ptr: *mut u8,
    old_len: usize,
    align: usize,
    new_len: usize,
) -> *mut u8 {
    use core::alloc::{GlobalAlloc, Layout};
    unsafe {
        if old_len == 0 {
            // Canonical ABI: a zero-size allocation returns the alignment as a
            // non-null, suitably-aligned dangling pointer.
            if new_len == 0 {
                return align as *mut u8;
            }
            return ALLOC.alloc(Layout::from_size_align_unchecked(new_len, align));
        }
        ALLOC.realloc(
            old_ptr,
            Layout::from_size_align_unchecked(old_len, align),
            new_len,
        )
    }
}

// Bindings generated by cargo-component as src/bindings.rs.
#[allow(warnings)]
#[cfg(feature = "bazel-bindings")]
use falcon_rate_bindings as bindings;
#[cfg(not(feature = "bazel-bindings"))]
mod bindings;

use core::cell::RefCell;

use bindings::exports::falcon::cascade::rate::Guest;
use bindings::falcon::cascade::types::{RateSetpoint, TorqueSetpoint, VehicleState};

use relay_rate::{RatePid, Timestamp};

/// A wasm component instance is single-threaded and never re-entered
/// concurrently by the runtime, so wrapping the per-instance state in a
/// `Sync` cell is sound. This replaces `thread_local!` (which is `std`, and
/// pulled in the WASI imports).
struct SingleThreaded<T>(RefCell<T>);
// SAFETY: the component model guarantees single-threaded, non-reentrant access.
unsafe impl<T> Sync for SingleThreaded<T> {}

static PID: SingleThreaded<RatePid> = SingleThreaded(RefCell::new(RatePid::new()));
static TICK_MS: SingleThreaded<u64> = SingleThreaded(RefCell::new(0));

fn next_timestamp() -> Timestamp {
    let ms = *TICK_MS.0.borrow();
    *TICK_MS.0.borrow_mut() = ms + 1;
    Timestamp {
        seconds: ms / 1000,
        fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32,
    }
}

struct Component;

impl Guest for Component {
    fn tick(state: VehicleState, sp: RateSetpoint) -> TorqueSetpoint {
        // Reclaim the previous call's spilled-parameter area. The canonical ABI
        // never frees what it allocates through cabi_realloc, so without this a
        // long-running component would walk the bump cursor to exhaustion.
        // Sound here because the arena only ever holds the CURRENT call's
        // argument/result area — nothing allocated by a previous tick outlives it.
        #[cfg(not(feature = "std"))]
        arena::BumpArena::reset();

        let torque = PID.0.borrow_mut().tick(
            next_timestamp(),
            [state.wx, state.wy, state.wz],
            [sp.rx, sp.ry, sp.rz],
        );
        TorqueSetpoint {
            tx: torque[0],
            ty: torque[1],
            tz: torque[2],
            // Thrust passes straight through the rate loop.
            thrust: sp.thrust,
        }
    }
}

bindings::export!(Component with_types_in bindings);

// no_std (cargo-component path) needs a panic handler; the Bazel path (std)
// provides its own.
#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
