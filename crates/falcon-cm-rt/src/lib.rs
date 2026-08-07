//! `no_std` Component-Model runtime support shared by the falcon cascade
//! components.
//!
//! Every published cascade stage needs the same three things to build `no_std`
//! and still satisfy the canonical ABI, and each is subtle enough that six
//! hand-written copies would be six chances to get the `unsafe` wrong:
//!
//! 1. **A bounded work-memory arena.** The platform rule is *bounded static
//!    work memory is acceptable, a heap or anything that grows is not*. A
//!    growing allocator emits `memory.grow`, and `memory.grow` is exactly what
//!    makes `meld fuse --memory shared --address-rebase` reject a component
//!    (gale#89, meld#299), blocking the single-address-space MCU lowering these
//!    components exist for.
//! 2. **A `cabi_realloc` export.** `wit-bindgen-rt` supplies it only when std
//!    is linked (`#[cfg(not(target_env = "p2"))]` — on `wasm32-wasip2` it
//!    expects wasi-libc to). Without it `wasm-component-ld` cannot encode a
//!    component and silently leaves a raw CORE MODULE, which is how
//!    `falcon-rate:1.129.0` shipped unusable.
//! 3. **A panic handler**, since `no_std` has none.
//!
//! Sizing: the ABI allocates here only for a call's SPILLED PARAMETER AREA —
//! when an export's flattened parameters exceed the canonical
//! `MAX_FLAT_PARAMS` (16), the host writes the arguments into our memory and we
//! return a pointer. That is a few hundred bytes per call. Exhaustion TRAPS
//! rather than growing: a bounded, analyzable failure mode.
//!
//! A component instance is never re-entered concurrently by the runtime, so
//! single-threaded access is sound.

#![no_std]

use core::alloc::{GlobalAlloc, Layout};
use core::cell::UnsafeCell;

/// Bounded backing store for the canonical ABI's spilled-parameter area.
const ARENA_BYTES: usize = 8 * 1024;

#[repr(align(16))]
struct Store(UnsafeCell<[u8; ARENA_BYTES]>);
// SAFETY: the component model guarantees single-threaded, non-reentrant access.
unsafe impl Sync for Store {}

struct Cursor(UnsafeCell<usize>);
// SAFETY: as above.
unsafe impl Sync for Cursor {}

static STORE: Store = Store(UnsafeCell::new([0; ARENA_BYTES]));
static NEXT: Cursor = Cursor(UnsafeCell::new(0));

/// Bump arena. Never grows; traps (via a null return) on exhaustion.
pub struct BumpArena;

impl BumpArena {
    /// Reclaim the previous call's parameter area.
    ///
    /// The canonical ABI never frees what it allocates through `cabi_realloc`,
    /// so without this a long-running component walks the cursor to
    /// exhaustion. Call it at the top of each exported entry point: sound
    /// because the arena only ever holds the CURRENT call's argument/result
    /// area — nothing allocated by a previous call outlives it.
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
                // Bounded failure: null rather than silently growing memory.
                return core::ptr::null_mut();
            }
            *NEXT.0.get() = end;
            base.add(aligned)
        }
    }

    /// Bump arena: individual frees are no-ops; `reset()` reclaims in bulk.
    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {}
}

/// The canonical-ABI reallocation entry point, backed by [`BumpArena`].
///
/// Exported by each component via [`export_cm_rt!`] rather than from here, so
/// the symbol lands in the component's own module.
///
/// # Safety
/// Callers must honour the canonical-ABI realloc contract.
pub unsafe fn cabi_realloc_impl(
    old_ptr: *mut u8,
    old_len: usize,
    align: usize,
    new_len: usize,
) -> *mut u8 {
    unsafe {
        if old_len == 0 {
            // Canonical ABI: a zero-size allocation returns the alignment as a
            // non-null, suitably-aligned dangling pointer.
            if new_len == 0 {
                return align as *mut u8;
            }
            return BumpArena.alloc(Layout::from_size_align_unchecked(new_len, align));
        }
        BumpArena.realloc(
            old_ptr,
            Layout::from_size_align_unchecked(old_len, align),
            new_len,
        )
    }
}

/// Install the global allocator, the `cabi_realloc` export and a panic handler.
///
/// These must be defined in the component crate itself — `#[global_allocator]`,
/// `#[no_mangle]` and `#[panic_handler]` are all per-final-artifact — so this
/// macro emits them while the logic stays in one audited place.
#[macro_export]
macro_rules! export_cm_rt {
    () => {
        #[global_allocator]
        static __FALCON_CM_ALLOC: $crate::BumpArena = $crate::BumpArena;

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn cabi_realloc(
            old_ptr: *mut u8,
            old_len: usize,
            align: usize,
            new_len: usize,
        ) -> *mut u8 {
            unsafe { $crate::cabi_realloc_impl(old_ptr, old_len, align, new_len) }
        }

        #[panic_handler]
        fn __falcon_cm_panic(_: &core::panic::PanicInfo) -> ! {
            loop {}
        }
    };
}
