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
//! # Arena reclamation: [`CallScope`] (#574)
//!
//! The canonical ABI allocates through `cabi_realloc` for a call's SPILLED
//! PARAMETER AREA — when an export's flattened parameters exceed
//! `MAX_FLAT_PARAMS` (16), the host calls `cabi_realloc` and writes the
//! arguments into our memory BEFORE the export runs. The generated glue then
//! calls `cabi_dealloc`, which is a no-op in a bump arena, so without
//! reclamation every such call leaks its parameter area: `position#tick`
//! (18 flat params) leaks 72 B per call and `attitude#tick` (19) 76 B, which
//! exhausts the 8 KiB arena after roughly a hundred calls.
//!
//! The arena is therefore reclaimed per CALL, by a state machine
//! ([`ArenaState`]) with a cursor, a `returned` flag and a nesting depth:
//!
//! * **An export opens a scope as its first statement**:
//!   `let _call = falcon_cm_rt::CallScope::enter();` — bound to a NAMED
//!   variable, never `let _ = ..`, which would drop it at once.
//! * **Dropping the scope marks the call returned.** It does NOT rewind. The
//!   call's results may still live in the arena while the host lifts them and
//!   runs the post-return function, so reclaiming is deferred.
//! * **The next host allocation reclaims.** A `cabi_realloc` from the host
//!   while the previous call has returned is, by the canonical ABI's
//!   sequencing, the lowering of the NEXT call's arguments: the host has
//!   finished lifting the previous results by then. So it rewinds the cursor
//!   to 0, clears `returned`, and allocates the new arguments at the start.
//! * **`enter` reclaims only if nothing has been allocated since the return.**
//!   If the host spilled this call's arguments, the allocation above already
//!   cleared `returned`, so `enter` keeps them. If the parameters were passed
//!   flat (no allocation), `returned` is still set and `enter` rewinds to 0.
//!
//! Results therefore stay valid until the next call allocates or enters, and
//! the soundness of reclamation no longer depends on the generated glue copying
//! scalar arguments out before the body runs (true of wit-bindgen 0.41 for the
//! all-scalar records on this seam today, but not part of the ABI contract and
//! false for any `list`/`string` parameter, which the guest BORROWS from the
//! argument area).
//!
//! ## Design decisions
//!
//! **Only the HOST allocation path reclaims.** Reclaim-on-allocation is done in
//! [`cabi_realloc_impl`] (the symbol the canonical options name as `realloc`),
//! never in the `GlobalAlloc` path the guest itself uses. Generated glue that
//! lowers a result can allocate AFTER the body has returned — i.e. after the
//! scope dropped — while the value it is lowering still lives in the arena; if
//! that allocation reclaimed, it would be handed memory overlapping its own
//! source. A post-return guest allocation therefore only bumps, and is
//! reclaimed with everything else by the next host allocation or `enter`.
//! (On this seam today results go to the glue's static `_RET_AREA`, so this is
//! forward-looking — it is what keeps a `list` result sound.) A host
//! `cabi_realloc` that RESIZES an existing block (`old_len != 0`) never
//! reclaims either: the old block is live.
//!
//! **Exhaustion is a bounded failure, never a wrap or an overlap.** The
//! allocator returns null when a request does not fit, its alignment is not a
//! power of two, or it exceeds the store's 16-byte alignment (the offset is
//! aligned against a 16-aligned base, so a larger alignment cannot be honoured
//! by offset arithmetic and is refused rather than silently mis-aligned). All
//! cursor arithmetic is checked. At the ABI boundary a null is NOT returned to
//! the host: address 0 is valid linear memory (the bottom of the shadow stack),
//! and a host would write the arguments there without complaint. So
//! `cabi_realloc` TRAPS (`unreachable`) instead.
//!
//! **Panic / trap mid-scope.** A trap ends the call without running `Drop`, so
//! `returned` stays false. The Component Model forbids re-entering an instance
//! that trapped; were it re-entered anyway, `enter` would not reclaim and
//! allocation would continue from the cursor — a leak that ends in the
//! exhaustion trap above, never an overlap with live data. The same holds for
//! the panic handler, which spins (`loop {}`) rather than trapping, under the
//! components' `panic = "abort"` release profile: the scope never drops.
//!
//! **Nesting.** The scope counts depth, so if one export's body ever calls
//! another export's `Guest` method in-crate, the inner drop does not mark the
//! outer call returned.
//!
//! ## Preconditions
//!
//! * **Single-threaded, non-reentrant.** A component instance is never entered
//!   concurrently, and the host does not call back into an export while one is
//!   running. This is the same assumption the components' `SingleThreaded`
//!   state cells rest on.
//! * **Synchronous lifting.** "Returned" means the core export function has
//!   returned. An async-lifted export whose task outlives the core call would
//!   need a different notion of when its arguments die.
//! * **Every export opens a scope.** An export without one runs with the
//!   previous call's `returned` still set, so a host allocation during it (an
//!   import's result) would reclaim memory its own body is using.
//! * **Nothing allocated in the arena outlives the call that allocated it.**
//!   The arena is transient work memory; it is not a heap for persistent
//!   state.

#![no_std]

#[cfg(test)]
extern crate std;

use core::alloc::{GlobalAlloc, Layout};
use core::cell::UnsafeCell;

/// Bounded backing store for the canonical ABI's spilled-parameter area.
const ARENA_BYTES: usize = 8 * 1024;

/// The alignment of [`Store`]. Offsets aligned to at most this are addresses
/// aligned to the same.
const STORE_ALIGN: usize = 16;

#[repr(align(16))]
struct Store(UnsafeCell<[u8; ARENA_BYTES]>);
// SAFETY: the component model guarantees single-threaded, non-reentrant access.
unsafe impl Sync for Store {}

struct Shared(UnsafeCell<ArenaState>);
// SAFETY: as above.
unsafe impl Sync for Shared {}

static STORE: Store = Store(UnsafeCell::new([0; ARENA_BYTES]));
static STATE: Shared = Shared(UnsafeCell::new(ArenaState::new()));

/// The arena's bookkeeping, as a plain value so it can be unit-tested and
/// model-checked (see `kani_proofs.rs`) without the static store.
///
/// Invariants, all established by the methods below:
/// * `cursor <= ARENA_BYTES`;
/// * `returned` implies `depth == 0`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ArenaState {
    /// Offset of the first free byte.
    pub(crate) cursor: usize,
    /// The previous call has returned and nothing has been allocated by the
    /// host or entered since — the arena holds nothing live.
    pub(crate) returned: bool,
    /// Number of open [`CallScope`]s.
    pub(crate) depth: u32,
}

impl ArenaState {
    /// Empty arena, no call in flight.
    pub(crate) const fn new() -> Self {
        ArenaState {
            cursor: 0,
            returned: true,
            depth: 0,
        }
    }

    /// Bump-allocate `size` bytes at `align`, returning the OFFSET. Never
    /// reclaims. `None` when the request does not fit, or `align` is not a
    /// power of two no larger than [`STORE_ALIGN`]. On `None` the cursor is
    /// unchanged.
    pub(crate) fn bump(&mut self, size: usize, align: usize) -> Option<usize> {
        if !align.is_power_of_two() || align > STORE_ALIGN {
            return None;
        }
        let aligned = self.cursor.checked_add(align - 1)? & !(align - 1);
        let end = aligned.checked_add(size)?;
        if end > ARENA_BYTES {
            return None;
        }
        self.cursor = end;
        Some(aligned)
    }

    /// A fresh allocation requested by the HOST through `cabi_realloc`. If the
    /// previous call has returned, this is the next call's argument lowering:
    /// reclaim the whole arena first.
    pub(crate) fn host_alloc(&mut self, size: usize, align: usize) -> Option<usize> {
        if self.returned {
            self.cursor = 0;
            self.returned = false;
        }
        self.bump(size, align)
    }

    /// An export begins. Reclaim only if nothing was allocated since the
    /// previous call returned (its parameters were passed flat).
    pub(crate) fn enter(&mut self) {
        if self.returned {
            self.cursor = 0;
        }
        self.returned = false;
        self.depth = self.depth.saturating_add(1);
    }

    /// An export's body has finished. Marks the call returned once the
    /// outermost scope closes; does not rewind.
    pub(crate) fn exit(&mut self) {
        self.depth = self.depth.saturating_sub(1);
        if self.depth == 0 {
            self.returned = true;
        }
    }
}

/// Run `f` on the process-wide arena state.
///
/// SAFETY of the `&mut`: single-threaded, non-reentrant access (see the crate
/// preconditions), and `f` never re-enters this function.
#[inline]
fn with_state<R>(f: impl FnOnce(&mut ArenaState) -> R) -> R {
    unsafe { f(&mut *STATE.0.get()) }
}

#[inline]
fn offset_to_ptr(off: Option<usize>) -> *mut u8 {
    match off {
        // In bounds: `bump` guarantees `off + size <= ARENA_BYTES`.
        Some(off) => unsafe { (STORE.0.get() as *mut u8).add(off) },
        None => core::ptr::null_mut(),
    }
}

/// The per-call guard every export opens as its first statement:
///
/// ```ignore
/// fn tick(state: VehicleState, sp: RateSetpoint) -> TorqueSetpoint {
///     #[cfg(not(feature = "std"))]
///     let _call = falcon_cm_rt::CallScope::enter();
///     ...
/// }
/// ```
///
/// See the crate documentation for the protocol and its preconditions. Bind it
/// to a named variable: `let _ = CallScope::enter()` drops it immediately,
/// which would mark the call returned while its arguments are still in use.
#[must_use = "bind the scope to a named variable (`let _call = ..`); `let _ =` drops it at once"]
pub struct CallScope {
    _private: (),
}

impl CallScope {
    /// Open the scope: reclaim the arena if the previous call returned and the
    /// host has not allocated this call's arguments in it.
    #[inline]
    pub fn enter() -> CallScope {
        with_state(ArenaState::enter);
        CallScope { _private: () }
    }
}

impl Drop for CallScope {
    #[inline]
    fn drop(&mut self) {
        with_state(ArenaState::exit);
    }
}

/// Bump arena. Never grows; returns null on exhaustion.
pub struct BumpArena;

impl BumpArena {
    /// Superseded by [`CallScope`]; kept so an out-of-tree caller still builds.
    ///
    /// UNSOUND WHEN ARGUMENTS ARE SPILLED, in its original form: called at the
    /// top of an export it rewound the cursor over the current call's argument
    /// area, which the host had already allocated and written. It was only safe
    /// while the generated glue copied every argument out before the body ran.
    ///
    /// It now rewinds only when the previous call has returned and nothing has
    /// been allocated since — the same condition [`CallScope::enter`] uses — so
    /// it can no longer free live arguments. It is NOT a replacement for the
    /// scope: without a `CallScope` nothing ever marks a call returned, so a
    /// component that only calls this stops reclaiming spilled-argument calls
    /// and ends at the exhaustion trap.
    #[deprecated(note = "unsound with spilled arguments; open a `CallScope` instead (#574)")]
    #[inline]
    pub fn reset() {
        with_state(|s| {
            if s.returned {
                s.cursor = 0;
            }
        });
    }
}

unsafe impl GlobalAlloc for BumpArena {
    /// Guest-side allocation: bumps, never reclaims (see "Only the HOST
    /// allocation path reclaims" in the crate docs).
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        offset_to_ptr(with_state(|s| s.bump(layout.size(), layout.align())))
    }

    /// Bump arena: individual frees are no-ops; the next call reclaims in bulk.
    unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {}
}

/// Abort the instance. On wasm this is the `unreachable` instruction — a trap
/// the host observes; natively (unit tests) a panic.
#[cold]
fn arena_exhausted() -> ! {
    #[cfg(target_arch = "wasm32")]
    core::arch::wasm32::unreachable();
    #[cfg(not(target_arch = "wasm32"))]
    panic!("falcon-cm-rt: arena exhausted");
}

/// The canonical-ABI reallocation entry point, backed by [`BumpArena`].
///
/// A fresh allocation (`old_len == 0`) is a HOST allocation and may reclaim
/// the arena (see the crate docs); a resize never does. Exhaustion traps rather
/// than returning null, because the host would accept address 0.
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
    let p = if old_len == 0 {
        // Canonical ABI: a zero-size allocation returns the alignment as a
        // non-null, suitably-aligned dangling pointer.
        if new_len == 0 {
            return align as *mut u8;
        }
        offset_to_ptr(with_state(|s| s.host_alloc(new_len, align)))
    } else {
        unsafe {
            BumpArena.realloc(
                old_ptr,
                Layout::from_size_align_unchecked(old_len, align),
                new_len,
            )
        }
    };
    if p.is_null() {
        arena_exhausted();
    }
    p
}

/// Install the global allocator, the `cabi_realloc` export and a panic handler.
///
/// These must be defined in the component crate itself — `#[global_allocator]`,
/// `#[no_mangle]` and `#[panic_handler]` are all per-final-artifact — so this
/// macro emits them while the logic stays in one audited place. Every export
/// must still open a [`CallScope`].
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

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests;
