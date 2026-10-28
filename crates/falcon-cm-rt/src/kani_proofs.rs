//! Kani proofs for the arena reclamation protocol (#574).
//!
//! A plain sibling declared `#[cfg(kani)]` from `lib.rs`, the relay
//! convention: `cargo build` / `cargo test` cannot see a break in `cfg(kani)`
//! code, so these are run with `cargo kani` and enforced by
//! `.github/workflows/kani.yml`.
//!
//! WHAT IS MODELLED. The harnesses drive [`ArenaState`] — the SAME value type
//! the global allocator, `cabi_realloc_impl` and `CallScope` delegate to, not a
//! copy of its logic — through bounded sequences of the operations a host and
//! a guest can perform: a host allocation (`host_alloc`, the `cabi_realloc`
//! path), a guest allocation (`bump`, the `GlobalAlloc` path), `enter` and
//! `exit`. Sizes and alignments are symbolic and unconstrained (any `usize`,
//! including non-powers-of-two and values that overflow), and the starting
//! state is any state satisfying the type's invariants, not just `new()`.
//!
//! WHAT IS NOT. The 8 KiB byte store itself, and the pointer formation in
//! `offset_to_ptr` (`base + offset`, in bounds by P2). Alignment of the
//! ADDRESS follows from offset alignment (P4) and the store's
//! `#[repr(align(16))]`, which the `static_path` unit test checks on the real
//! pointer.
//!
//! The properties, as numbered in the issue triage:
//! * P1 — no allocation handed out during call N overlaps an allocation still
//!   live for call N: the arguments the host allocated before `enter`, or
//!   anything allocated earlier in the same call, or (after the scope drops)
//!   the call's results while the host lifts them.
//! * P2 — exhaustion returns `None`; the cursor never exceeds `ARENA_BYTES`
//!   and never wraps.
//! * P3 — `enter` after the previous scope dropped (and no allocation since)
//!   reclaims to 0; `enter` before it does not move the cursor.
//! * P4 — every returned offset honours the requested alignment.

use super::{ArenaState, ARENA_BYTES, STORE_ALIGN};

/// Any state satisfying the invariants: `cursor <= ARENA_BYTES` and
/// `returned => depth == 0`. Depth is bounded so `saturating_add` is not the
/// thing being exercised.
fn any_state() -> ArenaState {
    let cursor: usize = kani::any();
    let returned: bool = kani::any();
    let depth: u32 = kani::any();
    kani::assume(cursor <= ARENA_BYTES);
    kani::assume(depth <= 4);
    kani::assume(!returned || depth == 0);
    ArenaState {
        cursor,
        returned,
        depth,
    }
}

/// Any invariant-satisfying state with no scope open.
fn any_state_between_calls() -> ArenaState {
    let s = any_state();
    kani::assume(s.depth == 0);
    s
}

fn invariant(s: &ArenaState) -> bool {
    s.cursor <= ARENA_BYTES && (!s.returned || s.depth == 0)
}

/// A live region: `[off, off + len)`.
#[derive(Clone, Copy)]
struct Region {
    off: usize,
    len: usize,
}

fn disjoint(a: Region, b: Region) -> bool {
    // Both are in bounds (P2), so these additions cannot overflow.
    a.off + a.len <= b.off || b.off + b.len <= a.off
}

const MAX_LIVE: usize = 6;

/// Fixed-capacity list of live regions (no heap in a `no_std` crate).
struct Live {
    r: [Region; MAX_LIVE],
    n: usize,
}

impl Live {
    fn new() -> Self {
        Live {
            r: [Region { off: 0, len: 0 }; MAX_LIVE],
            n: 0,
        }
    }
    /// Record a successful allocation, asserting it overlaps nothing live.
    fn add(&mut self, got: Option<usize>, len: usize) {
        if let Some(off) = got {
            let new = Region { off, len };
            let mut i = 0;
            while i < self.n {
                assert!(
                    disjoint(self.r[i], new),
                    "P1: allocation overlaps a live region"
                );
                i += 1;
            }
            self.r[self.n] = new;
            self.n += 1;
        }
    }
}

/// A symbolic, small allocation size. Small keeps the overlap reasoning about
/// placement rather than about the arithmetic P2 already covers; sizes up to
/// the whole arena still reach exhaustion.
fn any_size() -> usize {
    let n: usize = kani::any();
    kani::assume(n <= ARENA_BYTES);
    n
}

fn any_align() -> usize {
    let a: usize = kani::any();
    kani::assume(a.is_power_of_two() && a <= STORE_ALIGN);
    a
}

/// P1: one complete call N, from any starting state.
///
/// The host lowers 0..=2 argument areas (0 = flat params), the export enters,
/// performs three allocations each chosen as either a guest bump or a host
/// allocation (an import's result lowered into us mid-call), the scope drops,
/// and then post-return glue makes one more guest allocation while the
/// results are still being lifted. Every allocation in that window must be
/// disjoint from every earlier one.
#[kani::proof]
#[kani::unwind(8)]
fn p1_no_overlap_with_live_allocations_of_the_same_call() {
    // Between calls: no scope open. `returned` is symbolic, so this covers a
    // previous call that returned normally AND one that trapped mid-scope
    // (whose drop never ran).
    let mut s = any_state_between_calls();
    let mut live = Live::new();

    let n_args: u8 = kani::any();
    kani::assume(n_args <= 2);
    let mut i = 0;
    while i < n_args {
        let (sz, al) = (any_size(), any_align());
        live.add(s.host_alloc(sz, al), sz);
        i += 1;
    }
    kani::cover!(n_args == 0 && s.returned, "flat call: enter will reclaim");
    kani::cover!(
        n_args > 0 && live.n > 0,
        "spilled call: enter must keep args"
    );

    s.enter();

    let mut k = 0;
    while k < 3 {
        let (sz, al) = (any_size(), any_align());
        let host: bool = kani::any();
        let got = if host {
            s.host_alloc(sz, al)
        } else {
            s.bump(sz, al)
        };
        live.add(got, sz);
        k += 1;
    }

    s.exit();
    kani::cover!(live.n >= 2, "two disjoint live regions in one call");

    // Post-return glue (result lowering): must not reclaim over the results.
    let (sz, al) = (any_size(), any_align());
    live.add(s.bump(sz, al), sz);

    assert!(invariant(&s));
}

/// P1, nesting: an inner scope's drop does not mark the outer call returned,
/// so a host allocation during the outer call does not reclaim its arguments.
#[kani::proof]
#[kani::unwind(4)]
fn p1_nested_scope_keeps_outer_call_live() {
    let mut s = any_state_between_calls();
    let mut live = Live::new();
    let (sz, al) = (any_size(), any_align());
    live.add(s.host_alloc(sz, al), sz);
    s.enter();
    s.enter();
    s.exit();
    assert!(!s.returned);
    let (sz, al) = (any_size(), any_align());
    live.add(s.host_alloc(sz, al), sz);
    s.exit();
    assert!(s.returned);
}

/// P2: from any state, any of the four operations with ANY size and ANY
/// alignment keeps the cursor in bounds; a successful allocation lies wholly
/// inside the arena at or past the (possibly reclaimed) cursor; a failed one
/// leaves the cursor where the reclaim (if any) put it.
#[kani::proof]
fn p2_exhaustion_is_none_and_cursor_never_wraps() {
    let mut s = any_state();
    let size: usize = kani::any();
    let align: usize = kani::any();
    let op: u8 = kani::any();
    kani::assume(op < 4);

    match op {
        0 | 1 => {
            let host = op == 1;
            let reclaims = host && s.returned;
            let start = if reclaims { 0 } else { s.cursor };
            let got = if host {
                s.host_alloc(size, align)
            } else {
                s.bump(size, align)
            };
            match got {
                Some(off) => {
                    assert!(off >= start, "P2: allocation below the cursor (wrapped)");
                    assert!(
                        off <= ARENA_BYTES && size <= ARENA_BYTES - off,
                        "P2: out of bounds"
                    );
                    assert!(s.cursor == off + size);
                }
                None => {
                    assert!(
                        s.cursor == start,
                        "P2: a failed allocation moved the cursor"
                    );
                    // It really did not fit (or the alignment is unsupported).
                    let fits = align.is_power_of_two()
                        && align <= STORE_ALIGN
                        && start.checked_add(align - 1).is_some()
                        && {
                            let a = (start + align - 1) & !(align - 1);
                            a.checked_add(size).is_some_and(|e| e <= ARENA_BYTES)
                        };
                    assert!(!fits, "P2: refused an allocation that fits");
                    kani::cover!(
                        align.is_power_of_two() && align <= STORE_ALIGN,
                        "None from exhaustion"
                    );
                }
            }
        }
        2 => s.enter(),
        _ => s.exit(),
    }
    assert!(invariant(&s), "P2: invariant broken");
}

/// P3: `enter` after the previous scope dropped, with no allocation since,
/// reclaims to 0; `enter` while a call is live (arguments allocated, or a scope
/// open) leaves the cursor exactly where it was.
#[kani::proof]
fn p3_enter_reclaims_iff_previous_call_returned() {
    let mut s = any_state();
    let was_returned = s.returned;
    let before = s.cursor;
    s.enter();
    if was_returned {
        assert!(s.cursor == 0, "P3: enter after return did not reclaim");
    } else {
        assert!(
            s.cursor == before,
            "P3: enter during a live call moved the cursor"
        );
    }
    assert!(!s.returned && s.depth >= 1);
    kani::cover!(was_returned && before > 0, "reclaim from a non-empty arena");
    kani::cover!(!was_returned && before > 0, "keep a non-empty arena");
}

/// P3, the full sequence: drop marks returned without rewinding; a host
/// allocation after that reclaims and clears the flag, so the following
/// `enter` keeps it.
#[kani::proof]
fn p3_drop_then_host_alloc_then_enter() {
    let mut s = any_state_between_calls();
    s.enter();
    let before = s.cursor;
    s.exit();
    assert!(
        s.returned && s.cursor == before,
        "P3: drop rewound the arena"
    );
    let (sz, al) = (any_size(), any_align());
    let got = s.host_alloc(sz, al);
    assert!(!s.returned);
    if got.is_some() {
        assert!(
            got == Some(0),
            "P3: first host allocation after return not at 0"
        );
    }
    let mid = s.cursor;
    s.enter();
    assert!(s.cursor == mid, "P3: enter freed the spilled arguments");
}

/// P4: every successful allocation is aligned as requested, for any
/// power-of-two alignment the store supports, from any cursor.
#[kani::proof]
fn p4_alignment_honoured() {
    let mut s = any_state();
    let size: usize = kani::any();
    let align: usize = kani::any();
    let host: bool = kani::any();
    let got = if host {
        s.host_alloc(size, align)
    } else {
        s.bump(size, align)
    };
    if let Some(off) = got {
        assert!(align.is_power_of_two() && align <= STORE_ALIGN);
        assert!(off % align == 0, "P4: misaligned offset");
        // The store is 16-aligned, so the address is aligned too.
        assert!(STORE_ALIGN % align == 0);
    }
    kani::cover!(
        got.is_some() && align == 16 && s.cursor > 16,
        "16-aligned bump past the start"
    );
}

/// P1, inductive form (unbounded sequences). The sequence harness above is
/// bounded; this one-step lemma is what makes P1 hold for ANY number of
/// operations: from any state, every operation either leaves `[0, cursor)`
/// intact — a successful allocation lies at or past the old cursor, and the
/// cursor never moves backwards — or reclaims, and a reclaim happens ONLY when
/// the previous call had returned (`returned`) and the operation is a host
/// allocation or `enter`. A guest allocation never reclaims, in or out of a
/// call. So by induction everything live for the current call stays below the
/// cursor, and nothing below the cursor is handed out again within it.
#[kani::proof]
fn p1_step_only_a_returned_call_is_reclaimed() {
    let mut s = any_state();
    let before = s;
    let size: usize = kani::any();
    let align: usize = kani::any();
    let op: u8 = kani::any();
    kani::assume(op < 4);
    let got = match op {
        0 => s.bump(size, align),
        1 => s.host_alloc(size, align),
        2 => {
            s.enter();
            None
        }
        _ => {
            s.exit();
            None
        }
    };
    let reclaimed = s.cursor < before.cursor || (op == 1 && before.returned);
    if reclaimed {
        assert!(before.returned, "P1: reclaimed while a call was live");
        assert!(
            op == 1 || op == 2,
            "P1: reclaimed by a guest allocation or a drop"
        );
    } else {
        assert!(s.cursor >= before.cursor, "P1: cursor moved backwards");
        if let Some(off) = got {
            assert!(
                off >= before.cursor,
                "P1: handed out memory below the cursor"
            );
        }
    }
    kani::cover!(
        op == 0 && before.returned && got.is_some(),
        "post-return guest alloc"
    );
    kani::cover!(
        op == 1 && !before.returned && got.is_some(),
        "in-call host alloc"
    );
}
