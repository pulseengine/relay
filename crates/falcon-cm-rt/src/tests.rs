//! Unit tests for the arena reclamation protocol (#574).
//!
//! The protocol is tested on [`ArenaState`] directly, so the tests do not
//! share the process-wide static and can run in parallel. The static path
//! (`cabi_realloc_impl` + `CallScope` over the real store) is exercised by ONE
//! test function, because that state is global.

use super::*;

/// The spilled parameter area of `position#tick`: 18 flat f32 params.
const POSITION_ARGS: usize = 72;
/// ... and of `attitude#tick`: 19.
const ATTITUDE_ARGS: usize = 76;

/// One host call of an export whose arguments spill: the host allocates the
/// argument area, then the export runs inside a scope that also allocates
/// `body` bytes, then returns.
fn spilled_call(s: &mut ArenaState, args: usize, body: usize) -> (Option<usize>, Option<usize>) {
    let a = s.host_alloc(args, 4);
    s.enter();
    let b = if body > 0 { s.bump(body, 4) } else { None };
    s.exit();
    (a, b)
}

// (1) Entry does not reclaim the arguments the caller allocated for this call.
#[test]
fn enter_preserves_caller_allocated_args() {
    let mut s = ArenaState::new();
    // A previous call ran and returned, leaving data in the arena.
    s.enter();
    s.bump(200, 8).unwrap();
    s.exit();
    // The host lowers the next call's arguments: reclaims, places them at 0.
    let args = s.host_alloc(POSITION_ARGS, 4).unwrap();
    assert_eq!(args, 0);
    s.enter();
    assert_eq!(
        s.cursor, POSITION_ARGS,
        "enter must not rewind over this call's args"
    );
    let local = s.bump(16, 4).unwrap();
    assert!(
        local >= args + POSITION_ARGS,
        "in-call allocation overlaps the args"
    );
    s.exit();
}

// (2) The arena is reclaimed after a return — by the next call, not by the drop.
#[test]
fn reclaimed_after_return_not_at_drop() {
    let mut s = ArenaState::new();
    spilled_call(&mut s, ATTITUDE_ARGS, 40);
    // The drop marks returned but leaves the results in place for the host.
    assert!(s.returned);
    assert_eq!(s.cursor, ATTITUDE_ARGS + 40);
    // The next call's argument lowering reclaims.
    assert_eq!(s.host_alloc(ATTITUDE_ARGS, 4), Some(0));
}

// (3) A call with no spilled arguments reclaims at entry.
#[test]
fn flat_call_reclaims_at_entry() {
    let mut s = ArenaState::new();
    spilled_call(&mut s, POSITION_ARGS, 100);
    assert_ne!(s.cursor, 0);
    s.enter(); // no host allocation preceded this call
    assert_eq!(s.cursor, 0);
    s.exit();
}

// (4) Exhaustion within a single call returns None rather than wrapping.
#[test]
fn exhaustion_in_one_call_is_none_never_wraps() {
    let mut s = ArenaState::new();
    s.enter();
    let mut last_end = 0;
    let mut n = 0;
    while let Some(off) = s.bump(100, 4) {
        assert!(off >= last_end, "allocation wrapped or overlapped");
        last_end = off + 100;
        n += 1;
    }
    assert_eq!(n, ARENA_BYTES / 100);
    assert!(s.cursor <= ARENA_BYTES);
    let before = s;
    assert_eq!(s.bump(100, 4), None);
    assert_eq!(s.bump(usize::MAX, 1), None);
    assert_eq!(s, before, "a failed allocation must not move the cursor");
    // Still exhausted inside the same call: no hidden reclaim.
    assert_eq!(s.host_alloc(100, 4), None);
    s.exit();
}

#[test]
fn unsupported_alignment_is_refused() {
    let mut s = ArenaState::new();
    assert_eq!(s.bump(8, 3), None, "non-power-of-two");
    assert_eq!(s.bump(8, 0), None, "zero");
    assert_eq!(s.bump(8, 32), None, "above the store's 16-byte alignment");
    assert_eq!(s.cursor, 0);
    for align in [1, 2, 4, 8, 16] {
        s.bump(1, 1).unwrap();
        let off = s.bump(4, align).unwrap();
        assert_eq!(off % align, 0);
    }
}

#[test]
fn post_return_guest_alloc_does_not_reclaim() {
    // Glue lowering a result can allocate after the scope dropped. That must
    // not rewind over the value being lowered.
    let mut s = ArenaState::new();
    s.host_alloc(POSITION_ARGS, 4).unwrap();
    s.enter();
    let result = s.bump(32, 4).unwrap();
    s.exit();
    let lowered = s.bump(32, 4).unwrap();
    assert!(
        lowered >= result + 32,
        "post-return allocation overlapped the result"
    );
}

#[test]
fn nested_scope_does_not_mark_outer_returned() {
    let mut s = ArenaState::new();
    s.host_alloc(POSITION_ARGS, 4).unwrap();
    s.enter();
    s.enter();
    s.exit();
    assert!(!s.returned, "inner drop marked the outer call returned");
    // An import's result lowered during the outer call does not reclaim.
    assert_eq!(s.host_alloc(8, 4), Some(POSITION_ARGS));
    s.exit();
    assert!(s.returned);
}

#[test]
fn ten_thousand_spilled_calls_never_exhaust() {
    let mut s = ArenaState::new();
    for i in 0..10_000 {
        for args in [POSITION_ARGS, ATTITUDE_ARGS] {
            let (a, b) = spilled_call(&mut s, args, 24);
            assert_eq!(a, Some(0), "call {i}: args not at the arena start");
            assert_eq!(b, Some(args), "call {i}");
        }
    }
}

/// What `position` did before #574: the host spills 72 B per call and no
/// scope ever marks a call returned. Kept as the measured baseline the fix is
/// against — 8192 / 72 = 113 calls fit, the 114th does not.
#[test]
fn without_a_scope_the_parameter_area_leaks() {
    let mut s = ArenaState::new();
    let mut ok = 0;
    while s.host_alloc(POSITION_ARGS, 4).is_some() {
        ok += 1;
    }
    assert_eq!(ok, ARENA_BYTES / POSITION_ARGS);
    assert_eq!(ok, 113);
}

/// The real static path: `cabi_realloc_impl` + `CallScope` over the store.
/// One function, because the state is process-wide.
#[test]
#[allow(deprecated)]
fn static_path() {
    let base = STORE.0.get() as usize;
    let mut first = None;
    for _ in 0..10_000 {
        let p = unsafe { cabi_realloc_impl(core::ptr::null_mut(), 0, 4, POSITION_ARGS) };
        assert!(!p.is_null());
        assert_eq!(p as usize % 4, 0);
        assert!(p as usize >= base && p as usize + POSITION_ARGS <= base + ARENA_BYTES);
        assert_eq!(
            *first.get_or_insert(p),
            p,
            "argument area moved between calls"
        );
        unsafe { p.write_bytes(0xA5, POSITION_ARGS) };
        let _call = CallScope::enter();
        // reset() inside a spilled call must not free the args.
        BumpArena::reset();
        let q = unsafe { BumpArena.alloc(Layout::from_size_align(16, 16).unwrap()) };
        assert_eq!(q as usize % 16, 0);
        assert!(
            q as usize >= p as usize + POSITION_ARGS,
            "in-call alloc overlaps args"
        );
        assert!(unsafe { core::slice::from_raw_parts(p, POSITION_ARGS) }
            .iter()
            .all(|&b| b == 0xA5));
    }

    // Zero-size: the alignment as a dangling pointer, no arena use.
    assert_eq!(
        unsafe { cabi_realloc_impl(core::ptr::null_mut(), 0, 8, 0) } as usize,
        8
    );

    // Exhaustion at the ABI boundary traps (a panic natively) instead of
    // handing the host a null it would write through.
    let r = std::panic::catch_unwind(|| {
        let _call = CallScope::enter();
        loop {
            unsafe { cabi_realloc_impl(core::ptr::null_mut(), 0, 4, 1024) };
        }
    });
    assert!(r.is_err());
    // The unwound scope dropped and marked the call returned, so the next
    // host allocation reclaims.
    let p = unsafe { cabi_realloc_impl(core::ptr::null_mut(), 0, 4, POSITION_ARGS) };
    assert_eq!(p as usize, base);
}
