//! Kani harnesses for falcon-core (#638: the FDI logic no harness reached).
//!
//! Plain-only sibling, declared from `lib.rs` under `cfg(kani)`. Run with
//! `cargo kani` in `crates/falcon-core`; the `falcon-core` leg of
//! `.github/workflows/kani.yml` runs every harness here.

use crate::fdi::{FDI_STALL_CMD_MIN, stall_check};

/// The stall predicate as an INDEPENDENT spec, not a copy of the code's float
/// expression: `rpm / ESC_RPM_FULL` (8000) clamped to [0, 2] is below
/// `FDI_STALL_RPM_FRAC` (0.05) exactly when the integer RPM is below 400
/// (negative RPM clamps to 0). Exact in f32 for every i32 below 400.
fn stalled_spec(rpm: i32, cmd: f32) -> bool {
    rpm < 400 && cmd > FDI_STALL_CMD_MIN
}

/// One tick of the spin-up stall check over fully symbolic state: any run
/// counters, either `common`, any RPM (or none), any commands including NaN,
/// any window of at least one tick (`fdi_stall_steps` floors at 1).
#[kani::proof]
fn stall_check_isolates_only_a_lone_rotor_stalled_for_the_full_window() {
    let old_run: [u32; 4] = kani::any();
    let old_common: bool = kani::any();
    let rpm_now: Option<[i32; 4]> = kani::any();
    let cmd: [f32; 4] = kani::any();
    let stall_steps: u32 = kani::any();
    kani::assume(stall_steps >= 1);

    let mut run = old_run;
    let mut common = old_common;
    let out = stall_check(&mut run, &mut common, rpm_now, cmd, stall_steps);

    let Some(rpm) = rpm_now else {
        // A tick without telemetry is not evidence.
        assert!(out.is_none());
        // Element-wise: an array `==` lowers to memcmp, which Kani does not model.
        assert!(run[0] == 0 && run[1] == 0 && run[2] == 0 && run[3] == 0 && !common);
        return;
    };

    let mut n = 0u32;
    let mut i = 0;
    while i < 4 {
        let s = stalled_spec(rpm[i], cmd[i]);
        if s {
            n += 1;
            // A stalled rotor's run counts on, unless common mode cleared it.
            assert!(run[i] == old_run[i].saturating_add(1) || (common && run[i] == 0));
        } else {
            // Any other tick breaks the run.
            assert!(run[i] == 0);
        }
        i += 1;
    }

    // `common` is only ever set with two or more stalled.
    if common {
        assert!(n >= 2);
    }
    if n < 2 {
        assert!(!common);
    }

    match out {
        Some(j) => {
            assert!(j < 4);
            // Exactly one rotor stalled, it is the one returned, and its
            // unbroken run reached the window.
            assert!(n == 1);
            assert!(stalled_spec(rpm[j], cmd[j]));
            assert!(old_run[j].saturating_add(1) >= stall_steps);
        }
        None => {
            // A lone rotor at the full window is never missed.
            if n == 1 {
                let mut k = 0;
                while k < 4 {
                    if stalled_spec(rpm[k], cmd[k]) {
                        assert!(old_run[k].saturating_add(1) < stall_steps);
                    }
                    k += 1;
                }
            }
        }
    }

    kani::cover!(out.is_some(), "a lone stalled rotor is isolated");
    kani::cover!(common, "common mode latches");
    kani::cover!(
        n >= 2 && out.is_none() && !common,
        "common mode below the window"
    );
}
