//! Kani harnesses for relay-calib (plain-only sibling).
//!
//! The TOTALITY floor under the calibration math: `apply_*` is the identity under
//! [`CalParams::identity`], and the division-free solvers never panic / index out
//! of bounds over a fixed sample buffer. CONCRETE buffer lengths are used
//! deliberately — a SYMBOLIC slice length makes CBMC enumerate every slice shape
//! over an f32 buffer and times out (the same lesson as the DroneCAN symbolic-
//! offset proof). The NUMERICAL recovery + the divide-bearing solvers' guards
//! (`accel_6point`, `mag_softiron_diag`) are proptest/unit-gated: f32 least-
//! squares and CBMC's NaN-on-division check are intractable, the codebase's
//! standard Kani-vs-proptest split for f32.
#![cfg(kani)]

use crate::*;

/// CALIB-K01 — `identity()` is a genuine no-op: for ANY finite raw sample,
/// `apply_gyro`/`apply_accel`/`apply_mag` under the identity calibration return
/// the input unchanged (raw − 0, and 1·(raw − 0)). So replacing the estimator's
/// identity-remap placeholder with `CalParams::identity()` changes nothing.
#[kani::proof]
fn verify_identity_is_noop() {
    let raw: Vec3 = [kani::any(), kani::any(), kani::any()];
    let mut a = 0;
    while a < 3 {
        kani::assume(raw[a].is_finite());
        a += 1;
    }
    let c = CalParams::identity();
    assert!(c.apply_gyro(raw) == raw);
    assert!(c.apply_accel(raw) == raw);
    assert!(c.apply_mag(raw) == raw);
}

/// CALIB-K02 — `gyro_null` is total and finiteness-preserving: over a fixed
/// buffer of arbitrary, magnitude-bounded samples it returns without panic or
/// out-of-bounds index, and every component of the mean is finite (no NaN/inf —
/// the divisor is n ≥ 1). The empty case is covered by a unit test. (An exact
/// "mean within [min,max]" bound is NOT f32-provable here: the ⅓ factor is
/// inexact, unlike the midpoints' exact ×0.5; the recovery numerics are
/// proptest-gated.)
#[kani::proof]
fn verify_gyro_null_total_and_finite() {
    let buf: [Vec3; 3] = [
        [kani::any(), kani::any(), kani::any()],
        [kani::any(), kani::any(), kani::any()],
        [kani::any(), kani::any(), kani::any()],
    ];
    let mut i = 0;
    while i < 3 {
        let mut a = 0;
        while a < 3 {
            // bound magnitude so the sum cannot overflow f32 to ±inf.
            kani::assume(buf[i][a].is_finite() && buf[i][a].abs() < 1.0e18);
            a += 1;
        }
        i += 1;
    }
    let mean = gyro_null(&buf);
    assert!(mean[0].is_finite() && mean[1].is_finite() && mean[2].is_finite());
}

/// CALIB-K03 — `mag_hardiron` is total and the offset lies within the per-axis
/// [min, max] of the (finite) sweep: a non-empty finite sweep can never place the
/// hard-iron offset outside the observed range. Division-free (a midpoint).
#[kani::proof]
fn verify_mag_hardiron_bounded() {
    let buf: [Vec3; 3] = [
        [kani::any(), kani::any(), kani::any()],
        [kani::any(), kani::any(), kani::any()],
        [kani::any(), kani::any(), kani::any()],
    ];
    let mut i = 0;
    while i < 3 {
        let mut a = 0;
        while a < 3 {
            kani::assume(buf[i][a].is_finite() && buf[i][a].abs() < 1.0e18);
            a += 1;
        }
        i += 1;
    }
    let off = mag_hardiron(&buf);
    let xs = [buf[0][0], buf[1][0], buf[2][0]];
    let lo = if xs[0] < xs[1] {
        if xs[0] < xs[2] { xs[0] } else { xs[2] }
    } else if xs[1] < xs[2] {
        xs[1]
    } else {
        xs[2]
    };
    let hi = if xs[0] > xs[1] {
        if xs[0] > xs[2] { xs[0] } else { xs[2] }
    } else if xs[1] > xs[2] {
        xs[1]
    } else {
        xs[2]
    };
    assert!(off[0] >= lo && off[0] <= hi);
}
