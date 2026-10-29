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

/// CALIB-K04 — the ZERO off-diagonal selects the diagonal path, and nothing
/// else does: `mag_softiron_is_diagonal()` is true for EVERY sign pattern of
/// zeros (`+0.0` / `-0.0` in each slot) and false as soon as any term is
/// non-zero or NaN. `apply_mag` returns `hadamard(scale, raw − offset)` — the
/// pre-off-diagonal expression, unchanged — exactly when this is true, so a
/// host that sends `mag-offdiag = [0, 0, 0]` keeps today's numbers BIT FOR
/// BIT (`zero_offdiag_is_bit_identical_to_the_diagonal_path` checks the bits,
/// signed zero and inf included). Stated on the selector rather than on the
/// two f32 products: proving `a·b == a·b` across two separately-encoded f32
/// multipliers is a SAT miter that did not finish in 10 min (measured).
#[kani::proof]
fn verify_zero_offdiag_selects_the_diagonal_path() {
    let z = |neg: bool| if neg { -0.0f32 } else { 0.0f32 };
    let mut c = CalParams::identity();
    c.mag_offdiag = [z(kani::any()), z(kani::any()), z(kani::any())];
    assert!(c.mag_softiron_is_diagonal());

    let mut d = CalParams::identity();
    d.mag_offdiag = [kani::any(), kani::any(), kani::any()];
    kani::assume(d.mag_offdiag[0] != 0.0 || d.mag_offdiag[1] != 0.0 || d.mag_offdiag[2] != 0.0);
    assert!(!d.mag_softiron_is_diagonal());
}

/// CALIB-K05 — with a zero off-diagonal, `is_valid` is EXACTLY the rule of
/// earlier releases: every component finite and every scale `> 0`. So adding
/// the positive-definiteness check refuses nothing a diagonal calibration
/// could previously install, and accepts nothing it refused.
#[kani::proof]
fn verify_is_valid_with_zero_offdiag_is_the_diagonal_rule() {
    let c = CalParams {
        gyro_bias: [kani::any(), kani::any(), kani::any()],
        accel_bias: [kani::any(), kani::any(), kani::any()],
        accel_scale: [kani::any(), kani::any(), kani::any()],
        mag_offset: [kani::any(), kani::any(), kani::any()],
        mag_scale: [kani::any(), kani::any(), kani::any()],
        mag_offdiag: [0.0; 3],
    };
    let fin = |v: Vec3| v[0].is_finite() && v[1].is_finite() && v[2].is_finite();
    let pos = |v: Vec3| v[0] > 0.0 && v[1] > 0.0 && v[2] > 0.0;
    let old = fin(c.gyro_bias)
        && fin(c.accel_bias)
        && fin(c.accel_scale)
        && fin(c.mag_offset)
        && fin(c.mag_scale)
        && pos(c.accel_scale)
        && pos(c.mag_scale);
    assert!(c.is_valid() == old);
}

/// CALIB-K06 — refusal completeness for the off-diagonal: ANY non-finite
/// off-diagonal term is refused whatever the rest holds (a NaN soft-iron term
/// can never be installed as a calibration). The positive-definiteness
/// numerics themselves are unit/proptest-gated (`is_valid_rule`): a symbolic
/// 3x3 f32 determinant is the SAT heavy tail this crate's split avoids.
#[kani::proof]
fn verify_non_finite_offdiag_is_refused() {
    let mut c = CalParams::identity();
    c.mag_offdiag = [kani::any(), kani::any(), kani::any()];
    kani::assume(
        !(c.mag_offdiag[0].is_finite()
            && c.mag_offdiag[1].is_finite()
            && c.mag_offdiag[2].is_finite()),
    );
    assert!(!c.is_valid());
}
