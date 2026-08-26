//! Per-cycle throughput benchmarks for the falcon FLIGHT CASCADE.
//!
//! `engine-throughput` guards the cFS-lineage engines (LC/SCH/SC/HS/CFDP).
//! This one guards the control path that actually flies, which had NO timing
//! baseline at all until now (#8, PERF-P01) — a gap that made "performance"
//! the one axis on which the project could make no claim whatsoever.
//!
//! These are NOT microbenchmarks of isolated functions. Each bench drives one
//! cascade stage's per-cycle hot path with realistic inputs, and `full_cascade`
//! drives all five IN SEQUENCE — which is the number that matters, because the
//! cascade is single-rate: `cascade.step()` runs every stage once per tick
//! (verified in wasm/cm/cascade/src/lib.rs; there is no rate divider).
//!
//! So `full_cascade` IS the per-tick control-loop cost, and at a 1 kHz rate
//! loop it must fit inside 1 ms with margin for the HAL and scheduler.
//!
//! Run:   cargo bench -p cascade-throughput-bench
//! Baseline: benches/cascade-throughput/BASELINE.md
//! Traced by FV-FALCON-PERF-002 (PERF-P01).

use criterion::{black_box, criterion_group, criterion_main, Criterion};

use relay_att::{AttController, Timestamp as AttTime};
use relay_iekf::{Iekf, Imu};
use relay_mix_quad::QuadMixer;
use relay_pos::{Ned, PosController, PositionSetpoint, Timestamp as PosTime};
use relay_rate::{RatePid, Timestamp as RateTime};

/// A gently-moving hover sample. Deliberately NOT zeros: a zeroed IMU takes
/// different branches in the estimator's normalisation paths and would make the
/// benchmark measure an unrepresentative code path.
const GYRO: [f32; 3] = [0.012, -0.008, 0.003];
const ACCEL: [f32; 3] = [0.15, -0.09, -9.79];
const DT: f32 = 0.001; // 1 kHz rate loop

fn ts_rate(ms: u64) -> RateTime {
    RateTime { seconds: ms / 1000, fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32 }
}
fn ts_att(ms: u64) -> AttTime {
    AttTime { seconds: ms / 1000, fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32 }
}
fn ts_pos(ms: u64) -> PosTime {
    PosTime { seconds: ms / 1000, fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32 }
}

/// IEKF — one propagate step at the rate-loop dt. The estimator is the heaviest
/// stage (scry bounds it at 4192 B of stack against 16-112 B for the others),
/// so it dominates the cascade budget.
fn bench_iekf(c: &mut Criterion) {
    c.bench_function("iekf_propagate", |b| {
        let mut ekf = Iekf::level();
        let mut t = 0u64;
        b.iter(|| {
            t = t.wrapping_add(1);
            ekf.propagate(black_box(Imu { gyro: GYRO, accel: ACCEL }), black_box(DT));
            black_box(ekf.state())
        })
    });
}

/// Position loop — outer cascade stage, NED position/velocity to attitude
/// setpoint. Hover at 5 m with a small offset so the controller has real error
/// to act on.
fn bench_position(c: &mut Criterion) {
    c.bench_function("position_tick", |b| {
        let mut pos = PosController::new();
        let sp = PositionSetpoint {
            position_ned: [0.0, 0.0, -5.0] as Ned,
            velocity_ned: [0.0, 0.0, 0.0] as Ned,
            yaw_setpoint: 0.0,
        };
        let mut ms = 0u64;
        b.iter(|| {
            ms = ms.wrapping_add(20); // 50 Hz outer loop cadence
            black_box(pos.tick(
                black_box(ts_pos(ms)),
                black_box([0.3, -0.2, -4.8]),
                black_box([0.05, -0.03, 0.01]),
                black_box([1.0, 0.0, 0.0, 0.0]),
                black_box(sp),
            ))
        })
    });
}

/// Attitude loop — geometric SO(3), quaternion error to body-rate setpoint.
fn bench_attitude(c: &mut Criterion) {
    c.bench_function("attitude_tick", |b| {
        let mut att = AttController::new();
        let mut ms = 0u64;
        b.iter(|| {
            ms = ms.wrapping_add(5); // 200 Hz
            black_box(att.tick(
                black_box(ts_att(ms)),
                black_box([0.9998, 0.012, -0.008, 0.004]),
                black_box([1.0, 0.0, 0.0, 0.0]),
            ))
        })
    });
}

/// Rate loop — the innermost, fastest stage. PID on body rates.
fn bench_rate(c: &mut Criterion) {
    c.bench_function("rate_tick", |b| {
        let mut pid = RatePid::new();
        let mut ms = 0u64;
        b.iter(|| {
            ms = ms.wrapping_add(1); // 1 kHz
            black_box(pid.tick(black_box(ts_rate(ms)), black_box(GYRO), black_box([0.0, 0.0, 0.0])))
        })
    });
}

/// Mixer — control allocation, torque+thrust to four motor commands.
fn bench_mixer(c: &mut Criterion) {
    c.bench_function("mixer_mix", |b| {
        let mut mixer = QuadMixer::new();
        b.iter(|| black_box(mixer.mix(black_box([0.01, -0.02, 0.005]), black_box(0.55))))
    });
}

/// THE NUMBER THAT MATTERS — all five stages in sequence, exactly as
/// `cascade.step()` orders them. Single-rate: one execution of each per tick.
/// At 1 kHz this must fit in 1 ms with margin for the HAL and scheduler.
fn bench_full_cascade(c: &mut Criterion) {
    c.bench_function("full_cascade_tick", |b| {
        let mut ekf = Iekf::level();
        let mut pos = PosController::new();
        let mut att = AttController::new();
        let mut pid = RatePid::new();
        let mut mixer = QuadMixer::new();
        let sp = PositionSetpoint {
            position_ned: [0.0, 0.0, -5.0] as Ned,
            velocity_ned: [0.0, 0.0, 0.0] as Ned,
            yaw_setpoint: 0.0,
        };
        let mut ms = 0u64;
        b.iter(|| {
            ms = ms.wrapping_add(1);
            // 1. estimator
            ekf.propagate(black_box(Imu { gyro: GYRO, accel: ACCEL }), DT);
            let st = ekf.state();
            // 2. position -> attitude setpoint
            let att_sp = pos.tick(ts_pos(ms), [0.3, -0.2, -4.8], [0.05, -0.03, 0.01], [1.0, 0.0, 0.0, 0.0], sp);
            // 3. attitude -> rate setpoint
            let rate_sp = att.tick(ts_att(ms), att_sp.quaternion, [1.0, 0.0, 0.0, 0.0]);
            // 4. rate -> torque
            let torque = pid.tick(ts_rate(ms), GYRO, rate_sp);
            // 5. mixer -> motor commands
            black_box((mixer.mix(torque, att_sp.thrust), st))
        })
    });
}

criterion_group!(
    cascade,
    bench_iekf,
    bench_position,
    bench_attitude,
    bench_rate,
    bench_mixer,
    bench_full_cascade
);
criterion_main!(cascade);
