//! Per-cycle throughput benchmarks for the relay flight engines.
//!
//! These are NOT microbenchmarks of isolated functions. Each bench drives one
//! engine's per-cycle HOT PATH at a realistic table load — the same scan the
//! engine performs every control cycle — so the measured wall-time is directly
//! comparable to the cycle budget the Lean WCET proofs bound
//! (`proofs/lean/WcetAnalysis.lean`, `CompositionalWcet.lean`). A regression
//! here is an early warning that a deployed system is drifting toward its WCET
//! ceiling. Traced by FV-FALCON-PERF-001 (#8).
//!
//! Run:   cargo bench -p engine-throughput-bench
//! Baseline numbers are recorded in benches/engine-throughput/BASELINE.md.

use criterion::{black_box, criterion_group, criterion_main, Criterion};

/// LC — Limit Checker: violations per cycle. One cycle = scan a full sensor
/// frame (32 readings) against a loaded watchpoint table (64 watchpoints over
/// 32 sensors). persistence=1 so a tripped watchpoint latches immediately,
/// keeping branch behaviour stable across criterion iterations.
fn bench_lc(c: &mut Criterion) {
    use relay_lc::engine::{ComparisonOp, SensorReading, Watchpoint, WatchpointTable};

    let mut table = WatchpointTable::new();
    for i in 0..64u32 {
        table.add_watchpoint(Watchpoint {
            sensor_id: i % 32,
            op: ComparisonOp::GreaterThan,
            threshold: 100,
            enabled: true,
            persistence: 1,
            current_count: 0,
        });
    }
    // One control cycle's worth of sensor readings — half trip, half pass.
    let readings: [SensorReading; 32] = core::array::from_fn(|i| SensorReading {
        sensor_id: i as u32,
        value: if i % 2 == 0 { 200 } else { 50 },
    });

    c.bench_function("lc/evaluate_cycle__64wp_32readings", |b| {
        b.iter(|| {
            for r in readings.iter() {
                black_box(table.evaluate(black_box(*r)));
            }
        })
    });
}

/// SCH — Scheduler: actions per tick at max task load. One cycle = a full major
/// frame (10 minor ticks) scanned against 128 schedule slots.
fn bench_sch(c: &mut Criterion) {
    use relay_sch::engine::{ScheduleSlot, ScheduleTable};

    let mut table = ScheduleTable::new();
    for i in 0..128u32 {
        table.add_slot(ScheduleSlot {
            minor_frame: i % 10,
            major_frame: 0, // 0 = fire every major frame
            target_channel: i % 8,
            payload_offset: 0,
            payload_len: 0,
            enabled: true,
        });
    }

    c.bench_function("sch/major_frame__128slots_10ticks", |b| {
        b.iter(|| {
            for minor in 0..10u32 {
                black_box(table.process_tick(black_box(minor), black_box(0)));
            }
        })
    });
}

/// SC — Stored Command: dispatches per tick under load. Steady-state per-tick
/// cost = scan 256 ATS commands; here they are future-dated so the scan runs
/// every tick without consuming commands (the common case), giving a stable
/// per-cycle scan measurement.
fn bench_sc(c: &mut Criterion) {
    use relay_sc::engine::{AtsCommand, CommandStore};

    let mut store = CommandStore::new();
    for i in 0..256u32 {
        store.load_ats_command(AtsCommand {
            execute_at_sec: 1_000_000 + i as u64, // far future: scan, do not dispatch
            command_code: (i % 64) as u16,
            payload_offset: 0,
            payload_len: 0,
            dispatched: false,
        });
    }

    c.bench_function("sc/process_tick__256ats_scan", |b| {
        b.iter(|| black_box(store.process_tick(black_box(10))))
    });
}

/// HS — Health & Safety: alerts per check under load. One cycle = check 32
/// registered app monitors. Counters are refreshed each iteration so the apps
/// stay alive (stable scan; the no-alert steady state).
fn bench_hs(c: &mut Criterion) {
    use relay_hs::engine::{HealthTable, HsAction};

    let mut table = HealthTable::new();
    for i in 0..32u32 {
        table.register_app(i, 3, HsAction::Event);
    }

    c.bench_function("hs/check_health__32apps", |b| {
        b.iter(|| {
            for i in 0..32u32 {
                table.update_counter(i, i + 1); // heartbeat: stay alive
            }
            black_box(table.check_health(black_box(0)))
        })
    });
}

/// CFDP — File Delivery: PDU actions per event. The heaviest event is a NAK
/// (retransmit request). max_retransmit is set high so the retransmit path runs
/// every iteration without latching the transaction cancelled.
fn bench_cfdp(c: &mut Criterion) {
    use relay_cfdp::engine::TransactionTable;

    let mut table = TransactionTable::new();
    let txn = table
        .begin_send(/* file_size */ 65_536, /* max_retransmit */ 1_000_000)
        .expect("transaction slot available");

    c.bench_function("cfdp/process_nak__retransmit_event", |b| {
        b.iter(|| black_box(table.process_nak(black_box(txn), black_box(1024), black_box(512))))
    });
}

criterion_group!(
    engines,
    bench_lc,
    bench_sch,
    bench_sc,
    bench_hs,
    bench_cfdp
);
criterion_main!(engines);
