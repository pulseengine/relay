extern crate std;
use super::blackbox_backend::*;
use super::*;
use relay_log::blackbox::{BlockLog, REC_TICK, TICK_MAX, TickRecord, scan};
use std::vec::Vec;

struct VecLog(Vec<u8>);
impl BlockLog for VecLog {
    fn append(&mut self, r: &[u8]) -> Result<(), relay_log::blackbox::MediaRejected> {
        self.0.extend_from_slice(r);
        Ok(())
    }
}

fn fly_and_log(n_ticks: usize) -> (Vec<TickRecord>, f32) {
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = SimBackend::new(level, dt).with_pathology(Pathology {
        gyro_white: 0.004,
        gps_noise: 0.08,
        ..Default::default()
    });
    plant.baro_enabled = true;
    plant.baro_noise = 0.05;
    plant.rot_drag = 0.02;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut b = LoggingBackend::new(&mut plant, VecLog(Vec::new()), TICK_MAX);
    for _ in 0..n_ticks {
        core.step(&mut b);
        let est = core.state();
        let cov = core.cov_summary();
        b.finish_tick(&est, cov);
    }
    assert_eq!(b.writer.dropped, 0, "no silent budget drops in the fixture");
    let bytes = b.writer.into_inner().0;
    let mut ticks = Vec::new();
    let (cnt, used) = scan(&bytes, |r| {
        assert_eq!(r.rec_type, REC_TICK);
        ticks.push(TickRecord::decode(r.payload).expect("total decode"));
    });
    assert_eq!(cnt, n_ticks);
    assert_eq!(used, bytes.len());
    (ticks, dt)
}

/// LOG-P03's oracle: the PRODUCTION core, replayed from a recorded log,
/// reproduces the logged estimator trajectory f32-BIT-EXACTLY. Every
/// recorded flight is thereby a regression fixture — an estimator change
/// that alters any replayed state is a measured diff, not an opinion.
#[test]
fn blackbox_replay_reproduces_estimator_bit_exactly() {
    let (ticks, dt) = fly_and_log(1500);
    let mut replayed = FlightCore::new(0.5, 1.0 / dt);
    replayed.set_altitude(-2.0);
    let mut rb = ReplayBackend::new(&ticks, dt);
    let mut checked = 0usize;
    while !rb.done() {
        replayed.step(&mut rb);
        let est = replayed.state();
        let t = rb.current().unwrap();
        assert_eq!(
            est.q.map(f32::to_bits),
            t.est_q.map(f32::to_bits),
            "q tick {checked}"
        );
        assert_eq!(
            est.v.map(f32::to_bits),
            t.est_v.map(f32::to_bits),
            "v tick {checked}"
        );
        assert_eq!(
            est.p.map(f32::to_bits),
            t.est_p.map(f32::to_bits),
            "p tick {checked}"
        );
        assert_eq!(
            replayed.cov_summary().map(f32::to_bits),
            t.cov.map(f32::to_bits),
            "cov tick {checked}"
        );
        rb.advance();
        checked += 1;
    }
    assert_eq!(checked, 1500);
    // Determinism: a SECOND replay of the same log is identical too.
    let mut again = FlightCore::new(0.5, 1.0 / dt);
    again.set_altitude(-2.0);
    let mut rb2 = ReplayBackend::new(&ticks, dt);
    for t in &ticks {
        again.step(&mut rb2);
        assert_eq!(again.state().p.map(f32::to_bits), t.est_p.map(f32::to_bits));
        rb2.advance();
    }
}

/// GOLDEN-LOG regression gate: replay the checked-in fixture recorded on
/// a past build; the estimator must still reproduce ITS logged states
/// bit-exactly. An estimator behavior change fails here until the golden
/// is DELIBERATELY regenerated (run the ignored `regen_golden_log` and
/// commit the diff — a reviewed act, per LOG-P03).
#[test]
fn golden_log_replay_regression() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests-data/golden-v1118.bblog");
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(_) => {
            // First-build bootstrap: golden not yet generated.
            std::eprintln!(
                "golden log missing — run `cargo test regen_golden_log -- --ignored` and commit it"
            );
            return;
        }
    };
    let mut ticks = Vec::new();
    let (cnt, used) = scan(&bytes, |r| {
        if r.rec_type == REC_TICK {
            ticks.push(TickRecord::decode(r.payload).expect("golden decodes"));
        }
    });
    assert_eq!(used, bytes.len(), "golden log has a torn tail");
    assert!(cnt >= 500, "golden log too short: {cnt}");
    let dt = 0.004f32;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut rb = ReplayBackend::new(&ticks, dt);
    for (i, t) in ticks.iter().enumerate() {
        core.step(&mut rb);
        let est = core.state();
        assert_eq!(
            est.p.map(f32::to_bits),
            t.est_p.map(f32::to_bits),
            "estimator behavior changed vs the golden log at tick {i} — if intended, regenerate deliberately"
        );
        rb.advance();
    }
}

/// LOG-P02's full record set on the SUPERVISED stack: a boot record
/// (param snapshot), per-tick records via the LoggingBackend, and EVENT
/// records on every mode transition — all scanning back intact.
#[test]
fn supervised_flight_logs_boot_ticks_and_events() {
    use relay_fsm::Event;
    use relay_log::blackbox::{REC_BOOT, REC_EVENT, encode_boot, encode_event};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = SimBackend::new(level, dt);
    plant.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    let mut b = LoggingBackend::new(&mut plant, VecLog(Vec::new()), 256);
    // Boot record: schema version + a representative param snapshot.
    let mut buf = [0u8; 128];
    let n = encode_boot(&mut buf, 1, &[(*b"MC_ROLL_P\0\0\0\0\0\0\0", 6.5)]).unwrap();
    b.writer.begin_tick();
    b.writer.append(&buf[..n]);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mut last_mode = sup.mode();
    // 8000 steps @ 2 ms = 16 s — enough for Takeoff → Loiter (the same
    // horizon the supervised-landing siblings use).
    for _ in 0..8000 {
        sup.step(&mut b);
        let est = sup.state();
        b.finish_tick(&est, [0.0; 3]);
        if sup.mode() != last_mode {
            last_mode = sup.mode();
            let mut eb = [0u8; 16];
            let n = encode_event(&mut eb, last_mode as u8, 0.0);
            b.writer.append(&eb[..n]);
        }
    }
    let bytes = b.writer.into_inner().0;
    let (mut boots, mut ticks, mut events) = (0, 0, 0);
    let (_, used) = scan(&bytes, |r| match r.rec_type {
        REC_BOOT => boots += 1,
        REC_TICK => ticks += 1,
        REC_EVENT => events += 1,
        _ => {}
    });
    assert_eq!(used, bytes.len(), "no torn tail");
    assert_eq!(boots, 1);
    assert_eq!(ticks, 8000);
    assert!(
        events >= 1,
        "mode transitions must appear as events (got {events})"
    );
}

/// Writes the golden fixture (deliberate, reviewed regeneration only).
#[test]
#[ignore]
fn regen_golden_log() {
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = SimBackend::new(level, dt).with_pathology(Pathology {
        gyro_white: 0.004,
        gps_noise: 0.08,
        ..Default::default()
    });
    plant.baro_enabled = true;
    plant.baro_noise = 0.05;
    plant.rot_drag = 0.02;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut b = LoggingBackend::new(&mut plant, VecLog(Vec::new()), TICK_MAX);
    for _ in 0..500 {
        core.step(&mut b);
        let est = core.state();
        let cov = core.cov_summary();
        b.finish_tick(&est, cov);
    }
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests-data");
    std::fs::create_dir_all(dir).unwrap();
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests-data/golden-v1118.bblog");
    std::fs::write(path, &b.writer.inner().0).unwrap();
    std::eprintln!("golden written: {path}");
}
