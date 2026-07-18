//! Blackbox LOGGING + REPLAY around the [`FlightBackend`] seam (v1.118,
//! LOG-P02/LOG-P03).
//!
//! [`LoggingBackend`] wraps any backend and records, per control tick,
//! exactly what the flight loop SAW (each sensor read's returned value) and
//! COMMANDED (the motor write) — encoded via relay-log's blackbox codec into
//! a [`BlockLog`]. [`ReplayBackend`] plays a recorded stream back: the same
//! deterministic core, fed the identical input sequence at the identical dt,
//! must reproduce the logged estimator trajectory F32-BIT-EXACTLY — that
//! equality is LOG-P03's oracle, and it makes every recorded flight a
//! regression fixture for the estimator (any change that alters a replayed
//! trajectory is a measured behavioral diff, never an opinion).

use crate::{FlightBackend, ImuSample, NavState};
use relay_iekf::Vec3;
use relay_log::blackbox::{BlackboxWriter, BlockLog, TickRecord, TICK_MAX};

/// Wraps a live backend; call [`LoggingBackend::finish_tick`] after each
/// `FlightCore::step` with the estimator state to seal that tick's record.
pub struct LoggingBackend<'a, B: FlightBackend, L: BlockLog> {
    inner: &'a mut B,
    pub writer: BlackboxWriter<L>,
    cur: TickRecord,
}

fn blank_tick() -> TickRecord {
    TickRecord {
        gyro: [0.0; 3],
        accel: [0.0; 3],
        pos: None,
        mag: None,
        heading: None,
        baro: None,
        rpm: None,
        motors: [0.0; 4],
        est_q: [1.0, 0.0, 0.0, 0.0],
        est_v: [0.0; 3],
        est_p: [0.0; 3],
        cov: [0.0; 3],
    }
}

impl<'a, B: FlightBackend, L: BlockLog> LoggingBackend<'a, B, L> {
    /// `budget_per_tick` bytes may enter the media per tick (drops counted).
    pub fn new(inner: &'a mut B, log: L, budget_per_tick: usize) -> Self {
        LoggingBackend {
            inner,
            writer: BlackboxWriter::new(log, budget_per_tick),
            cur: blank_tick(),
        }
    }

    /// Seal the current tick with the post-step estimator state + covariance
    /// summary and write it.
    pub fn finish_tick(&mut self, est: &NavState, cov: [f32; 3]) {
        self.cur.est_q = est.q;
        self.cur.est_v = est.v;
        self.cur.est_p = est.p;
        self.cur.cov = cov;
        let mut buf = [0u8; TICK_MAX];
        let n = self.cur.encode(&mut buf);
        self.writer.begin_tick();
        self.writer.append(&buf[..n]);
        self.cur = blank_tick();
    }
}

impl<B: FlightBackend, L: BlockLog> FlightBackend for LoggingBackend<'_, B, L> {
    fn read_imu(&mut self) -> ImuSample {
        let s = self.inner.read_imu();
        self.cur.gyro = s.gyro;
        self.cur.accel = s.accel;
        s
    }
    fn read_position(&mut self) -> Option<Vec3> {
        let p = self.inner.read_position();
        self.cur.pos = p;
        p
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        let m = self.inner.read_mag();
        self.cur.mag = m;
        m
    }
    fn read_heading(&mut self) -> Option<f32> {
        let h = self.inner.read_heading();
        self.cur.heading = h;
        h
    }
    fn read_baro(&mut self) -> Option<f32> {
        let b = self.inner.read_baro();
        self.cur.baro = b;
        b
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        let r = self.inner.read_motor_rpm();
        self.cur.rpm = r;
        r
    }
    fn read_battery_v(&mut self) -> f32 {
        self.inner.read_battery_v()
    }
    fn read_gnss_dual(
        &mut self,
    ) -> Option<(Option<falcon_gnss_ubx::dual::NedFix>, Option<falcon_gnss_ubx::dual::NedFix>)>
    {
        // Forwarded, NOT logged: the TickRecord carries the SELECTED position
        // (what the estimator consumed); per-receiver raw lanes are part of
        // the schema-v2 slice (#277).
        self.inner.read_gnss_dual()
    }
    fn read_range(&mut self) -> Option<f32> {
        // Forwarded, NOT logged yet (schema-v2, #277).
        self.inner.read_range()
    }
    fn read_battery_i(&mut self) -> Option<f32> {
        // Forwarded, NOT logged yet: battery samples are not in the v1.118
        // TickRecord schema (extending it regenerates the shipped golden —
        // a deliberate schema-v2 slice, tracked on the campaign board).
        self.inner.read_battery_i()
    }
    fn write_motors(&mut self, motors: &[f32]) {
        for (i, m) in self.cur.motors.iter_mut().enumerate() {
            *m = motors.get(i).copied().unwrap_or(0.0);
        }
        self.inner.write_motors(motors);
    }
    fn dt(&self) -> f32 {
        self.inner.dt()
    }
}

/// Plays a recorded tick sequence back into the core. The physics is gone —
/// only the recorded sensor stream remains — so this is an ESTIMATOR replay:
/// the core's control outputs go nowhere, and the logged `est_*` fields are
/// the per-tick oracle the replayed estimator must match bit-exactly.
pub struct ReplayBackend<'a> {
    ticks: &'a [TickRecord],
    at: usize,
    dt: f32,
}

impl<'a> ReplayBackend<'a> {
    pub fn new(ticks: &'a [TickRecord], dt: f32) -> Self {
        ReplayBackend { ticks, at: 0, dt }
    }
    /// The record currently being replayed (its `est_*` = the oracle).
    pub fn current(&self) -> Option<&TickRecord> {
        self.ticks.get(self.at)
    }
    /// Advance to the next tick (call after comparing the oracle).
    pub fn advance(&mut self) {
        self.at += 1;
    }
    pub fn done(&self) -> bool {
        self.at >= self.ticks.len()
    }
}

impl FlightBackend for ReplayBackend<'_> {
    fn read_imu(&mut self) -> ImuSample {
        let t = &self.ticks[self.at.min(self.ticks.len() - 1)];
        ImuSample { accel: t.accel, gyro: t.gyro }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.ticks.get(self.at).and_then(|t| t.pos)
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.ticks.get(self.at).and_then(|t| t.mag)
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.ticks.get(self.at).and_then(|t| t.heading)
    }
    fn read_baro(&mut self) -> Option<f32> {
        self.ticks.get(self.at).and_then(|t| t.baro)
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.ticks.get(self.at).and_then(|t| t.rpm)
    }
    fn write_motors(&mut self, _motors: &[f32]) {
        // Replay has no plant; the tick advances via `advance()` so the
        // harness can compare the oracle FIRST.
    }
    fn dt(&self) -> f32 {
        self.dt
    }
}
