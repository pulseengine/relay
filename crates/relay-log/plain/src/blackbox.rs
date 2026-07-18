//! BLACKBOX record codec (LOG-P02, v1.118) — the persistent flight log.
//!
//! [`FlightLog`](crate::FlightLog) (LOG-P01) is the fixed-16-byte RAM ring for
//! events; a crash post-mortem and estimator REPLAY (LOG-P03) need more: the
//! full per-tick sensor inputs, actuator outputs, and estimator state, framed
//! so that a power cut mid-write truncates — never corrupts — the record
//! stream. This module owns the relay half of the storage split agreed in
//! jess#144 (their REQ-PIX-022): the RECORD CODEC, the per-tick rate BUDGET
//! (drops are counted, never silent), and the ANY-BYTE-OFFSET truncation
//! decoder. jess owns the media below the [`BlockLog`] seam (flush ordering,
//! torn-record detectability at the block level).
//!
//! ## Framing
//!
//! ```text
//! [magic 0xB5][type u8][len u16 LE][payload: len bytes][crc32 u32 LE]
//! ```
//!
//! The CRC covers `type|len|payload`. The scanner walks records from the
//! front and STOPS CLEANLY at the first magic/length/CRC failure or
//! incomplete tail — so a log cut at any byte offset yields exactly the
//! complete records before the cut (`scan`'s contract, property-tested over
//! every cut point).
//!
//! ## Record types
//!
//! * `Boot` — schema/version marker + the parameter snapshot at boot.
//! * `Tick` — one control tick: raw IMU, the optional sensor reads exactly as
//!   the flight loop saw them (fix/mag/heading/baro/RPM present-flags), the
//!   motor commands, and the estimator state (q, v, p) — the last being the
//!   REPLAY ORACLE: a bit-exact re-run must reproduce it.
//! * `Event` — mode changes / failsafe firings (code + argument).

use crate::crc32;

/// Byte-stream sink for the blackbox — the media seam (jess REQ-PIX-022
/// binds SD/NOR below it; tests use a Vec-backed mock with torn-write
/// injection). `append` must either take the WHOLE record or reject it.
pub trait BlockLog {
    /// Append one encoded record. `Err(MediaRejected)` = media full/failed
    /// (the writer counts it as a drop — loudly).
    fn append(&mut self, record: &[u8]) -> Result<(), MediaRejected>;
}

/// The media refused an append (full / write failure) — the whole record was
/// rejected, never a partial write (that contract is jess's side of the
/// REQ-PIX-022 split; the writer's side is to COUNT the drop).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MediaRejected;

/// Record type tags.
pub const REC_BOOT: u8 = 0x01;
pub const REC_TICK: u8 = 0x02;
pub const REC_EVENT: u8 = 0x03;

const MAGIC: u8 = 0xB5;
const HDR: usize = 4; // magic + type + len u16
const CRC_LEN: usize = 4;

/// One control tick as the flight loop saw it. Optionals mirror the
/// [`FlightBackend`] reads (`None` = the backend offered nothing that tick),
/// so a replay presents the estimator with the IDENTICAL input sequence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickRecord {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub pos: Option<[f32; 3]>,
    pub mag: Option<[f32; 3]>,
    pub heading: Option<f32>,
    pub baro: Option<f32>,
    pub rpm: Option<[i32; 4]>,
    pub motors: [f32; 4],
    /// Estimator output AFTER this tick — the replay oracle (q, v, p).
    pub est_q: [f32; 4],
    pub est_v: [f32; 3],
    pub est_p: [f32; 3],
    /// Covariance-diagonal summary [attitude, velocity, position] — logged
    /// estimator health (LOG-P02), also replay-oracle-covered.
    pub cov: [f32; 3],
}

/// Max encoded tick size (all optionals present).
pub const TICK_MAX: usize = HDR + 1 + 24 + 12 + 12 + 4 + 4 + 16 + 16 + 52 + CRC_LEN;

fn put_f32(buf: &mut [u8], at: &mut usize, v: f32) {
    buf[*at..*at + 4].copy_from_slice(&v.to_le_bytes());
    *at += 4;
}
fn get_f32(buf: &[u8], at: &mut usize) -> f32 {
    let v = f32::from_le_bytes([buf[*at], buf[*at + 1], buf[*at + 2], buf[*at + 3]]);
    *at += 4;
    v
}

impl TickRecord {
    /// Encode into `out` (needs `>= TICK_MAX`); returns the encoded length.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let mut flags = 0u8;
        if self.pos.is_some() {
            flags |= 1;
        }
        if self.mag.is_some() {
            flags |= 2;
        }
        if self.heading.is_some() {
            flags |= 4;
        }
        if self.baro.is_some() {
            flags |= 8;
        }
        if self.rpm.is_some() {
            flags |= 16;
        }
        let mut at = HDR;
        out[at] = flags;
        at += 1;
        for v in self.gyro.iter().chain(self.accel.iter()) {
            put_f32(out, &mut at, *v);
        }
        if let Some(p) = self.pos {
            for v in p {
                put_f32(out, &mut at, v);
            }
        }
        if let Some(m) = self.mag {
            for v in m {
                put_f32(out, &mut at, v);
            }
        }
        if let Some(h) = self.heading {
            put_f32(out, &mut at, h);
        }
        if let Some(b) = self.baro {
            put_f32(out, &mut at, b);
        }
        if let Some(r) = self.rpm {
            for v in r {
                out[at..at + 4].copy_from_slice(&v.to_le_bytes());
                at += 4;
            }
        }
        for v in self.motors {
            put_f32(out, &mut at, v);
        }
        for v in self
            .est_q
            .iter()
            .chain(self.est_v.iter())
            .chain(self.est_p.iter())
            .chain(self.cov.iter())
        {
            put_f32(out, &mut at, *v);
        }
        let len = (at - HDR) as u16;
        out[0] = MAGIC;
        out[1] = REC_TICK;
        out[2..4].copy_from_slice(&len.to_le_bytes());
        let crc = crc32(&out[1..at]);
        out[at..at + 4].copy_from_slice(&crc.to_le_bytes());
        at + 4
    }

    /// Decode a VERIFIED payload (the scanner has already checked the CRC).
    /// Returns `None` on a structurally short payload (defensive totality).
    pub fn decode(payload: &[u8]) -> Option<TickRecord> {
        if payload.is_empty() {
            return None;
        }
        let flags = payload[0];
        let mut need = 1 + 24 + 16 + 52;
        if flags & 1 != 0 {
            need += 12;
        }
        if flags & 2 != 0 {
            need += 12;
        }
        if flags & 4 != 0 {
            need += 4;
        }
        if flags & 8 != 0 {
            need += 4;
        }
        if flags & 16 != 0 {
            need += 16;
        }
        if payload.len() != need {
            return None;
        }
        let mut at = 1usize;
        let mut g = [0.0f32; 3];
        let mut a = [0.0f32; 3];
        for v in g.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        for v in a.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        let pos = (flags & 1 != 0).then(|| {
            let mut p = [0.0f32; 3];
            for v in p.iter_mut() {
                *v = get_f32(payload, &mut at);
            }
            p
        });
        let mag = (flags & 2 != 0).then(|| {
            let mut m = [0.0f32; 3];
            for v in m.iter_mut() {
                *v = get_f32(payload, &mut at);
            }
            m
        });
        let heading = (flags & 4 != 0).then(|| get_f32(payload, &mut at));
        let baro = (flags & 8 != 0).then(|| get_f32(payload, &mut at));
        let rpm = (flags & 16 != 0).then(|| {
            let mut r = [0i32; 4];
            for v in r.iter_mut() {
                *v = i32::from_le_bytes([
                    payload[at],
                    payload[at + 1],
                    payload[at + 2],
                    payload[at + 3],
                ]);
                at += 4;
            }
            r
        });
        let mut motors = [0.0f32; 4];
        for v in motors.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        let mut est_q = [0.0f32; 4];
        let mut est_v = [0.0f32; 3];
        let mut est_p = [0.0f32; 3];
        for v in est_q.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        for v in est_v.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        for v in est_p.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        let mut cov = [0.0f32; 3];
        for v in cov.iter_mut() {
            *v = get_f32(payload, &mut at);
        }
        Some(TickRecord {
            gyro: g,
            accel: a,
            pos,
            mag,
            heading,
            baro,
            rpm,
            motors,
            est_q,
            est_v,
            est_p,
            cov,
        })
    }
}

/// Encode a Boot record: schema version + `(id, value)` parameter snapshot.
/// Returns the encoded length, or `None` if `out` is too small.
pub fn encode_boot(
    out: &mut [u8],
    schema_version: u32,
    params: &[([u8; 16], f32)],
) -> Option<usize> {
    let payload_len = 4 + 2 + params.len() * 20;
    let total = HDR + payload_len + CRC_LEN;
    if out.len() < total || params.len() > u16::MAX as usize {
        return None;
    }
    let mut at = HDR;
    out[at..at + 4].copy_from_slice(&schema_version.to_le_bytes());
    at += 4;
    out[at..at + 2].copy_from_slice(&(params.len() as u16).to_le_bytes());
    at += 2;
    for (id, v) in params {
        out[at..at + 16].copy_from_slice(id);
        at += 16;
        put_f32(out, &mut at, *v);
    }
    out[0] = MAGIC;
    out[1] = REC_BOOT;
    out[2..4].copy_from_slice(&(payload_len as u16).to_le_bytes());
    let crc = crc32(&out[1..at]);
    out[at..at + 4].copy_from_slice(&crc.to_le_bytes());
    Some(at + 4)
}

/// Encode an Event record (mode change / failsafe: code + argument).
pub fn encode_event(out: &mut [u8; 16], code: u8, arg: f32) -> usize {
    let mut at = HDR;
    out[at] = code;
    at += 1;
    put_f32(out, &mut at, arg);
    out[0] = MAGIC;
    out[1] = REC_EVENT;
    out[2..4].copy_from_slice(&5u16.to_le_bytes());
    let crc = crc32(&out[1..at]);
    out[at..at + 4].copy_from_slice(&crc.to_le_bytes());
    at + 4
}

/// A scanned record: type tag + payload slice (CRC already verified).
pub struct ScannedRecord<'a> {
    pub rec_type: u8,
    pub payload: &'a [u8],
}

/// Walk the byte stream from the front, yielding each CRC-valid record to
/// `sink`; STOP at the first invalid/incomplete record. Returns
/// `(records_yielded, bytes_consumed)` — the truncation contract: a stream
/// cut at ANY byte offset yields exactly the complete records before the
/// cut. Total for arbitrary bytes; never panics.
pub fn scan<F: FnMut(ScannedRecord<'_>)>(bytes: &[u8], mut sink: F) -> (usize, usize) {
    let mut at = 0usize;
    let mut n = 0usize;
    loop {
        if bytes.len() - at < HDR + CRC_LEN {
            return (n, at);
        }
        if bytes[at] != MAGIC {
            return (n, at);
        }
        let rec_type = bytes[at + 1];
        let len = u16::from_le_bytes([bytes[at + 2], bytes[at + 3]]) as usize;
        let total = HDR + len + CRC_LEN;
        if bytes.len() - at < total {
            return (n, at);
        }
        let body = &bytes[at + 1..at + HDR + len];
        let stored = u32::from_le_bytes([
            bytes[at + HDR + len],
            bytes[at + HDR + len + 1],
            bytes[at + HDR + len + 2],
            bytes[at + HDR + len + 3],
        ]);
        if crc32(body) != stored {
            return (n, at);
        }
        sink(ScannedRecord { rec_type, payload: &bytes[at + HDR..at + HDR + len] });
        n += 1;
        at += total;
    }
}

/// Rate-budgeted blackbox writer: at most `budget_per_tick` bytes enter the
/// media per tick; overflow and media failures are COUNTED drops (loud),
/// never silent. The budget models the media's sustained write rate so a
/// burst cannot starve the control loop.
pub struct BlackboxWriter<L: BlockLog> {
    log: L,
    budget_per_tick: usize,
    spent_this_tick: usize,
    pub dropped: u32,
    pub written: u32,
}

impl<L: BlockLog> BlackboxWriter<L> {
    pub fn new(log: L, budget_per_tick: usize) -> Self {
        BlackboxWriter { log, budget_per_tick, spent_this_tick: 0, dropped: 0, written: 0 }
    }

    /// Start a new control tick (resets the budget window).
    pub fn begin_tick(&mut self) {
        self.spent_this_tick = 0;
    }

    /// Append one encoded record within the tick budget.
    pub fn append(&mut self, record: &[u8]) {
        if self.spent_this_tick + record.len() > self.budget_per_tick {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        match self.log.append(record) {
            Ok(()) => {
                self.spent_this_tick += record.len();
                self.written = self.written.saturating_add(1);
            }
            Err(MediaRejected) => self.dropped = self.dropped.saturating_add(1),
        }
    }

    pub fn into_inner(self) -> L {
        self.log
    }
    pub fn inner(&self) -> &L {
        &self.log
    }
}
