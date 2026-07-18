//! Relay Log — bounded-ring onboard flight logger (black box).
//!
//! The PX4 ulog role: every flight records a timestamped stream of state +
//! events so a flight (or a SITL run, or a failsafe) is reviewable after the
//! fact. This is a no_std/no_alloc fixed-capacity RING logger (the realistic
//! embedded shape — bounded RAM, oldest entries overwritten when full, with a
//! dropped-count so nothing is silently lost), plus a deterministic 16-byte
//! binary record format and a replay decoder.
//!
//! Verified (Kani): the ring is total — `record` never panics / never indexes
//! out of bounds for any sequence, and the held count never exceeds the
//! capacity; the encode/decode round-trips EXACTLY (a logged entry replays to
//! the same value — a black box must not silently corrupt a field).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// One log record: a millisecond timestamp, a kind tag, and two payload floats
/// (their meaning is per-`kind`, e.g. a position pair, a setpoint, or a failsafe
/// code + value). Fixed 16 bytes on the wire, losslessly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LogEntry {
    /// Timestamp, milliseconds since boot.
    pub t_ms: u32,
    /// Record kind tag.
    pub kind: u8,
    /// Two payload values (kind-specific).
    pub data: [f32; 2],
}

/// Encoded record length: `t_ms` 4 + `kind` 1 + `data` 8 = 13, padded to 16 for
/// alignment. Lossless.
pub const ENTRY_LEN: usize = 16;

impl LogEntry {
    /// Encode to the 16-byte wire record (little-endian); the last 3 bytes pad.
    pub fn encode(&self) -> [u8; ENTRY_LEN] {
        let mut b = [0u8; ENTRY_LEN];
        b[0..4].copy_from_slice(&self.t_ms.to_le_bytes());
        b[4] = self.kind;
        b[5..9].copy_from_slice(&self.data[0].to_le_bytes());
        b[9..13].copy_from_slice(&self.data[1].to_le_bytes());
        b
    }

    /// Decode a 16-byte record. Total: any bytes decode to a defined entry.
    pub fn decode(b: &[u8; ENTRY_LEN]) -> LogEntry {
        LogEntry {
            t_ms: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            kind: b[4],
            data: [
                f32::from_le_bytes([b[5], b[6], b[7], b[8]]),
                f32::from_le_bytes([b[9], b[10], b[11], b[12]]),
            ],
        }
    }
}

/// A fixed-capacity ring flight logger (`N` records).
pub struct FlightLog<const N: usize> {
    buf: [LogEntry; N],
    /// Index of the oldest held record.
    start: usize,
    /// Number of records currently held (≤ N).
    len: usize,
    /// Total records dropped because the ring was full.
    dropped: u32,
}

impl<const N: usize> Default for FlightLog<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FlightLog<N> {
    /// An empty logger.
    pub fn new() -> Self {
        let blank = LogEntry { t_ms: 0, kind: 0, data: [0.0; 2] };
        FlightLog { buf: [blank; N], start: 0, len: 0, dropped: 0 }
    }

    /// Record an entry. When the ring is full the OLDEST entry is overwritten and
    /// `dropped` increments. Total: never panics for any sequence.
    pub fn record(&mut self, e: LogEntry) {
        if N == 0 {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        let write = (self.start + self.len) % N;
        self.buf[write] = e;
        if self.len < N {
            self.len += 1;
        } else {
            self.start = (self.start + 1) % N;
            self.dropped = self.dropped.saturating_add(1);
        }
    }

    /// Number of records currently held (≤ N).
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether the log is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// How many records were dropped (ring overflow).
    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    /// The i-th held record, oldest first (i in 0..len). For replay/export.
    pub fn nth(&self, i: usize) -> Option<LogEntry> {
        if i < self.len && N > 0 {
            Some(self.buf[(self.start + i) % N])
        } else {
            None
        }
    }
}

/// Replay: decode a byte stream of concatenated 16-byte records, invoking `sink`
/// for each complete record. Returns the number decoded. A trailing partial
/// record is ignored (total — never reads past the buffer).
pub fn replay<F: FnMut(LogEntry)>(bytes: &[u8], mut sink: F) -> usize {
    let mut n = 0;
    let mut off = 0;
    while off + ENTRY_LEN <= bytes.len() {
        let mut rec = [0u8; ENTRY_LEN];
        rec.copy_from_slice(&bytes[off..off + ENTRY_LEN]);
        sink(LogEntry::decode(&rec));
        n += 1;
        off += ENTRY_LEN;
    }
    n
}

pub mod blackbox;

/// CRC-32 (IEEE, bitwise, table-free) — shared by the blackbox record framing.
pub(crate) fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        let mut i = 0;
        while i < 8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
            i += 1;
        }
    }
    !crc
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn e(t: u32, k: u8) -> LogEntry {
        LogEntry { t_ms: t, kind: k, data: [t as f32, -(t as f32)] }
    }

    #[test]
    fn records_in_order() {
        let mut log: FlightLog<8> = FlightLog::new();
        for t in 0..5 {
            log.record(e(t, 1));
        }
        assert_eq!(log.len(), 5);
        assert_eq!(log.dropped(), 0);
        for i in 0..5 {
            assert_eq!(log.nth(i).unwrap().t_ms, i as u32);
        }
    }

    #[test]
    fn ring_overwrites_oldest_when_full() {
        let mut log: FlightLog<4> = FlightLog::new();
        for t in 0..6 {
            log.record(e(t, 1));
        }
        assert_eq!(log.len(), 4);
        assert_eq!(log.dropped(), 2);
        assert_eq!(log.nth(0).unwrap().t_ms, 2); // 0,1 dropped
        assert_eq!(log.nth(3).unwrap().t_ms, 5);
        assert!(log.nth(4).is_none());
    }

    #[test]
    fn encode_decode_roundtrips_exactly() {
        let original = LogEntry { t_ms: 123456, kind: 7, data: [1.5, -2.25] };
        assert_eq!(LogEntry::decode(&original.encode()), original);
    }

    #[test]
    fn replay_decodes_a_stream() {
        let a = LogEntry { t_ms: 10, kind: 1, data: [1.0, 2.0] };
        let b = LogEntry { t_ms: 20, kind: 2, data: [3.0, 4.0] };
        let mut stream = [0u8; 32];
        stream[0..16].copy_from_slice(&a.encode());
        stream[16..32].copy_from_slice(&b.encode());
        let mut seen = [0u32; 2];
        let mut i = 0;
        let n = replay(&stream, |rec| {
            if i < 2 {
                seen[i] = rec.t_ms;
                i += 1;
            }
        });
        assert_eq!(n, 2);
        assert_eq!(seen, [10, 20]);
    }
}

#[cfg(test)]
mod blackbox_tests {
    extern crate std;
    use super::blackbox::*;
    use std::vec::Vec;

    struct VecLog(Vec<u8>, Option<usize>); // bytes, fail-after budget
    impl BlockLog for VecLog {
        fn append(&mut self, r: &[u8]) -> Result<(), MediaRejected> {
            if let Some(cap) = self.1 {
                if self.0.len() + r.len() > cap {
                    return Err(MediaRejected);
                }
            }
            self.0.extend_from_slice(r);
            Ok(())
        }
    }

    fn tick(i: u32) -> TickRecord {
        let f = i as f32;
        TickRecord {
            gyro: [0.01 * f, -0.02, 0.005],
            accel: [0.1, 0.0, -9.81 + 0.01 * f],
            pos: (i % 5 == 0).then(|| [1.0 + f, 2.0, -2.0]),
            mag: None,
            heading: (i % 3 == 0).then_some(0.9),
            baro: Some(-2.0 - 0.001 * f),
            rpm: (i % 2 == 0).then(|| [4000 + i as i32, 4001, 4002, 4003]),
            motors: [0.5, 0.51, 0.49, 0.5],
            est_q: [1.0, 0.0, 0.001 * f, 0.0],
            est_v: [0.0, 0.1, -0.05],
            est_p: [f, 2.0, -2.0],
            cov: [0.01, 0.02 + 0.001 * f, 0.03],
        }
    }

    #[test]
    fn tick_roundtrip_bit_exact_all_flag_combos() {
        for i in 0..30u32 {
            let t = tick(i);
            let mut buf = [0u8; TICK_MAX];
            let n = t.encode(&mut buf);
            let mut got = None;
            let (cnt, used) = scan(&buf[..n], |r| {
                assert_eq!(r.rec_type, REC_TICK);
                got = TickRecord::decode(r.payload);
            });
            assert_eq!((cnt, used), (1, n));
            assert_eq!(got.unwrap(), t, "tick {i}");
        }
    }

    #[test]
    fn stream_truncated_at_every_byte_yields_only_complete_records() {
        // Build a stream: boot + 20 ticks + an event.
        let mut stream = Vec::new();
        let mut buf = [0u8; 1024];
        let n = encode_boot(&mut buf, 7, &[(*b"MC_ROLL_P\0\0\0\0\0\0\0", 6.5)]).unwrap();
        stream.extend_from_slice(&buf[..n]);
        let mut boundaries = std::vec![stream.len()];
        for i in 0..20 {
            let mut tb = [0u8; TICK_MAX];
            let n = tick(i).encode(&mut tb);
            stream.extend_from_slice(&tb[..n]);
            boundaries.push(stream.len());
        }
        let mut eb = [0u8; 16];
        let n = encode_event(&mut eb, 3, 1.5);
        stream.extend_from_slice(&eb[..n]);
        boundaries.push(stream.len());

        // Cut at EVERY byte offset: decoded count == records fully before the
        // cut; consumed bytes == that record boundary; never a panic.
        for cut in 0..=stream.len() {
            let expect = boundaries.iter().filter(|&&b| b <= cut).count();
            let (cnt, used) = scan(&stream[..cut], |_| {});
            assert_eq!(cnt, expect, "cut at {cut}");
            let expect_used = boundaries.iter().filter(|&&b| b <= cut).max().copied().unwrap_or(0);
            assert_eq!(used, expect_used, "cut at {cut}");
        }
    }

    #[test]
    fn corrupt_byte_stops_cleanly_never_out_of_bounds() {
        let mut stream = Vec::new();
        for i in 0..5 {
            let mut tb = [0u8; TICK_MAX];
            let n = tick(i).encode(&mut tb);
            stream.extend_from_slice(&tb[..n]);
        }
        for pos in 0..stream.len() {
            let mut evil = stream.clone();
            evil[pos] ^= 0xFF;
            let (cnt, used) = scan(&evil, |r| {
                // Whatever still decodes must decode TOTALLY (no panic).
                if r.rec_type == REC_TICK {
                    let _ = TickRecord::decode(r.payload);
                }
            });
            assert!(cnt <= 5 && used <= evil.len());
        }
    }

    #[test]
    fn writer_budget_drops_are_counted_never_silent() {
        // Budget fits exactly one tick record per tick.
        let mut w = BlackboxWriter::new(VecLog(Vec::new(), None), TICK_MAX);
        let t = tick(0);
        let mut buf = [0u8; TICK_MAX];
        let n = t.encode(&mut buf);
        w.begin_tick();
        w.append(&buf[..n]); // fits
        w.append(&buf[..n]); // over budget -> dropped
        assert_eq!((w.written, w.dropped), (1, 1));
        // Media failure also counts as a drop.
        let mut w2 = BlackboxWriter::new(VecLog(Vec::new(), Some(10)), TICK_MAX);
        w2.begin_tick();
        w2.append(&buf[..n]);
        assert_eq!((w2.written, w2.dropped), (0, 1));
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For any sequence of records, the held count never exceeds capacity and
        /// held + dropped equals the number recorded.
        #[test]
        fn ring_invariant(seq in prop::collection::vec(0u32..1000, 0..200)) {
            let mut log: FlightLog<16> = FlightLog::new();
            let total = seq.len();
            for t in &seq {
                log.record(LogEntry { t_ms: *t, kind: 0, data: [0.0; 2] });
            }
            prop_assert!(log.len() <= 16);
            prop_assert_eq!(log.len() as u32 + log.dropped(), total as u32);
        }

        /// encode∘decode is the identity for any record (lossless black box).
        #[test]
        fn encode_decode_identity(t in any::<u32>(), k in any::<u8>(), x in any::<f32>(), y in any::<f32>()) {
            let e = LogEntry { t_ms: t, kind: k, data: [x, y] };
            let back = LogEntry::decode(&e.encode());
            prop_assert_eq!(back.t_ms, e.t_ms);
            prop_assert_eq!(back.kind, e.kind);
            prop_assert_eq!(back.data[0].to_bits(), e.data[0].to_bits());
            prop_assert_eq!(back.data[1].to_bits(), e.data[1].to_bits());
        }
    }
}
