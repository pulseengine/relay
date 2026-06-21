//! Falcon GNSS driver — u-blox UBX protocol parser.
//!
//! A streaming byte-at-a-time UBX frame parser (the realistic UART driver shape)
//! that decodes UBX-NAV-PVT (position/velocity/time) and validates the UBX
//! 8-bit Fletcher checksum. This is one of the real driver BODIES the readiness
//! dossier flagged as the #1 hardware-boundary gap (F6) — previously only the
//! ICM-42688 IMU had a real driver. It is verified the same way: a protocol
//! mock-stream test (feed exact UBX bytes, assert the decoded fix) plus Kani
//! totality (the parser never panics on ANY byte stream). On-silicon validation
//! over a real UART remains the honest open item (the byte-level protocol is
//! what this crate pins down).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`. A fixed 128-byte frame buffer; UBX
//! payloads longer than that are skipped (the parser resyncs), never overflow.

#![no_std]
#![forbid(unsafe_code)]

const SYNC1: u8 = 0xB5;
const SYNC2: u8 = 0x62;
const CLASS_NAV: u8 = 0x01;
const ID_PVT: u8 = 0x07;
const PVT_LEN: usize = 92;
const BUF: usize = 128;

/// A decoded UBX-NAV-PVT fix (the fields the estimator/flight stack consumes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NavPvt {
    /// Latitude, 1e-7 degrees.
    pub lat_deg_e7: i32,
    /// Longitude, 1e-7 degrees.
    pub lon_deg_e7: i32,
    /// Height above ellipsoid, mm.
    pub height_mm: i32,
    /// Height above mean sea level, mm.
    pub h_msl_mm: i32,
    /// NED velocity north, mm/s.
    pub vel_n_mmps: i32,
    /// NED velocity east, mm/s.
    pub vel_e_mmps: i32,
    /// NED velocity down, mm/s.
    pub vel_d_mmps: i32,
    /// Fix type (0 none, 2 2D, 3 3D, …).
    pub fix_type: u8,
    /// Satellites used.
    pub num_sv: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Sync1,
    Sync2,
    Header, // class, id, len_l, len_h (4 bytes)
    Payload,
    CkA,
    CkB,
}

/// Streaming UBX parser. Feed bytes with [`push`](Self::push); it returns
/// `Some(NavPvt)` exactly when a complete, checksum-valid NAV-PVT frame ends.
pub struct UbxParser {
    state: State,
    hdr: [u8; 4],
    hdr_n: usize,
    class: u8,
    id: u8,
    len: usize,
    buf: [u8; BUF],
    idx: usize,
    ck_a: u8,
    ck_b: u8,
    rx_ck_a: u8,
}

impl Default for UbxParser {
    fn default() -> Self {
        Self::new()
    }
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            state: State::Sync1,
            hdr: [0; 4],
            hdr_n: 0,
            class: 0,
            id: 0,
            len: 0,
            buf: [0; BUF],
            idx: 0,
            ck_a: 0,
            ck_b: 0,
            rx_ck_a: 0,
        }
    }

    fn reset(&mut self) {
        self.state = State::Sync1;
        self.hdr_n = 0;
        self.idx = 0;
        self.ck_a = 0;
        self.ck_b = 0;
    }

    // UBX 8-bit Fletcher: accumulated over class, id, len(2), payload.
    #[inline]
    fn ck(&mut self, b: u8) {
        self.ck_a = self.ck_a.wrapping_add(b);
        self.ck_b = self.ck_b.wrapping_add(self.ck_a);
    }

    /// Feed one received byte. Returns the decoded fix when a valid NAV-PVT
    /// frame completes; `None` otherwise. Total: never panics for any input.
    pub fn push(&mut self, byte: u8) -> Option<NavPvt> {
        match self.state {
            State::Sync1 => {
                if byte == SYNC1 {
                    self.state = State::Sync2;
                }
            }
            State::Sync2 => {
                self.state = if byte == SYNC2 { State::Header } else { State::Sync1 };
                self.hdr_n = 0;
                self.ck_a = 0;
                self.ck_b = 0;
            }
            State::Header => {
                self.hdr[self.hdr_n] = byte;
                self.ck(byte);
                self.hdr_n += 1;
                if self.hdr_n == 4 {
                    self.class = self.hdr[0];
                    self.id = self.hdr[1];
                    self.len = self.hdr[2] as usize | ((self.hdr[3] as usize) << 8);
                    self.idx = 0;
                    if self.len == 0 {
                        self.state = State::CkA;
                    } else {
                        self.state = State::Payload;
                    }
                }
            }
            State::Payload => {
                if self.idx < BUF {
                    self.buf[self.idx] = byte;
                }
                self.ck(byte);
                self.idx += 1;
                if self.idx == self.len {
                    self.state = State::CkA;
                }
            }
            State::CkA => {
                self.rx_ck_a = byte;
                self.state = State::CkB;
            }
            State::CkB => {
                let ok = self.rx_ck_a == self.ck_a && byte == self.ck_b;
                let is_pvt = self.class == CLASS_NAV && self.id == ID_PVT;
                let fits = self.len >= PVT_LEN && self.len <= BUF;
                let out = if ok && is_pvt && fits {
                    Some(decode_nav_pvt(&self.buf))
                } else {
                    None
                };
                self.reset();
                return out;
            }
        }
        None
    }
}

#[inline]
fn le_i32(b: &[u8], off: usize) -> i32 {
    i32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

/// Decode the NAV-PVT payload fields (UBX spec offsets). `payload` is at least
/// `PVT_LEN` bytes (guaranteed by the caller).
fn decode_nav_pvt(p: &[u8; BUF]) -> NavPvt {
    NavPvt {
        fix_type: p[20],
        num_sv: p[23],
        lon_deg_e7: le_i32(p, 24),
        lat_deg_e7: le_i32(p, 28),
        height_mm: le_i32(p, 32),
        h_msl_mm: le_i32(p, 36),
        vel_n_mmps: le_i32(p, 48),
        vel_e_mmps: le_i32(p, 52),
        vel_d_mmps: le_i32(p, 56),
    }
}

/// Build a complete UBX-NAV-PVT frame into `out` from a 92-byte payload,
/// computing the sync words, header, and Fletcher checksum. Returns the frame
/// length. Test/encoder helper — mirrors what a u-blox receiver emits on the
/// wire, so the parser round-trips against it. Available to integrators too.
pub fn encode_nav_pvt_frame(payload: &[u8; PVT_LEN], out: &mut [u8; PVT_LEN + 8]) -> usize {
    out[0] = SYNC1;
    out[1] = SYNC2;
    out[2] = CLASS_NAV;
    out[3] = ID_PVT;
    out[4] = (PVT_LEN & 0xff) as u8;
    out[5] = (PVT_LEN >> 8) as u8;
    out[6..6 + PVT_LEN].copy_from_slice(payload);
    let mut a = 0u8;
    let mut b = 0u8;
    for &byte in &out[2..6 + PVT_LEN] {
        a = a.wrapping_add(byte);
        b = b.wrapping_add(a);
    }
    out[6 + PVT_LEN] = a;
    out[7 + PVT_LEN] = b;
    PVT_LEN + 8
}

/// Async UBX NAV-PVT reader over an **`embedded-io-async` `Read`** byte source —
/// the hardware path (relay#214 / jess#62). jess binds the i.MX RT1176 LPUART
/// (DMA RX) under `Read`; this wraps the SAME pure [`UbxParser`] the sync path
/// uses, so the verified frame parser lives once above the seam and only the
/// transport differs. Fallible: a read error returns `R::Error`, the per-device
/// health source the estimator votes on.
pub struct UbxReader<R> {
    rx: R,
    parser: UbxParser,
}

impl<R: embedded_io_async::Read> UbxReader<R> {
    /// Wrap an async byte source.
    pub fn new(rx: R) -> Self {
        Self { rx, parser: UbxParser::new() }
    }

    /// Read bytes until a complete, checksum-valid NAV-PVT frame decodes, then
    /// return it. Every received byte is fed through the verified parser, so a
    /// fix mid-buffer is returned immediately (a real GNSS stream is continuous;
    /// a bounded source must deliver a full frame before it stops yielding).
    pub async fn read_fix(&mut self) -> Result<NavPvt, R::Error> {
        let mut buf = [0u8; 64];
        loop {
            let n = self.rx.read(&mut buf).await?;
            for &b in &buf[..n] {
                if let Some(fix) = self.parser.push(b) {
                    return Ok(fix);
                }
            }
        }
    }

    /// Consume the reader, returning the byte source.
    pub fn release(self) -> R {
        self.rx
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_payload() -> [u8; PVT_LEN] {
        let mut p = [0u8; PVT_LEN];
        p[20] = 3; // fix_type 3D
        p[23] = 11; // num_sv
        p[24..28].copy_from_slice(&(85_000_000_i32).to_le_bytes()); // lon 8.5°
        p[28..32].copy_from_slice(&(472_000_000_i32).to_le_bytes()); // lat 47.2°
        p[32..36].copy_from_slice(&(540_000_i32).to_le_bytes()); // height 540 m
        p[36..40].copy_from_slice(&(492_000_i32).to_le_bytes()); // hMSL 492 m
        p[48..52].copy_from_slice(&(1_500_i32).to_le_bytes()); // velN 1.5 m/s
        p[52..56].copy_from_slice(&(-800_i32).to_le_bytes()); // velE -0.8 m/s
        p[56..60].copy_from_slice(&(200_i32).to_le_bytes()); // velD 0.2 m/s
        p
    }

    #[test]
    fn parses_a_valid_nav_pvt_frame() {
        let payload = sample_payload();
        let mut frame = [0u8; PVT_LEN + 8];
        let n = encode_nav_pvt_frame(&payload, &mut frame);

        let mut parser = UbxParser::new();
        let mut got = None;
        for &b in &frame[..n] {
            if let Some(pvt) = parser.push(b) {
                got = Some(pvt);
            }
        }
        let pvt = got.expect("a valid frame must decode");
        assert_eq!(pvt.fix_type, 3);
        assert_eq!(pvt.num_sv, 11);
        assert_eq!(pvt.lon_deg_e7, 85_000_000);
        assert_eq!(pvt.lat_deg_e7, 472_000_000);
        assert_eq!(pvt.height_mm, 540_000);
        assert_eq!(pvt.h_msl_mm, 492_000);
        assert_eq!(pvt.vel_n_mmps, 1_500);
        assert_eq!(pvt.vel_e_mmps, -800);
        assert_eq!(pvt.vel_d_mmps, 200);
    }

    #[test]
    fn rejects_a_corrupted_frame() {
        let payload = sample_payload();
        let mut frame = [0u8; PVT_LEN + 8];
        let n = encode_nav_pvt_frame(&payload, &mut frame);
        frame[30] ^= 0xff; // flip a payload byte → checksum mismatch

        let mut parser = UbxParser::new();
        let mut got = None;
        for &b in &frame[..n] {
            if let Some(pvt) = parser.push(b) {
                got = Some(pvt);
            }
        }
        assert!(got.is_none(), "a checksum-invalid frame must be rejected");
    }

    #[test]
    fn resyncs_after_garbage() {
        let payload = sample_payload();
        let mut frame = [0u8; PVT_LEN + 8];
        let n = encode_nav_pvt_frame(&payload, &mut frame);

        let mut parser = UbxParser::new();
        // Feed leading garbage (incl. a stray sync), then the real frame.
        for b in [0x00, 0xB5, 0x11, 0xFF, 0xB5, 0x00] {
            parser.push(b);
        }
        let mut got = None;
        for &b in &frame[..n] {
            if let Some(pvt) = parser.push(b) {
                got = Some(pvt);
            }
        }
        assert!(got.is_some(), "the parser must resync and decode the real frame");
    }

    // ── async stream path (embedded-io-async Read) + inter-core carrier ──────

    /// A mock `embedded-io-async` Read that yields a byte buffer once, then 0.
    struct MockRx {
        data: [u8; 128],
        len: usize,
        pos: usize,
        fail: bool,
    }

    #[derive(Debug)]
    struct MockRxError;
    impl core::fmt::Display for MockRxError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str("MockRxError")
        }
    }
    impl core::error::Error for MockRxError {}
    impl embedded_io::Error for MockRxError {
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }
    impl embedded_io::ErrorType for MockRx {
        type Error = MockRxError;
    }
    impl embedded_io_async::Read for MockRx {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, MockRxError> {
            if self.fail {
                return Err(MockRxError);
            }
            let remaining = self.len - self.pos;
            let n = remaining.min(buf.len());
            buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
            self.pos += n;
            Ok(n)
        }
    }

    /// Drive a future that completes synchronously (the mock delivers the whole
    /// frame in one read, so `read_fix` returns on the first poll).
    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, Waker};
        let mut fut = core::pin::pin!(fut);
        let mut cx = Context::from_waker(Waker::noop());
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    fn one_frame() -> ([u8; 128], usize) {
        let payload = sample_payload();
        let mut frame = [0u8; PVT_LEN + 8];
        let n = encode_nav_pvt_frame(&payload, &mut frame);
        let mut data = [0u8; 128];
        data[..n].copy_from_slice(&frame[..n]);
        (data, n)
    }

    /// The async Read path and the sync push loop decode an identical frame to a
    /// byte-identical NavPvt — the stream re-target preserves the verified parser.
    #[test]
    fn gnss_async_read_matches_sync_parse() {
        let (data, n) = one_frame();
        // sync push loop
        let mut parser = UbxParser::new();
        let mut sync_fix = None;
        for &b in &data[..n] {
            if let Some(p) = parser.push(b) {
                sync_fix = Some(p);
            }
        }
        let sync_fix = sync_fix.unwrap();
        // async Read path
        let mut rdr = UbxReader::new(MockRx { data, len: n, pos: 0, fail: false });
        let async_fix = block_on(rdr.read_fix()).unwrap();
        assert_eq!(sync_fix, async_fix);
    }

    /// The async path is fallible: a read transport error propagates as Err.
    #[test]
    fn gnss_async_propagates_read_error() {
        let mut rdr = UbxReader::new(MockRx { data: [0; 128], len: 0, pos: 0, fail: true });
        assert!(block_on(rdr.read_fix()).is_err());
    }

    /// relay#177 — a decoded NAV-PVT fix (a larger HAL payload than MagField)
    /// rides the relay-bus MessageTransport ring across the M4→M7 seam,
    /// byte-identical + FIFO.
    #[test]
    fn nav_pvt_rides_the_intercore_ring() {
        use relay_bus::{MessageTransport, SpscRing};
        let (data, n) = one_frame();
        let mut parser = UbxParser::new();
        let mut fix = None;
        for &b in &data[..n] {
            if let Some(p) = parser.push(b) {
                fix = Some(p);
            }
        }
        let fix = fix.unwrap();
        let mut ring: SpscRing<NavPvt, 2> = SpscRing::new();
        assert!(ring.push(fix));
        assert_eq!(ring.pop(), Some(fix));
        assert!(ring.pop().is_none());
    }
}
