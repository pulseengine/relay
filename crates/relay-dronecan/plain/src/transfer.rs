//! DroneCAN v0 multi-frame transfer reassembly — the fixed-buffer state machine.
//!
//! Accepts [`CanFrame`]s and emits a completed [`Transfer`] when an
//! end-of-transfer frame closes a valid transfer. The verified core of the
//! peripheral bus, and the prime Kani target (`kani_proofs.rs`):
//!   * **Totality** — `push` never panics for ANY frame from ANY state.
//!   * **Buffer-bound** — the accumulated payload never exceeds [`MAX_PAYLOAD`];
//!     an oversized transfer is dropped, never overflowed (the AADL bound sink).
//!   * **CRC-gating** — a multi-frame transfer only completes if its transfer
//!     CRC matches (the AADL CRC sink); a corrupt one is dropped.
//!   * **Sequencing** — the toggle bit and transfer id must follow the protocol;
//!     a re-ordered/duplicated frame is dropped (the AADL sequence sink).
//!
//! v1.92 is a SINGLE-SLOT reassembler (one in-flight transfer): a fresh
//! start-of-transfer preempts any active one. Per-source-node slotting (for
//! interleaved peripherals) is the mechanical v1.93+ extension; the verified
//! state-machine core proven here is unchanged by it.

use crate::crc::{crc16_add, crc16_signature};
use crate::id::decode_message_id;
use crate::tail::decode_tail;

/// The CAN frame value type is owned by the relay-hal seam (jess binds FlexCAN
/// underneath); the transfer layer decodes these into reassembled transfers.
pub use relay_hal::CanFrame;

/// The fixed reassembly buffer size. Covers the falcon DroneCAN message set
/// (the largest, gnss.Fix2, is ~62 bytes); an oversized transfer is dropped.
pub const MAX_PAYLOAD: usize = 128;

/// A reassembled DroneCAN transfer: the concatenated payload (CRC-checked for
/// multi-frame) tagged with its data-type id, source node, and transfer id.
#[derive(Clone, Copy)]
pub struct Transfer {
    /// The message data-type id (e.g. 341 NodeStatus).
    pub data_type_id: u16,
    /// The source node id, 0..=127.
    pub source_node_id: u8,
    /// The transfer id, 0..=31.
    pub transfer_id: u8,
    /// Number of valid payload bytes in `payload` (always `<= MAX_PAYLOAD`).
    pub len: usize,
    /// The reassembled payload (only `len` bytes are valid).
    pub payload: [u8; MAX_PAYLOAD],
}

/// A single-slot DroneCAN transfer reassembler for one message type (one 64-bit
/// data-type signature, used to verify the multi-frame transfer CRC).
pub struct Reassembler {
    /// The data-type signature for the CRC check (this reassembler's message).
    pub(crate) signature: u64,
    /// Whether a multi-frame transfer is currently being accumulated.
    pub(crate) active: bool,
    /// The transfer id of the active multi-frame transfer.
    pub(crate) transfer_id: u8,
    /// The data-type id / source node of the active transfer.
    pub(crate) data_type_id: u16,
    pub(crate) source_node_id: u8,
    /// The toggle bit expected on the next continuation frame.
    pub(crate) expected_toggle: bool,
    /// The transfer CRC carried in the first frame, to verify at end-of-transfer.
    pub(crate) expected_crc: u16,
    /// The reassembly buffer and its current fill (`len <= MAX_PAYLOAD` invariant).
    pub(crate) buf: [u8; MAX_PAYLOAD],
    pub(crate) len: usize,
}

impl Reassembler {
    /// A fresh reassembler for the message with the given data-type signature.
    pub fn new(signature: u64) -> Self {
        Self {
            signature,
            active: false,
            transfer_id: 0,
            data_type_id: 0,
            source_node_id: 0,
            expected_toggle: false,
            expected_crc: 0,
            buf: [0; MAX_PAYLOAD],
            len: 0,
        }
    }

    fn drop_active(&mut self) {
        self.active = false;
        self.len = 0;
    }

    /// Append `chunk` to the buffer, dropping the active transfer (returning
    /// `false`) if it would exceed [`MAX_PAYLOAD`]. The buffer-bound guard: the
    /// copy slice upper bound is checked `<= MAX_PAYLOAD` before the copy, so it
    /// can never index out of bounds. (pub(crate) so the Kani harness can prove
    /// the bound invariant on it directly — the inductive core of DC-K03.)
    pub(crate) fn accumulate(&mut self, chunk: &[u8]) -> bool {
        if self.len > MAX_PAYLOAD || chunk.len() > MAX_PAYLOAD - self.len {
            self.drop_active();
            return false;
        }
        self.buf[self.len..self.len + chunk.len()].copy_from_slice(chunk);
        self.len += chunk.len();
        true
    }

    /// Feed one CAN frame. Returns a completed [`Transfer`] when an
    /// end-of-transfer frame closes a valid transfer; `None` otherwise.
    ///
    /// Total: never panics for ANY frame from ANY state, and the `len`
    /// invariant (`<= MAX_PAYLOAD`) is preserved (Kani DC-K01/K02).
    pub fn push(&mut self, frame: &CanFrame) -> Option<Transfer> {
        let mid = decode_message_id(frame.id)?; // ignore service frames
        let dlc = (frame.dlc as usize).min(8);
        if dlc == 0 {
            return None; // need at least the tail byte
        }
        let tail = decode_tail(frame.data[dlc - 1]);
        let payload = &frame.data[..dlc - 1]; // data minus the tail byte (0..=7 bytes)

        // Single-frame transfer: SOT & EOT — emit directly (no transfer CRC).
        if tail.start_of_transfer && tail.end_of_transfer {
            self.drop_active();
            let n = payload.len().min(MAX_PAYLOAD);
            let mut out = Transfer {
                data_type_id: mid.data_type_id,
                source_node_id: mid.source_node_id,
                transfer_id: tail.transfer_id,
                len: n,
                payload: [0; MAX_PAYLOAD],
            };
            out.payload[..n].copy_from_slice(&payload[..n]);
            return Some(out);
        }

        // Start of a multi-frame transfer: SOT & !EOT.
        if tail.start_of_transfer {
            self.drop_active();
            // The first frame's payload begins with the 2-byte transfer CRC; the
            // first frame's toggle must be 0. A malformed start is dropped.
            if tail.toggle || payload.len() < 2 {
                return None;
            }
            self.active = true;
            self.transfer_id = tail.transfer_id;
            self.data_type_id = mid.data_type_id;
            self.source_node_id = mid.source_node_id;
            self.expected_crc = u16::from_le_bytes([payload[0], payload[1]]);
            self.expected_toggle = true; // next frame toggles to 1
            self.len = 0;
            self.accumulate(&payload[2..]);
            return None;
        }

        // Continuation frame: !SOT. Must match an active transfer's id + toggle.
        if !self.active
            || tail.transfer_id != self.transfer_id
            || tail.toggle != self.expected_toggle
        {
            // A stray, re-ordered, or duplicated frame — drop any active transfer.
            if self.active && !tail.start_of_transfer {
                self.drop_active();
            }
            return None;
        }
        if !self.accumulate(payload) {
            return None; // overflowed the buffer bound — dropped
        }
        self.expected_toggle = !self.expected_toggle;

        if tail.end_of_transfer {
            // Verify the transfer CRC over the data-type signature + payload.
            let crc = crc16_add(crc16_signature(self.signature), &self.buf[..self.len]);
            let out = if crc == self.expected_crc {
                Some(Transfer {
                    data_type_id: self.data_type_id,
                    source_node_id: self.source_node_id,
                    transfer_id: self.transfer_id,
                    len: self.len,
                    payload: self.buf,
                })
            } else {
                None // corrupt multi-frame — dropped (the CRC sink)
            };
            self.drop_active();
            return out;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::id::{encode_message_id, MessageId};
    use crate::tail::{encode_tail, single_frame_tail, Tail};

    const SIG: u64 = 0x0102_0304_0506_0708;

    fn frame(dtid: u16, node: u8, tailb: u8, body: &[u8]) -> CanFrame {
        let mut data = [0u8; 8];
        let n = body.len().min(7);
        data[..n].copy_from_slice(&body[..n]);
        data[n] = tailb;
        CanFrame {
            id: encode_message_id(&MessageId { priority: 16, data_type_id: dtid, source_node_id: node }),
            dlc: (n + 1) as u8,
            data,
        }
    }

    #[test]
    fn single_frame_transfer_emits_payload() {
        let mut r = Reassembler::new(SIG);
        let t = r.push(&frame(341, 42, single_frame_tail(3), &[0xAA, 0xBB, 0xCC])).unwrap();
        assert_eq!(t.data_type_id, 341);
        assert_eq!(t.source_node_id, 42);
        assert_eq!(t.transfer_id, 3);
        assert_eq!(&t.payload[..t.len], &[0xAA, 0xBB, 0xCC]);
    }

    /// Build a valid 2-frame transfer for `payload` and feed it; assert it
    /// reassembles byte-identical with a matching CRC.
    #[test]
    fn multi_frame_transfer_reassembles_with_crc() {
        let payload: [u8; 9] = [1, 2, 3, 4, 5, 6, 7, 8, 9];
        let crc = crc16_add(crc16_signature(SIG), &payload);
        let crc_b = crc.to_le_bytes();
        let mut r = Reassembler::new(SIG);
        // frame 1: SOT, toggle 0 — [crc_lo, crc_hi, p0..p4]
        let f1_body = [crc_b[0], crc_b[1], payload[0], payload[1], payload[2], payload[3], payload[4]];
        let t1 = encode_tail(&Tail { start_of_transfer: true, end_of_transfer: false, toggle: false, transfer_id: 5 });
        assert!(r.push(&frame(1063, 7, t1, &f1_body)).is_none());
        // frame 2: EOT, toggle 1 — [p5..p8]
        let t2 = encode_tail(&Tail { start_of_transfer: false, end_of_transfer: true, toggle: true, transfer_id: 5 });
        let out = r.push(&frame(1063, 7, t2, &payload[5..])).expect("transfer completes");
        assert_eq!(&out.payload[..out.len], &payload);
        assert_eq!(out.data_type_id, 1063);
    }

    #[test]
    fn corrupt_crc_is_dropped() {
        let payload: [u8; 9] = [1, 2, 3, 4, 5, 6, 7, 8, 9];
        let bad = 0x0000u16.to_le_bytes(); // wrong CRC
        let mut r = Reassembler::new(SIG);
        let f1 = [bad[0], bad[1], payload[0], payload[1], payload[2], payload[3], payload[4]];
        let t1 = encode_tail(&Tail { start_of_transfer: true, end_of_transfer: false, toggle: false, transfer_id: 5 });
        r.push(&frame(1063, 7, t1, &f1));
        let t2 = encode_tail(&Tail { start_of_transfer: false, end_of_transfer: true, toggle: true, transfer_id: 5 });
        assert!(r.push(&frame(1063, 7, t2, &payload[5..])).is_none()); // CRC sink drops it
    }

    #[test]
    fn wrong_toggle_drops_the_transfer() {
        let mut r = Reassembler::new(SIG);
        let t1 = encode_tail(&Tail { start_of_transfer: true, end_of_transfer: false, toggle: false, transfer_id: 5 });
        r.push(&frame(1063, 7, t1, &[0, 0, 1, 2, 3]));
        // continuation with the WRONG toggle (expected 1, send 0)
        let bad = encode_tail(&Tail { start_of_transfer: false, end_of_transfer: true, toggle: false, transfer_id: 5 });
        assert!(r.push(&frame(1063, 7, bad, &[4, 5])).is_none());
        assert!(!r.active); // dropped
    }

    #[test]
    fn service_frame_is_ignored() {
        let mut r = Reassembler::new(SIG);
        let mut f = frame(341, 1, single_frame_tail(0), &[1, 2]);
        f.id |= 1 << 7; // service-not-message bit
        assert!(r.push(&f).is_none());
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// Fuzz the reassembler with arbitrary frames from arbitrary state: push
        /// never panics and the buffer invariant (len <= MAX_PAYLOAD) holds, for
        /// ANY id/dlc/data — a garbage/babbling bus can never overflow or panic.
        /// (The proptest companion to Kani DC-K03/K04.)
        #[test]
        fn reassembler_never_panics_and_stays_bounded(
            sig: u64,
            frames in proptest::collection::vec(
                (any::<u32>(), 0u8..=8, any::<[u8; 8]>()), 0..40),
        ) {
            let mut r = Reassembler::new(sig);
            for (id, dlc, data) in frames {
                let out = r.push(&CanFrame { id, dlc, data });
                prop_assert!(r.len <= MAX_PAYLOAD);
                if let Some(t) = out {
                    prop_assert!(t.len <= MAX_PAYLOAD);
                }
            }
        }
    }
}
