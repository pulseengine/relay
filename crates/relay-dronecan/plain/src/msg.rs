//! DroneCAN v0 message decoders/encoders for the ESC control path (DC-P02).
//!
//! Pure fixed-bit-offset DSDL codecs over reassembled transfer payloads — the
//! verified core falcon's flight loop uses to command ESCs and observe node
//! presence. The transfer framing (id/tail/CRC/reassembly, single-frame TX) is
//! [`crate::transfer`]; these are the per-message bodies on top.
//!
//! falcon's ESC command path:
//!   * **out** — [`encode_raw_command`] packs the verified mixer's per-motor
//!     throttles as `uavcan.equipment.esc.RawCommand` (int14), emitted via
//!     [`crate::transfer::encode_single_frame`].
//!   * **in** — [`decode_node_status`] reads `uavcan.protocol.NodeStatus` (the
//!     one mandatory message): node uptime/health/mode — the bus-presence +
//!     node-health source for FDI.

use crate::dsdl;

/// Data-type id of uavcan.protocol.NodeStatus.
pub const DTID_NODE_STATUS: u16 = 341;
/// Data-type id of uavcan.equipment.esc.RawCommand.
pub const DTID_ESC_RAW_COMMAND: u16 = 1030;
/// Maximum ESC channels in an esc.RawCommand (the DSDL array bound).
pub const MAX_ESC: usize = 20;
/// int14 command range (signed 14-bit): -8192..=8191.
pub const ESC_CMD_MIN: i16 = -8192;
pub const ESC_CMD_MAX: i16 = 8191;

/// Decoded uavcan.protocol.NodeStatus (7-byte single-frame payload).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeStatus {
    /// Seconds since node power-on.
    pub uptime_sec: u32,
    /// Node health: 0 OK, 1 WARNING, 2 ERROR, 3 CRITICAL (2 bits).
    pub health: u8,
    /// Node mode: 0 OPERATIONAL, 1 INITIALIZATION, ... (3 bits).
    pub mode: u8,
    /// Vendor-defined sub-mode (3 bits).
    pub sub_mode: u8,
    /// Vendor-specific status code.
    pub vendor_status: u16,
}

/// Decode a NodeStatus payload. Returns `None` if shorter than 7 bytes. Total:
/// never panics; every bit-field is masked to its DSDL width.
pub fn decode_node_status(payload: &[u8]) -> Option<NodeStatus> {
    if payload.len() < 7 {
        return None;
    }
    Some(NodeStatus {
        // byte-aligned fields are little-endian (== DSDL be_from_le_bits); the
        // sub-byte fields use the DSDL bit codec (MSB-first within the byte) —
        // the conformance fix vs the canonical pydronecan encoding.
        uptime_sec: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
        health: dsdl::read_uint(payload, 32, 2) as u8,
        mode: dsdl::read_uint(payload, 34, 3) as u8,
        sub_mode: dsdl::read_uint(payload, 37, 3) as u8,
        vendor_status: u16::from_le_bytes([payload[5], payload[6]]),
    })
}

/// Pack up to [`MAX_ESC`] int14 throttle commands (LSB-first) into `out`,
/// returning the number of bytes written (0 if `out` is too small or no
/// channels). Each value is clamped to the int14 range. Total: never panics;
/// writes at most ceil(MAX_ESC*14/8) = 35 bytes. The verified mixer's output
/// (scaled to int14) becomes the esc.RawCommand wire payload.
pub fn encode_raw_command(channels: &[i16], out: &mut [u8]) -> usize {
    let n = channels.len().min(MAX_ESC);
    let nbytes = (n * 14).div_ceil(8);
    if nbytes > out.len() {
        return 0;
    }
    for b in out[..nbytes].iter_mut() {
        *b = 0;
    }
    for (i, &c) in channels.iter().take(n).enumerate() {
        let v = (c.clamp(ESC_CMD_MIN, ESC_CMD_MAX) as u16 & 0x3FFF) as u64; // int14 two's complement
        dsdl::write_uint(out, i * 14, 14, v); // DSDL bit packing (conformance fix)
    }
    nbytes
}

/// Decoded esc.RawCommand: the per-ESC int14 throttle commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawCommand {
    /// Per-ESC commands, sign-extended from int14 (only `count` are valid).
    pub channels: [i16; MAX_ESC],
    /// Number of valid channels (`payload_bits / 14`, capped at MAX_ESC).
    pub count: usize,
}

/// Decode an esc.RawCommand payload into its int14 channels (sign-extended).
/// Total: never panics; bounded reads only.
pub fn decode_raw_command(payload: &[u8]) -> RawCommand {
    let n = (payload.len() * 8 / 14).min(MAX_ESC);
    let mut channels = [0i16; MAX_ESC];
    for (i, ch) in channels.iter_mut().take(n).enumerate() {
        // DSDL int14, sign-extended (conformance fix)
        *ch = dsdl::read_int(payload, i * 14, 14) as i16;
    }
    RawCommand { channels, count: n }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CONFORMANCE: the canonical pydronecan NodeStatus(uptime=0x01020304,
    /// health=2, mode=3, sub_mode=1, vendor=0xBEEF) is `0403020199efbe` —
    /// byte 4 = 0x99 (MSB-first sub-byte fields). Catches the LSB-first bug.
    #[test]
    fn node_status_decodes_fields() {
        let payload = [0x04, 0x03, 0x02, 0x01, 0x99, 0xEF, 0xBE];
        let ns = decode_node_status(&payload).unwrap();
        assert_eq!(ns.uptime_sec, 0x0102_0304);
        assert_eq!(ns.health, 2);
        assert_eq!(ns.mode, 3);
        assert_eq!(ns.sub_mode, 1);
        assert_eq!(ns.vendor_status, 0xBEEF);
    }

    /// CONFORMANCE (flight-critical): the canonical pydronecan RawCommand
    /// [8191, 0, -8192, 4096] is `ff7c0000080010`. Decoding it must recover the
    /// commands; encoding must reproduce the bytes. This is the bug that would
    /// have sent garbage throttles to a real ESC.
    #[test]
    fn raw_command_matches_pydronecan_reference() {
        let canonical = [0xff, 0x7c, 0x00, 0x00, 0x08, 0x00, 0x10];
        let decoded = decode_raw_command(&canonical);
        assert_eq!(&decoded.channels[..4], &[8191, 0, -8192, 4096]);
        let mut buf = [0u8; 7];
        let n = encode_raw_command(&[8191, 0, -8192, 4096], &mut buf);
        assert_eq!(n, 7);
        assert_eq!(buf, canonical);
    }

    #[test]
    fn node_status_rejects_short_payload() {
        assert_eq!(decode_node_status(&[0; 6]), None);
    }

    #[test]
    fn raw_command_round_trips_quad() {
        // a quad: 4 int14 throttles (incl a negative reverse value)
        let cmds = [8191i16, 0, -8192, 4096];
        let mut buf = [0u8; 35];
        let n = encode_raw_command(&cmds, &mut buf);
        assert_eq!(n, (4usize * 14).div_ceil(8)); // 7 bytes -> single frame
        let decoded = decode_raw_command(&buf[..n]);
        assert_eq!(decoded.count, 4);
        assert_eq!(&decoded.channels[..4], &cmds);
    }

    #[test]
    fn raw_command_clamps_out_of_range() {
        let cmds = [30000i16, -30000];
        let mut buf = [0u8; 35];
        let n = encode_raw_command(&cmds, &mut buf);
        let decoded = decode_raw_command(&buf[..n]);
        assert_eq!(decoded.channels[0], ESC_CMD_MAX);
        assert_eq!(decoded.channels[1], ESC_CMD_MIN);
    }

    #[test]
    fn raw_command_caps_at_max_esc() {
        let cmds = [1i16; 25]; // more than MAX_ESC
        let mut buf = [0u8; 64];
        let n = encode_raw_command(&cmds, &mut buf);
        assert_eq!(n, (MAX_ESC * 14usize).div_ceil(8));
        assert_eq!(decode_raw_command(&buf[..n]).count, MAX_ESC);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// encode_raw_command never panics and the int14 round-trip preserves
        /// every clamped channel, for ANY channel values + count.
        #[test]
        fn raw_command_encode_decode_round_trip(
            cmds in proptest::collection::vec(any::<i16>(), 0..MAX_ESC),
        ) {
            let mut buf = [0u8; 35];
            let n = encode_raw_command(&cmds, &mut buf);
            let decoded = decode_raw_command(&buf[..n]);
            prop_assert_eq!(decoded.count, cmds.len());
            for (i, &c) in cmds.iter().enumerate() {
                prop_assert_eq!(decoded.channels[i], c.clamp(ESC_CMD_MIN, ESC_CMD_MAX));
            }
        }

        /// decode_node_status never panics for any payload; on success every
        /// bit-field is within its DSDL width.
        #[test]
        fn node_status_decode_never_panics(bytes in proptest::collection::vec(any::<u8>(), 0..32)) {
            if let Some(ns) = decode_node_status(&bytes) {
                prop_assert!(ns.health <= 3);
                prop_assert!(ns.mode <= 7);
                prop_assert!(ns.sub_mode <= 7);
            }
        }
    }
}
