//! DroneCAN v0 tail byte — the last data byte of every CAN frame in a transfer.
//!
//! ```text
//!   bit 7    start of transfer (SOT)
//!   bit 6    end of transfer   (EOT)
//!   bit 5    toggle
//!   bit 4..0 transfer id        (5 bits, 0..=31)
//! ```
//! Single-frame transfer: SOT=1, EOT=1, toggle=0. Multi-frame: first frame
//! SOT=1/EOT=0/toggle=0, then toggle alternates each frame, last frame EOT=1.

/// A decoded DroneCAN tail byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tail {
    /// Start-of-transfer flag (first frame of a transfer).
    pub start_of_transfer: bool,
    /// End-of-transfer flag (last frame of a transfer).
    pub end_of_transfer: bool,
    /// Toggle bit — alternates each frame within a multi-frame transfer.
    pub toggle: bool,
    /// Transfer id, 0..=31 (5 bits) — distinguishes consecutive transfers.
    pub transfer_id: u8,
}

/// Decode a tail byte. Total: never panics for any `u8`.
pub fn decode_tail(byte: u8) -> Tail {
    Tail {
        start_of_transfer: byte & 0x80 != 0,
        end_of_transfer: byte & 0x40 != 0,
        toggle: byte & 0x20 != 0,
        transfer_id: byte & 0x1F,
    }
}

/// Encode a tail byte (the transfer id is masked to 5 bits).
pub fn encode_tail(t: &Tail) -> u8 {
    let mut b = t.transfer_id & 0x1F;
    if t.start_of_transfer {
        b |= 0x80;
    }
    if t.end_of_transfer {
        b |= 0x40;
    }
    if t.toggle {
        b |= 0x20;
    }
    b
}

/// Convenience: the tail byte of a single-frame transfer (SOT|EOT, toggle=0).
pub fn single_frame_tail(transfer_id: u8) -> u8 {
    encode_tail(&Tail {
        start_of_transfer: true,
        end_of_transfer: true,
        toggle: false,
        transfer_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tail_round_trips() {
        for raw in 0u8..=255 {
            let t = decode_tail(raw);
            // re-encoding masks the same bits we read, so it is idempotent
            assert_eq!(encode_tail(&t), raw & 0xE0 | (raw & 0x1F));
        }
    }

    #[test]
    fn single_frame_markers() {
        let t = decode_tail(single_frame_tail(7));
        assert!(t.start_of_transfer && t.end_of_transfer && !t.toggle);
        assert_eq!(t.transfer_id, 7);
    }

    #[test]
    fn transfer_id_is_five_bits() {
        // transfer id 0x1F is the max; bit 5 would be the toggle
        assert_eq!(decode_tail(0x1F).transfer_id, 0x1F);
        assert_eq!(decode_tail(0x3F).transfer_id, 0x1F); // toggle set, tid unchanged
        assert!(decode_tail(0x3F).toggle);
    }
}
