//! DroneCAN v0 CAN ID — the 29-bit extended-identifier message-frame layout.
//!
//! Message frames pack the 29-bit ID as:
//! ```text
//!   bit 28..24  priority            (5 bits)
//!   bit 23..8   data type id        (16 bits)
//!   bit 7       service-not-message (1 bit, 0 for a message frame)
//!   bit 6..0    source node id      (7 bits, 0 = anonymous)
//! ```

/// A decoded DroneCAN message-frame identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MessageId {
    /// Transfer priority, 0 (highest) .. 31 (lowest); 5 bits.
    pub priority: u8,
    /// The message data-type id (e.g. 341 NodeStatus, 1030 esc.RawCommand).
    pub data_type_id: u16,
    /// Source node id, 0..=127 (0 = anonymous, e.g. during node-id allocation).
    pub source_node_id: u8,
}

/// Bit 7 of the 29-bit ID: set on a service frame, clear on a message frame.
const SERVICE_NOT_MESSAGE: u32 = 1 << 7;

/// Decode a 29-bit CAN extended ID as a DroneCAN message-frame id.
///
/// Returns `None` if the service-not-message bit is set (a service request/
/// response frame, not a broadcast message — out of this transfer layer's
/// scope). Total: never panics for any `u32`. Only the low 29 bits are read.
pub fn decode_message_id(raw: u32) -> Option<MessageId> {
    if raw & SERVICE_NOT_MESSAGE != 0 {
        return None;
    }
    Some(MessageId {
        source_node_id: (raw & 0x7F) as u8,
        data_type_id: ((raw >> 8) & 0xFFFF) as u16,
        priority: ((raw >> 24) & 0x1F) as u8,
    })
}

/// Encode a DroneCAN message-frame id into a 29-bit CAN extended ID (the
/// service-not-message bit is left clear). Each field is masked to its width.
pub fn encode_message_id(m: &MessageId) -> u32 {
    ((m.priority as u32 & 0x1F) << 24)
        | ((m.data_type_id as u32) << 8)
        | (m.source_node_id as u32 & 0x7F)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nodestatus_id_round_trips() {
        // priority 16, DTID 341 (NodeStatus), node 42
        let m = MessageId { priority: 16, data_type_id: 341, source_node_id: 42 };
        let raw = encode_message_id(&m);
        assert_eq!(decode_message_id(raw), Some(m));
    }

    #[test]
    fn service_frame_is_rejected() {
        let raw = encode_message_id(&MessageId { priority: 0, data_type_id: 1, source_node_id: 1 })
            | SERVICE_NOT_MESSAGE;
        assert_eq!(decode_message_id(raw), None);
    }

    #[test]
    fn fields_are_masked_to_width() {
        // node id is 7 bits: 0x7F is the max
        let m = MessageId { priority: 31, data_type_id: 0xFFFF, source_node_id: 0x7F };
        assert_eq!(decode_message_id(encode_message_id(&m)), Some(m));
    }
}
