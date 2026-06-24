//! The DroneCAN node runtime (DC-P04) — the async pump tying the verified
//! transfer layer to the relay-hal CanBus seam, plus the NodeStatus heartbeat.
//!
//! `DroneCanNode<C: CanBus>` mirrors `UbxReader`: `poll` receives one CAN frame
//! and drives the verified [`Reassembler`], returning a completed [`Transfer`];
//! `send_heartbeat` encodes + emits this node's own NodeStatus at the
//! DroneCAN-spec 1 Hz (the caller drives the cadence). Static node-id; dynamic
//! node-id allocation (uavcan.protocol.dynamic_node_id.Allocation) is a
//! follow-on. The decode/dispatch step is the verified reassembler — the async
//! path adds only the await on the bus.

use crate::msg::{encode_node_status, NodeStatus, DTID_NODE_STATUS};
use crate::transfer::{encode_single_frame, Reassembler, Transfer};
use relay_hal::CanBus;

/// DroneCAN transfer priority used for the NodeStatus heartbeat (mid priority).
const HEARTBEAT_PRIORITY: u8 = 16;

/// A static-node-id DroneCAN node over a [`CanBus`] seam (jess binds FlexCAN).
pub struct DroneCanNode<C> {
    bus: C,
    reassembler: Reassembler,
    node_id: u8,
    heartbeat_tid: u8,
}

impl<C: CanBus> DroneCanNode<C> {
    /// A node with the given static node id, reassembling one message type (its
    /// 64-bit data-type signature, for the multi-frame transfer CRC).
    pub fn new(bus: C, node_id: u8, signature: u64) -> Self {
        Self {
            bus,
            reassembler: Reassembler::new(signature),
            node_id,
            heartbeat_tid: 0,
        }
    }

    /// Receive one CAN frame and feed the verified reassembler. Returns a
    /// completed [`Transfer`] when an end-of-transfer frame closes a valid one;
    /// `None` otherwise. The async path adds only the bus await — the dispatch
    /// is the v1.92 reassembler step (totality + buffer-bound proven).
    pub async fn poll(&mut self) -> Result<Option<Transfer>, C::Error> {
        let frame = self.bus.recv().await?;
        Ok(self.reassembler.push(&frame))
    }

    /// Broadcast this node's NodeStatus heartbeat: encode the 7-byte payload and
    /// emit a single-frame transfer, advancing the 5-bit transfer id. The caller
    /// drives the 1 Hz cadence (spar models the Heartbeat thread's timing).
    pub async fn send_heartbeat(&mut self, status: &NodeStatus) -> Result<(), C::Error> {
        let payload = encode_node_status(status);
        // a 7-byte payload always fits one frame, so encode_single_frame -> Some
        if let Some(frame) = encode_single_frame(
            DTID_NODE_STATUS,
            self.node_id,
            HEARTBEAT_PRIORITY,
            self.heartbeat_tid,
            &payload,
        ) {
            self.heartbeat_tid = (self.heartbeat_tid + 1) & 0x1F;
            self.bus.send(&frame).await?;
        }
        Ok(())
    }

    /// Borrow the underlying bus.
    pub fn bus(&mut self) -> &mut C {
        &mut self.bus
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::decode_node_status;
    use crate::transfer::Reassembler;
    use relay_hal::CanFrame;

    /// A mock CanBus: yields a preset RX frame sequence and records TX frames.
    /// no_std/no_alloc — fixed buffers.
    struct MockCanBus {
        rx: [CanFrame; 4],
        rx_len: usize,
        rx_pos: usize,
        tx: [CanFrame; 4],
        tx_len: usize,
    }

    impl MockCanBus {
        fn new() -> Self {
            let empty = CanFrame { id: 0, dlc: 0, data: [0; 8] };
            Self { rx: [empty; 4], rx_len: 0, rx_pos: 0, tx: [empty; 4], tx_len: 0 }
        }
    }

    impl CanBus for MockCanBus {
        type Error = ();
        async fn send(&mut self, frame: &CanFrame) -> Result<(), ()> {
            if self.tx_len < 4 {
                self.tx[self.tx_len] = *frame;
                self.tx_len += 1;
            }
            Ok(())
        }
        async fn recv(&mut self) -> Result<CanFrame, ()> {
            if self.rx_pos < self.rx_len {
                let f = self.rx[self.rx_pos];
                self.rx_pos += 1;
                Ok(f)
            } else {
                Err(())
            }
        }
    }

    /// Drive a future that completes synchronously (the mock delivers in one
    /// poll). Mirrors falcon-gnss-ubx's test executor.
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

    /// The async pump dispatches a frame to the same Transfer the sync
    /// reassembler produces — the runtime preserves the verified reassembler.
    #[test]
    fn poll_reassembles_like_sync_push() {
        let payload = [1u8, 2, 3];
        let frame = encode_single_frame(341, 9, 16, 0, &payload).unwrap();

        let mut r = Reassembler::new(0);
        let sync = r.push(&frame).expect("single-frame completes");

        let mut bus = MockCanBus::new();
        bus.rx[0] = frame;
        bus.rx_len = 1;
        let mut node = DroneCanNode::new(bus, 9, 0);
        let asyncd = block_on(node.poll()).unwrap().expect("poll yields the transfer");

        assert_eq!(asyncd.data_type_id, sync.data_type_id);
        assert_eq!(asyncd.len, sync.len);
        assert_eq!(&asyncd.payload[..asyncd.len], &payload);
    }

    /// A heartbeat emitted by the node reassembles + decodes back to the original
    /// NodeStatus — the TX path round-trips through the RX path.
    #[test]
    fn heartbeat_round_trips_through_reassembler() {
        let status = NodeStatus {
            uptime_sec: 42,
            health: 1,
            mode: 0,
            sub_mode: 0,
            vendor_status: 7,
        };
        let mut node = DroneCanNode::new(MockCanBus::new(), 5, 0);
        block_on(node.send_heartbeat(&status)).unwrap();

        let sent = node.bus().tx[0];
        assert!(node.bus().tx_len == 1);
        let mut r = Reassembler::new(0);
        let t = r.push(&sent).expect("heartbeat reassembles");
        assert_eq!(t.data_type_id, DTID_NODE_STATUS);
        assert_eq!(t.source_node_id, 5);
        assert_eq!(decode_node_status(&t.payload[..t.len]), Some(status));
    }

    /// A bus recv error propagates (the async path is fallible — the health
    /// source).
    #[test]
    fn poll_propagates_bus_error() {
        let mut node = DroneCanNode::new(MockCanBus::new(), 1, 0); // empty rx -> Err
        assert!(block_on(node.poll()).is_err());
    }
}
