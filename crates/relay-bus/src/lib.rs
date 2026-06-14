//! Relay Bus — the `no_std` transport seam under the Software Bus.
//!
//! The Software Bus WIT (`wit/interfaces/relay-software-bus/bus.wit`) declares
//! a transport-neutral `stream<T>` contract: *"On embedded (Synth → ARM →
//! Gale): `stream<T>` → Gale `ring_buf`."* This crate is the Rust realisation
//! of that boundary — a single [`MessageTransport`] trait that the host
//! Software Bus queue and an embedded shared-memory ring both implement, so a
//! carrier can be chosen by the host/linker WITHOUT changing the `stream<T>`
//! contract above it.
//!
//! Ownership split (relay#177): **relay owns this seam; Gale owns the ring**
//! (gale#63). [`SpscRing`] here is the `no_alloc` reference backend that mirrors
//! a Gale `ring_buf`'s shape — it proves the trait admits an embeddable,
//! bounded, backpressure-honest carrier (the thing jess's inter-core M7↔M4
//! link needs) without pulling Gale in. The production embedded backend is a
//! verified Gale `ring_buf` behind the same trait.
//!
//! Verified: Kani BUS-K01 (a bounded sequence of arbitrary push/pop ops is
//! total — no panic — and `len` stays within `[0, N]`); BUS-K02 (a full ring
//! refuses the push, never overwrites, FIFO preserved).
#![no_std]
#![forbid(unsafe_code)]

mod ring;
pub use ring::SpscRing;

#[cfg(kani)]
mod kani_proofs;

/// A bounded single-producer/single-consumer message carrier — the pluggable
/// transport behind the Software Bus `stream<T>` seam.
///
/// One trait, many backends: the host bus's `VecDeque` queue (`relay-sb`) and
/// an embedded shared-memory ring (a Gale `ring_buf`, or [`SpscRing`]) are
/// interchangeable carriers. `push` is non-blocking and **never panics**: a
/// full carrier refuses the item (`false`) rather than overwrite or grow —
/// backpressure is explicit, matching `sb-error::backpressure` in the WIT.
pub trait MessageTransport {
    /// The message type carried.
    type Item;

    /// Enqueue `item`. Returns `false` if the carrier is full (backpressure);
    /// never blocks, never overwrites, never panics.
    fn push(&mut self, item: Self::Item) -> bool;

    /// Dequeue the oldest item in FIFO order, or `None` if empty.
    fn pop(&mut self) -> Option<Self::Item>;

    /// Number of queued items.
    fn len(&self) -> usize;

    /// Maximum capacity.
    fn capacity(&self) -> usize;

    /// Whether the carrier cannot accept another item.
    fn is_full(&self) -> bool {
        self.len() >= self.capacity()
    }

    /// Whether the carrier holds no items.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
