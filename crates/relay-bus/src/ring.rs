//! [`SpscRing`] — a `no_alloc`, fixed-capacity single-producer/single-consumer
//! ring buffer. The embeddable reference backend behind [`MessageTransport`],
//! mirroring a Gale `ring_buf`'s shape (relay#177 / gale#63).
//!
//! Safe Rust only (`#![forbid(unsafe_code)]` at the crate root): slots are
//! `Option<T>`, so no `MaybeUninit` and no `T: Copy` bound. Total by
//! construction — a full `push` is refused, an empty `pop` is `None`, and the
//! degenerate `N == 0` ring short-circuits before any modulo (so even a
//! zero-capacity ring never panics).

use crate::MessageTransport;

/// A bounded SPSC ring of capacity `N`, no allocation.
pub struct SpscRing<T, const N: usize> {
    buf: [Option<T>; N],
    head: usize, // index of the oldest item (next pop)
    tail: usize, // index of the next free slot (next push)
    len: usize,
}

impl<T, const N: usize> SpscRing<T, N> {
    /// An empty ring.
    pub fn new() -> Self {
        SpscRing {
            buf: core::array::from_fn(|_| None),
            head: 0,
            tail: 0,
            len: 0,
        }
    }
}

impl<T, const N: usize> Default for SpscRing<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> MessageTransport for SpscRing<T, N> {
    type Item = T;

    fn push(&mut self, item: T) -> bool {
        if self.len >= N {
            return false; // full (and the only path when N == 0)
        }
        self.buf[self.tail] = Some(item);
        self.tail = (self.tail + 1) % N;
        self.len += 1;
        true
    }

    fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None; // empty (and the only path when N == 0)
        }
        let item = self.buf[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        item
    }

    fn len(&self) -> usize {
        self.len
    }

    fn capacity(&self) -> usize {
        N
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_order() {
        let mut r: SpscRing<u8, 4> = SpscRing::new();
        assert!(r.push(1));
        assert!(r.push(2));
        assert!(r.push(3));
        assert_eq!(r.pop(), Some(1));
        assert_eq!(r.pop(), Some(2));
        assert_eq!(r.pop(), Some(3));
        assert_eq!(r.pop(), None);
    }

    #[test]
    fn backpressure_when_full() {
        let mut r: SpscRing<u8, 2> = SpscRing::new();
        assert!(r.push(10));
        assert!(r.push(20));
        assert!(r.is_full());
        assert!(!r.push(30)); // refused, not overwritten
        assert_eq!(r.len(), 2);
        assert_eq!(r.pop(), Some(10)); // FIFO preserved
    }

    #[test]
    fn wraparound_reuses_slots() {
        let mut r: SpscRing<u8, 3> = SpscRing::new();
        assert!(r.push(1));
        assert!(r.push(2));
        assert_eq!(r.pop(), Some(1));
        assert!(r.push(3));
        assert!(r.push(4)); // tail wraps past the freed slot
        assert!(!r.push(5)); // full again (2,3,4)
        assert_eq!(r.pop(), Some(2));
        assert_eq!(r.pop(), Some(3));
        assert_eq!(r.pop(), Some(4));
    }

    #[test]
    fn empty_pop_is_none() {
        let mut r: SpscRing<u8, 4> = SpscRing::new();
        assert!(r.is_empty());
        assert_eq!(r.pop(), None);
        assert_eq!(r.len(), 0);
        assert_eq!(r.capacity(), 4);
    }

    #[test]
    fn zero_capacity_is_total() {
        let mut r: SpscRing<u8, 0> = SpscRing::new();
        assert!(r.is_full());
        assert!(!r.push(1)); // never panics on the modulo
        assert_eq!(r.pop(), None);
    }
}
