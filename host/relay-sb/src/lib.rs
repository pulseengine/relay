//! Relay Software Bus — host-native message routing service.
//!
//! Routes messages between components via a push-based subscription model.
//! Maintains a subscription table and dispatches messages to subscribers.
//!
//! This is a host-side crate (std allowed).

pub mod core;
