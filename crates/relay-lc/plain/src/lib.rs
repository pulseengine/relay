#![no_std]

// Re-export primitives so engine.rs can use `crate::compare::...` and
// `crate::persistence::...` — the same paths src/engine.rs uses in the
// Verus tree (where they come from #[path] imports).
pub use relay_primitives::compare;
pub use relay_primitives::persistence;

pub mod engine;

#[cfg(feature = "c-api")]
pub mod c_api;
