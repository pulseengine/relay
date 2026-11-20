//! The falcon SITL plant as a library (SWREQ-FALCON-PLANT-P01, relay#705).
//!
//! `physics` and `pace` moved here unchanged from `examples/falcon-sitl-gz`, so
//! the native bench, `tests/cascade-sitl-wasm` and a harness outside relay all
//! step the same plant. `lockstep` is the tick interface for that outside
//! harness: `start` and `step` exchange whole frames of the published
//! falcon-hitl binding, so the harness relays bytes and converts nothing.
//!
//! Only `MockPhysics` can run lockstep. The gz bridge ignores `dt` because
//! gz-sim runs on its own clock.

pub mod lockstep;
pub mod pace;
pub mod physics;
