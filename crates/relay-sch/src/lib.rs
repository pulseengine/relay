#![no_std]

// Relay Scheduler — stream transformer: ticks → scheduled actions.
//
// Pure stream processor: reads tick events, consults schedule table,
// emits actions to fire. No persistent state between ticks needed
// beyond the table configuration.
//
// On embedded: compiled by Synth to ARM, backed by Gale ring_buf
// for stream I/O. Runs in P3 callback mode — single stack, no alloc.

// TODO: wit_bindgen::generate! once WIT interfaces are stable
// TODO: Implement schedule table as static array (no alloc)
// TODO: Verus verification of scheduling invariants
