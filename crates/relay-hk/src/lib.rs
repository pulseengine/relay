#![no_std]

// Relay Housekeeping — stream combiner: heartbeats + sensors → hk packets.
//
// Reads from multiple input streams, extracts configured fields via
// copy table, assembles combined housekeeping packets, writes to
// output stream.
//
// Stateless per-invocation. Copy table is static configuration.
// No alloc — all buffers are stack-local or table-provided.

// TODO: wit_bindgen::generate! once WIT interfaces are stable
// TODO: Copy table as static array with field offset descriptors
// TODO: Verus verification of copy table bounds
