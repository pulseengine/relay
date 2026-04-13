#![no_std]

//! Relay Scheduler — stream transformer: stream<tick> → stream<scheduled-action>.
//!
//! Architecture (following Verification Guide):
//!
//! ```text
//! core.rs          ← Verus-annotated verified logic (no async, no alloc)
//!   │
//!   ├── verus-strip ──→ plain/src/core.rs → cargo test + Kani + coq_of_rust
//!   │
//!   └── lib.rs     ← P3 async wrapper (NOT verified, thin binding layer)
//! ```
//!
//! The verified core (`core.rs`) contains:
//!   - ScheduleTable: fixed-size schedule definition table
//!   - process_tick(): finds matching slots, emits bounded actions
//!
//! This file (`lib.rs`) wraps the verified core with:
//!   - wit-bindgen P3 async stream bindings
//!   - Stream read/write glue (outside verification boundary)

pub mod core;

// TODO: Enable when WIT interfaces are finalized and wit-bindgen P3
// bindings are generated via rules_wasm_component:
//
// wit_bindgen::generate!({
//     world: "scheduler",
//     path: "../../wit",
// });
//
// struct Scheduler {
//     table: core::ScheduleTable,
// }
//
// impl Guest for Scheduler {
//     // P3 stream transformer:
//     // Reads tick events from input stream,
//     // calls core::ScheduleTable::process_tick() (verified),
//     // writes scheduled actions to output stream.
//     //
//     // #[cfg(target_arch = "wasm32")]
//     // async fn run(
//     //     ticks: StreamReader<TickEvent>,
//     //     config: StreamReader<TableUpdate>,
//     // ) -> StreamWriter<ScheduledAction> {
//     //     loop {
//     //         let tick = ticks.read().await;
//     //         let result = self.table.process_tick(tick.frame, tick.major);
//     //         for i in 0..result.action_count {
//     //             output.write(result.actions[i]).await;
//     //         }
//     //     }
//     // }
// }
