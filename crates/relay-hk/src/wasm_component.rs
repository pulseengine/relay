// Relay Housekeeping — P3 WASM component (self-contained).
//
// This file contains both:
//   1. The verified core engine (from plain/src/engine.rs)
//   2. The P3 async Guest trait implementation
//
// Built by: bazel build //:relay-hk (rules_wasm_component, wasi_version="p3")
// Verified by: bazel test //:relay_hk_verus_test (src/engine.rs with verus!)

// ═══════════════════════════════════════════════════════════════
// Verified core engine (plain Rust, identical to plain/src/engine.rs)
// ═══════════════════════════════════════════════════════════════

// Run the REAL Verus+Kani-verified engine (#168), not an inline copy.
use relay_hk::engine;

// ═══════════════════════════════════════════════════════════════
// P3 WASM component binding — delegates to verified engine
// ═══════════════════════════════════════════════════════════════

use relay_hk_bindings::exports::pulseengine::relay_housekeeping::housekeeping::{
    Guest, CopyEntry as WitCopyEntry, SourceData as WitSourceData, HkPacket as WitHkPacket,
};

struct Component;

static mut TABLE: Option<engine::CopyTable> = None;
static mut PACKET: Option<engine::HkPacket> = None;

fn get_table() -> &'static mut engine::CopyTable {
    unsafe {
        if TABLE.is_none() {
            TABLE = Some(engine::CopyTable::new());
        }
        TABLE.as_mut().unwrap()
    }
}

fn get_packet() -> &'static mut engine::HkPacket {
    unsafe {
        if PACKET.is_none() {
            PACKET = Some(engine::HkPacket::new());
        }
        PACKET.as_mut().unwrap()
    }
}

impl Guest for Component {
    #[cfg(target_arch = "wasm32")]
    async fn init() -> Result<(), String> {
        unsafe {
            TABLE = Some(engine::CopyTable::new());
            PACKET = Some(engine::HkPacket::new());
        }
        Ok(())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn init() -> Result<(), String> {
        unsafe {
            TABLE = Some(engine::CopyTable::new());
            PACKET = Some(engine::HkPacket::new());
        }
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    async fn add_copy_entry(entry: WitCopyEntry) -> bool {
        Self::do_add_entry(entry)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn add_copy_entry(entry: WitCopyEntry) -> bool {
        Self::do_add_entry(entry)
    }

    #[cfg(target_arch = "wasm32")]
    async fn collect(sources: Vec<WitSourceData>) -> Option<WitHkPacket> {
        Self::do_collect(sources)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn collect(sources: Vec<WitSourceData>) -> Option<WitHkPacket> {
        Self::do_collect(sources)
    }
}

impl Component {
    fn do_add_entry(entry: WitCopyEntry) -> bool {
        get_table().add_entry(engine::CopyEntry {
            source_id: entry.source_id,
            source_offset: entry.source_offset,
            length: entry.length,
            output_offset: entry.output_offset,
        })
    }

    fn do_collect(sources: Vec<WitSourceData>) -> Option<WitHkPacket> {
        // Convert WIT sources to engine sources
        let mut engine_sources = Vec::with_capacity(sources.len());
        for src in &sources {
            let mut sd = engine::SourceData::empty();
            sd.source_id = src.source_id;
            let copy_len = core::cmp::min(src.data.len(), engine::SOURCE_DATA_SIZE);
            let mut b = 0;
            while b < copy_len {
                sd.data[b] = src.data[b];
                b += 1;
            }
            engine_sources.push(sd);
        }

        let table = get_table();
        let packet = get_packet();
        if table.collect(&engine_sources, packet) {
            Some(WitHkPacket {
                data: packet.data[..packet.length as usize].to_vec(),
                length: packet.length,
                sequence: packet.sequence,
            })
        } else {
            None
        }
    }
}

relay_hk_bindings::export!(Component with_types_in relay_hk_bindings);
