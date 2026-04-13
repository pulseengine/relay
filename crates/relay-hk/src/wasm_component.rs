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

mod engine {
    pub const MAX_COPY_ENTRIES: usize = 128;
    pub const MAX_OUTPUT_SIZE: usize = 256;
    pub const SOURCE_DATA_SIZE: usize = 64;

    #[derive(Clone, Copy)]
    pub struct CopyEntry {
        pub source_id: u32,
        pub source_offset: u32,
        pub length: u32,
        pub output_offset: u32,
    }

    pub struct CopyTable {
        entries: [CopyEntry; MAX_COPY_ENTRIES],
        entry_count: u32,
    }

    #[derive(Clone, Copy)]
    pub struct SourceData {
        pub source_id: u32,
        pub data: [u8; SOURCE_DATA_SIZE],
    }

    pub struct HkPacket {
        pub data: [u8; MAX_OUTPUT_SIZE],
        pub length: u32,
        pub sequence: u32,
    }

    impl CopyEntry {
        pub const fn empty() -> Self {
            CopyEntry { source_id: 0, source_offset: 0, length: 0, output_offset: 0 }
        }
    }

    impl SourceData {
        pub const fn empty() -> Self {
            SourceData { source_id: 0, data: [0u8; SOURCE_DATA_SIZE] }
        }
    }

    impl HkPacket {
        pub fn new() -> Self {
            HkPacket { data: [0u8; MAX_OUTPUT_SIZE], length: 0, sequence: 0 }
        }
    }

    impl CopyTable {
        pub fn new() -> Self {
            CopyTable {
                entries: [CopyEntry::empty(); MAX_COPY_ENTRIES],
                entry_count: 0,
            }
        }

        pub fn add_entry(&mut self, entry: CopyEntry) -> bool {
            if self.entry_count as usize >= MAX_COPY_ENTRIES { return false; }
            self.entries[self.entry_count as usize] = entry;
            self.entry_count = self.entry_count + 1;
            true
        }

        pub fn entry_count(&self) -> u32 { self.entry_count }

        pub fn collect(&self, sources: &[SourceData], packet: &mut HkPacket) -> bool {
            let count = self.entry_count;
            let mut i: u32 = 0;
            while i < count {
                let entry = self.entries[i as usize];

                // Bounds check: output region must fit in packet
                let out_end = entry.output_offset as usize + entry.length as usize;
                if out_end > MAX_OUTPUT_SIZE { return false; }

                // Bounds check: source region must fit in source data
                let src_end = entry.source_offset as usize + entry.length as usize;
                if src_end > SOURCE_DATA_SIZE { return false; }

                // Find matching source
                let mut found = false;
                let mut s: usize = 0;
                while s < sources.len() {
                    if sources[s].source_id == entry.source_id {
                        // Copy bytes from source to packet
                        let mut b: usize = 0;
                        while b < entry.length as usize {
                            packet.data[entry.output_offset as usize + b] =
                                sources[s].data[entry.source_offset as usize + b];
                            b = b + 1;
                        }
                        found = true;
                        break;
                    }
                    s = s + 1;
                }

                if !found { return false; }

                // Track the high-water mark for packet length
                if out_end as u32 > packet.length {
                    packet.length = out_end as u32;
                }

                i = i + 1;
            }
            packet.sequence = packet.sequence + 1;
            true
        }
    }
}

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
