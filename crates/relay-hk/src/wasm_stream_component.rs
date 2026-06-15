// Relay Housekeeping — P3 Stream Transformer WASM component.
//
// Takes stream<source-batch>, emits stream<hk-packet>.
// The verified engine (collect, Verus HK-P02/HK-P04) assembles each packet.
//
// This is the stream-native P3 interface — the future of Relay.

// Run the REAL Verus+Kani-verified engine (#168) — not an inline copy.
use relay_hk::engine;

// P3 Stream binding
use relay_hk_stream_bindings::exports::pulseengine::relay_housekeeping::housekeeping_stream::{
    Guest, CopyEntry as WitCopyEntry, HkPacket as WitHkPacket, SourceBatch as WitBatch,
};

struct Component;

static mut TABLE: Option<engine::CopyTable> = None;
static mut PACKET: Option<engine::HkPacket> = None;

fn get_table() -> &'static mut engine::CopyTable {
    unsafe {
        if TABLE.is_none() { TABLE = Some(engine::CopyTable::new()); }
        TABLE.as_mut().unwrap()
    }
}

fn get_packet() -> &'static mut engine::HkPacket {
    unsafe {
        if PACKET.is_none() { PACKET = Some(engine::HkPacket::new()); }
        PACKET.as_mut().unwrap()
    }
}

impl Guest for Component {
    async fn init() -> Result<(), String> {
        unsafe {
            TABLE = Some(engine::CopyTable::new());
            PACKET = Some(engine::HkPacket::new());
        }
        Ok(())
    }

    async fn add_copy_entry(entry: WitCopyEntry) -> bool {
        get_table().add_entry(engine::CopyEntry {
            source_id: entry.source_id,
            source_offset: entry.source_offset,
            length: entry.length,
            output_offset: entry.output_offset,
        })
    }

    /// STREAM TRANSFORMER: reads source batches, collects each through the
    /// copy table, writes assembled HK packets to the output stream.
    async fn monitor(
        mut batches: wit_bindgen::rt::async_support::StreamReader<WitBatch>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitHkPacket> {
        let (mut writer, reader) = relay_hk_stream_bindings::wit_stream::new::<WitHkPacket>();

        while let Some(batch) = batches.next().await {
            // Convert WIT sources to engine sources (mirrors the sync collect).
            let mut engine_sources = Vec::with_capacity(batch.sources.len());
            for src in &batch.sources {
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
                let out = WitHkPacket {
                    data: packet.data[..packet.length as usize].to_vec(),
                    length: packet.length,
                    sequence: packet.sequence,
                };
                let _ = writer.write(vec![out]).await;
            }
        }

        reader
    }
}

relay_hk_stream_bindings::export!(Component with_types_in relay_hk_stream_bindings);
