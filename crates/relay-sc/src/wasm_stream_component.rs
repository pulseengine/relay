// Relay Stored Command — P3 Stream Transformer WASM component.
//
// Takes stream<time-tick>, emits stream<dispatched-command>.
// The verified engine (process_tick, Verus-proven) fires due ATS/RTS commands.
//
// This is the stream-native P3 interface — the future of Relay.

// Run the REAL Verus+Kani-verified engine (#168) — not an inline copy.
use relay_sc::engine;

// P3 Stream binding
use relay_sc_stream_bindings::exports::pulseengine::relay_stored_command::stored_command_stream::{
    Guest, AtsCommand as WitAtsCmd, DispatchedCommand as WitDisp, TimeTick as WitTick,
};

struct Component;

static mut STORE: Option<engine::CommandStore> = None;

fn get_store() -> &'static mut engine::CommandStore {
    unsafe {
        if STORE.is_none() { STORE = Some(engine::CommandStore::new()); }
        STORE.as_mut().unwrap()
    }
}

impl Guest for Component {
    async fn init() -> Result<(), String> {
        unsafe { STORE = Some(engine::CommandStore::new()); }
        Ok(())
    }

    async fn load_ats_command(cmd: WitAtsCmd) -> bool {
        get_store().load_ats_command(engine::AtsCommand {
            execute_at_sec: cmd.execute_at_sec,
            command_code: cmd.command_code,
            payload_offset: cmd.payload_offset,
            payload_len: cmd.payload_len,
            dispatched: cmd.dispatched,
        })
    }

    async fn start_rts(rts_id: u32, current_time: u64) -> bool {
        get_store().start_rts(rts_id, current_time)
    }

    async fn stop_rts(rts_id: u32) -> bool {
        get_store().stop_rts(rts_id)
    }

    /// STREAM TRANSFORMER: reads time ticks, processes the stored sequences on
    /// each, writes the dispatched commands to the output stream.
    async fn monitor(
        mut ticks: wit_bindgen::rt::async_support::StreamReader<WitTick>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitDisp> {
        let (mut writer, reader) = relay_sc_stream_bindings::wit_stream::new::<WitDisp>();

        while let Some(t) = ticks.next().await {
            let result = get_store().process_tick(t.current_time);
            let mut cmds = Vec::new();
            for i in 0..result.dispatch_count as usize {
                let d = &result.dispatched[i];
                cmds.push(WitDisp {
                    command_code: d.command_code,
                    payload_offset: d.payload_offset,
                    payload_len: d.payload_len,
                });
            }
            if !cmds.is_empty() {
                let _ = writer.write(cmds).await;
            }
        }

        reader
    }
}

relay_sc_stream_bindings::export!(Component with_types_in relay_sc_stream_bindings);
