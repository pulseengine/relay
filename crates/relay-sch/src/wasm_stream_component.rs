// Relay Scheduler — P3 Stream Transformer WASM component.
//
// Takes stream<tick>, emits stream<scheduled-action>.
// The verified engine (process_tick, Verus SCH-P03..P06) processes each tick.
//
// This is the stream-native P3 interface — the future of Relay.

// Run the REAL Verus+Kani-verified engine (#168) — not an inline copy.
use relay_sch::engine;

// P3 Stream binding
use relay_sch_stream_bindings::exports::pulseengine::relay_scheduler::scheduler_stream::{
    Guest, ScheduleSlot as WitSlot, ScheduledAction as WitAction, Tick as WitTick,
};

struct Component;

static mut TABLE: Option<engine::ScheduleTable> = None;

fn get_table() -> &'static mut engine::ScheduleTable {
    unsafe {
        if TABLE.is_none() { TABLE = Some(engine::ScheduleTable::new()); }
        TABLE.as_mut().unwrap()
    }
}

impl Guest for Component {
    async fn init() -> Result<(), String> {
        unsafe { TABLE = Some(engine::ScheduleTable::new()); }
        Ok(())
    }

    async fn add_slot(slot: WitSlot) -> bool {
        get_table().add_slot(engine::ScheduleSlot {
            minor_frame: slot.minor_frame,
            major_frame: slot.major_frame,
            target_channel: slot.target_channel,
            payload_offset: slot.payload_offset,
            payload_len: slot.payload_len,
            enabled: slot.enabled,
        })
    }

    async fn set_enabled(index: u32, enabled: bool) -> bool {
        get_table().set_enabled(index, enabled)
    }

    /// STREAM TRANSFORMER: reads from the input tick stream, processes each
    /// tick against the schedule table, writes fired actions to the output.
    async fn monitor(
        mut ticks: wit_bindgen::rt::async_support::StreamReader<WitTick>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitAction> {
        let (mut writer, reader) = relay_sch_stream_bindings::wit_stream::new::<WitAction>();

        // Process ticks inline — monitor is already async.
        while let Some(tick) = ticks.next().await {
            let result = get_table().process_tick(tick.minor_frame, tick.major_frame);
            let mut actions = Vec::new();
            for i in 0..result.action_count as usize {
                let a = &result.actions[i];
                actions.push(WitAction {
                    target_channel: a.target_channel,
                    payload_offset: a.payload_offset,
                    payload_len: a.payload_len,
                });
            }
            if !actions.is_empty() {
                let _ = writer.write(actions).await;
            }
        }

        reader
    }
}

relay_sch_stream_bindings::export!(Component with_types_in relay_sch_stream_bindings);
