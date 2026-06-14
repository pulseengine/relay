// Relay Stored Command — P3 WASM component (self-contained).
//
// This file contains both:
//   1. The verified core engine (from plain/src/engine.rs)
//   2. The P3 async Guest trait implementation
//
// Built by: bazel build //:relay-sc (rules_wasm_component, wasi_version="p3")
// Verified by: bazel test //:relay_sc_verus_test (src/engine.rs with verus!)

// ═══════════════════════════════════════════════════════════════
// Verified core engine (plain Rust, identical to plain/src/engine.rs)
// ═══════════════════════════════════════════════════════════════

// Run the REAL Verus+Kani-verified engine (#168), not an inline copy.
use relay_sc::engine;

// ═══════════════════════════════════════════════════════════════
// P3 WASM component binding — delegates to verified engine
// ═══════════════════════════════════════════════════════════════

use relay_sc_bindings::exports::pulseengine::relay_stored_command::stored_command::{
    Guest, AtsCommand as WitAtsCmd, DispatchedCommand as WitDisp,
};

struct Component;

static mut STORE: Option<engine::CommandStore> = None;

fn get_store() -> &'static mut engine::CommandStore {
    unsafe {
        if STORE.is_none() {
            STORE = Some(engine::CommandStore::new());
        }
        STORE.as_mut().unwrap()
    }
}

impl Guest for Component {
    #[cfg(target_arch = "wasm32")]
    async fn init() -> Result<(), String> {
        unsafe { STORE = Some(engine::CommandStore::new()); }
        Ok(())
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn init() -> Result<(), String> {
        unsafe { STORE = Some(engine::CommandStore::new()); }
        Ok(())
    }

    #[cfg(target_arch = "wasm32")]
    async fn load_ats_command(cmd: WitAtsCmd) -> bool {
        Self::do_load_ats(cmd)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn load_ats_command(cmd: WitAtsCmd) -> bool {
        Self::do_load_ats(cmd)
    }

    #[cfg(target_arch = "wasm32")]
    async fn start_rts(rts_id: u32, current_time: u64) -> bool {
        Self::do_start_rts(rts_id, current_time)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn start_rts(rts_id: u32, current_time: u64) -> bool {
        Self::do_start_rts(rts_id, current_time)
    }

    #[cfg(target_arch = "wasm32")]
    async fn stop_rts(rts_id: u32) -> bool {
        Self::do_stop_rts(rts_id)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn stop_rts(rts_id: u32) -> bool {
        Self::do_stop_rts(rts_id)
    }

    #[cfg(target_arch = "wasm32")]
    async fn process_tick(current_time: u64) -> Vec<WitDisp> {
        Self::do_process_tick(current_time)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn process_tick(current_time: u64) -> Vec<WitDisp> {
        Self::do_process_tick(current_time)
    }
}

impl Component {
    fn do_load_ats(cmd: WitAtsCmd) -> bool {
        get_store().load_ats_command(engine::AtsCommand {
            execute_at_sec: cmd.execute_at_sec,
            command_code: cmd.command_code,
            payload_offset: cmd.payload_offset,
            payload_len: cmd.payload_len,
            dispatched: cmd.dispatched,
        })
    }

    fn do_start_rts(rts_id: u32, current_time: u64) -> bool {
        get_store().start_rts(rts_id, current_time)
    }

    fn do_stop_rts(rts_id: u32) -> bool {
        get_store().stop_rts(rts_id)
    }

    fn do_process_tick(current_time: u64) -> Vec<WitDisp> {
        let result = get_store().process_tick(current_time);
        let mut cmds = Vec::with_capacity(result.dispatch_count as usize);
        for i in 0..result.dispatch_count as usize {
            cmds.push(WitDisp {
                command_code: result.dispatched[i].command_code,
                payload_offset: result.dispatched[i].payload_offset,
                payload_len: result.dispatched[i].payload_len,
            });
        }
        cmds
    }
}

relay_sc_bindings::export!(Component with_types_in relay_sc_bindings);
