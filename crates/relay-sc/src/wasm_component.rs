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

mod engine {
    pub const MAX_ATS_COMMANDS: usize = 256;
    pub const MAX_RTS_SEQUENCES: usize = 16;
    pub const MAX_RTS_COMMANDS: usize = 64;
    pub const MAX_DISPATCH_PER_TICK: usize = 8;

    #[derive(Clone, Copy)]
    pub struct AtsCommand {
        pub execute_at_sec: u64,
        pub command_code: u16,
        pub payload_offset: u32,
        pub payload_len: u32,
        pub dispatched: bool,
    }

    #[derive(Clone, Copy)]
    pub struct RtsCommand {
        pub delay_sec: u32,
        pub command_code: u16,
        pub payload_offset: u32,
        pub payload_len: u32,
    }

    #[derive(Clone, Copy)]
    pub struct RtsSequence {
        pub commands: [RtsCommand; MAX_RTS_COMMANDS],
        pub command_count: u32,
        pub running: bool,
        pub start_time_sec: u64,
        pub current_index: u32,
    }

    pub struct CommandStore {
        ats_table: [AtsCommand; MAX_ATS_COMMANDS],
        ats_count: u32,
        rts_sequences: [RtsSequence; MAX_RTS_SEQUENCES],
    }

    #[derive(Clone, Copy)]
    pub struct DispatchedCommand {
        pub command_code: u16,
        pub payload_offset: u32,
        pub payload_len: u32,
    }

    pub struct DispatchResult {
        pub dispatched: [DispatchedCommand; MAX_DISPATCH_PER_TICK],
        pub dispatch_count: u32,
    }

    impl AtsCommand {
        pub const fn empty() -> Self {
            AtsCommand { execute_at_sec: 0, command_code: 0, payload_offset: 0, payload_len: 0, dispatched: false }
        }
    }

    impl RtsCommand {
        pub const fn empty() -> Self {
            RtsCommand { delay_sec: 0, command_code: 0, payload_offset: 0, payload_len: 0 }
        }
    }

    impl RtsSequence {
        pub const fn empty() -> Self {
            RtsSequence {
                commands: [RtsCommand::empty(); MAX_RTS_COMMANDS],
                command_count: 0,
                running: false,
                start_time_sec: 0,
                current_index: 0,
            }
        }
    }

    impl DispatchedCommand {
        pub const fn empty() -> Self {
            DispatchedCommand { command_code: 0, payload_offset: 0, payload_len: 0 }
        }
    }

    impl CommandStore {
        pub fn new() -> Self {
            CommandStore {
                ats_table: [AtsCommand::empty(); MAX_ATS_COMMANDS],
                ats_count: 0,
                rts_sequences: [RtsSequence::empty(); MAX_RTS_SEQUENCES],
            }
        }

        pub fn load_ats_command(&mut self, cmd: AtsCommand) -> bool {
            if self.ats_count as usize >= MAX_ATS_COMMANDS { return false; }
            self.ats_table[self.ats_count as usize] = cmd;
            self.ats_count = self.ats_count + 1;
            true
        }

        pub fn start_rts(&mut self, rts_id: u32, current_time_sec: u64) -> bool {
            if rts_id as usize >= MAX_RTS_SEQUENCES { return false; }
            if self.rts_sequences[rts_id as usize].command_count == 0 { return false; }
            self.rts_sequences[rts_id as usize].running = true;
            self.rts_sequences[rts_id as usize].start_time_sec = current_time_sec;
            self.rts_sequences[rts_id as usize].current_index = 0;
            true
        }

        pub fn stop_rts(&mut self, rts_id: u32) -> bool {
            if rts_id as usize >= MAX_RTS_SEQUENCES { return false; }
            self.rts_sequences[rts_id as usize].running = false;
            true
        }

        pub fn load_rts_command(&mut self, rts_id: u32, cmd: RtsCommand) -> bool {
            if rts_id as usize >= MAX_RTS_SEQUENCES { return false; }
            let seq = &mut self.rts_sequences[rts_id as usize];
            if seq.command_count as usize >= MAX_RTS_COMMANDS { return false; }
            seq.commands[seq.command_count as usize] = cmd;
            seq.command_count = seq.command_count + 1;
            true
        }

        pub fn ats_count(&self) -> u32 { self.ats_count }

        pub fn process_tick(&mut self, current_time_sec: u64) -> DispatchResult {
            let mut result = DispatchResult {
                dispatched: [DispatchedCommand::empty(); MAX_DISPATCH_PER_TICK],
                dispatch_count: 0,
            };

            // Check ATS commands
            let ats_count = self.ats_count;
            let mut i: u32 = 0;
            while i < ats_count {
                if result.dispatch_count as usize >= MAX_DISPATCH_PER_TICK { break; }
                let cmd = self.ats_table[i as usize];
                if !cmd.dispatched && cmd.execute_at_sec <= current_time_sec {
                    let idx = result.dispatch_count as usize;
                    result.dispatched[idx] = DispatchedCommand {
                        command_code: cmd.command_code,
                        payload_offset: cmd.payload_offset,
                        payload_len: cmd.payload_len,
                    };
                    result.dispatch_count = result.dispatch_count + 1;
                    self.ats_table[i as usize].dispatched = true;
                }
                i = i + 1;
            }

            // Check RTS sequences
            let mut r: u32 = 0;
            while r < MAX_RTS_SEQUENCES as u32 {
                if result.dispatch_count as usize >= MAX_DISPATCH_PER_TICK { break; }
                let seq = self.rts_sequences[r as usize];
                if seq.running && seq.current_index < seq.command_count {
                    let cmd = seq.commands[seq.current_index as usize];
                    let elapsed = current_time_sec.wrapping_sub(seq.start_time_sec);
                    if elapsed >= cmd.delay_sec as u64 {
                        let idx = result.dispatch_count as usize;
                        result.dispatched[idx] = DispatchedCommand {
                            command_code: cmd.command_code,
                            payload_offset: cmd.payload_offset,
                            payload_len: cmd.payload_len,
                        };
                        result.dispatch_count = result.dispatch_count + 1;
                        self.rts_sequences[r as usize].current_index = seq.current_index + 1;
                        // If sequence is exhausted, stop it
                        if seq.current_index + 1 >= seq.command_count {
                            self.rts_sequences[r as usize].running = false;
                        }
                    }
                }
                r = r + 1;
            }

            result
        }
    }
}

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
