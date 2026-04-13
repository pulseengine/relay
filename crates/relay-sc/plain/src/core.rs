//! Relay Stored Command — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/core.rs (Verus-annotated). Do not edit manually.

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_store_no_dispatches() {
        let mut store = CommandStore::new();
        let result = store.process_tick(100);
        assert_eq!(result.dispatch_count, 0);
    }

    #[test]
    fn test_ats_dispatch_at_correct_time() {
        let mut store = CommandStore::new();
        store.load_ats_command(AtsCommand {
            execute_at_sec: 50,
            command_code: 0x01,
            payload_offset: 0,
            payload_len: 8,
            dispatched: false,
        });
        let result = store.process_tick(50);
        assert_eq!(result.dispatch_count, 1);
        assert_eq!(result.dispatched[0].command_code, 0x01);
    }

    #[test]
    fn test_ats_not_dispatched_early() {
        let mut store = CommandStore::new();
        store.load_ats_command(AtsCommand {
            execute_at_sec: 100,
            command_code: 0x02,
            payload_offset: 0,
            payload_len: 4,
            dispatched: false,
        });
        let result = store.process_tick(99);
        assert_eq!(result.dispatch_count, 0);
    }

    #[test]
    fn test_ats_not_redispatched() {
        let mut store = CommandStore::new();
        store.load_ats_command(AtsCommand {
            execute_at_sec: 10,
            command_code: 0x03,
            payload_offset: 0,
            payload_len: 0,
            dispatched: false,
        });
        let r1 = store.process_tick(10);
        assert_eq!(r1.dispatch_count, 1);
        let r2 = store.process_tick(20);
        assert_eq!(r2.dispatch_count, 0);
    }

    #[test]
    fn test_rts_sequence_execution() {
        let mut store = CommandStore::new();
        store.load_rts_command(0, RtsCommand {
            delay_sec: 0,
            command_code: 0x10,
            payload_offset: 0,
            payload_len: 4,
        });
        store.load_rts_command(0, RtsCommand {
            delay_sec: 5,
            command_code: 0x11,
            payload_offset: 4,
            payload_len: 4,
        });
        assert!(store.start_rts(0, 100));

        // First command fires immediately (delay=0, elapsed=0)
        let r1 = store.process_tick(100);
        assert_eq!(r1.dispatch_count, 1);
        assert_eq!(r1.dispatched[0].command_code, 0x10);

        // Second command not ready yet (delay=5, elapsed=4)
        let r2 = store.process_tick(104);
        assert_eq!(r2.dispatch_count, 0);

        // Second command fires (delay=5, elapsed=5)
        let r3 = store.process_tick(105);
        assert_eq!(r3.dispatch_count, 1);
        assert_eq!(r3.dispatched[0].command_code, 0x11);
    }

    #[test]
    fn test_rts_stop() {
        let mut store = CommandStore::new();
        store.load_rts_command(0, RtsCommand {
            delay_sec: 0,
            command_code: 0x20,
            payload_offset: 0,
            payload_len: 0,
        });
        store.load_rts_command(0, RtsCommand {
            delay_sec: 10,
            command_code: 0x21,
            payload_offset: 0,
            payload_len: 0,
        });
        assert!(store.start_rts(0, 0));
        let r1 = store.process_tick(0);
        assert_eq!(r1.dispatch_count, 1);

        assert!(store.stop_rts(0));
        let r2 = store.process_tick(100);
        assert_eq!(r2.dispatch_count, 0);
    }

    #[test]
    fn test_dispatch_count_bounded() {
        let mut store = CommandStore::new();
        for i in 0..(MAX_DISPATCH_PER_TICK as u32 + 4) {
            store.load_ats_command(AtsCommand {
                execute_at_sec: 0,
                command_code: i as u16,
                payload_offset: 0,
                payload_len: 0,
                dispatched: false,
            });
        }
        let result = store.process_tick(0);
        assert_eq!(result.dispatch_count, MAX_DISPATCH_PER_TICK as u32);
    }

    #[test]
    fn test_ats_table_full_returns_false() {
        let mut store = CommandStore::new();
        for _ in 0..MAX_ATS_COMMANDS {
            assert!(store.load_ats_command(AtsCommand::empty()));
        }
        assert!(!store.load_ats_command(AtsCommand::empty()));
    }

    #[test]
    fn test_start_rts_empty_sequence_returns_false() {
        let mut store = CommandStore::new();
        assert!(!store.start_rts(0, 0));
    }

    #[test]
    fn test_start_rts_invalid_id_returns_false() {
        let mut store = CommandStore::new();
        assert!(!store.start_rts(MAX_RTS_SEQUENCES as u32, 0));
    }
}
