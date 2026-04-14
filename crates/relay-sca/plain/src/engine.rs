//! Relay Stored Command Absolute — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_COMMANDS: usize = 256;
pub const MAX_DISPATCH_PER_TICK: usize = 8;

#[derive(Clone, Copy)]
pub struct AbsCommand {
    pub execute_at_sec: u64,
    pub command_code: u16,
    pub args: [u8; 32],
    pub arg_len: u8,
    pub dispatched: bool,
    pub enabled: bool,
}

pub struct AbsTable {
    pub commands: [AbsCommand; MAX_COMMANDS],
    pub command_count: u32,
}

#[derive(Clone, Copy)]
pub struct DispatchedCommand {
    pub command_code: u16,
    pub args: [u8; 32],
    pub arg_len: u8,
}

pub struct DispatchResult {
    pub dispatched: [DispatchedCommand; MAX_DISPATCH_PER_TICK],
    pub dispatch_count: u32,
}

impl AbsCommand {
    pub fn empty() -> Self {
        AbsCommand {
            execute_at_sec: 0,
            command_code: 0,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: false,
        }
    }
}

impl DispatchedCommand {
    pub fn empty() -> Self {
        DispatchedCommand {
            command_code: 0,
            args: [0u8; 32],
            arg_len: 0,
        }
    }
}

impl AbsTable {
    pub fn new() -> Self {
        AbsTable {
            commands: [AbsCommand::empty(); MAX_COMMANDS],
            command_count: 0,
        }
    }

    pub fn add_command(&mut self, cmd: AbsCommand) -> bool {
        if self.command_count as usize >= MAX_COMMANDS {
            return false;
        }
        self.commands[self.command_count as usize] = cmd;
        self.command_count = self.command_count + 1;
        true
    }

    pub fn count(&self) -> u32 {
        self.command_count
    }

    pub fn process_tick(&mut self, current_time_sec: u64) -> DispatchResult {
        let mut result = DispatchResult {
            dispatched: [DispatchedCommand::empty(); MAX_DISPATCH_PER_TICK],
            dispatch_count: 0,
        };

        let cmd_count = self.command_count;
        let mut i: u32 = 0;

        while i < cmd_count {
            if result.dispatch_count as usize >= MAX_DISPATCH_PER_TICK {
                break;
            }

            let cmd = self.commands[i as usize];

            if cmd.enabled && !cmd.dispatched && cmd.execute_at_sec <= current_time_sec {
                let idx = result.dispatch_count as usize;
                result.dispatched[idx] = DispatchedCommand {
                    command_code: cmd.command_code,
                    args: cmd.args,
                    arg_len: cmd.arg_len,
                };
                result.dispatch_count = result.dispatch_count + 1;
                self.commands[i as usize].dispatched = true;
            }

            i = i + 1;
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table_no_dispatches() {
        let mut table = AbsTable::new();
        let result = table.process_tick(100);
        assert_eq!(result.dispatch_count, 0);
    }

    #[test]
    fn test_dispatch_at_exact_time() {
        let mut table = AbsTable::new();
        table.add_command(AbsCommand {
            execute_at_sec: 50,
            command_code: 0x01,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        let result = table.process_tick(50);
        assert_eq!(result.dispatch_count, 1);
        assert_eq!(result.dispatched[0].command_code, 0x01);
    }

    #[test]
    fn test_not_dispatched_early() {
        let mut table = AbsTable::new();
        table.add_command(AbsCommand {
            execute_at_sec: 100,
            command_code: 0x02,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        let result = table.process_tick(99);
        assert_eq!(result.dispatch_count, 0);
    }

    #[test]
    fn test_not_redispatched() {
        let mut table = AbsTable::new();
        table.add_command(AbsCommand {
            execute_at_sec: 10,
            command_code: 0x03,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        let r1 = table.process_tick(10);
        assert_eq!(r1.dispatch_count, 1);
        let r2 = table.process_tick(20);
        assert_eq!(r2.dispatch_count, 0);
    }

    #[test]
    fn test_disabled_not_dispatched() {
        let mut table = AbsTable::new();
        table.add_command(AbsCommand {
            execute_at_sec: 0,
            command_code: 0x04,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: false,
        });
        let result = table.process_tick(100);
        assert_eq!(result.dispatch_count, 0);
    }

    #[test]
    fn test_dispatch_count_bounded() {
        let mut table = AbsTable::new();
        for i in 0..(MAX_DISPATCH_PER_TICK as u32 + 4) {
            table.add_command(AbsCommand {
                execute_at_sec: 0,
                command_code: i as u16,
                args: [0u8; 32],
                arg_len: 0,
                dispatched: false,
                enabled: true,
            });
        }
        let result = table.process_tick(0);
        assert_eq!(result.dispatch_count, MAX_DISPATCH_PER_TICK as u32);
    }

    #[test]
    fn test_table_full_returns_false() {
        let mut table = AbsTable::new();
        for _ in 0..MAX_COMMANDS {
            assert!(table.add_command(AbsCommand::empty()));
        }
        assert!(!table.add_command(AbsCommand::empty()));
    }

    #[test]
    fn test_multiple_commands_different_times() {
        let mut table = AbsTable::new();
        table.add_command(AbsCommand {
            execute_at_sec: 10,
            command_code: 0x10,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        table.add_command(AbsCommand {
            execute_at_sec: 20,
            command_code: 0x20,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        table.add_command(AbsCommand {
            execute_at_sec: 30,
            command_code: 0x30,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled: true,
        });
        let r1 = table.process_tick(15);
        assert_eq!(r1.dispatch_count, 1);
        assert_eq!(r1.dispatched[0].command_code, 0x10);
        let r2 = table.process_tick(25);
        assert_eq!(r2.dispatch_count, 1);
        assert_eq!(r2.dispatched[0].command_code, 0x20);
        let r3 = table.process_tick(35);
        assert_eq!(r3.dispatch_count, 1);
        assert_eq!(r3.dispatched[0].command_code, 0x30);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// SCA-P01: dispatch_count never exceeds MAX_DISPATCH_PER_TICK
    #[kani::proof]
    fn verify_dispatch_bounded() {
        let mut table = AbsTable::new();
        let execute_at: u64 = kani::any();
        let code: u16 = kani::any();
        let enabled: bool = kani::any();
        table.add_command(AbsCommand {
            execute_at_sec: execute_at,
            command_code: code,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled,
        });
        let current_time: u64 = kani::any();
        let result = table.process_tick(current_time);
        assert!(result.dispatch_count as usize <= MAX_DISPATCH_PER_TICK);
    }

    /// SCA-P02: no panics for any symbolic input
    #[kani::proof]
    fn verify_no_panic() {
        let mut table = AbsTable::new();
        let execute_at: u64 = kani::any();
        let code: u16 = kani::any();
        let enabled: bool = kani::any();
        table.add_command(AbsCommand {
            execute_at_sec: execute_at,
            command_code: code,
            args: [0u8; 32],
            arg_len: 0,
            dispatched: false,
            enabled,
        });
        let _ = table.count();
        let _ = table.process_tick(kani::any());
    }
}
