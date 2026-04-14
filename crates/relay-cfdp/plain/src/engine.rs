//! Relay CFDP Protocol Core — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_TRANSACTIONS: usize = 16;
pub const MAX_ACTIONS: usize = 4;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum TransactionState {
    Idle = 0,
    MetadataSent = 1,
    DataSending = 2,
    EofSent = 3,
    Finished = 4,
    Cancelled = 5,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PduType {
    Metadata = 0,
    FileData = 1,
    Eof = 2,
    Ack = 3,
    Nak = 4,
    Finished = 5,
}

#[derive(Clone, Copy)]
pub struct Transaction {
    pub id: u32,
    pub state: TransactionState,
    pub file_size: u32,
    pub bytes_sent: u32,
    pub eof_checksum: u32,
    pub retransmit_count: u32,
    pub max_retransmit: u32,
}

pub struct TransactionTable {
    pub transactions: [Transaction; MAX_TRANSACTIONS],
    pub count: u32,
}

#[derive(Clone, Copy, Debug)]
pub enum CfdpAction {
    SendMetadata,
    SendData { offset: u32, length: u32 },
    SendEof,
    SendAck,
    Retransmit { offset: u32, length: u32 },
    Complete,
    Cancel,
}

pub struct CfdpResult {
    pub actions: [CfdpAction; MAX_ACTIONS],
    pub action_count: u32,
}

impl CfdpResult {
    pub fn new() -> Self {
        CfdpResult {
            actions: [CfdpAction::Cancel; MAX_ACTIONS],
            action_count: 0,
        }
    }

    pub fn add_action(&mut self, action: CfdpAction) -> bool {
        if self.action_count as usize >= MAX_ACTIONS {
            return false;
        }
        self.actions[self.action_count as usize] = action;
        self.action_count = self.action_count + 1;
        true
    }
}

impl Transaction {
    pub const fn empty() -> Self {
        Transaction {
            id: 0,
            state: TransactionState::Idle,
            file_size: 0,
            bytes_sent: 0,
            eof_checksum: 0,
            retransmit_count: 0,
            max_retransmit: 0,
        }
    }
}

impl TransactionTable {
    pub fn new() -> Self {
        TransactionTable {
            transactions: [Transaction::empty(); MAX_TRANSACTIONS],
            count: 0,
        }
    }

    fn find_transaction(&self, transaction_id: u32) -> u32 {
        let mut i: u32 = 0;
        while i < self.count {
            if self.transactions[i as usize].id == transaction_id {
                return i;
            }
            i = i + 1;
        }
        MAX_TRANSACTIONS as u32
    }

    pub fn begin_send(&mut self, file_size: u32, max_retransmit: u32) -> Option<u32> {
        if self.count as usize >= MAX_TRANSACTIONS {
            return None;
        }
        let id = self.count;
        self.transactions[self.count as usize] = Transaction {
            id,
            state: TransactionState::Idle,
            file_size,
            bytes_sent: 0,
            eof_checksum: 0,
            retransmit_count: 0,
            max_retransmit,
        };
        self.count = self.count + 1;
        Some(id)
    }

    pub fn process_ack(&mut self, transaction_id: u32) -> CfdpResult {
        let mut result = CfdpResult::new();
        let idx = self.find_transaction(transaction_id);
        if idx as usize >= MAX_TRANSACTIONS || idx >= self.count {
            return result;
        }

        let txn = self.transactions[idx as usize];
        match txn.state {
            TransactionState::Idle => {
                self.transactions[idx as usize].state = TransactionState::MetadataSent;
                result.add_action(CfdpAction::SendMetadata);
            },
            TransactionState::MetadataSent => {
                self.transactions[idx as usize].state = TransactionState::DataSending;
                let len = txn.file_size;
                if len > 0 {
                    result.add_action(CfdpAction::SendData { offset: 0, length: len });
                }
            },
            TransactionState::DataSending => {
                self.transactions[idx as usize].bytes_sent = txn.file_size;
                self.transactions[idx as usize].state = TransactionState::EofSent;
                result.add_action(CfdpAction::SendEof);
            },
            TransactionState::EofSent => {
                self.transactions[idx as usize].state = TransactionState::Finished;
                result.add_action(CfdpAction::Complete);
            },
            TransactionState::Finished => {},
            TransactionState::Cancelled => {},
        }

        result
    }

    pub fn process_nak(
        &mut self,
        transaction_id: u32,
        offset: u32,
        length: u32,
    ) -> CfdpResult {
        let mut result = CfdpResult::new();
        let idx = self.find_transaction(transaction_id);
        if idx as usize >= MAX_TRANSACTIONS || idx >= self.count {
            return result;
        }

        let txn = self.transactions[idx as usize];

        if txn.state != TransactionState::DataSending && txn.state != TransactionState::EofSent {
            return result;
        }

        if txn.retransmit_count >= txn.max_retransmit {
            self.transactions[idx as usize].state = TransactionState::Cancelled;
            result.add_action(CfdpAction::Cancel);
            return result;
        }

        self.transactions[idx as usize].retransmit_count = txn.retransmit_count + 1;

        let clamped_length = if offset < txn.file_size {
            let remaining = txn.file_size - offset;
            if length < remaining { length } else { remaining }
        } else {
            0
        };

        if clamped_length > 0 {
            result.add_action(CfdpAction::Retransmit { offset, length: clamped_length });
        }

        result
    }

    pub fn tick(&mut self, transaction_id: u32) -> CfdpResult {
        let mut result = CfdpResult::new();
        let idx = self.find_transaction(transaction_id);
        if idx as usize >= MAX_TRANSACTIONS || idx >= self.count {
            return result;
        }

        let txn = self.transactions[idx as usize];
        match txn.state {
            TransactionState::Idle => {
                self.transactions[idx as usize].state = TransactionState::MetadataSent;
                result.add_action(CfdpAction::SendMetadata);
            },
            TransactionState::MetadataSent => {
                result.add_action(CfdpAction::SendMetadata);
            },
            TransactionState::DataSending => {
                let remaining = txn.file_size - txn.bytes_sent;
                if remaining > 0 {
                    result.add_action(CfdpAction::SendData { offset: txn.bytes_sent, length: remaining });
                } else {
                    self.transactions[idx as usize].state = TransactionState::EofSent;
                    result.add_action(CfdpAction::SendEof);
                }
            },
            TransactionState::EofSent => {
                result.add_action(CfdpAction::SendAck);
            },
            TransactionState::Finished => {
                result.add_action(CfdpAction::Complete);
            },
            TransactionState::Cancelled => {
                result.add_action(CfdpAction::Cancel);
            },
        }

        result
    }

    pub fn get_state(&self, transaction_id: u32) -> Option<TransactionState> {
        let idx = self.find_transaction(transaction_id);
        if idx as usize >= MAX_TRANSACTIONS || idx >= self.count {
            return None;
        }
        Some(self.transactions[idx as usize].state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_transaction() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 3);
        assert!(id.is_some());
        assert_eq!(id.unwrap(), 0);
        assert_eq!(table.count, 1);
    }

    #[test]
    fn test_state_progression() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 3).unwrap();

        assert_eq!(table.get_state(id), Some(TransactionState::Idle));

        // Idle -> MetadataSent
        table.process_ack(id);
        assert_eq!(table.get_state(id), Some(TransactionState::MetadataSent));

        // MetadataSent -> DataSending
        table.process_ack(id);
        assert_eq!(table.get_state(id), Some(TransactionState::DataSending));

        // DataSending -> EofSent
        table.process_ack(id);
        assert_eq!(table.get_state(id), Some(TransactionState::EofSent));

        // EofSent -> Finished
        table.process_ack(id);
        assert_eq!(table.get_state(id), Some(TransactionState::Finished));
    }

    #[test]
    fn test_nak_retransmit() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 3).unwrap();

        // Move to DataSending
        table.process_ack(id);
        table.process_ack(id);
        assert_eq!(table.get_state(id), Some(TransactionState::DataSending));

        // NAK: request retransmit
        let result = table.process_nak(id, 100, 200);
        assert_eq!(result.action_count, 1);
        assert_eq!(table.get_state(id), Some(TransactionState::DataSending));
    }

    #[test]
    fn test_max_retransmit_cancels() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 2).unwrap();

        // Move to DataSending
        table.process_ack(id);
        table.process_ack(id);

        // Exhaust retransmits
        table.process_nak(id, 0, 100);
        table.process_nak(id, 0, 100);

        // Third NAK should cancel
        let result = table.process_nak(id, 0, 100);
        assert_eq!(result.action_count, 1);
        assert_eq!(table.get_state(id), Some(TransactionState::Cancelled));
    }

    #[test]
    fn test_eof_transition() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 3).unwrap();

        // Advance to EofSent
        table.process_ack(id); // -> MetadataSent
        table.process_ack(id); // -> DataSending
        table.process_ack(id); // -> EofSent

        assert_eq!(table.get_state(id), Some(TransactionState::EofSent));

        // Tick in EofSent should send ACK
        let result = table.tick(id);
        assert_eq!(result.action_count, 1);
    }

    #[test]
    fn test_cancel_on_max_retransmit() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(512, 0).unwrap();

        // Move to DataSending
        table.process_ack(id);
        table.process_ack(id);

        // max_retransmit = 0, first NAK should cancel
        let result = table.process_nak(id, 0, 100);
        assert_eq!(result.action_count, 1);
        assert_eq!(table.get_state(id), Some(TransactionState::Cancelled));
    }

    #[test]
    fn test_bounded_transactions() {
        let mut table = TransactionTable::new();
        for _i in 0..MAX_TRANSACTIONS {
            assert!(table.begin_send(100, 1).is_some());
        }
        // Table is full
        assert!(table.begin_send(100, 1).is_none());
    }

    #[test]
    fn test_tick_idle_sends_metadata() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(256, 3).unwrap();
        let result = table.tick(id);
        assert_eq!(result.action_count, 1);
        assert_eq!(table.get_state(id), Some(TransactionState::MetadataSent));
    }

    #[test]
    fn test_get_state_invalid_id() {
        let table = TransactionTable::new();
        assert_eq!(table.get_state(999), None);
    }

    #[test]
    fn test_nak_in_wrong_state_ignored() {
        let mut table = TransactionTable::new();
        let id = table.begin_send(1024, 3).unwrap();

        // In Idle state, NAK should be ignored
        let result = table.process_nak(id, 0, 100);
        assert_eq!(result.action_count, 0);
    }
}
