//! Relay Telemetry Output — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_SUBSCRIPTIONS: usize = 128;

#[derive(Clone, Copy)]
pub struct Subscription {
    pub msg_id: u32,
    pub priority: u8,
    pub enabled: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ToDecision {
    Include = 0,
    Exclude = 1,
    NotSubscribed = 2,
}

pub struct SubscriptionTable {
    entries: [Subscription; MAX_SUBSCRIPTIONS],
    entry_count: u32,
}

impl Subscription {
    pub const fn empty() -> Self {
        Subscription { msg_id: 0, priority: 0, enabled: false }
    }
}

impl SubscriptionTable {
    pub fn new() -> Self {
        SubscriptionTable {
            entries: [Subscription::empty(); MAX_SUBSCRIPTIONS],
            entry_count: 0,
        }
    }

    /// Add a subscription. Returns true on success, false if full.
    pub fn subscribe(&mut self, msg_id: u32, priority: u8) -> bool {
        if self.entry_count as usize >= MAX_SUBSCRIPTIONS {
            return false;
        }
        let idx = self.entry_count as usize;
        self.entries[idx] = Subscription { msg_id, priority, enabled: true };
        self.entry_count = self.entry_count + 1;
        true
    }

    /// Remove a subscription by msg_id. Returns true if found and removed.
    pub fn unsubscribe(&mut self, msg_id: u32) -> bool {
        let count = self.entry_count;
        let mut i: u32 = 0;
        while i < count {
            let idx = i as usize;
            if self.entries[idx].msg_id == msg_id {
                self.entries[idx].enabled = false;
                return true;
            }
            i = i + 1;
        }
        false
    }

    /// Evaluate whether a message should be included in downlink.
    pub fn evaluate(&self, msg_id: u32) -> ToDecision {
        let count = self.entry_count;
        let mut i: u32 = 0;
        while i < count {
            let idx = i as usize;
            if self.entries[idx].msg_id == msg_id {
                if self.entries[idx].enabled {
                    return ToDecision::Include;
                } else {
                    return ToDecision::Exclude;
                }
            }
            i = i + 1;
        }
        ToDecision::NotSubscribed
    }

    /// Count currently enabled subscriptions.
    pub fn get_active_count(&self) -> u32 {
        let count = self.entry_count;
        let mut active: u32 = 0;
        let mut i: u32 = 0;
        while i < count {
            if self.entries[i as usize].enabled {
                active = active + 1;
            }
            i = i + 1;
        }
        active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table() {
        let table = SubscriptionTable::new();
        assert_eq!(table.get_active_count(), 0);
        assert_eq!(table.evaluate(42), ToDecision::NotSubscribed);
    }

    #[test]
    fn test_subscribe_and_include() {
        let mut table = SubscriptionTable::new();
        assert!(table.subscribe(0x0800, 1));
        assert_eq!(table.evaluate(0x0800), ToDecision::Include);
    }

    #[test]
    fn test_not_subscribed() {
        let mut table = SubscriptionTable::new();
        table.subscribe(0x0800, 1);
        assert_eq!(table.evaluate(0x0900), ToDecision::NotSubscribed);
    }

    #[test]
    fn test_disabled_subscription() {
        let mut table = SubscriptionTable::new();
        table.subscribe(0x0800, 1);
        table.unsubscribe(0x0800);
        assert_eq!(table.evaluate(0x0800), ToDecision::Exclude);
    }

    #[test]
    fn test_unsubscribe() {
        let mut table = SubscriptionTable::new();
        table.subscribe(0x0800, 1);
        assert!(table.unsubscribe(0x0800));
        assert!(!table.unsubscribe(0x9999));
        assert_eq!(table.get_active_count(), 0);
    }

    #[test]
    fn test_bounded_table() {
        let mut table = SubscriptionTable::new();
        for i in 0..MAX_SUBSCRIPTIONS {
            assert!(table.subscribe(i as u32, 1));
        }
        assert!(!table.subscribe(9999, 1));
    }

    #[test]
    fn test_active_count() {
        let mut table = SubscriptionTable::new();
        table.subscribe(1, 1);
        table.subscribe(2, 1);
        table.subscribe(3, 1);
        assert_eq!(table.get_active_count(), 3);
        table.unsubscribe(2);
        assert_eq!(table.get_active_count(), 2);
    }
}
