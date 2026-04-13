//! Relay Software Bus — routing core.
//!
//! Host-native service that routes messages between components.
//! Push-based: maintains a subscription table, dispatches to subscribers.
//!
//! Source mapping: NASA cFS Software Bus (cfe_sb_api.c, cfe_sb_task.c)

use std::collections::VecDeque;

pub const MAX_CHANNELS: usize = 256;
pub const MAX_SUBSCRIBERS_PER_CHANNEL: usize = 16;
pub const MAX_PENDING_MESSAGES: usize = 64;

pub type ChannelId = u32;
pub type SubscriberId = u32;

/// Error type for Software Bus operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SbError {
    /// Channel has reached maximum subscriber count.
    ChannelFull,
    /// Maximum number of channels reached.
    TooManyChannels,
    /// Subscriber not found on channel.
    NotSubscribed,
    /// Message queue is full, message dropped.
    QueueFull,
    /// No subscribers on the target channel.
    NoSubscribers,
}

/// A subscription entry.
#[derive(Clone, Debug)]
pub struct Subscription {
    pub channel: ChannelId,
    pub subscriber: SubscriberId,
    pub active: bool,
}

/// A channel's subscriber list (fixed-capacity).
#[derive(Clone, Debug)]
struct ChannelEntry {
    channel_id: ChannelId,
    subscribers: [SubscriberId; MAX_SUBSCRIBERS_PER_CHANNEL],
    subscriber_count: usize,
}

impl ChannelEntry {
    fn new(channel_id: ChannelId) -> Self {
        ChannelEntry {
            channel_id,
            subscribers: [0; MAX_SUBSCRIBERS_PER_CHANNEL],
            subscriber_count: 0,
        }
    }
}

/// The subscription table — maps channels to subscriber lists.
pub struct SubscriptionTable {
    channels: Vec<ChannelEntry>,
}

/// A message routed through the bus.
#[derive(Clone, Debug)]
pub struct Message {
    pub channel: ChannelId,
    pub payload: Vec<u8>,
    pub timestamp: u64,
    pub source: SubscriberId,
}

/// Bounded message queue.
pub struct MessageQueue {
    messages: VecDeque<Message>,
    capacity: usize,
}

/// Bus statistics.
#[derive(Clone, Debug, Default)]
pub struct BusStats {
    pub messages_routed: u64,
    pub messages_dropped: u64,
    pub subscriptions_active: u32,
}

/// The Software Bus — routes messages between components.
pub struct SoftwareBus {
    subscriptions: SubscriptionTable,
    pending: MessageQueue,
    stats: BusStats,
}

impl SubscriptionTable {
    fn new() -> Self {
        SubscriptionTable {
            channels: Vec::new(),
        }
    }

    fn find_channel(&self, channel: ChannelId) -> Option<usize> {
        self.channels.iter().position(|c| c.channel_id == channel)
    }

    fn get_or_create_channel(&mut self, channel: ChannelId) -> Result<usize, SbError> {
        if let Some(idx) = self.find_channel(channel) {
            return Ok(idx);
        }
        if self.channels.len() >= MAX_CHANNELS {
            return Err(SbError::TooManyChannels);
        }
        self.channels.push(ChannelEntry::new(channel));
        Ok(self.channels.len() - 1)
    }

    fn subscribe(&mut self, channel: ChannelId, subscriber: SubscriberId) -> Result<(), SbError> {
        let idx = self.get_or_create_channel(channel)?;
        let entry = &mut self.channels[idx];
        // Check if already subscribed
        for i in 0..entry.subscriber_count {
            if entry.subscribers[i] == subscriber {
                return Ok(());
            }
        }
        if entry.subscriber_count >= MAX_SUBSCRIBERS_PER_CHANNEL {
            return Err(SbError::ChannelFull);
        }
        entry.subscribers[entry.subscriber_count] = subscriber;
        entry.subscriber_count += 1;
        Ok(())
    }

    fn unsubscribe(&mut self, channel: ChannelId, subscriber: SubscriberId) -> Result<(), SbError> {
        let idx = self.find_channel(channel).ok_or(SbError::NotSubscribed)?;
        let entry = &mut self.channels[idx];
        let mut found = false;
        for i in 0..entry.subscriber_count {
            if entry.subscribers[i] == subscriber {
                // Swap-remove
                entry.subscribers[i] = entry.subscribers[entry.subscriber_count - 1];
                entry.subscriber_count -= 1;
                found = true;
                break;
            }
        }
        if !found {
            return Err(SbError::NotSubscribed);
        }
        Ok(())
    }

    fn get_subscribers(&self, channel: ChannelId) -> &[SubscriberId] {
        match self.find_channel(channel) {
            Some(idx) => {
                let entry = &self.channels[idx];
                &entry.subscribers[..entry.subscriber_count]
            }
            None => &[],
        }
    }

    fn subscriber_count(&self) -> u32 {
        let mut total: u32 = 0;
        for entry in &self.channels {
            total += entry.subscriber_count as u32;
        }
        total
    }
}

impl MessageQueue {
    fn new(capacity: usize) -> Self {
        MessageQueue {
            messages: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn push(&mut self, msg: Message) -> bool {
        if self.messages.len() >= self.capacity {
            return false;
        }
        self.messages.push_back(msg);
        true
    }
}

impl SoftwareBus {
    /// Create a new Software Bus with the given message queue capacity.
    pub fn new(capacity: usize) -> Self {
        SoftwareBus {
            subscriptions: SubscriptionTable::new(),
            pending: MessageQueue::new(capacity),
            stats: BusStats::default(),
        }
    }

    /// Subscribe a component to a channel.
    pub fn subscribe(&mut self, channel: ChannelId, subscriber: SubscriberId) -> Result<(), SbError> {
        let result = self.subscriptions.subscribe(channel, subscriber);
        if result.is_ok() {
            self.stats.subscriptions_active = self.subscriptions.subscriber_count();
        }
        result
    }

    /// Unsubscribe a component from a channel.
    pub fn unsubscribe(&mut self, channel: ChannelId, subscriber: SubscriberId) -> Result<(), SbError> {
        let result = self.subscriptions.unsubscribe(channel, subscriber);
        if result.is_ok() {
            self.stats.subscriptions_active = self.subscriptions.subscriber_count();
        }
        result
    }

    /// Publish a message to a channel.
    /// Returns the number of subscribers the message was delivered to.
    pub fn publish(
        &mut self,
        channel: ChannelId,
        payload: Vec<u8>,
        source: SubscriberId,
    ) -> Result<usize, SbError> {
        let subscribers = self.subscriptions.get_subscribers(channel);
        if subscribers.is_empty() {
            return Err(SbError::NoSubscribers);
        }
        let count = subscribers.len();
        let msg = Message {
            channel,
            payload,
            timestamp: 0, // Timestamp set by caller or host time service
            source,
        };
        if self.pending.push(msg) {
            self.stats.messages_routed += 1;
        } else {
            self.stats.messages_dropped += 1;
            return Err(SbError::QueueFull);
        }
        Ok(count)
    }

    /// Get the list of subscribers for a channel.
    pub fn get_subscribers(&self, channel: ChannelId) -> &[SubscriberId] {
        self.subscriptions.get_subscribers(channel)
    }

    /// Get bus statistics.
    pub fn stats(&self) -> &BusStats {
        &self.stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_subscribe_and_get() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(1, 200).unwrap();
        let subs = bus.get_subscribers(1);
        assert_eq!(subs.len(), 2);
        assert!(subs.contains(&100));
        assert!(subs.contains(&200));
    }

    #[test]
    fn test_unsubscribe() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(1, 200).unwrap();
        bus.unsubscribe(1, 100).unwrap();
        let subs = bus.get_subscribers(1);
        assert_eq!(subs.len(), 1);
        assert!(subs.contains(&200));
    }

    #[test]
    fn test_publish_no_subscribers() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        let result = bus.publish(99, vec![1, 2, 3], 0);
        assert_eq!(result, Err(SbError::NoSubscribers));
    }

    #[test]
    fn test_publish_multiple_subscribers() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(1, 200).unwrap();
        bus.subscribe(1, 300).unwrap();
        let count = bus.publish(1, vec![0xAA], 50).unwrap();
        assert_eq!(count, 3);
    }

    #[test]
    fn test_channel_isolation() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(2, 200).unwrap();
        let subs1 = bus.get_subscribers(1);
        assert_eq!(subs1.len(), 1);
        assert!(subs1.contains(&100));
        let subs2 = bus.get_subscribers(2);
        assert_eq!(subs2.len(), 1);
        assert!(subs2.contains(&200));
    }

    #[test]
    fn test_stats_tracking() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        assert_eq!(bus.stats().subscriptions_active, 1);
        bus.publish(1, vec![1], 0).unwrap();
        assert_eq!(bus.stats().messages_routed, 1);
        assert_eq!(bus.stats().messages_dropped, 0);
    }

    #[test]
    fn test_unsubscribe_nonexistent() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        let result = bus.unsubscribe(1, 999);
        assert_eq!(result, Err(SbError::NotSubscribed));
    }

    #[test]
    fn test_max_subscribers_per_channel() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        for i in 0..MAX_SUBSCRIBERS_PER_CHANNEL {
            bus.subscribe(1, i as u32).unwrap();
        }
        let result = bus.subscribe(1, 999);
        assert_eq!(result, Err(SbError::ChannelFull));
    }

    #[test]
    fn test_duplicate_subscribe_idempotent() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(1, 100).unwrap(); // duplicate, should be no-op
        let subs = bus.get_subscribers(1);
        assert_eq!(subs.len(), 1);
    }

    #[test]
    fn test_queue_full_drops_message() {
        let mut bus = SoftwareBus::new(2); // tiny capacity
        bus.subscribe(1, 100).unwrap();
        bus.publish(1, vec![1], 0).unwrap();
        bus.publish(1, vec![2], 0).unwrap();
        let result = bus.publish(1, vec![3], 0);
        assert_eq!(result, Err(SbError::QueueFull));
        assert_eq!(bus.stats().messages_dropped, 1);
    }

    #[test]
    fn test_stats_after_unsubscribe() {
        let mut bus = SoftwareBus::new(MAX_PENDING_MESSAGES);
        bus.subscribe(1, 100).unwrap();
        bus.subscribe(1, 200).unwrap();
        assert_eq!(bus.stats().subscriptions_active, 2);
        bus.unsubscribe(1, 100).unwrap();
        assert_eq!(bus.stats().subscriptions_active, 1);
    }
}
