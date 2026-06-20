//! Kani harnesses for relay-hal (plain `cfg(kani)` sibling — cargo build/test
//! never see these; run `cargo kani -p relay-hal`). Proves the relay-owned
//! `Presence` device-set logic total + bounded (FV-RELAY-HAL-001).

use crate::{DeviceDescriptor, DeviceKind, Health, Presence};

const N: usize = 4;

fn any_kind() -> DeviceKind {
    match kani::any::<u8>() % 6 {
        0 => DeviceKind::Imu,
        1 => DeviceKind::Baro,
        2 => DeviceKind::Mag,
        3 => DeviceKind::Gnss,
        4 => DeviceKind::PowerMonitor,
        _ => DeviceKind::Other,
    }
}

fn any_health() -> Health {
    match kani::any::<u8>() % 3 {
        0 => Health::Ok,
        1 => Health::Degraded,
        _ => Health::Failed,
    }
}

fn any_device() -> DeviceDescriptor {
    DeviceDescriptor {
        bus: kani::any(),
        part_id: kani::any(),
        kind: any_kind(),
        health: any_health(),
    }
}

/// HAL-K01 — an arbitrary sequence of pushes is total (no panic) and bounded:
/// `count` stays in `[0, N]`, and `push` succeeds exactly when there was room.
#[kani::proof]
#[kani::unwind(7)]
fn push_bounded_and_total() {
    let mut p: Presence<N> = Presence::new();
    let ops: usize = kani::any();
    kani::assume(ops <= 6);
    let mut i = 0;
    while i < ops {
        let before = p.count();
        let ok = p.push(any_device());
        assert!(p.count() <= N);
        if before < N {
            assert!(ok);
            assert!(p.count() == before + 1);
        } else {
            assert!(!ok);
            assert!(p.count() == before);
        }
        i += 1;
    }
}

/// HAL-K02 — `get(i)` is `Some` iff `i < count`; any index (incl. `>= N`)
/// returns `None` rather than panic / out-of-bounds.
#[kani::proof]
#[kani::unwind(5)]
fn get_in_range_only() {
    let mut p: Presence<N> = Presence::new();
    let pushes: usize = kani::any();
    kani::assume(pushes <= N);
    let mut k = 0;
    while k < pushes {
        let _ = p.push(any_device());
        k += 1;
    }
    let i: usize = kani::any();
    kani::assume(i < 16);
    let g = p.get(i);
    if i < p.count() {
        assert!(g.is_some());
    } else {
        assert!(g.is_none());
    }
}

/// HAL-K03 — the healthy voter set never exceeds what the host enumerated
/// present: `count_healthy(kind) <= count` for any kind and any device set.
#[kani::proof]
#[kani::unwind(5)]
fn healthy_set_bounded() {
    let mut p: Presence<N> = Presence::new();
    let pushes: usize = kani::any();
    kani::assume(pushes <= N);
    let mut k = 0;
    while k < pushes {
        let _ = p.push(any_device());
        k += 1;
    }
    let kind = any_kind();
    assert!(p.count_healthy(kind) <= p.count());
}
