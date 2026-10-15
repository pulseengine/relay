//! The wire contract of `pulseengine:falcon-cascade@0.12.0`'s supervisor
//! fields, pinned against the Rust the component actually runs.
//!
//! WHY THIS FILE EXISTS. SUPERVISOR-P01 publishes two integers — `mode` and
//! `failsafe` — that an EXTERNAL CONSUMER decodes directly out of a seqlock in
//! shared memory. The consumer (pulseengine/jess#167) has taken an explicit
//! decoder obligation against the numbering below, and has already been burned
//! once by a layout inferred from prose rather than measured (their AFD-127).
//! So the numbering must not live only in a WIT doc comment, because a WIT doc
//! comment cannot fail.
//!
//! THE SPECIFIC HAZARD, measured: `relay_fsm::Mode` carries NO `#[repr]` and
//! writes NO explicit discriminants. Its wire values are therefore its
//! DECLARATION ORDER, and inserting or reordering a variant upstream would
//! silently renumber every mode a consumer has already shipped a decoder for —
//! with no compile error anywhere in this repo. These tests assert each variant
//! BY NAME so that a reorder fails here, loudly, on our side of the seam.
//!
//! The failsafe bits are pinned for the same reason, one bit at a time, against
//! the `falcon_core::failsafe_flags` constants the supervisor actually ORs.

use falcon_core::failsafe_flags as ff;
use falcon_core::relay_fsm::Mode;

/// Every mode discriminant the seam publishes, by name.
///
/// Asserted individually rather than as a loop over a list, because a loop
/// comparing a list to itself is the mirror defect: it would pass after a
/// reorder. The literal on the right is the contract.
#[test]
fn mode_discriminants_are_the_published_wire_values() {
    assert_eq!(Mode::Disarmed as u8, 0, "mode 0 is disarmed");
    assert_eq!(Mode::Armed as u8, 1, "mode 1 is armed");
    assert_eq!(Mode::Takeoff as u8, 2, "mode 2 is takeoff");
    assert_eq!(Mode::Loiter as u8, 3, "mode 3 is loiter");
    assert_eq!(Mode::Mission as u8, 4, "mode 4 is mission");
    assert_eq!(Mode::Land as u8, 5, "mode 5 is land");
    assert_eq!(Mode::Rtl as u8, 6, "mode 6 is rtl");
    assert_eq!(Mode::Terminated as u8, 7, "mode 7 is terminated");
}

/// SUPERVISOR-P01 clause 3, as a test rather than a sentence.
///
/// `Disarmed` is a SAFE on-ground state. `Terminated` is a deliberate AIRBORNE
/// thrust cut that nothing recovers from without an on-ground reset. A seam
/// that let them collapse would report a terminated vehicle as safely
/// disarmed — the absence-looks-like-data shape this release line has already
/// paid for twice (#413 a battery read as healthy, #452 an accelerometer read
/// as gravity).
///
/// This is not implied by the test above: that one pins today's numbers, this
/// one states the property that must survive any future renumbering.
#[test]
fn disarmed_and_terminated_are_distinguishable_on_the_wire() {
    assert_ne!(
        Mode::Disarmed as u8,
        Mode::Terminated as u8,
        "disarmed and terminated MUST NOT share a wire value — a consumer would \
         report a terminated vehicle as safely on the ground"
    );
}

/// Every failsafe bit the seam publishes, against the constant the supervisor ORs.
#[test]
fn failsafe_bits_are_the_published_wire_values() {
    assert_eq!(ff::RTL_LATCHED, 1 << 0, "failsafe bit 0 is rtl-latched");
    assert_eq!(ff::BATT_LOW, 1 << 1, "failsafe bit 1 is batt-low");
    assert_eq!(ff::BATT_CRITICAL, 1 << 2, "failsafe bit 2 is batt-critical");
    assert_eq!(ff::BATT_SILENT, 1 << 3, "failsafe bit 3 is batt-silent");
    assert_eq!(
        ff::GEOFENCE_BREACH,
        1 << 4,
        "failsafe bit 4 is geofence-breach"
    );
    assert_eq!(
        ff::RUNAWAY_ARMING,
        1 << 5,
        "failsafe bit 5 is runaway-arming"
    );
    assert_eq!(ff::WIND_ARMING, 1 << 6, "failsafe bit 6 is wind-arming");
    assert_eq!(ff::ARM_BLOCKED, 1 << 7, "failsafe bit 7 is arm-blocked");
}

/// The bits must be DISJOINT, which is the property that makes a bitmask the
/// right shape rather than an enum. Two causes sharing a bit would make a flat
/// pack inside a geofence breach indistinguishable from either alone.
#[test]
fn failsafe_bits_are_disjoint() {
    let all = [
        ff::RTL_LATCHED,
        ff::BATT_LOW,
        ff::BATT_CRITICAL,
        ff::BATT_SILENT,
        ff::GEOFENCE_BREACH,
        ff::RUNAWAY_ARMING,
        ff::WIND_ARMING,
        ff::ARM_BLOCKED,
    ];
    let mut seen: u32 = 0;
    for (i, bit) in all.iter().enumerate() {
        assert_eq!(bit.count_ones(), 1, "flag {i} must be a single bit");
        assert_eq!(seen & bit, 0, "flag {i} collides with an earlier flag");
        seen |= bit;
    }
    // All eight present and none above bit 7 — a ninth cause must extend the
    // WIT doc and this test together, not arrive silently.
    assert_eq!(seen, 0x00FF, "exactly bits 0..=7 are allocated");
}

/// The lever the published component uses to refuse arming while nobody has
/// configured the supervisor.
///
/// WHY THE COMPONENT NEEDS THIS, and why it is not paranoia.
/// `FlightSupervisor::new` deliberately defaults the first six pre-arm rows to
/// TRUE for bring-up back-compat, and `update_preflight` never writes the
/// `FailsafeConfigured` row — so that default is what a host's very first
/// `arm` would be judged against. The component therefore declares this row
/// explicitly on every entry through the `pilot` interface, from
/// `ensure_supervisor`.
///
/// Without it, a host that called `pilot.command(arm, ..)` before ever calling
/// `pilot.configure-supervisor(..)` would arm a vehicle whose geofence centre,
/// fence radius, RTL altitude and battery thresholds are all unset. That is the
/// "a supervisor on defaults is worse than no supervisor" case SUPERVISOR-P01
/// names, and it is the one shape of this change that could ship a REGRESSION
/// against the stabiliser it replaces.
#[test]
fn an_unconfigured_supervisor_refuses_to_arm() {
    use falcon_core::relay_preflight::CheckId;
    use falcon_core::{FlightSupervisor, relay_fsm::Event};

    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    // Exactly what the component does for an unconfigured host.
    sup.set_check(CheckId::FailsafeConfigured, false);
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Disarmed,
        "an unconfigured supervisor must refuse to arm; it reported {:?}",
        sup.mode()
    );
    assert_eq!(
        sup.arm_blocked_check(),
        Some(CheckId::FailsafeConfigured),
        "the refusal must be NAMED as FailsafeConfigured, not some other row — \
         otherwise this test could pass because an unrelated row happened to \
         block, and would stop proving anything the day that row changed"
    );

    // THE CONTROL. Flipping only the row under test must let the same arm
    // through, so the refusal above is attributable to it and not to the
    // supervisor being unarmable in this state for some other reason.
    sup.set_check(CheckId::FailsafeConfigured, true);
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Armed,
        "with the row satisfied the identical arm must succeed — if this fails, \
         the refusal above was not caused by FailsafeConfigured"
    );
}
