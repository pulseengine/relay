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

/// `cruise-alt` is metres AGL and POSITIVE — the one altitude in this seam that
/// is NOT NED-signed.
///
/// WHY THIS TEST EXISTS: I documented the field the other way round in the WIT
/// and had to correct it. The surrounding convention makes the wrong answer look
/// obvious — `pos-d`, `home.z` and every setpoint in this package are NED with
/// negative up — but `FlightSupervisor` documents the constructor argument as
/// "m AGL" and NEGATES it itself when deriving the setpoint
/// (`[p0, p1, -cruise_alt]` for Takeoff, Loiter, Rtl and Land). So a consumer
/// following the convention and passing -30.0 to mean "30 m up" would command
/// +30.0 NED: thirty metres BELOW the ground.
///
/// A prose fix is not enough for a sign error on a safety seam, so this pins the
/// BEHAVIOUR: a positive `cruise_alt` must make the vehicle climb. If the value
/// were ever consumed as NED directly, the same positive input would command a
/// descent and this test fails.
#[test]
fn a_positive_cruise_alt_commands_a_climb_not_a_descent() {
    use falcon_core::{FlightSupervisor, SimBackend, relay_fsm::Event};

    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    // cruise_alt = 2.0, POSITIVE, meaning 2 m above the ground.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);

    let start_d = backend.pos[2];
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    let end_d = backend.pos[2];

    // NED: down is POSITIVE, so climbing means pos[2] DECREASES.
    assert!(
        end_d < start_d,
        "a positive cruise_alt must command a CLIMB: pos-d went {start_d} -> {end_d}. \
         If pos-d increased, cruise_alt is being consumed as an NED coordinate \
         instead of an AGL magnitude, and a consumer passing a negative value \
         for 'up' would be commanded into the ground."
    );
    // And it should settle near the commanded altitude, not merely move upward —
    // otherwise a drift would satisfy the inequality above.
    assert!(
        (end_d - (-2.0)).abs() < 0.5,
        "should hold near -2.0 m NED (2 m AGL) for cruise_alt = 2.0; got {end_d}"
    );
}

/// MEASURED GAP, pinned so it cannot be forgotten: the `@0.12.0` seam cannot
/// supply a CALIBRATION, so a host that warms its estimator before arming — the
/// way a real GCS does — is refused by `CalibrationPresent`.
///
/// This test asserts a LIMITATION, not a feature, and it is here because I
/// published the wrong cause. I claimed the component "cannot arm" without
/// `battery-v`, which is true but mis-attributed: with a warmed estimator the
/// FIRST row to refuse is `CalibrationPresent`, and adding battery does not
/// move it. `update_preflight` sets
/// `calibration_present = core.calibration() != CalParams::identity()`, and
/// nothing in `vehicle-config` or `supervisor-config` carries a calibration —
/// the component never calls `set_calibration` because it has nothing to pass.
///
/// WHY IT MATTERS BEYOND THE SEAM. The published component can therefore only
/// be armed on the *arm-before-first-step* path, where `FlightSupervisor::new`'s
/// deliberate bring-up defaults (the legacy six rows forced true) are what the
/// pre-arm table is judged against. That is SUPERVISOR-P01's own falsification
/// condition — "the supervisor runs against default inputs the host cannot
/// supply" — so a GREEN SITL GATE IS NOT EVIDENCE that the pre-arm table works
/// through the seam. It shows the defaults path works.
///
/// The fix is a seam change (a calibration input) and therefore not this test's
/// to make. Until it lands, this test is the honest record of where the V is
/// open.
#[test]
fn the_seam_cannot_supply_a_calibration_so_a_warmed_host_is_refused() {
    use falcon_core::relay_calib::CalParams;
    use falcon_core::relay_preflight::CheckId;
    use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, relay_fsm::Event};

    const G: f32 = 9.80665;
    /// Everything the @0.12.0 sensor frame CAN carry, and nothing it cannot.
    struct SeamHost;
    impl FlightBackend for SeamHost {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -G],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<[f32; 3]> {
            Some([0.0, 0.0, 0.0])
        }
        fn read_mag(&mut self) -> Option<[f32; 3]> {
            None
        }
        fn read_heading(&mut self) -> Option<f32> {
            Some(0.0)
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.0025
        }
        // The seam DOES carry these two at @0.12.0.
        fn read_battery_v(&mut self) -> Option<f32> {
            Some(16.8)
        }
        fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
            Some([3000; 4])
        }
    }

    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 14.0);
    let mut b = SeamHost;
    // Warm the estimator first, which is what `step_estimate_only` exists for
    // and what a real host does before arming. Motors are zero throughout.
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.set_check(CheckId::FailsafeConfigured, true);
    sup.command(Event::Arm, true, true);

    assert_eq!(
        sup.arm_blocked_check(),
        Some(CheckId::CalibrationPresent),
        "a warmed host supplying everything the seam carries must still be \
         refused, and the row must be CalibrationPresent — if this changes, the \
         seam gained a calibration path (good: delete this test and verify the \
         new one) or a DIFFERENT row now refuses first (bad: the gap moved and \
         the published justification is stale again)"
    );
    assert_eq!(sup.mode(), Mode::Disarmed, "and it must not be armed");

    // THE CONTROL — install a calibration, the one thing the seam cannot pass,
    // and the identical arm succeeds. This is what attributes the refusal above
    // to the missing calibration rather than to the host being unarmable.
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..10 {
        sup.step(&mut b);
    }
    sup.set_check(CheckId::FailsafeConfigured, true);
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Armed,
        "with a calibration installed the identical arm must succeed — if it \
         does not, the refusal above was not caused by the missing calibration"
    );
}
