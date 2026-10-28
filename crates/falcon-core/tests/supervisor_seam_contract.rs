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

/// `CalibrationPresent` means "a VALID calibration has been EXPLICITLY
/// configured" (jess#167, #611), and a host that warms its estimator before
/// arming (the way a real GCS does, and since #609 finding 2 the ONLY way the
/// published component arms) is judged on exactly that:
///
///   * NEVER configured            -> refused, by `CalibrationPresent`;
///   * an explicit IDENTITY         -> ARMS: per-sensor calibration is applied
///     below the seam, so the identity means "calibrated upstream, no
///     residual", not "not calibrated";
///   * an INVALID one afterwards    -> refused again (`set_calibration`
///     returns false, installs the identity, and REVOKES the earlier valid
///     one);
///   * a valid non-identity one     -> arms.
///
/// HISTORY, kept because it is why the seam changed. This test was
/// `the_seam_cannot_supply_a_calibration_so_a_warmed_host_is_refused`, then
/// `a_warmed_host_arms_only_with_a_non_identity_calibration`, which pinned
/// `calibration_present = core.calibration() != CalParams::identity()`. That
/// rule could not tell "never configured" from "calibrated upstream", and it
/// counted a NaN calibration as present. The component installs the
/// calibration with `set_calibration`, the call this test makes.
#[test]
fn a_warmed_host_arms_only_with_a_configured_valid_calibration() {
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

    /// Step, then try to arm; the deciding row (None = armed).
    fn try_arm(sup: &mut FlightSupervisor, b: &mut SeamHost, steps: u32) -> Option<CheckId> {
        for _ in 0..steps {
            sup.step(b);
        }
        sup.set_check(CheckId::FailsafeConfigured, true);
        sup.command(Event::Arm, true, true);
        let row = sup.arm_blocked_check();
        assert_eq!(
            sup.mode() == Mode::Armed,
            row.is_none(),
            "the mode and the table's verdict disagree (row {row:?}, mode {:?})",
            sup.mode()
        );
        row
    }

    let mut b = SeamHost;

    // 1. NEVER configured: refused, and by CalibrationPresent — if a
    //    DIFFERENT row refuses first, the cascade.wit account of what a host
    //    must send is stale.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 14.0);
    assert!(!sup.calibration_configured());
    assert_eq!(
        try_arm(&mut sup, &mut b, 1500),
        Some(CheckId::CalibrationPresent),
        "a warmed host that never configured a calibration must be refused"
    );

    // 2. An explicitly configured IDENTITY is a valid calibration: arms.
    assert!(
        sup.set_calibration(CalParams::identity()),
        "identity is valid"
    );
    assert!(sup.calibration_configured());
    assert_eq!(
        try_arm(&mut sup, &mut b, 10),
        None,
        "an explicitly configured IDENTITY must allow arming"
    );

    // 3. An INVALID calibration on a fresh, warmed supervisor that HAD a valid
    //    one: not installed, the earlier one revoked, refused.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 14.0);
    assert!(sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    }));
    let bad = CalParams {
        mag_offdiag: [f32::NAN, 0.0, 0.0],
        ..CalParams::identity()
    };
    assert!(
        !sup.set_calibration(bad),
        "an invalid calibration was installed"
    );
    assert!(
        !sup.calibration_configured(),
        "the valid one was not revoked"
    );
    assert_eq!(
        sup.calibration(),
        CalParams::identity(),
        "an invalid calibration must leave the identity applied, never itself"
    );
    assert_eq!(
        try_arm(&mut sup, &mut b, 1500),
        Some(CheckId::CalibrationPresent),
        "an invalid calibration must refuse arming"
    );

    // 4. THE CONTROL — a valid non-identity calibration on the same warmed
    //    supervisor arms, attributing step 3's refusal to the calibration.
    assert!(sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    }));
    assert_eq!(try_arm(&mut sup, &mut b, 10), None, "the control must arm");
}

/// #609 finding 6 — bit 7 (`ARM_BLOCKED`) must report the verdict of the table
/// `command(Arm)` actually gates on, while Disarmed, and nothing once armed.
mod arm_blocked_bit {
    use super::ff;
    use falcon_core::FlightSupervisor;
    use falcon_core::relay_fsm::{Event, Mode};
    use falcon_core::relay_preflight::CheckId;

    fn sup() -> FlightSupervisor {
        FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0)
    }

    #[test]
    fn motors_healthy_refusal_sets_bit_7() {
        let mut s = sup();
        s.set_check(CheckId::MotorsHealthy, false);
        s.command(Event::Arm, true, true);
        assert_eq!(s.mode(), Mode::Disarmed);
        assert_eq!(s.arm_blocked_check(), Some(CheckId::MotorsHealthy));
        assert_ne!(s.failsafe_flags() & ff::ARM_BLOCKED, 0, "bit 7 must be set");
    }

    #[test]
    fn failsafe_configured_refusal_sets_bit_7() {
        let mut s = sup();
        s.set_check(CheckId::FailsafeConfigured, false);
        s.command(Event::Arm, true, true);
        assert_eq!(s.mode(), Mode::Disarmed);
        assert_eq!(s.arm_blocked_check(), Some(CheckId::FailsafeConfigured));
        assert_ne!(s.failsafe_flags() & ff::ARM_BLOCKED, 0, "bit 7 must be set");
    }

    #[test]
    fn disarmed_with_every_row_passing_leaves_bit_7_clear() {
        let mut s = sup();
        assert_eq!(s.arm_blocked_check(), None, "control: the table allows");
        assert_eq!(s.failsafe_flags() & ff::ARM_BLOCKED, 0);
        s.command(Event::Arm, true, true);
        assert_eq!(s.mode(), Mode::Armed, "control: arm really is allowed");
    }

    /// A supervisor with NO calibration configured. `step` runs
    /// `update_preflight`, which marks the LEGACY struct `calibration_present =
    /// false`; on main that held bit 7 set on every tick while armed and
    /// airborne. Armed, the bit must be clear.
    #[test]
    fn armed_airborne_without_a_configured_calibration_leaves_bit_7_clear() {
        use falcon_core::{FlightBackend, ImuSample};
        struct Host;
        impl FlightBackend for Host {
            fn read_imu(&mut self) -> ImuSample {
                ImuSample {
                    accel: [0.0, 0.0, -9.80665],
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
            fn read_battery_v(&mut self) -> Option<f32> {
                Some(16.8)
            }
            fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
                Some([3000; 4])
            }
        }
        let mut s = sup();
        let mut b = Host;
        s.command(Event::Arm, true, true);
        assert_eq!(s.mode(), Mode::Armed);
        s.command(Event::RequestTakeoff, true, true);
        assert_eq!(s.mode(), Mode::Takeoff);
        for _ in 0..20 {
            s.step(&mut b);
        }
        assert!(
            matches!(s.mode(), Mode::Takeoff | Mode::Loiter | Mode::Armed),
            "still flying, got {:?}",
            s.mode()
        );
        // Precondition that makes this the component's shape: the legacy
        // struct DOES fail (no calibration configured) while we are airborne.
        assert!(s.arm_blocked_reason().is_some(), "legacy struct fails");
        assert_eq!(s.failsafe_flags() & ff::ARM_BLOCKED, 0, "airborne");
        // Even a failing table row must not raise the bit once past the gate.
        s.set_check(CheckId::MotorsHealthy, false);
        assert_eq!(s.failsafe_flags() & ff::ARM_BLOCKED, 0, "row failing");
    }
}
