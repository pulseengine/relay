extern crate std;

use super::*;

/// The SAME verified cascade, run through the HAL seam against the sim
/// backend, recovers a tilted body to level — demonstrating the flight
/// core is backend-agnostic (the seam carries the real IEKF + geometric +
/// ADRC + mixer, not a stub).
#[test]
fn flight_core_stabilizes_through_the_hal() {
    let dt = 0.002f32;
    // start tilted ~23° about x
    let th = 0.4f32;
    let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
    let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
    let mut backend = SimBackend::new(r0, dt);
    let mut core = FlightCore::new(0.5, 1.0 / dt);

    let tilt0 = backend.tilt();
    assert!(tilt0 > 0.35, "should start tilted, {tilt0}");
    for _ in 0..6000 {
        core.step(&mut backend);
    }
    let tilt = backend.tilt();
    assert!(
        tilt < 0.1,
        "core must recover to level through the HAL: {tilt} rad (start {tilt0})"
    );
}

/// The backend is a SEAM, not a fixed simulator: a trivial stand-in
/// backend (constant level IMU, no fixes) drives the core with zero panics
/// and bounded motors — i.e. any `FlightBackend` impl works.
/// v1.98 — the estimator CONSUMES the calibration. A constant yaw-gyro bias
/// is unobservable without a heading reference, so it integrates into yaw
/// drift; with a calibration that cancels the bias the estimate does NOT
/// drift — so the calibrated and uncalibrated trajectories diverge. Proves the
/// calibration is in the estimation loop (replacing the identity placeholder),
/// not silently ignored.
#[test]
fn estimator_consumes_calibration() {
    use relay_calib::CalParams;
    struct BiasedGyro {
        bias_z: f32,
    }
    impl FlightBackend for BiasedGyro {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0, 0.0, self.bias_z],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            None
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
    }
    let bias = 0.2_f32; // rad/s yaw-gyro bias → ~0.2 rad yaw over 1 s

    let mut uncal = FlightCore::new(0.5, 250.0);
    let mut bu = BiasedGyro { bias_z: bias };
    for _ in 0..250 {
        uncal.step(&mut bu);
    }

    let mut cal = FlightCore::new(0.5, 250.0);
    cal.set_calibration(CalParams {
        gyro_bias: [0.0, 0.0, bias],
        ..CalParams::identity()
    });
    let mut bc = BiasedGyro { bias_z: bias };
    for _ in 0..250 {
        cal.step(&mut bc);
    }

    let qu = uncal.state().q;
    let qc = cal.state().q;
    let diff = (qu[0] - qc[0]).abs()
        + (qu[1] - qc[1]).abs()
        + (qu[2] - qc[2]).abs()
        + (qu[3] - qc[3]).abs();
    assert!(
        diff > 0.05,
        "calibration must change the estimate (yaw drift suppressed): diff {diff}"
    );
    // the installed calibration is reported back.
    assert_eq!(cal.calibration().gyro_bias, [0.0, 0.0, bias]);
}

/// v1.103 — the supervisor LANDS on a motor failure: an airborne vehicle that
/// loses a rotor (detected by the core's FDI) is commanded to Land (not RTL —
/// a 3-rotor quad has no yaw authority to navigate home).
#[test]
fn motor_failure_commands_land() {
    use relay_calib::CalParams;
    struct RpmBackend {
        last: [f32; 4],
        inject: bool,
    }
    impl FlightBackend for RpmBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, -2.0]) // at the 2 m cruise altitude
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn read_battery_v(&mut self) -> Option<f32> {
            Some(16.0)
        }
        fn write_motors(&mut self, m: &[f32]) {
            self.last = [m[0], m[1], m[2], m[3]];
        }
        fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
            let mut rpm = [0i32; 4];
            let mut i = 0;
            while i < 4 {
                rpm[i] = (self.last[i] * ESC_RPM_FULL) as i32;
                i += 1;
            }
            if self.inject {
                rpm[1] = 0; // rotor 1 dies
            }
            Some(rpm)
        }
        fn dt(&self) -> f32 {
            0.004
        }
    }
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 200.0, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    let mut b = RpmBackend {
        last: [0.5; 4],
        inject: false,
    };

    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(sup.mode(), relay_fsm::Mode::Armed);
    sup.command(relay_fsm::Event::RequestTakeoff, true, false);
    assert!(sup.mode().is_airborne(), "airborne after takeoff");

    // Fly past the FDI's SPIN-UP guard first: the warmup counts from the
    // FLIGHT loop's start (v1.117 — idle-mode steps are estimate-only and
    // no longer count; before, disarmed warm-up steps silently neutered
    // the spin-up guard). Then a rotor dies → isolation → Land.
    for _ in 0..250 {
        sup.step(&mut b);
    }
    b.inject = true;
    for _ in 0..40 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Land,
        "motor failure commands Land"
    );
}

/// v1.103 — the production FlightCore detects a dead rotor from ESC RPM and
/// reconfigures: healthy rotors are never isolated, but a rotor whose RPM
/// drops to 0 under a nonzero command is isolated by the CUSUM FDI within a
/// few cycles, after which the reconfigured allocator commands that rotor 0
/// and keeps the healthy three in [floor, 1] (MIX-P08). Deterministic — the
/// RPM is a direct backend input, not estimator-derived.
#[test]
fn fdi_isolates_dead_rotor_and_reconfigures() {
    struct RpmBackend {
        last: [f32; 4],
        failed: usize,
        inject: bool,
    }
    impl FlightBackend for RpmBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, -5.0]) // holding 5 m up
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn write_motors(&mut self, m: &[f32]) {
            self.last = [m[0], m[1], m[2], m[3]];
        }
        fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
            // reported RPM tracks the last command, except a dead rotor reads 0.
            let mut rpm = [0i32; 4];
            let mut i = 0;
            while i < 4 {
                rpm[i] = (self.last[i] * ESC_RPM_FULL) as i32;
                i += 1;
            }
            if self.inject {
                rpm[self.failed] = 0;
            }
            Some(rpm)
        }
        fn dt(&self) -> f32 {
            0.004
        }
    }
    let mut core = FlightCore::new(0.5, 250.0);
    core.set_altitude(-5.0); // hold 5 m → commands hover thrust to all rotors
    let mut b = RpmBackend {
        last: [0.5; 4],
        failed: 2,
        inject: false,
    };

    // healthy: the FDI never isolates a rotor.
    for _ in 0..100 {
        core.step(&mut b);
    }
    assert_eq!(
        core.failed_motor(),
        None,
        "healthy rotors must not be isolated"
    );

    // rotor 2 dies (RPM → 0 under a nonzero command) → isolated quickly.
    b.inject = true;
    for _ in 0..30 {
        core.step(&mut b);
    }
    assert_eq!(
        core.failed_motor(),
        Some(2),
        "the CUSUM FDI isolates the dead rotor"
    );

    // the reconfigured allocator commands the failed rotor 0 and the three
    // healthy rotors within [floor, 1] (MIX-P08 in the production loop).
    core.step(&mut b);
    assert_eq!(b.last[2], 0.0, "failed rotor commanded 0");
    for (i, &v) in b.last.iter().enumerate() {
        if i != 2 {
            assert!(
                (ROTOR_OUT_FLOOR - 1e-6..=1.0 + 1e-6).contains(&v),
                "healthy rotor {i} = {v}"
            );
        }
    }
}

/// v1.114 — the RECOVERY, not just the isolation: the production core flown
/// on the analytic rigid-body plant survives a real single-rotor loss
/// without flipping. This is the offline twin of the gz rotor-out bench —
/// before v1.114 the rotor-out mixer injected a parasitic moment ≈
/// collective toward the dead corner (proven by the mix-quad diagnostic),
/// tipping the vehicle monotonically past 90° into an inverted crash on gz.
/// The rank-3 allocation removes that moment, so the reduced-attitude law
/// keeps the thrust axis upright while the body spins freely in yaw.
#[test]
fn survives_single_rotor_failure_without_flipping() {
    let dt = 0.004f32;
    let r0 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]; // level
    let mut backend = SimBackend::new(r0, dt);
    // Realistic aero for a rotor-out: rotational drag bounds the yaw spin
    // (both surviving lift rotors share a spin sense), and ground contact
    // lets the controlled spin-descent settle instead of falling through.
    backend.rot_drag = 0.02;
    backend.ground_contact = true;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-3.0); // hold ~3 m

    // Reach a settled, level hover first (the FDI's gate only isolates a
    // dead rotor from a near-level, low-rate state — you cannot diagnose
    // one mid-tumble).
    for _ in 0..3000 {
        core.step(&mut backend);
    }
    let hover_tilt = backend.tilt();
    assert!(
        hover_tilt < 0.1,
        "must reach a level hover first: {hover_tilt} rad"
    );
    assert_eq!(
        core.failed_motor(),
        None,
        "no false isolation while healthy"
    );

    // Rotor 0 dies. A 3-rotor quad cannot hover, so the honest recovery is
    // a controlled spin-DESCENT: hold the thrust axis near-level while the
    // vehicle comes down and lands. Track the worst tilt + yaw rate while
    // AIRBORNE, and capture the attitude at touchdown.
    backend.fail_rotor(0);
    let mut peak_tilt = 0.0f32;
    let mut peak_yaw = 0.0f32;
    let mut touchdown_tilt: Option<f32> = None;
    for _ in 0..2000 {
        core.step(&mut backend);
        let alt = -backend.pos[2];
        if alt > 0.15 {
            // airborne — this is the recovery flight we're grading
            peak_tilt = peak_tilt.max(backend.tilt());
            peak_yaw = peak_yaw.max(relay_math::fabsf(backend.omega[2]));
        } else if touchdown_tilt.is_none() {
            touchdown_tilt = Some(backend.tilt()); // attitude as it lands
        }
    }

    assert_eq!(
        core.failed_motor(),
        Some(0),
        "FDI must isolate the dead rotor"
    );
    // The whole point: the thrust axis never tips anywhere near inverted. A
    // pre-v1.114 run blew through 90° to ~180° (parasitic-moment flip); the
    // rank-3 allocation holds it near-level throughout the descent (the
    // trajectory stayed < ~6° in steady descent, with a brief startup
    // transient).
    assert!(
        peak_tilt < 0.5,
        "vehicle must stay near-level after rotor-out: peak tilt {peak_tilt} \
         rad ({} deg)",
        peak_tilt * 180.0 / core::f32::consts::PI,
    );
    // Yaw is relinquished, so the body spins — but rotational drag bounds
    // it (an unbounded spin corrupts the estimator). A few rad/s, not tens.
    assert!(
        peak_yaw < 10.0,
        "rotor-out yaw spin must stay bounded: peak {peak_yaw} rad/s",
    );
    // It descended and LANDED (a 3-rotor quad must come down), arriving
    // near-upright — a controlled spin-landing, not a sideways crash.
    let td = touchdown_tilt.expect("vehicle must descend and land");
    assert!(td < 0.35, "must land near-upright: touchdown tilt {td} rad");
}

#[test]
fn arbitrary_backend_drives_the_core() {
    struct NullBackend {
        motors: [f32; 4],
    }
    impl FlightBackend for NullBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            None
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn write_motors(&mut self, motors: &[f32]) {
            for (d, &s) in self.motors.iter_mut().zip(motors.iter()) {
                *d = s;
            }
        }
        fn dt(&self) -> f32 {
            0.001
        }
    }
    let mut b = NullBackend { motors: [0.0; 4] };
    let mut core = FlightCore::new(0.5, 1000.0);
    for _ in 0..100 {
        core.step(&mut b);
    }
    for &m in &b.motors {
        assert!((0.0..=1.0).contains(&m), "motor out of range: {m}");
    }
}

// ── v1.2 altitude hold + disturbance rejection ───────────────────────

/// The backend-agnostic core climbs to and holds a commanded altitude
/// through the HAL (the thrust/altitude loop, decoupled from tilt).
#[test]
fn altitude_hold_climbs_to_setpoint() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0); // 2 m up (NED z negative)
    for _ in 0..15000 {
        core.step(&mut backend);
    }
    assert!(
        (backend.pos[2] + 2.0).abs() < 0.25,
        "altitude must reach −2 m: {}",
        backend.pos[2]
    );
    assert!(
        backend.tilt() < 0.1,
        "should stay level while holding altitude: {}",
        backend.tilt()
    );
}

/// v1.3 — full 6-DoF: the backend-agnostic core flies to and holds a
/// horizontal position setpoint through the HAL (tilts to translate, the
/// estimator + position loop bring it home). Honest scope: the SimBackend
/// uses the near-hover accelerometer model; the realistic specific-force +
/// acceleration-compensated path is exercised in the gz bench (v0.30–35).
#[test]
fn position_hold_flies_to_setpoint() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([2.0, -1.5, -2.0]); // 2 m N, 1.5 m W, 2 m up
    for _ in 0..30000 {
        core.step(&mut backend);
    }
    let e = [
        backend.pos[0] - 2.0,
        backend.pos[1] + 1.5,
        backend.pos[2] + 2.0,
    ];
    let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    assert!(
        err < 0.5,
        "must reach the position setpoint through the HAL: {err} m, pos {:?}",
        backend.pos
    );
    assert!(
        backend.tilt() < 0.15,
        "settle near level: {} rad",
        backend.tilt()
    );
}

/// The verified ADRC inner loop REJECTS a sustained body-torque
/// disturbance and holds the body near level — through the HAL. The ESO
/// estimates and cancels the disturbance, so the tilt stays bounded.
#[test]
fn disturbance_rejected_holds_level() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    backend.disturbance = [0.25, -0.15, 0.0]; // sustained roll+pitch disturbance
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    let mut peak_after = 0.0f32;
    for k in 0..8000 {
        core.step(&mut backend);
        if k > 4000 {
            let t = backend.tilt();
            if t > peak_after {
                peak_after = t;
            }
        }
    }
    // after the ESO converges, the disturbance is cancelled and the body
    // holds near level (a plain proportional loop would sit at a steady
    // offset; ADRC drives it out).
    assert!(
        peak_after < 0.12,
        "ESO must reject the disturbance: steady tilt {peak_after} rad"
    );
}

// ── v1.8 supervisor: geofence→RTL actuation + battery failsafe ────────

/// A geofence breach ACTUATES a return-to-launch (the audit's gap: the
/// geofence detected but never commanded the vehicle home). Commanded to a
/// mission point OUTSIDE the fence, the drone crosses it, the supervisor
/// fires Failsafe → the FSM commands RTL → it flies home and lands — never
/// reaching the out-of-bounds target.
#[test]
fn geofence_breach_actuates_rtl_home() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.5, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "should reach Loiter after takeoff"
    );
    sup.set_mission([4.0, 0.0, -2.0]); // OUTSIDE the 1.5 m fence
    sup.command(Event::RequestMission, true, false);
    for _ in 0..40000 {
        sup.step(&mut backend);
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    let dh = relay_math::sqrtf(backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1]);
    assert!(
        dh < 1.0,
        "RTL must bring it home, not to [4,0]: horiz {dh} m, pos {:?}",
        backend.pos
    );
    assert!(
        matches!(sup.mode(), Mode::Land | Mode::Disarmed),
        "RTL should be landing/landed, mode {:?}",
        sup.mode()
    );
}

/// v1.30 — a multi-leg mission: the supervisor flies a stored sequence of
/// waypoints IN ORDER, then autonomously returns home, lands, and disarms.
/// A full autonomous sortie with no per-waypoint commanding.
#[test]
fn flies_a_multi_waypoint_mission_then_returns_and_disarms() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true; // so the autonomous landing can settle
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 1.0);

    // A three-leg path (NED, 2 m AGL) that is NOT in straight-line order, so
    // visiting in sequence is observable.
    let wps = [[3.0, 0.0, -2.0], [3.0, 3.0, -2.0], [0.0, 3.0, -2.0]];
    sup.set_mission_waypoints(&wps);
    assert_eq!(sup.waypoint_count(), 3);

    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "should reach Loiter after takeoff"
    );
    sup.command(Event::RequestMission, true, false);

    // Fly the mission. Track the closest approach to each waypoint and the
    // order the sequencer advanced through the legs.
    let mut min_d = [f32::MAX; 3];
    let mut last_index = 0usize;
    let mut order_ok = true;
    let mut disarmed = false;
    for _ in 0..150000 {
        sup.step(&mut b);
        let p = sup.state().p;
        for (i, w) in wps.iter().enumerate() {
            let dx = p[0] - w[0];
            let dy = p[1] - w[1];
            let dz = p[2] - w[2];
            let d = relay_math::sqrtf(dx * dx + dy * dy + dz * dz);
            if d < min_d[i] {
                min_d[i] = d;
            }
        }
        let idx = sup.waypoint_index();
        if idx < last_index {
            order_ok = false; // the leg index must never go backwards
        }
        last_index = idx;
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }

    for (i, d) in min_d.iter().enumerate() {
        assert!(
            *d < WAYPOINT_RADIUS + 0.2,
            "waypoint {i} not visited: min dist {d} m"
        );
    }
    assert!(
        order_ok,
        "waypoints must be flown in order (leg index monotonic)"
    );
    assert!(
        disarmed,
        "mission must complete autonomously: return home + land + disarm (mode {:?})",
        sup.mode()
    );
}

/// v1.31 — keep-out (no-fly) zone avoidance: a mission whose straight path
/// runs through a no-fly zone is still flown to completion, but the vehicle
/// ARCS AROUND the zone — its closest approach to the zone centre stays
/// outside the zone radius for the whole flight (out and back).
#[test]
fn mission_avoids_a_keepout_zone() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 1.0);

    // A single far waypoint straight across a zone that sits on the path.
    sup.set_mission_waypoints(&[[10.0, 0.0, -2.0]]);
    let zone = KeepoutZone {
        center: [5.0, 0.0, -2.0],
        radius: 2.0,
    };
    sup.set_keepout_zones(&[zone]);

    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    sup.command(Event::RequestMission, true, false);

    let mut min_zone = f32::MAX; // closest approach to the zone centre
    let mut min_wp = f32::MAX; // closest approach to the waypoint
    let mut disarmed = false;
    for _ in 0..260000 {
        sup.step(&mut b);
        let p = sup.state().p;
        let dz = relay_math::sqrtf((p[0] - 5.0) * (p[0] - 5.0) + (p[1] - 0.0) * (p[1] - 0.0));
        if dz < min_zone {
            min_zone = dz;
        }
        let dw = relay_math::sqrtf((p[0] - 10.0) * (p[0] - 10.0) + p[1] * p[1]);
        if dw < min_wp {
            min_wp = dw;
        }
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    // visited the far waypoint (so it really crossed the obstacle field) …
    assert!(
        min_wp < WAYPOINT_RADIUS + 0.3,
        "waypoint not reached: min dist {min_wp} m"
    );
    // … but never entered the no-fly zone …
    assert!(
        min_zone > zone.radius,
        "entered the keep-out zone: min dist {min_zone} m < {} m",
        zone.radius
    );
    // … and still completed the sortie autonomously.
    assert!(
        disarmed,
        "mission with avoidance must still complete (mode {:?})",
        sup.mode()
    );
}

/// A low battery actuates a failsafe (the audit's "no battery failsafe"):
/// while loitering, the pack sags below threshold → Failsafe → RTL.
#[test]
fn low_battery_actuates_failsafe() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    backend.battery_v = 13.2; // sag below the 14 V threshold
    // 4 s: past the estimator's 2 s sustained-excursion debounce (a raw
    // instantaneous compare would fire in one step — BATTERY-P02 requires
    // the sustained excursion so a transient can never false-trigger).
    for _ in 0..2000 {
        sup.step(&mut backend);
    }
    assert!(
        matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed),
        "low battery must trigger a failsafe recovery, mode {:?}",
        sup.mode()
    );
}

/// **A rig that never had a battery sense must not be failsafed for it.**
/// The in-flight silence rule is declare-on-first-sight: the SITL and HITL
/// backends report `None` always, and the first version of that rule
/// grounded them one second after takeoff — measured on the gz bench's
/// `supervised-rotorout`, which went to `Land` at 1.0 s and `Disarmed` at
/// 1.6 s, long before the rotor it was testing even failed. Refusing to
/// ARM without a pack is the pre-arm gate's job (#413); the air is not the
/// place to discover it.
#[test]
fn a_vehicle_that_never_had_a_battery_sense_is_not_failsafed_in_flight() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    backend.battery_lost = true; // no reading, ever — as the SITL backends do
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend); // 16 s, far past the 1 s silence window
    }
    assert!(
        matches!(sup.mode(), Mode::Takeoff | Mode::Loiter),
        "a rig with no battery sense must keep flying, mode {:?}",
        sup.mode()
    );
}

/// **Losing the voltage sense in flight must raise the failsafe** — the
/// in-flight half of "absence is not health" (#413 closed the pre-arm
/// half). A pack that stops reporting is not a healthy pack: the estimator
/// stops being updated, so its low/critical latches freeze at whatever
/// they last saw, and without this the vehicle flies on until the pack is
/// flat with no failsafe ever raised. Found in the v1.139 candidate review.
#[test]
fn a_vehicle_that_never_had_a_battery_sense_does_not_report_a_silent_pack() {
    use relay_fsm::Event;
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    backend.battery_lost = true; // no reading, EVER — as the SITL backends do
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend); // 16 s — `batt_missing_s` reaches ~16, far
        // past the 1 s window, so ONLY `batt_seen`
        // keeps BATT_SILENT clear.
    }
    // AN EARLIER VERSION OF THIS TEST USED A FRESH SUPERVISOR AND WAS
    // VACUOUS: `batt_missing_s` is 0 before any step, so `0 > 1.0` is false
    // whatever the guard does, and dropping `batt_seen &&` would not have
    // moved it. Flying without a sense is what exercises the guard.
    assert_eq!(
        sup.failsafe_flags() & failsafe_flags::BATT_SILENT,
        0,
        "a vehicle that NEVER had a battery sense must not report BATT_SILENT \
         after 16 s of silence — #413 shipped absence-as-healthy, #475 \
         corrected it to declare-on-first-sight, and the observer must honour \
         the same distinction. flags {:#010b}",
        sup.failsafe_flags()
    );
}

#[test]
fn a_pack_that_goes_silent_in_flight_is_reported_as_the_cause_not_just_the_outcome() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    assert_eq!(sup.mode(), Mode::Loiter, "airborne with a healthy pack");
    assert_eq!(
        sup.failsafe_flags() & failsafe_flags::BATT_SILENT,
        0,
        "a reporting pack is not silent"
    );

    backend.battery_lost = true;
    for _ in 0..2000 {
        sup.step(&mut backend); // 4 s, well past BATT_SILENCE_S
    }

    let bits = sup.failsafe_flags();
    // The CAUSE is readable, which is the whole point: `mode()` says Rtl or
    // Land, but only this says WHY. Before `failsafe_flags` a telemetry
    // consumer saw the vehicle divert and could not tell a flat pack from a
    // geofence breach from a wind failsafe.
    assert_ne!(
        bits & failsafe_flags::BATT_SILENT,
        0,
        "a pack silent for 4 s must report BATT_SILENT, got {bits:#010b}"
    );
    assert_ne!(
        bits & failsafe_flags::RTL_LATCHED,
        0,
        "and the supervisor latched a recovery, got {bits:#010b}"
    );
    assert!(
        matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed),
        "outcome and cause must agree, mode {:?}",
        sup.mode()
    );
}

#[test]
fn losing_the_battery_sense_in_flight_actuates_failsafe() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    assert_eq!(sup.mode(), Mode::Loiter, "airborne with a healthy pack");

    // The ADC/harness fails: no reading at all from here on.
    backend.battery_lost = true;
    for _ in 0..2000 {
        sup.step(&mut backend); // 4 s
    }
    assert!(
        matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed),
        "a lost voltage sense must trigger a failsafe recovery, mode {:?}",
        sup.mode()
    );
}

/// PREARM-P03 integration criterion: a SITL cold boot whose parameter
/// store fell back to defaults (blank NVM) CANNOT arm — with the
/// distinct operator reason — and after a save + genuine NVM load it
/// can. Runs the REAL PARAM-P03 persistence seam (two-slot FRAM image
/// over the ArrayNvm mock), not a hand-set flag.
#[test]
fn defaults_fallback_params_block_arming() {
    use relay_param::ParamStore;
    use relay_param::persist::{ArrayNvm, Layout, LoadOutcome, load, save};
    use relay_preflight::CheckId;

    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..relay_calib::CalParams::identity()
    });

    // COLD BOOT: blank NVM → defaults-fallback; the integration reports
    // the load outcome into the check table.
    let mut nvm: ArrayNvm<2048> = ArrayNvm::new();
    let layout = Layout::new(16);
    let mut store: ParamStore<8> = ParamStore::new();
    assert!(falcon_core_tuning_register(&mut store));
    let rep = load(&mut store, &nvm, layout, 1);
    assert_eq!(rep.outcome, LoadOutcome::FreshDefaults);
    sup.set_check(CheckId::ParamsFromNvm, rep.outcome == LoadOutcome::Loaded);

    for _ in 0..2000 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Disarmed,
        "defaults-fallback must not arm"
    );
    assert_eq!(sup.arm_blocked_check(), Some(CheckId::ParamsFromNvm));
    assert_eq!(
        CheckId::ParamsFromNvm.reason_text(),
        "PREARM: params are defaults (no NVM)"
    );

    // field save + reboot-reload: a GENUINE NVM image now loads.
    save(&store, &mut nvm, layout, 1).expect("save");
    let mut store2: ParamStore<8> = ParamStore::new();
    assert!(falcon_core_tuning_register(&mut store2));
    let rep2 = load(&mut store2, &nvm, layout, 1);
    assert_eq!(rep2.outcome, LoadOutcome::Loaded);
    sup.set_check(CheckId::ParamsFromNvm, rep2.outcome == LoadOutcome::Loaded);
    sup.step(&mut b);
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Armed,
        "arms after the NVM load"
    );
}

/// PART-P02 (a): the F100 pass-through conformance fixture for gale#65.
/// A DETERMINISTIC per-motor command stream from the REAL mixer —
/// nominal hover, a saturated attitude transient, and the rank-3
/// rotor-out asymmetric sets including legitimately ZERO motors — with
/// f32 bit patterns, because pass-through fidelity is byte-exact:
/// gust must forward these unmodified (no re-mix, no per-motor floor;
/// an independent allocation authority downstream would reintroduce
/// the v1.114 parasitic-moment failure class). This test is the drift
/// gate: it regenerates the stream and asserts it matches the
/// checked-in fixture gale's tests consume.
#[test]
fn f100_passthrough_fixture_is_current() {
    let generated = gen_f100_fixture();
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../bench-evidence/fixtures/f100-passthrough-v1.csv"
    );
    let on_disk = std::fs::read_to_string(path)
        .expect("fixture missing — run the ignored regen test")
        .replace("\r\n", "\n"); // Windows checkout normalization (belt to the .gitattributes eol=lf)
    assert_eq!(
        generated, on_disk,
        "f100 pass-through fixture drifted from the real mixer output — \
         regen deliberately (ignored test) and coordinate gale#65"
    );
}

#[test]
#[ignore]
fn regen_f100_fixture() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../bench-evidence/fixtures/f100-passthrough-v1.csv"
    );
    std::fs::write(path, gen_f100_fixture()).unwrap();
}

fn gen_f100_fixture() -> std::string::String {
    use std::fmt::Write as _;
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.rot_drag = 0.02;
    b.ground_contact = true;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-3.0);
    let mut out = std::string::String::from(
        "# F100/gust pass-through conformance fixture (PART-P02, falcon v1.124)\n\
         # phase,m0_bits,m1_bits,m2_bits,m3_bits  — f32 bit patterns (hex);\n\
         # EXPECTED F100 OUTPUT == INPUT, byte-exact (no re-mix, no floors).\n",
    );
    let cap = |core: &mut FlightCore,
               b: &mut SimBackend,
               phase: &str,
               n: usize,
               every: usize,
               out: &mut std::string::String| {
        for k in 0..n {
            core.step(b);
            if k % every == 0 {
                let m = core.casc.last_motor_cmd;
                writeln!(
                    out,
                    "{phase},{:08x},{:08x},{:08x},{:08x}",
                    m[0].to_bits(),
                    m[1].to_bits(),
                    m[2].to_bits(),
                    m[3].to_bits()
                )
                .unwrap();
            }
        }
    };
    cap(&mut core, &mut b, "hover", 3000, 300, &mut out);
    // saturated transient: a hard lateral setpoint step
    core.set_position([6.0, -4.0, -3.0]);
    cap(&mut core, &mut b, "saturated", 1200, 100, &mut out);
    // rotor-out: the asymmetric rank-3 sets incl. the legitimate zeros
    b.fail_rotor(0);
    cap(&mut core, &mut b, "rotor_out", 2400, 100, &mut out);
    out
}

/// PART-P02 (b): the SOFTWARE half of the silence safe-state — the
/// Kani-proven FSM Terminate reaches motors-off actuation within the
/// proposed 100 ms bound (the F100's own heartbeat-timeout backstop
/// for a silent M7 is gale's evidence, gale#65/TEST-PIX-019).
#[test]
fn terminate_reaches_motors_off_within_bound() {
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(relay_fsm::Event::Arm, true, true);
    sup.command(relay_fsm::Event::RequestTakeoff, true, true);
    for _ in 0..4000 {
        sup.step(&mut b);
    }
    assert!(sup.mode() != relay_fsm::Mode::Disarmed, "airborne first");
    sup.command(relay_fsm::Event::Terminate, false, false);
    let bound_ticks = (0.100 / dt) as usize; // 100 ms
    let mut off_at = None;
    for k in 0..bound_ticks {
        sup.step(&mut b);
        if b.last_collective == 0.0 {
            off_at = Some(k);
            break;
        }
    }
    let k = off_at.expect("motors must reach OFF within 100 ms of Terminate");
    assert!(
        (k as f32 + 1.0) * dt <= 0.100,
        "motors-off took {} s",
        (k as f32 + 1.0) * dt
    );
}

/// PART-P01 equivalence oracle: the PARTITIONED core at ZERO transport
/// delay reproduces the fused FlightCore trajectory f32-BIT-EXACTLY on
/// the analytic plant — same inputs, same motor stream, same estimate,
/// step for step. (The golden-log replay separately pins the fused
/// core itself to the pre-partition v1.118 fixture.)
#[test]
fn partitioned_zero_delay_is_bit_exact_with_fused() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b1 = SimBackend::new(level, dt);
    b1.path.gps_noise = 0.08; // identical RNG streams: same seed, same draws
    let mut b2 = SimBackend::new(level, dt);
    b2.path.gps_noise = 0.08;
    let mut fused = FlightCore::new(0.5, 1.0 / dt);
    fused.set_altitude(-3.0);
    let mut part = PartitionedCore::new(0.5, 1.0 / dt, 0, false, 1);
    part.set_altitude(-3.0);
    for k in 0..6000 {
        fused.step(&mut b1);
        part.step(&mut b2);
        let (e1, e2) = (fused.state(), part.state());
        for i in 0..3 {
            assert_eq!(
                e1.p[i].to_bits(),
                e2.p[i].to_bits(),
                "estimate diverged at tick {k} axis {i}"
            );
        }
        for m in 0..4 {
            assert_eq!(
                b1.motor_state[m].to_bits(),
                b2.motor_state[m].to_bits(),
                "motors diverged at tick {k} rotor {m}"
            );
        }
    }
}

/// PART-P01 delay robustness (the requirement's teeth): with NavState
/// delayed 1 and 2 ticks and jittered ±1 tick across the transport
/// hop, the vehicle holds hover within the existing envelopes —
/// dispersed over noise seeds (measured, not asserted).
#[test]
fn partitioned_delay_holds_hover() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for delay in [1usize, 2] {
        for (t, jitter) in [(11u32, false), (29, true), (47, true)] {
            let mut b = SimBackend::new(level, dt);
            b.path.gps_noise = 0.08;
            b.path.gyro_white = 0.004;
            let mut core = PartitionedCore::new(0.5, 1.0 / dt, delay, jitter, t);
            core.set_altitude(-3.0);
            let mut worst_tilt = 0.0f32;
            for k in 0..15000 {
                core.step(&mut b);
                if k > 5000 {
                    worst_tilt = worst_tilt.max(b.tilt());
                    let alt_err = (-b.pos[2] - 3.0).abs();
                    assert!(
                        alt_err < 1.0,
                        "delay {delay} jitter {jitter} seed {t}: altitude ran away ({alt_err} m)"
                    );
                }
            }
            assert!(
                worst_tilt < 0.25,
                "delay {delay} jitter {jitter} seed {t}: settled tilt {worst_tilt} rad"
            );
        }
    }
}

/// PART-P01 delay robustness under FAULT: the rotor-out recovery
/// envelope (near-level thrust axis, upright touchdown — the v1.114
/// bounds) holds with the estimator output delayed 2 ticks and
/// jittered — the transport hop must not reintroduce the flip.
#[test]
fn partitioned_delay_rotor_out_recovers() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for (seed, jitter) in [(3u32, true), (17, true), (31, false)] {
        let mut b = SimBackend::new(level, dt);
        b.rot_drag = 0.02;
        b.ground_contact = true;
        let mut core = PartitionedCore::new(0.5, 1.0 / dt, 2, jitter, seed);
        core.set_altitude(-3.0);
        for _ in 0..3000 {
            core.step(&mut b);
        }
        assert!(b.tilt() < 0.1, "seed {seed}: hover first");
        b.fail_rotor(0);
        let mut peak_tilt = 0.0f32;
        let mut touchdown_tilt = None;
        for _ in 0..4000 {
            core.step(&mut b);
            let alt = -b.pos[2];
            if alt > 0.15 {
                peak_tilt = peak_tilt.max(b.tilt());
            } else if touchdown_tilt.is_none() {
                touchdown_tilt = Some(b.tilt());
            }
        }
        assert!(
            peak_tilt < 0.8,
            "seed {seed} jitter {jitter}: thrust axis must stay recoverable ({peak_tilt} rad)"
        );
        let td = touchdown_tilt.expect("must land");
        assert!(td < 0.35, "seed {seed}: upright touchdown ({td} rad)");
    }
}

/// NOTCH-P01 closed-loop probe — DISABLED pending root-cause (#290).
///
/// This asserted the notch reduces motor thrash under a vib=0.8 rotor
/// line while holding hover. The v1.125 f32-kernel work exposed that
/// the assertion is a NON-DETERMINISTIC oracle: the notch-on hover in
/// this analytic plant is chaotically fragile across the vib band, and
/// the flip is NOT kernel-specific — the baseline libm build flips
/// notch-OFF at vib=0.4/0.6, and both kernels flip notch-ON at several
/// injections (probe data in #290). RPM-tracking smoothing did NOT fix
/// it, so the mechanism is unknown — likely a notch-integration defect
/// the <10° crossover-phase unit test does not capture (transient /
/// full-bank stacking), NOT steady-state filtering. The ROBUST NOTCH-P01
/// evidence is the relay-notch frequency-domain suite (≥20 dB, sweep
/// within 3 dB, <10° phase, bit-exact bypass, line rejection), which is
/// unaffected. Do NOT re-enable until #290 is root-caused; do NOT lower
/// vib to green it.
#[test]
#[ignore = "non-deterministic oracle — see #290 (notch closed-loop fragility)"]
fn notch_reduces_rate_loop_noise_under_rotor_vibration() {
    fn hover_thrash(rpm_telemetry: bool) -> f32 {
        let dt = 0.004f32; // 250 Hz — the flight loop rate
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.rotor_vib = 0.8; // rad/s per motor line — a rough 1045 airframe
        b.rpm_telemetry = rpm_telemetry;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-3.0);
        for _ in 0..4000 {
            core.step(&mut b);
        }
        // measure: RMS deviation of motor 0's ACHIEVED state over 8 s of
        // settled hover — command thrash is the limit-cycle/vibration
        // symptom (#270's regression metric shape).
        let n = 2000usize;
        let mut sum = 0.0f64;
        let mut sum2 = 0.0f64;
        for _ in 0..n {
            core.step(&mut b);
            let m = b.motor_state[0] as f64;
            sum += m;
            sum2 += m * m;
        }
        let mean = sum / n as f64;
        let var = (sum2 / n as f64 - mean * mean).max(0.0);
        assert!(
            b.tilt() < 0.2,
            "hover must hold (telemetry={rpm_telemetry})"
        );
        (var.sqrt()) as f32
    }
    let with_notch = hover_thrash(true);
    let baseline = hover_thrash(false);
    assert!(
        with_notch < 0.6 * baseline,
        "tracking notch must cut motor thrash substantially: \
         with {with_notch:.4} vs bypass {baseline:.4}"
    );
}

/// GNSS-P02 failover: receiver A dies mid-hover — the estimator input
/// fails over within one fix interval with NO position-estimate step
/// beyond the healthy receiver's accuracy, and no estimator reset.
#[test]
fn dual_gnss_failover_holds_estimate() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.gnss_dual_enabled = true;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-3.0);
    for _ in 0..4000 {
        core.step(&mut b);
    }
    let before = core.state().p;
    b.gnss_a_down = true; // receiver A dies
    let mut max_step = 0.0f32;
    let mut prev = before;
    for _ in 0..1000 {
        core.step(&mut b);
        let p = core.state().p;
        let d = relay_math::sqrtf((p[0] - prev[0]).powi(2) + (p[1] - prev[1]).powi(2));
        max_step = max_step.max(d);
        prev = p;
    }
    assert!(
        max_step < 1.0,
        "failover must not step the estimate beyond B's accuracy: {max_step} m"
    );
    assert!(!core.nav_compromised(), "a clean failover is not a spoof");
}

/// GNSS-P02 divergence: receiver B walks 12 m off. The selector latches
/// divergence, the estimate stays with the truth-consistent receiver,
/// and PRE-ARM blocks (nav_compromised via sensors_healthy).
#[test]
fn gnss_divergence_latches_and_blocks_arming() {
    use relay_calib::CalParams;
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.gnss_dual_enabled = true;
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..2000 {
        sup.step(&mut b);
    }
    assert_eq!(sup.arm_blocked_check(), None, "healthy dual-GNSS arms");
    b.gnss_b_offset = [12.0, 0.0]; // B starts lying
    for _ in 0..2000 {
        sup.step(&mut b);
    }
    assert!(
        sup.core().gnss_diverged(),
        "sustained 12 m split must latch"
    );
    // The legacy struct reported this as `CheckFail::Sensors`; the table's
    // first failing row for a nav-integrity latch is `SensorsHealthy`.
    assert_eq!(
        sup.arm_blocked_check(),
        Some(relay_preflight::CheckId::SensorsHealthy),
        "divergence blocks arming"
    );
    let e = sup.core().state();
    assert!(
        relay_math::fabsf(e.p[0]) < 2.0,
        "estimate must stay with the consistent receiver: {} m",
        e.p[0]
    );
}

/// GNSS-P02 dispersion mini-campaign (CI-sized): 40 LCG-dispersed trials
/// over fault type (A dropout / A accuracy collapse / B divergence walk /
/// nominal) and onset. The divergence flag fires in 100% of walk trials
/// and 0% of the others; no trial steps the estimate discontinuously.
#[test]
fn dual_gnss_dispersion_campaign() {
    let mut lcg: u32 = 0xC0FFEE;
    let mut rnd = move || {
        lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
        (lcg >> 8) as f32 / 16777216.0
    };
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for trial in 0..40 {
        let fault = trial % 4; // 0 nominal, 1 A-drop, 2 A-acc-collapse, 3 B-walk
        let onset = 2000 + (rnd() * 1000.0) as usize;
        let mut b = SimBackend::new(level, dt);
        b.gnss_dual_enabled = true;
        b.path.gps_noise = 0.05 + rnd() * 0.1;
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_altitude(-3.0);
        let mut prev: Option<Vec3> = None;
        let mut max_step = 0.0f32;
        for k in 0..5000 {
            if k == onset {
                match fault {
                    1 => b.gnss_a_down = true,
                    2 => b.gnss_a_acc = 50.0, // over the health ceiling
                    3 => b.gnss_b_offset = [15.0, 0.0],
                    _ => {}
                }
            }
            core.step(&mut b);
            let p = core.state().p;
            if let Some(q) = prev {
                let d = relay_math::sqrtf((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2));
                max_step = max_step.max(d);
            }
            prev = Some(p);
        }
        assert!(
            max_step < 1.5,
            "trial {trial} fault {fault}: estimate stepped {max_step} m"
        );
        if fault == 3 {
            assert!(
                core.gnss_diverged(),
                "trial {trial}: walk must raise the flag"
            );
        } else {
            assert!(
                !core.gnss_diverged(),
                "trial {trial} fault {fault}: false flag"
            );
        }
    }
}

/// RANGEDRV-P01 landing assist: the baro lies by 2 m (reads HIGH — the
/// dangerous direction: the vehicle believes it is 2 m above its true
/// altitude and would fly into the ground). With the TF02 range anchor
/// the touchdown fires within 15 cm of TRUE ground.
#[test]
fn biased_baro_range_assisted_touchdown() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    // WITH the range anchor:
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true;
    b.baro_enabled = true;
    b.baro_bias = -2.0; // baro reads 2 m HIGHER than truth (NED z minus)
    b.gps_pos_bias = [0.0, 0.0, -2.0]; // GNSS vertical carries the same lie
    b.range_enabled = true;
    b.range_noise = 0.02;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-3.0);
    for _ in 0..4000 {
        core.step(&mut b);
    }
    core.set_landing(true);
    // Touchdown recognition = the landing law cutting to idle (the
    // terrain-relative AGL crossing 0.12), visible as the collective
    // dropping to 4×0.1. Record the TRUE altitude at that moment.
    let mut true_alt_at_touchdown = None;
    for _ in 0..20000 {
        core.step(&mut b);
        if b.last_collective < 0.45 {
            true_alt_at_touchdown = Some(-b.pos[2]);
            break;
        }
    }
    let ta = true_alt_at_touchdown.expect("must recognize touchdown");
    assert!(
        ta.abs() <= 0.15,
        "range-assisted touchdown must fire within 15 cm of true ground: {ta} m"
    );

    // WITHOUT range (the failure this slice fixes): the same biased baro
    // leaves the estimate ~2 m high — touchdown never triggers before
    // the vehicle is ground-clamped, i.e. it flies INTO the ground.
    let mut b2 = SimBackend::new(level, dt);
    b2.ground_contact = true;
    b2.baro_enabled = true;
    b2.baro_bias = -2.0;
    b2.gps_pos_bias = [0.0, 0.0, -2.0];
    let mut core2 = FlightCore::new(0.5, 1.0 / dt);
    core2.set_altitude(-3.0);
    for _ in 0..4000 {
        core2.step(&mut b2);
    }
    core2.set_landing(true);
    let mut touch_true_alt = None;
    for _ in 0..20000 {
        core2.step(&mut b2);
        if b2.last_collective < 0.45 {
            touch_true_alt = Some(-b2.pos[2]);
            break;
        }
    }
    match touch_true_alt {
        None => {} // never recognized touchdown — flew into the clamp: the failure
        Some(t) => assert!(
            t.abs() > 0.15,
            "without range the biased-vertical landing should NOT be clean ({t} m) — \
             if it is, the scenario no longer discriminates"
        ),
    }
}

/// RANGEDRV-P01 innovation gate: an obstacle overflight (sudden 4 m
/// short return) mid-descent must NOT fold the altitude estimate.
#[test]
fn obstacle_overflight_does_not_fold_altitude() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true;
    b.baro_enabled = true;
    b.range_enabled = true;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-8.0);
    for _ in 0..6000 {
        core.step(&mut b);
    }
    let est_before = -core.state().p[2];
    // fly over a 5 m obstacle for 0.5 s: the range suddenly reads ~3 m.
    b.range_fault = Some(3.0);
    for _ in 0..250 {
        core.step(&mut b);
    }
    assert!(
        core.range_agl().is_none(),
        "the innovation gate must REJECT the obstacle's short return"
    );
    let est_after = -core.state().p[2];
    assert!(
        (est_after - est_before).abs() < 0.5,
        "altitude estimate folded over the obstacle: {est_before} -> {est_after}"
    );
}

/// BATTERY-P02's sag criterion AT THE SUPERVISOR: a 60C punch on a
/// half-full pack sags the terminal voltage far below the raw threshold
/// (which would have fired the OLD `v < low_batt_v` failsafe) — the
/// sag-compensated estimate does NOT trip; the SAME terminal voltage
/// sustained at near-zero current (a genuinely spent, rebounding pack
/// reads even lower resting) DOES.
#[test]
fn sag_punch_does_not_trip_supervisor_failsafe() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    backend.battery_i = Some(20.0); // cruise draw, current sense present
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut backend);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    assert!(
        !sup.battery().degraded,
        "current sense present ⇒ compensated path"
    );

    // 4-second 300 A punch: terminal sags to 8.2 V — WAY below the 14 V
    // raw threshold that used to gate the failsafe directly.
    backend.battery_v = 3.85 * 4.0 - 300.0 * 0.024;
    backend.battery_i = Some(300.0);
    for _ in 0..2000 {
        sup.step(&mut backend);
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "sag must not false-trigger the failsafe"
    );

    // Same terminal voltage, near-zero current, sustained: nothing to
    // credit back — a pack genuinely THIS low at rest is an emergency.
    backend.battery_i = Some(1.0);
    for _ in 0..2000 {
        sup.step(&mut backend);
    }
    assert!(
        matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed),
        "the same voltage at rest must trip, mode {:?}",
        sup.mode()
    );
}

// ── v1.9 robustness: injected sensor pathologies through the HAL ──────
//
// The audit found the estimator was only ever shown against a perfect
// world. These drive the SAME verified cascade through the HAL with each
// pathology injected, and assert it HOLDS (bounded tilt / position) or
// degrades gracefully (recovers after a GPS dropout).

/// Broadband accelerometer vibration (1.5 m/s² per axis, ≈15 % of g):
/// altitude-holding, the verified IEKF's gravity update must reject the
/// zero-mean noise and keep the body near level through the HAL.
#[test]
fn holds_through_accelerometer_vibration() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt).with_pathology(Pathology {
        vibration: 1.5,
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut peak = 0.0f32;
    for k in 0..15000 {
        core.step(&mut backend);
        if k > 5000 {
            peak = peak.max(backend.tilt());
        }
    }
    assert!(
        peak < 0.15,
        "IEKF must reject accel vibration: peak tilt {peak} rad"
    );
    assert!(
        (backend.pos[2] + 2.0).abs() < 0.4,
        "altitude held under vibration: {}",
        backend.pos[2]
    );
}

/// A slow gyro bias drift (0.004 rad/s², ≈0.5°/s after 12 s): the IEKF's
/// gyro-bias state must track it, else the attitude walks off. Compares
/// against a clean run to show the bias is the only added error.
#[test]
fn iekf_tracks_gyro_bias_drift() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt).with_pathology(Pathology {
        gyro_bias_drift: 0.004,
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut peak = 0.0f32;
    for k in 0..15000 {
        core.step(&mut backend);
        if k > 8000 {
            peak = peak.max(backend.tilt());
        }
    }
    // injected bias reaches ≈0.004·30 = 0.12 rad/s; without bias estimation
    // the attitude would integrate that into a steady tilt. The IEKF holds.
    assert!(
        peak < 0.15,
        "IEKF gyro-bias state must track the drift: peak tilt {peak} rad"
    );
}

/// A GPS dropout mid-flight (2 s, steps 6000–7000): while holding position,
/// the fix vanishes → the IEKF dead-reckons → position drifts, but on the
/// fix's return the position update RE-CONVERGES. Graceful degradation, the
/// honest claim (not "no drift" — dead-reckoning on the near-hover accel
/// model does drift; the test asserts bounded drift + recovery).
#[test]
fn survives_gps_dropout_and_recovers() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt).with_pathology(Pathology {
        gps_dropout_start: 6000,
        gps_dropout_len: 1000, // 2 s without a fix
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([0.0, 0.0, -2.0]);
    // settle, then fly through the dropout window and well past it
    let mut peak_drift = 0.0f32;
    for k in 0..15000 {
        core.step(&mut backend);
        if (6000..8000).contains(&k) {
            let d = relay_math::sqrtf(
                backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1],
            );
            peak_drift = peak_drift.max(d);
        }
    }
    let final_d =
        relay_math::sqrtf(backend.pos[0] * backend.pos[0] + backend.pos[1] * backend.pos[1]);
    assert!(
        peak_drift < 2.0,
        "dropout drift must stay bounded: {peak_drift} m"
    );
    assert!(
        final_d < 0.5,
        "position must re-converge after the fix returns: {final_d} m"
    );
}

/// Magnetometer interference (0.3 of unit field per axis): the heading
/// update variance must tolerate it — the body holds level + altitude and
/// does not tumble under the corrupted yaw reference.
#[test]
fn tolerates_mag_interference() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt).with_pathology(Pathology {
        mag_interference: 0.3,
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut peak = 0.0f32;
    for k in 0..15000 {
        core.step(&mut backend);
        if k > 5000 {
            peak = peak.max(backend.tilt());
        }
    }
    assert!(
        peak < 0.15,
        "mag interference must not destabilise attitude: peak tilt {peak} rad"
    );
    assert!(
        (backend.pos[2] + 2.0).abs() < 0.4,
        "altitude held under mag interference: {}",
        backend.pos[2]
    );
}

// ── v1.11 — the real-hardware backend SEAM, composed end-to-end ───────

/// The verified flight core stabilises through the **`HardwareBackend`**
/// (the real-board seam), not just `SimBackend`. Five mock drivers — the
/// exact contracts a real board implements (IMU/GNSS/mag/ESC/battery) —
/// share one simulated plant through a `core::cell::RefCell`; each borrows
/// it only for its own call, so the motor write steps the physics that the
/// next IMU read observes. The closed loop runs entirely through the driver
/// traits, proving the seam carries a real control loop. The ONLY thing
/// that changes to fly a board is swapping these mocks for silicon drivers.
#[test]
fn flight_core_stabilizes_through_the_hardware_seam() {
    use core::cell::RefCell;

    struct MockImu<'a>(&'a RefCell<SimBackend>);
    impl ImuDriver for MockImu<'_> {
        fn read(&mut self) -> ImuSample {
            self.0.borrow_mut().read_imu()
        }
    }
    struct MockPos<'a>(&'a RefCell<SimBackend>);
    impl PositionDriver for MockPos<'_> {
        fn read(&mut self) -> Option<Vec3> {
            self.0.borrow_mut().read_position()
        }
    }
    struct MockMag<'a>(&'a RefCell<SimBackend>);
    impl MagDriver for MockMag<'_> {
        fn read(&mut self) -> Option<Vec3> {
            self.0.borrow_mut().read_mag()
        }
    }
    struct MockEsc<'a>(&'a RefCell<SimBackend>);
    impl MotorDriver for MockEsc<'_> {
        fn write(&mut self, motors: &[f32]) {
            self.0.borrow_mut().write_motors(motors) // steps the shared plant
        }
    }
    struct MockBatt; // uses the default healthy-pack voltage
    impl BatteryDriver for MockBatt {}

    let dt = 0.002f32;
    // start tilted ~23° about x — same plant as the SimBackend HAL test
    let th = 0.4f32;
    let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
    let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
    let plant = RefCell::new(SimBackend::new(r0, dt));

    let mut hw = HardwareBackend {
        imu: MockImu(&plant),
        gnss: MockPos(&plant),
        mag: MockMag(&plant),
        motors: MockEsc(&plant),
        battery: MockBatt,
        dt,
    };
    let mut core = FlightCore::new(0.5, 1.0 / dt);

    let tilt0 = plant.borrow().tilt();
    assert!(tilt0 > 0.35, "should start tilted, {tilt0}");
    for _ in 0..6000 {
        core.step(&mut hw); // the WHOLE loop runs through the driver traits
    }
    // The battery seam answers through the trait too — and with no driver
    // override it reports ABSENT, not a healthy pack. This assertion used to
    // require 16.0 V and so encoded the fail-unsafe default it was meant to
    // exercise (#413).
    assert!(
        hw.read_battery_v().is_none(),
        "a HardwareBackend with no battery driver must report absence, not health"
    );
    let tilt = plant.borrow().tilt();
    assert!(
        tilt < 0.1,
        "core must recover to level through the HARDWARE seam: {tilt} rad (start {tilt0})"
    );
}

// ── v1.16 wind: a translational FORCE disturbance the position loop must
// reject (the ESO handles torque, not this). The integral term is the fix.

/// Hold position at the origin under a given wind for N steps; return the
/// final horizontal distance from home. `ki=Some(0.0)` ⇒ P-D only.
fn fly_under_wind(wind: Vec3, gust: f32, ki: Option<f32>, steps: usize) -> f32 {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.wind = wind;
    b.gust_amp = gust;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    if let Some(k) = ki {
        core.set_position_integral_gain(k);
    }
    core.set_position([0.0, 0.0, -2.0]);
    for _ in 0..steps {
        core.step(&mut b);
    }
    relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1])
}

/// The P-I-D position loop REJECTS a steady 5 m/s wind — the integral winds
/// up to counter the constant force, returning near the setpoint.
#[test]
fn holds_position_under_steady_wind() {
    let d = fly_under_wind([3.0, 4.0, 0.0], 0.0, None, 30000); // 5 m/s
    assert!(d < 0.6, "P-I-D must reject steady wind: {d} m from home");
}

/// FALSIFICATION (the honest motivation): with the integral DISABLED
/// (ki=0, P-D only) the same wind leaves a much larger steady offset — a
/// P-D loop cannot reject a constant force. This is why v1.16 adds the
/// integral, and the integral must cut the offset by more than half.
#[test]
fn bare_pd_loop_offsets_under_wind() {
    let d_pd = fly_under_wind([3.0, 4.0, 0.0], 0.0, Some(0.0), 30000);
    let d_pid = fly_under_wind([3.0, 4.0, 0.0], 0.0, None, 30000);
    assert!(d_pd > 1.5, "P-D alone should offset under wind: {d_pd} m");
    assert!(
        d_pid < d_pd * 0.5,
        "the integral must cut the offset: PID {d_pid} m vs PD {d_pd} m"
    );
}

/// Gusts on top of the steady wind stay bounded — the integral tracks the
/// mean, the loop rides out the deterministic broadband gust.
#[test]
fn rejects_wind_gusts() {
    let d = fly_under_wind([3.0, 0.0, 0.0], 2.0, None, 30000); // 3 m/s + 2 m/s gusts
    assert!(
        d < 1.0,
        "P-I-D must keep gusty wind bounded: {d} m from home"
    );
}

// ── v1.17 aerodynamic drag (quadratic, ∝ v²) ──────────────────────────

/// The position loop TRACKS a far setpoint through realistic quadratic
/// aerodynamic drag — the drag opposes the transit and caps top speed, but
/// the P-I-D overcomes it and settles on target.
#[test]
fn tracks_setpoint_through_aerodynamic_drag() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.drag_quad = 0.05;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([4.0, 0.0, -2.0]);
    for _ in 0..40000 {
        core.step(&mut b);
    }
    let e = [b.pos[0] - 4.0, b.pos[1], b.pos[2] + 2.0];
    let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    assert!(err < 0.6, "must track the setpoint through drag: {err} m");
}

/// Quadratic drag DAMPS the vehicle: a horizontally-kicked drone sheds the
/// kick faster WITH drag than without — the peak excursion is smaller.
/// Demonstrates the v² damping physics (drag is also a stabilising force).
#[test]
fn quadratic_drag_damps_a_kick() {
    let run = |cd: f32| -> f32 {
        let dt = 0.002f32;
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt);
        b.drag_quad = cd;
        b.vel = [4.0, 0.0, 0.0]; // a 4 m/s horizontal kick
        let mut core = FlightCore::new(0.5, 1.0 / dt);
        core.set_position([0.0, 0.0, -2.0]);
        let mut peak = 0.0f32;
        for _ in 0..15000 {
            core.step(&mut b);
            let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
            peak = peak.max(d);
        }
        peak
    };
    let peak_no_drag = run(0.0);
    let peak_drag = run(0.15);
    assert!(
        peak_drag < peak_no_drag,
        "drag must damp the kick: peak {peak_drag} m vs no-drag {peak_no_drag} m"
    );
}

// ── v1.18 IMU bias-instability: a STOCHASTIC random-walk gyro bias + white
// noise (richer than v1.9's constant ramp). The IEKF must continuously
// re-track the wandering bias.

/// The IEKF holds attitude under a random-walk gyro bias-instability +
/// white gyro noise — it tracks the wandering bias (a moving target) so the
/// body stays level through a long hover, not just a fixed-ramp bias.
#[test]
fn iekf_holds_under_random_walk_gyro_bias() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
        gyro_bias_rw: 0.01, // bias-instability random walk (rad/s/√s)
        gyro_white: 0.01,   // white gyro noise (rad/s)
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    let mut peak = 0.0f32;
    for k in 0..20000 {
        core.step(&mut b);
        if k > 6000 {
            peak = peak.max(b.tilt());
        }
    }
    // after ~40 s the injected bias has random-walked to ≈0.01·√40 ≈ 0.06
    // rad/s; the IEKF tracks it, so the steady tilt stays bounded.
    assert!(
        peak < 0.15,
        "IEKF must hold under random-walk gyro bias: peak tilt {peak} rad"
    );
}

// ── v1.19 GNSS realism: continuous position noise + INTERMITTENT periodic
// outage (recurring loss), beyond v1.9's single dropout window.

/// Shared scenario: hold at the origin under 0.3 m noisy GNSS + a recurring
/// 0.6 s outage every 3 s; `pos_var` is the filter's assumed fix variance.
/// Returns the peak horizontal drift after settling.
fn fly_noisy_gps(pos_var: f32) -> f32 {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
        gps_noise: 0.3,           // 30 cm per-axis fix noise
        gps_dropout_period: 1500, // every 3 s …
        gps_dropout_len: 300,     // … lose the fix for 0.6 s
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_pos_var(pos_var);
    core.set_position([0.0, 0.0, -2.0]);
    let mut peak = 0.0f32;
    for k in 0..30000 {
        core.step(&mut b);
        if k > 4000 {
            let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
            peak = peak.max(d);
        }
    }
    peak
}

/// **#434's oracle, with a realistic accelerometer.** Hold a fixed point
/// with position fixes at 5 Hz — the rate an M9N actually delivers, and the
/// rate at which the pre-#452 core diverged without bound (75 m at 20 s,
/// 3.4 km at 60 s on this harness). The accelerometer now reports specific
/// force, so the vehicle's tilt is invisible to it and the estimator takes
/// tilt from the position innovations instead.
///
/// Falsifies the fix: if `EstimatorPartition::step` fuses the accelerometer
/// as gravity while flying, this run diverges again.
#[test]
fn hold_converges_with_5hz_position_aiding() {
    for &dt in &[0.002f32, 0.01] {
        let hz = 1.0 / dt;
        let period = (hz / 5.0) as u32; // fixes at 5 Hz
        let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
            gps_dropout_period: period,
            gps_dropout_len: period - 1,
            ..Pathology::default()
        });
        let mut core = FlightCore::new(0.5, hz);
        core.set_pos_var(0.25);
        core.set_process_floor(0.30, 0.05);
        core.set_position([0.0, 0.0, -2.0]);
        b.pos[0] = 1.0; // start 1 m off, as #434 does
        let mut peak = 0.0f32;
        for i in 0..(60.0 / dt) as usize {
            core.step(&mut b);
            let h = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
            if i as f32 * dt > 5.0 && h > peak {
                peak = h;
            }
        }
        let h = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
        assert!(
            h < 0.5 && peak < 2.0,
            "5 Hz aiding must converge (dt {dt}): h@60s {h} m, peak after 5 s {peak} m"
        );
        let alt_err = (-b.pos[2] - 2.0).abs();
        assert!(
            alt_err < 0.3,
            "altitude must hold (dt {dt}): off by {alt_err} m"
        );
    }
}

/// **The accelerometer comes back when the fixes stop.** Suppressing the
/// gravity update is conditional on fresh aiding (#452); through a GNSS
/// outage the accelerometer is the only attitude reference there is, so the
/// update must resume and the vehicle must stay controllable.
#[test]
fn gnss_outage_restores_the_gravity_update_and_stays_bounded() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    // A PLANT THAT CAN ACTUALLY DRIFT (#483). Without this the default
    // Pathology has zero gyro noise and zero bias, so 15 s of dead
    // reckoning is exact and the outage stays bounded WHETHER OR NOT the
    // gravity update resumes — the test passed identically with the
    // stale-aiding fallback deleted (clean-room mutation test, 2026-09-23;
    // the compiler even reported `gravity_hold_ticks` as never read). A
    // test for "the accelerometer is the only attitude reference there is"
    // has to be run on a plant where losing that reference costs something:
    // the gyro bias ramp is unobservable without the gravity update, so
    // attitude walks off and the horizontal error grows.
    b.path.gyro_bias_drift = 0.0004;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    for _ in 0..10_000 {
        core.step(&mut b); // 20 s of aided hold
    }
    // 15 s with no fixes at all.
    b.path.gps_dropout_start = 0;
    b.path.gps_dropout_len = u32::MAX;
    b.path.gps_dropout_period = 0;
    // ASSERT ON ATTITUDE, NOT POSITION (#483). The stale-aiding fallback
    // restores the ATTITUDE reference; it cannot restore position, because
    // with no fixes the position is dead-reckoned by double integration and
    // drifts on any real plant. The old bound (`peak < 5.0` m of horizontal
    // over 15 s) was therefore only satisfiable on a NOISELESS plant — which
    // is exactly why the plant was noiseless, and why deleting the branch
    // changed nothing. Measure the quantity the branch actually controls:
    // the error between the ESTIMATED thrust axis and the TRUE one.
    let mut peak_att_err = 0.0f32;
    for _ in 0..7_500 {
        core.step(&mut b);
        // cos(tilt) of each: R[2][2]. Estimate from the quaternion, truth
        // straight off the plant's attitude matrix.
        let q = core.state().q;
        let est_c = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
        let true_c = b.r[2][2];
        let err = (est_c - true_c).abs();
        if err > peak_att_err {
            peak_att_err = err;
        }
    }
    assert!(
        peak_att_err < 0.02,
        "through a 15 s outage the accelerometer must keep the attitude \
         estimate anchored: peak |cos(tilt) est - true| = {peak_att_err}"
    );
}

/// **A misconfigured hover thrust must not silently re-enable the in-flight
/// gravity update (#481).** `flying()` gates `in_flight`, which is the only
/// thing admitting that update (#483). Every shipped entry point hardcodes
/// `hover_thrust = 0.5` — `wasm/cm/flight`, `embedded/falcon-cortex-m`,
/// `falcon-param`, `falcon-hitl`, `FlightSupervisor::new` — and
/// `wasm/cm/cascade` falls back to 0.5 when `configure` was never called.
/// An airframe that actually hovers at 0.35 mean collective then sat below
/// the old `0.7 * hover_thrust` = 0.35 threshold in a STEADY HOVER, read as
/// not-flying, and got the gravity update in flight — the full #452
/// divergence, silently, on the artifact an integrator runs.
#[test]
fn an_overestimated_hover_thrust_does_not_reenable_the_inflight_gravity_update() {
    // Same excitation as `hold_converges_with_5hz_position_aiding`, which is
    // the configuration known to expose this divergence: 5 Hz aiding, and
    // the vehicle starts 1 m off so it must TILT to fly back. Without that
    // tilt nothing is excited and the gravity update does no harm, so a
    // level undisturbed plant cannot tell the two policies apart.
    let dt = 0.002f32;
    let hz = 1.0 / dt;
    let period = (hz / 5.0) as u32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
        gps_dropout_period: period,
        gps_dropout_len: period - 1,
        ..Pathology::default()
    });
    // A LIGHT AIRFRAME: hovers at 0.35 mean collective, not 0.5.
    b.k_thrust = GRAVITY / (4.0 * 0.35);
    // ...flown by a core that was told 0.5, as every shipped entry point does.
    let mut core = FlightCore::new(0.5, hz);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    b.pos[0] = 1.0;
    let mut peak = 0.0f32;
    for i in 0..(60.0 / dt) as usize {
        core.step(&mut b);
        let h = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
        if i as f32 * dt > 5.0 && h > peak {
            peak = h;
        }
    }
    // HORIZONTAL only. A wrong hover feedforward also leaves a steady
    // ALTITUDE offset (this plant settles ~3 m high) — that is a separate,
    // already-recorded consequence of misconfiguration
    // (SWREQ-FALCON-TRANSPORT-P01 measured 58.8 m), not the property under
    // test. The #452 divergence signature is horizontal and unbounded.
    assert!(
        peak < 1.0,
        "a 0.35-hover airframe told hover_thrust=0.5 must still be seen as \
         FLYING, so the gravity update stays suppressed and the hold does \
         not diverge: peak horizontal {peak} m"
    );
}

/// **The filters follow the backend's rate, not the caller's claim.**
///
/// THIS TEST EXISTS BECAUSE THE GREPS COULD NOT FAIL. Independent review
/// reverted `resync_loop_rate` to an early `return` and every `steps:` entry
/// of FV-FALCON-RATE-002 and FV-FALCON-FAULT-005 still passed rc=0 — the
/// grepped tokens are all still there (#499). This asserts the EFFECT.
///
/// `gyro_lpf` and the harmonic `notch` are DESIGNS built from a sample rate.
/// `FlightSupervisor::new` declares 1000 Hz while the gz bench steps at
/// 250 Hz, which put a 60 Hz low-pass's real corner at a quarter of the
/// intended frequency and was the whole of the attitude limit cycle (#270).
#[test]
fn the_filters_follow_the_backend_rate_not_the_declared_one() {
    let dt = 0.004f32; // 250 Hz, as the gz bench runs
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    // DECLARE A RATE THAT IS WRONG, exactly as FlightSupervisor::new does.
    let mut core = FlightCore::new(0.5, 1000.0);
    assert_eq!(
        core.designed_loop_hz(),
        1000.0,
        "precondition: the core must start out believing the declared rate"
    );
    core.step(&mut b);
    let got = core.designed_loop_hz();
    assert!(
        (got - 1.0 / dt).abs() < 1.0,
        "after one tick the filters must be designed for the rate the \
         backend is ACTUALLY stepped at ({} Hz), not the declared 1000 Hz; \
         got {got} Hz",
        1.0 / dt
    );
}

/// Position hold survives noisy + intermittently-dropping GNSS WHEN the
/// filter's measurement variance matches the receiver: the IEKF smooths the
/// 0.3 m fix noise and dead-reckons through the recurring outage, bounded.
#[test]
fn holds_under_noisy_intermittent_gps() {
    let peak = fly_noisy_gps(0.09); // var = (0.3 m)² — matched to the fix noise
    assert!(
        peak < 1.5,
        "variance-matched filter must hold under noisy GNSS: peak {peak} m"
    );
}

/// FALSIFICATION: the optimistic default variance (0.01 = 1 cm²) over-trusts
/// a metre-class fix — the filter injects the noise, the v1.16 integral
/// winds up on it, and the loop DIVERGES (>10 m). The honest motivation for
/// matching the measurement variance to the real sensor (set_pos_var).
#[test]
fn variance_matching_helps_and_over_trust_no_longer_diverges() {
    let peak_optimistic = fly_noisy_gps(0.01); // default 1 cm² — over-trusting
    let peak_matched = fly_noisy_gps(0.09); // matched to the 0.3 m fix noise
    // BEFORE #452 the over-trusting filter DIVERGED here (> 10 m, ~2400 m
    // observed at v1.19), and that divergence was the stated motivation for
    // set_pos_var. It was not the variance alone: the filter chased the fix
    // noise into a tilt estimate corrupted by reading specific force as
    // gravity. With the gravity update suppressed in flight, over-trust
    // costs accuracy (0.40 m vs 0.18 m peak) and no longer diverges.
    assert!(
        peak_matched < peak_optimistic,
        "matching the variance must still help: {peak_matched} vs {peak_optimistic} m"
    );
    assert!(
        peak_optimistic < 1.5,
        "over-trust must no longer diverge: {peak_optimistic} m"
    );
}

// ── v1.20 barometer fusion: an independent vertical source so altitude
// survives GPS-vertical loss.

/// Max altitude error during/after a long GPS outage, with baro on/off.
fn alt_err_through_gps_loss(baro: bool) -> f32 {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.baro_enabled = baro;
    b.baro_noise = 0.2;
    b.path.gps_dropout_start = 8000; // GPS drops after settling …
    b.path.gps_dropout_len = 20000; // … for 40 s
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-3.0);
    let mut maxerr = 0.0f32;
    for k in 0..30000 {
        core.step(&mut b);
        if k > 9000 {
            maxerr = maxerr.max((b.pos[2] + 3.0).abs());
        }
    }
    maxerr
}

/// Barometer fusion keeps altitude through a 40 s GPS-vertical outage: the
/// baro (an independent vertical source) anchors the altitude loop, holding
/// the commanded altitude — and is no worse than the GPS-only dead-reckoning.
#[test]
fn baro_holds_altitude_through_gps_loss() {
    let err_baro = alt_err_through_gps_loss(true);
    let err_nobaro = alt_err_through_gps_loss(false);
    assert!(
        err_baro < 1.5,
        "baro must hold altitude through GPS loss: {err_baro} m"
    );
    assert!(
        err_baro < err_nobaro,
        "baro must beat GPS-only dead-reckoning: baro {err_baro} vs no-baro {err_nobaro} m"
    );
}

// ── v1.21 battery drain: the v1.8 failsafe fires on a REAL endurance limit
// (a depleting pack whose voltage sags under load), not a set value.

/// A draining battery actuates the endurance failsafe: the supervisor arms,
/// takes off and loiters while the pack depletes; the terminal voltage sags
/// (open-circuit drop + load sag) below the threshold and the supervisor
/// commands a recovery — fired by genuine energy use, not a set voltage.
/// #562: a SUSTAINED geofence breach must not demote the RTL it just
/// commanded. Every trigger in `step` is level-triggered, and a vehicle
/// flying home is by definition still outside the fence, so the breach
/// re-sent `Failsafe` every tick and `(Rtl, Failsafe) => Land` fired 4 ms
/// later -- landing OUTSIDE the fence it was recovering from.
///
/// The geometry matters and a first attempt got it wrong: with a 1 cm
/// fence, "breached" and "arrived home" (`dist_home < 0.4` at the Rtl
/// arrival handoff) are true SIMULTANEOUSLY, so RTL ends via the arrival
/// path and the demotion is invisible. The vehicle is ~2 m from home
/// here: the breach is sustained AND the arrival radius cannot fire.
///
/// ARMED INSIDE THE FENCE (#609 finding 5): this used to arm 2 m from
/// home, outside the fence, which `InsideGeofence` now refuses. It arms at
/// home and is carried 2 m out in flight instead (the plant is displaced;
/// the estimate follows GNSS).
#[test]
fn sustained_geofence_breach_does_not_demote_rtl() {
    use relay_calib::CalParams;
    use relay_fsm::Mode;
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 0.5, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Armed, "{:?}", sup.arm_blocked_check());
    sup.command(relay_fsm::Event::RequestTakeoff, true, false);
    for _ in 0..20_000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    b.pos[0] += 2.0;
    let mut entered = false;
    for _ in 0..2000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Rtl {
            entered = true;
            break;
        }
    }
    assert!(entered, "a fence breach must actuate RTL at all");
    // 50 ticks = 0.2 s. The vehicle cannot have covered 2 m home in that
    // time, so any exit from Rtl here is the demotion, not an arrival.
    for i in 0..50 {
        sup.step(&mut b);
        assert_eq!(
            sup.mode(),
            Mode::Rtl,
            "RTL was demoted {} ticks in while the breach was still true \
             (dist_home ~ 2 m, so this is not the arrival handoff)",
            i + 1
        );
    }
}

/// `rtl_latched` is an IN-FLIGHT monotonicity device (a sustained breach must
/// not re-fire and demote the Rtl it just commanded, #562) — NOT an arming
/// interlock. It clears on disarm, per the field's own contract, so a
/// failsafe whose condition has completely cleared does not brick the
/// airframe for the life of the process (#564: it did, and the pre-arm
/// table reported "PREARM: failsafe latched" forever).
///
/// What still blocks re-arming is the hazard itself: `MotorsHealthy` for
/// an isolated rotor (#565), `InsideGeofence` (#609 finding 5), and the
/// battery terms, which LATCH until a process restart (#609 finding 5).
/// High wind is permitted deliberately — it has no ground-observable
/// signal, so a transient gust must not be a permanent block
/// (maintainer decision, #564).
#[test]
fn re_arming_is_allowed_once_the_failsafe_condition_has_cleared() {
    use relay_calib::CalParams;
    use relay_fsm::{Event, Mode};
    let dt = 0.004f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_contact = true;
    // Home at the origin, 0.5 m fence. ARMED INSIDE IT (#609 finding 5:
    // `InsideGeofence` refuses a first arm outside the fence, which is how
    // this test used to provoke the breach); the vehicle is carried 2 m
    // out in flight below, distinguishable from "arrived home".
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 0.5, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Armed, "first arm must succeed");
    sup.command(Event::RequestTakeoff, true, false);

    // Step 2: climb to Loiter, carry the vehicle out of the fence, and fly
    // until the geofence failsafe latches.
    for _ in 0..5000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    b.pos[0] += 2.0;
    let mut latched = false;
    for _ in 0..2000 {
        sup.step(&mut b);
        if sup.rtl_latched {
            latched = true;
            break;
        }
    }
    assert!(latched, "geofence breach must latch rtl_latched");

    // Step 3: let the failsafe complete to Disarmed.
    let mut disarmed = false;
    for _ in 0..60000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    assert!(
        disarmed && sup.mode() == Mode::Disarmed,
        "failsafe must end Disarmed: mode {:?}, alt {} m, pos {:?}",
        sup.mode(),
        -b.pos[2],
        b.pos
    );
    // Settle on the ground so the pre-arm table refreshes.
    for _ in 0..10 {
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Disarmed);

    // Step 4: the hazard is unambiguously gone.
    let dx = b.pos[0];
    let dy = b.pos[1];
    let dist_home = (dx * dx + dy * dy).sqrt();
    assert!(
        dist_home < 0.5,
        "vehicle must be inside the fence around home: {dist_home} m"
    );
    assert!(-b.pos[2] < 0.3, "must be on the ground: {} m", -b.pos[2]);
    let batt = sup.battery();
    assert!(
        !batt.low && !batt.critical,
        "battery must be healthy: low {} critical {}",
        batt.low,
        batt.critical
    );
    // Was `assert!(sup.rtl_latched)` while this reproduced the defect — the
    // latch staying set WAS the cause. Inverted now: clearing it on disarm is
    // the property under test, so this fails if the clear regresses.
    assert!(
        !sup.rtl_latched,
        "the latch must CLEAR on disarm: it is an in-flight device, not an arming interlock"
    );

    // Step 5/6: re-arm through the real path; DESIRED = it arms.
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Armed,
        "vehicle cannot re-arm after a cleared failsafe; pre-arm reason: {:?} ({})",
        sup.arm_blocked_check(),
        sup.arm_blocked_check()
            .map(|c| c.reason_text())
            .unwrap_or("none")
    );
}

#[test]
fn battery_drain_actuates_endurance_failsafe() {
    use relay_fsm::Mode;
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut backend = SimBackend::new(level, dt);
    backend.battery_drain = true; // a real depleting pack (starts full)
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(relay_fsm::Event::Arm, true, true);
    sup.command(relay_fsm::Event::RequestTakeoff, true, true);
    let mut min_v = 99.0f32;
    let mut fired = false;
    for _ in 0..30000 {
        sup.step(&mut backend);
        min_v = min_v.min(backend.battery_v);
        if matches!(sup.mode(), Mode::Rtl | Mode::Land | Mode::Disarmed) {
            fired = true;
            break;
        }
    }
    assert!(fired, "draining battery must actuate the failsafe");
    assert!(
        min_v < 14.0,
        "voltage must have SAGGED below threshold from drain, not set: {min_v} V"
    );
    assert!(
        backend.battery_charge < 0.9,
        "charge must have genuinely depleted: {}",
        backend.battery_charge
    );
}

// ── v1.22 air-density thrust lapse: thrust falls with altitude; the
// altitude integral compensates, and beyond the margin a service ceiling.

/// Final altitude (m up) after climbing to a target under a thrust lapse.
fn final_altitude_under_lapse(target_up_m: f32, lapse: f32, ki_alt: Option<f32>) -> f32 {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.thrust_lapse = lapse;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    if let Some(k) = ki_alt {
        core.set_altitude_integral_gain(k);
    }
    core.set_altitude(-target_up_m);
    for _ in 0..40000 {
        core.step(&mut b);
    }
    -b.pos[2]
}

/// The altitude integral rejects the thrust lapse: the vehicle reaches and
/// holds 20 m even though air-density loss reduces thrust there.
/// v1.99 expanded failsafe — an attitude RUNAWAY (a sustained, unrecoverable
/// tilt while airborne) cuts the motors: the FSM reaches Terminated. A level
/// airborne vehicle is NOT terminated; only the sustained tumble is.
#[test]
fn attitude_runaway_terminates_in_flight() {
    use relay_calib::CalParams;
    struct TumbleBackend {
        tilted: bool,
        /// Attitude reached so far (rad), ramped while `tilted`.
        tilt_now: f32,
    }
    impl FlightBackend for TumbleBackend {
        fn read_imu(&mut self) -> ImuSample {
            // A TUMBLE IS A ROTATION (#452). The first version reported
            // gravity along body-x with a zero gyro — a vehicle tilted 90°
            // that never rotated, readable only by an estimator that takes
            // the accelerometer for gravity. A tumbling vehicle's
            // accelerometer reads its thrust axis; the tumble is in the
            // gyro. So this rolls at 2 rad/s to ~90° (0.8 s, inside the
            // 200-step level phase's successor) and holds.
            let dt = 0.004;
            let rate = if self.tilted && self.tilt_now < core::f32::consts::FRAC_PI_2 {
                self.tilt_now += 2.0 * dt;
                2.0
            } else {
                0.0
            };
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0, rate, 0.0],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, 0.0])
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn read_battery_v(&mut self) -> Option<f32> {
            Some(16.0)
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
    }
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    let mut b = TumbleBackend {
        tilted: false,
        tilt_now: 0.0,
    };

    // converge level, then arm + take off (airborne, still level).
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(sup.mode(), relay_fsm::Mode::Armed);
    sup.command(relay_fsm::Event::RequestTakeoff, true, false);
    assert_eq!(sup.mode(), relay_fsm::Mode::Takeoff, "airborne");
    // a level airborne vehicle is NOT terminated.
    for _ in 0..200 {
        sup.step(&mut b);
    }
    assert_ne!(
        sup.mode(),
        relay_fsm::Mode::Terminated,
        "level flight must not terminate"
    );

    // now a sustained tumble → flight termination. The tilt blows through the
    // high-wind band (~47 cycles, < the wind debounce) into the runaway range,
    // so termination — not RTL — fires (the runaway path strictly dominates).
    b.tilted = true;
    for _ in 0..3000 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Terminated,
        "sustained attitude runaway cuts motors"
    );
}

/// v1.101 expanded failsafe — HIGH WIND: control saturation while leaning hard
/// (but NOT tumbling) is a disturbance beyond the rate loop's authority →
/// RTL. A sustained moderate tilt with the motors pinned, away from home,
/// drives the FSM to Rtl (it does not terminate — the tilt stays below the
/// runaway limit — and does not land, being away from home).
#[test]
fn high_wind_saturation_commands_rtl() {
    use relay_calib::CalParams;
    struct WindBackend {
        windy: bool,
        /// Control ticks since the wind came on, so the tilt can be
        /// established by INTEGRATING GYRO rather than asserted through
        /// the accelerometer.
        windy_ticks: u32,
    }
    /// Pitch rate during the ramp, and how long it runs: 0.61 rad/s for
    /// 200 ticks at dt=0.004 s reaches the ~0.488 rad tilt the wind band
    /// [0.30, 0.70] expects, gently enough not to look like a runaway.
    const WIND_RAMP_RATE: f32 = 0.61;
    const WIND_RAMP_TICKS: u32 = 200;
    impl FlightBackend for WindBackend {
        fn read_imu(&mut self) -> ImuSample {
            // THIS BACKEND USED TO CONVEY TILT THROUGH THE ACCELEROMETER
            // ALONE, WITH `gyro: [0.0; 3]` (#483). That is idealized by
            // construction: a multirotor accelerometer reads specific force
            // along the THRUST axis, so a real vehicle cannot learn its tilt
            // that way in flight — and once the in-flight gravity update was
            // removed (it made attitude 24x worse through an outage) this
            // test had no way left to express a tilt at all, and stopped
            // firing the failsafe it names.
            //
            // Establish the tilt the way a vehicle actually does: rotate
            // into it on the gyro, then hold. The accelerometer then
            // reports the specific force consistent with that attitude,
            // which is what a real IMU would produce.
            if self.windy {
                self.windy_ticks = self.windy_ticks.saturating_add(1);
            }
            let (accel, gyro) = if !self.windy {
                ([0.0, 0.0, -GRAVITY], [0.0; 3])
            } else if self.windy_ticks <= WIND_RAMP_TICKS {
                // rotating INTO the tilt about body +y
                let th = WIND_RAMP_RATE * (self.windy_ticks as f32) * 0.004;
                (
                    [
                        GRAVITY * relay_math::sinf(th),
                        0.0,
                        -GRAVITY * relay_math::cosf(th),
                    ],
                    [0.0, WIND_RAMP_RATE, 0.0],
                )
            } else {
                // holding the tilt: no further rotation
                ([GRAVITY * 0.469, 0.0, -GRAVITY * 0.883], [0.0; 3])
            };
            ImuSample { accel, gyro }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([100.0, 0.0, 0.0]) // away from home: an RTL flies, never lands
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn read_battery_v(&mut self) -> Option<f32> {
            Some(16.0)
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
    }
    // effectively-infinite fence so the tilt-corrupted position estimate can
    // never trip the GEOFENCE failsafe — isolating the high-wind path.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.0e9, 2.0, 14.0);
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    let mut b = WindBackend {
        windy: false,
        windy_ticks: 0,
    };
    // converge level, arm, take off.
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(sup.mode(), relay_fsm::Mode::Armed);
    sup.command(relay_fsm::Event::RequestTakeoff, true, false);
    assert_eq!(sup.mode(), relay_fsm::Mode::Takeoff, "airborne");
    // sustained high wind: moderate tilt + saturation fires the RTL-class
    // failsafe (rtl_latched), and the vehicle RECOVERS — it does NOT terminate
    // (the tilt is below the runaway limit). It also leaves normal flight. The
    // exact recovery mode then follows the standard RTL path (Rtl→Land as the
    // tilt-corrupted position estimate reads near home); the discriminating
    // property vs attitude-runaway is recover-not-terminate.
    b.windy = true;
    let mut fired_at_takeoff = false;
    for _ in 0..400 {
        let before = sup.mode();
        sup.step(&mut b);
        // the FIRST failsafe to fire does so from Takeoff → Rtl (recovery).
        if sup.rtl_latched && before == relay_fsm::Mode::Takeoff {
            fired_at_takeoff =
                sup.mode() == relay_fsm::Mode::Rtl || sup.mode() == relay_fsm::Mode::Land;
        }
        if sup.rtl_latched {
            break;
        }
    }
    assert!(
        sup.rtl_latched,
        "sustained high-wind saturation must fire the RTL-class failsafe"
    );
    assert!(
        fired_at_takeoff,
        "the failsafe fired from normal flight (Takeoff → RTL recovery)"
    );
    assert_ne!(
        sup.mode(),
        relay_fsm::Mode::Terminated,
        "high wind recovers (RTL), it does NOT terminate"
    );
}

/// #413 — ABSENCE IS NOT HEALTH. A backend with no battery sense must make
/// the BatteryOk pre-arm row FAIL, and must block arming.
///
/// This is the falsification criterion from the issue: before the fix,
/// `read_battery_v` defaulted to 16.0 V — a healthy 4S pack — so a vehicle
/// that could not see its battery at all reported that the battery was
/// fine, forever, and armed happily. The published wasm component, gz SITL
/// and any HardwareBackend without a battery driver were all in that state.
///
/// Note what this test does NOT do: it does not override `read_battery_v`.
/// The whole point is the DEFAULT, so the test must inherit it — overriding
/// it here would verify a path no real backend takes.
#[test]
fn absent_battery_blocks_arming() {
    use relay_calib::CalParams;
    struct NoBatteryBackend;
    impl FlightBackend for NoBatteryBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, 0.0])
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
        // read_battery_v deliberately NOT overridden — inherit the default.
    }

    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    let mut b = NoBatteryBackend;
    // A calibration is installed so that CALIBRATION is not what blocks us —
    // otherwise this test would pass for the wrong reason.
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..1500 {
        sup.step(&mut b);
    }

    assert_eq!(
        sup.arm_blocked_check(),
        Some(relay_preflight::CheckId::BatteryOk),
        "a vehicle that cannot see its pack must be blocked ON THE BATTERY row"
    );
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Disarmed,
        "must refuse to arm with no battery data"
    );
}

/// v1.99 — the pre-arm gate is fed by REAL vehicle state via update_preflight
/// (called each step). A fresh, uncalibrated vehicle is refused arming
/// (no calibration, estimator not converged); after installing a calibration
/// and letting the estimator settle, with a healthy battery + a loaded fence,
/// arming is permitted. Proves the gate is no longer inert all-pass defaults.
#[test]
fn prearm_gate_fed_by_real_state() {
    use relay_calib::CalParams;
    struct RestBackend {
        batt: f32,
    }
    impl FlightBackend for RestBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -GRAVITY],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            Some([0.0, 0.0, 0.0])
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn read_battery_v(&mut self) -> Option<f32> {
            Some(self.batt)
        }
        fn write_motors(&mut self, _: &[f32]) {}
        fn dt(&self) -> f32 {
            0.004
        }
    }
    // fence_radius 50 (loaded), low_batt_v 14.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    let mut b = RestBackend { batt: 16.0 };

    // settle the estimator, but with NO calibration installed.
    for _ in 0..1500 {
        sup.step(&mut b);
    }
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Disarmed,
        "must not arm without calibration"
    );
    assert_eq!(
        sup.arm_blocked_check(),
        Some(relay_preflight::CheckId::CalibrationPresent)
    );

    // install a (valid) calibration → step once to refresh → arms.
    sup.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    sup.step(&mut b);
    assert_eq!(sup.arm_blocked_check(), None, "all real checks pass");
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Armed,
        "arms once the real signals are good"
    );

    // a low battery (read each step) blocks re-arming after a disarm.
    let mut low = RestBackend { batt: 13.0 };
    let mut sup2 = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup2.set_calibration(CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..CalParams::identity()
    });
    for _ in 0..1500 {
        sup2.step(&mut low);
    }
    sup2.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup2.mode(),
        relay_fsm::Mode::Disarmed,
        "low battery blocks arming"
    );
    assert_eq!(
        sup2.arm_blocked_check(),
        Some(relay_preflight::CheckId::BatteryOk)
    );
}

/// v1.97 pre-arm gate (the seam): the FlightSupervisor refuses to arm unless
/// every commander check passes, surfaces the first failing reason, and arms
/// once they all pass — even when the vehicle is physically level + idle.
///
/// Driven through the PREARM-P03 table's always-required six rows (the
/// legacy `set_preflight(PreflightChecks)` seam is gone, #609
/// single-source refactor); the gate and the reason are the table's.
#[test]
fn prearm_checks_gate_arming() {
    use relay_preflight::CheckId;
    const SIX: [CheckId; 6] = [
        CheckId::SensorsHealthy,
        CheckId::EstimatorConverged,
        CheckId::CalibrationPresent,
        CheckId::GeofenceLoaded,
        CheckId::BatteryOk,
        CheckId::FailsafeConfigured,
    ];
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);

    // all checks FAILING: arming refused, reason = the first row.
    for id in SIX {
        sup.set_check(id, false);
    }
    sup.command(relay_fsm::Event::Arm, true, true); // level + throttle idle
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Disarmed,
        "no arm with failed pre-arm checks"
    );
    assert_eq!(sup.arm_blocked_check(), Some(CheckId::SensorsHealthy));

    // only the battery failing → blocked on BatteryOk, still won't arm.
    for id in SIX {
        sup.set_check(id, id != CheckId::BatteryOk);
    }
    assert_eq!(sup.arm_blocked_check(), Some(CheckId::BatteryOk));
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(sup.mode(), relay_fsm::Mode::Disarmed);

    // every check passing → arms.
    for id in SIX {
        sup.set_check(id, true);
    }
    assert_eq!(sup.arm_blocked_check(), None);
    sup.command(relay_fsm::Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        relay_fsm::Mode::Armed,
        "arms once every pre-arm check passes"
    );
}

#[test]
fn holds_altitude_under_thrust_lapse() {
    let alt = final_altitude_under_lapse(20.0, 0.01, Some(0.02)); // integral ON
    assert!(
        (alt - 20.0).abs() < 1.0,
        "must hold 20 m despite thrust lapse: {alt} m"
    );
}

/// FALSIFICATION: with the altitude integral DISABLED (P-D only) the thrust
/// lapse leaves the vehicle sagging below the target (a steady thrust
/// deficit the P-D loop cannot reject); the integral closes the gap.
#[test]
fn bare_altitude_pd_sags_under_lapse() {
    let alt_pd = final_altitude_under_lapse(20.0, 0.01, Some(0.0)); // integral OFF
    let alt_pid = final_altitude_under_lapse(20.0, 0.01, Some(0.02)); // integral ON
    assert!(
        alt_pd < 19.0,
        "P-D alone should sag below target under lapse: {alt_pd} m"
    );
    assert!(
        (alt_pid - 20.0).abs() < 1.0 && alt_pid > alt_pd,
        "the integral must close the gap: pid {alt_pid} vs pd {alt_pd} m"
    );
}

/// The thrust lapse imposes a SERVICE CEILING: commanded to 80 m, the
/// vehicle cannot climb past where full thrust just hovers (~50 m), even
/// with the integral — an honest physical limit, not a control failure.
#[test]
fn thrust_lapse_imposes_service_ceiling() {
    let alt = final_altitude_under_lapse(80.0, 0.01, Some(0.02)); // integral ON, still ceiling
    assert!(
        (35.0..65.0).contains(&alt),
        "thrust lapse must cap altitude well below the 80 m command: {alt} m"
    );
}

// ── v1.23 motor dynamics: first-order spin-up lag — the actuator lag the
// ADRC ESO is designed to absorb (a robustness confirmation).

/// The ADRC ESO ABSORBS realistic motor lag: with a 40 ms first-order motor
/// time constant (the rotor speed lags the command), the verified inner loop
/// still recovers a tilted body to level — exactly the actuator lag the ESO
/// was built for (v0.25).
#[test]
fn adrc_absorbs_motor_lag() {
    let dt = 0.002f32;
    let th = 0.4f32;
    let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
    let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
    let mut b = SimBackend::new(r0, dt);
    b.motor_tau = 0.04; // 40 ms motor lag — significant
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    for _ in 0..8000 {
        core.step(&mut b);
    }
    assert!(
        b.tilt() < 0.1,
        "ADRC ESO must absorb motor lag and recover to level: {} rad",
        b.tilt()
    );
}

/// Position hold stays bounded under motor lag — the lag the v0.25 work
/// found could destabilise a naive loop is absorbed by the ESO; the vehicle
/// holds its setpoint.
#[test]
fn position_hold_stable_under_motor_lag() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.motor_tau = 0.03; // 30 ms motor lag
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([1.5, -1.0, -2.0]);
    for _ in 0..30000 {
        core.step(&mut b);
    }
    let e = [b.pos[0] - 1.5, b.pos[1] + 1.0, b.pos[2] + 2.0];
    let err = relay_math::sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    assert!(
        err < 0.6,
        "position hold must stay bounded under motor lag: {err} m"
    );
}

// ── v1.24 ground effect: a thrust cushion near the surface (landing/takeoff).

/// Ground effect AIDS takeoff: the reflected-downwash thrust boost near the
/// surface helps the climb; the vehicle reaches and holds its commanded
/// altitude with no instability.
#[test]
fn ground_effect_aids_takeoff() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_effect = 0.4;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    for _ in 0..15000 {
        core.step(&mut b);
    }
    let alt = -b.pos[2];
    assert!(
        (alt - 2.0).abs() < 0.3,
        "ground effect must aid takeoff to altitude: {alt} m"
    );
}

/// HONEST LIMITATION (documented, not faked): ground effect cushions the
/// landing — the position-based altitude loop reaches a hover equilibrium
/// ABOVE the surface and FLOATS on the cushion rather than touching down. It
/// stays bounded and stable (not diverging). A clean touchdown needs a
/// velocity-based landing controller (future work); the altitude integral
/// is NOT a fix here (it runs away when wound up during the climb).
#[test]
fn ground_effect_cushions_landing_into_a_float() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_effect = 0.4;
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    for _ in 0..10000 {
        core.step(&mut b);
    }
    core.set_altitude(0.0); // command landing
    for _ in 0..20000 {
        core.step(&mut b);
    }
    let alt = -b.pos[2];
    // floats on the cushion, bounded + stable — the documented limitation
    assert!(
        (0.2..2.5).contains(&alt),
        "ground effect floats the landing on a bounded cushion: {alt} m"
    );
}

// ── v1.25 turbulence: a Dryden-like COLORED gust spectrum (OU, correlated),
// harder than v1.16's white-noise gust because the gusts persist.

/// Position hold rides out a Dryden-like turbulence spectrum (2 m/s RMS,
/// ~1 s correlation). Honest claim: the persistent, correlated gusts push
/// the vehicle into a larger excursion than a steady wind (peak ~3 m here),
/// but it stays BOUNDED — it does not diverge — and returns. The worst-case
/// environment of the realism arc; bounded-not-crisp, stated plainly.
#[test]
fn holds_position_under_turbulence() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt).with_pathology(Pathology {
        turbulence: 2.0,
        ..Default::default()
    });
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([0.0, 0.0, -2.0]);
    let mut peak = 0.0f32;
    for k in 0..40000 {
        core.step(&mut b);
        if k > 4000 {
            let d = relay_math::sqrtf(b.pos[0] * b.pos[0] + b.pos[1] * b.pos[1]);
            peak = peak.max(d);
        }
    }
    // bounded under continuous turbulence — it does not diverge (an
    // over-authority wind blew the vehicle to 100s of metres; this rides
    // out the persistent gusts within a few metres).
    assert!(
        peak < 4.0,
        "turbulence must stay bounded (not diverge): peak {peak} m"
    );
}

// ── v1.27 velocity-based touchdown: the clean landing the v1.24 ground-
// effect float needs.

/// The velocity-based touchdown controller LANDS through the ground-effect
/// cushion the v1.24 position loop floated on: commanded to land, the
/// vehicle descends at a controlled rate, pushes through the cushion, and
/// settles on the surface — vs the ~1.3 m float without it.
#[test]
fn velocity_landing_touches_down_through_ground_effect() {
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_effect = 0.4; // the cushion that floated v1.24
    b.ground_contact = true; // model the ground so a touchdown can settle
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_altitude(-2.0);
    for _ in 0..10000 {
        core.step(&mut b); // climb to 2 m
    }
    core.set_landing(true); // engage the velocity-based touchdown
    for _ in 0..20000 {
        core.step(&mut b);
    }
    let alt = -b.pos[2];
    assert!(
        alt < 0.15,
        "velocity touchdown must reach the surface through ground effect: {alt} m"
    );
    assert!(
        b.vel[2].abs() < 0.3,
        "should settle on touchdown: vz {} m/s",
        b.vel[2]
    );
}

// ── v1.29: wire the v1.27 velocity-landing into the FlightSupervisor ──
//
// v1.27 added the velocity-based touchdown controller to FlightCore but the
// FlightSupervisor still left the core in position mode for Land — so the
// INTEGRATED stack (the thing a real vehicle runs) still descended on the
// slow altitude P-I-D that floats short through ground effect
// (`ground_effect_cushions_landing_into_a_float` above shows that float).
// The supervisor now engages set_landing in Land mode, so a supervised
// landing through the same cushion reaches the surface and DISARMS.

/// A full supervised mission — arm → takeoff → loiter → land — touches down
/// and disarms through the ground-effect cushion the position loop floats
/// on. This is the integration the v1.27 core fix needed: before v1.29 the
/// supervised Land floated (never reaching Disarmed); now it settles.
#[test]
fn supervised_landing_disarms_through_ground_effect() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.ground_effect = 0.4; // the cushion that floated the position-land
    b.ground_contact = true; // model the ground so a touchdown can settle
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "should reach Loiter after takeoff"
    );

    sup.command(Event::RequestLand, true, false); // → Land (velocity touchdown)
    let mut disarmed = false;
    for _ in 0..20000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    assert!(
        disarmed,
        "supervised landing must touch down + disarm through ground effect \
         (it floated before v1.29): mode {:?}, alt {} m",
        sup.mode(),
        -b.pos[2]
    );
    // Touchdown→Disarmed interrupts the descent at the 0.15 m trigger and the
    // disarmed position-hold settles just above it through ground effect — on
    // the surface (vs the ~1.3 m float without the velocity-landing).
    assert!(
        -b.pos[2] < 0.25,
        "must settle on the surface (not float): {} m",
        -b.pos[2]
    );
}

#[test]
fn a_rotor_fault_isolated_during_landing_blocks_re_arming() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.rot_drag = 0.02;
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    // A calibrated airframe, so the ONLY thing that could block a re-arm
    // is the fault under test (an uncalibrated one is refused for an
    // unrelated reason).
    sup.set_calibration(relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..relay_calib::CalParams::identity()
    });
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Loiter, "should reach Loiter first");
    assert!(
        sup.core().failed_motor().is_none(),
        "healthy before the fault"
    );

    // Command the landing; the fault must NOT be injected until Land.
    sup.command(Event::RequestLand, true, false);
    sup.step(&mut b);
    assert_eq!(sup.mode(), Mode::Land, "must be in Land BEFORE the fault");
    assert!(!sup.rtl_latched, "no latch before the fault");

    // Rotor 1 dies during the landing descent.
    b.fail_rotor(1);
    let mut isolated = false;
    for _ in 0..15000 {
        sup.step(&mut b);
        if sup.core().failed_motor().is_some() {
            isolated = true;
            break;
        }
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    assert!(
        isolated,
        "FDI must isolate the rotor (mode {:?}, alt {} m)",
        sup.mode(),
        -b.pos[2]
    );
    assert_eq!(
        sup.mode(),
        Mode::Land,
        "must still be in Land at isolation (else the scenario is void)"
    );
    assert!(
        !sup.rtl_latched,
        "the fault slipped past the failsafe latch (mode != Land guard)"
    );

    let mut disarmed = false;
    for _ in 0..15000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    assert!(
        disarmed,
        "must disarm after touchdown: mode {:?}",
        sup.mode()
    );
    assert_eq!(sup.mode(), Mode::Disarmed);
    assert!(
        sup.core().failed_motor().is_some(),
        "the dead motor is still latched at the arm attempt"
    );

    // Let the pre-arm table refresh, then try to re-arm.
    sup.step(&mut b);
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Disarmed,
        "a vehicle with a latched dead motor must REFUSE to arm; \
         arm_blocked_check={:?} failed_motor={:?}",
        sup.arm_blocked_check(),
        sup.core().failed_motor()
    );
}

/// #564/#565 — the latch CLEARS on disarm, so for a rotor that failed IN
/// FLIGHT (latch set) the `MotorsHealthy` row alone must keep the vehicle
/// from re-arming. Opposite of the during-Land test, where the latch is
/// never set.
#[test]
fn a_rotor_that_failed_in_flight_still_blocks_re_arming_after_the_latch_clears() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.rot_drag = 0.02;
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    // Calibrated, so a refusal cannot be CalibrationPresent.
    sup.set_calibration(relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..relay_calib::CalParams::identity()
    });
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Loiter, "should reach Loiter first");
    assert!(sup.core().failed_motor().is_none(), "healthy before fault");
    assert!(!sup.rtl_latched, "no latch before the fault");

    // Rotor 1 dies in normal flight (NOT Land).
    b.fail_rotor(1);
    let mut isolated = false;
    for _ in 0..15000 {
        sup.step(&mut b);
        if sup.core().failed_motor().is_some() {
            isolated = true;
            break;
        }
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    assert!(
        isolated,
        "FDI must isolate the rotor (mode {:?}, alt {} m)",
        sup.mode(),
        -b.pos[2]
    );
    assert!(
        sup.core().failed_motor().is_some() && sup.rtl_latched,
        "the in-flight fault must go through the latch path: \
         failed_motor={:?} rtl_latched={} mode={:?} alt={} m",
        sup.core().failed_motor(),
        sup.rtl_latched,
        sup.mode(),
        -b.pos[2]
    );
    assert!(-b.pos[2] > 0.3, "must still be airborne at isolation");

    let mut disarmed = false;
    for _ in 0..30000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    assert!(disarmed, "must disarm: mode {:?}", sup.mode());
    for _ in 0..10 {
        sup.step(&mut b);
    }
    assert!(
        !sup.rtl_latched,
        "the latch must have CLEARED on disarm, else the refusal below proves nothing"
    );
    assert!(
        sup.core().failed_motor().is_some(),
        "the dead motor is still latched at the arm attempt"
    );

    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Disarmed,
        "a vehicle with a latched dead motor must REFUSE to arm; \
         arm_blocked_check={:?} failed_motor={:?}",
        sup.arm_blocked_check(),
        sup.core().failed_motor()
    );
    assert_eq!(
        sup.arm_blocked_check(),
        Some(relay_preflight::CheckId::MotorsHealthy),
        "the refusal must be NAMED as MotorsHealthy, not some other row"
    );
}

/// v1.117 (FAULT-P04) — the SUPERVISED rotor-out chain ends on the ground:
/// a rotor dies in Loiter → the FDI isolates → the motor failsafe commands
/// LAND → the vehicle makes a CONTROLLED descent (bounded sink rate, thrust
/// axis near-level on the rank-3 allocation) → velocity touchdown →
/// Disarmed. This closes the ALTITUDE scope the v1.114 campaign
/// deliberately reported-but-did-not-gate ("altitude is the supervisor's
/// LAND job" — this is that job, verified end to end on the full
/// production supervisor + rot_drag plant).
#[test]
fn supervised_rotor_out_lands_and_disarms() {
    use relay_fsm::{Event, Mode};
    let dt = 0.002f32; // the rate the supervisor's loops are tuned at
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = SimBackend::new(level, dt);
    b.rot_drag = 0.02; // drag-bounded rotor-out yaw spin (v1.114)
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "should reach Loiter after takeoff"
    );
    // The failure must be injected from a SETTLED hover — otherwise the
    // sink metrics measure the pre-existing limit cycle, not the recovery
    // (this precondition caught exactly that on first write).
    assert!(
        (-b.pos[2] - 2.0).abs() < 0.3 && b.vel[2].abs() < 0.3,
        "hover must be settled before the failure: alt {} vz {}",
        -b.pos[2],
        b.vel[2]
    );

    // Rotor 1 dies mid-hover.
    b.fail_rotor(1);
    let (mut landed, mut land_cmded) = (false, false);
    // Two sink metrics, deliberately separate: the LIFT-LOSS TRANSIENT
    // (the first moments after the rotor dies — physics guarantees a drop
    // until the FDI isolates and collective compensates; it must be
    // BOUNDED and ARRESTED) vs the FINAL APPROACH (below 0.5 m — this is
    // the "controlled descent" claim and must be GENTLE).
    let (mut peak_sink_transient, mut approach_sink, mut peak_tilt) = (0.0f32, 0.0f32, 0.0f32);
    for _ in 0..15000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Land {
            land_cmded = true;
        }
        let alt = -b.pos[2];
        if alt > 0.15 {
            peak_sink_transient = peak_sink_transient.max(b.vel[2]); // NED +z = down
            peak_tilt = peak_tilt.max(b.tilt());
            if alt < 0.5 {
                approach_sink = approach_sink.max(b.vel[2]);
            }
        }
        if sup.mode() == Mode::Disarmed {
            landed = true;
            break;
        }
    }
    assert!(
        land_cmded,
        "motor failsafe must command Land (mode {:?})",
        sup.mode()
    );
    assert!(
        landed,
        "supervised rotor-out must touch down + disarm: mode {:?}, alt {} m",
        sup.mode(),
        -b.pos[2]
    );
    assert!(
        -b.pos[2] < 0.25,
        "must settle on the surface: {} m",
        -b.pos[2]
    );
    // The lift-loss transient is bounded (arrested well short of freefall
    // from 2 m ≈ 6.3 m/s) and the FINAL APPROACH is gentle (the velocity
    // landing commands 0.5 m/s; allow margin for the rotor-out wobble).
    assert!(
        peak_sink_transient < 4.0,
        "lift-loss transient must be arrested: peak {peak_sink_transient} m/s"
    );
    assert!(
        approach_sink < 1.0,
        "final approach must be gentle: {approach_sink} m/s"
    );
    assert!(
        peak_tilt < 0.5,
        "must stay near-level during the descent: peak {peak_tilt} rad"
    );
    // Upright on the ground.
    assert!(
        b.tilt() < 0.35,
        "must be upright at touchdown: {} rad",
        b.tilt()
    );
}

/// TRANSPORT-P02: the failing-row mask a binding host reads through `state` is
/// EXACTLY the verdict `command(Arm)` is judged on. Over pseudo-random tables
/// (each row left undeclared, declared passing, or declared failing):
///
/// - mask == 0 iff `arm_blocked_check()` is None;
/// - a set bit is exactly a row that is required and not passing, at bit
///   `id as usize` (the discriminant, not the gate position);
/// - the row `arm_blocked_check()` names is the first set bit in
///   `CheckId::ALL` (gate) order;
/// - and the mask agrees with what `Arm` actually does.
#[test]
fn prearm_failing_mask_is_the_arm_gate_verdict() {
    use relay_preflight::{CHECK_COUNT, CheckId};
    let mut x: u32 = 0x9e37_79b9;
    let mut blocked_runs = 0;
    let mut allowed_runs = 0;
    for run in 0..600 {
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
        let mut want = 0u32;
        for id in CheckId::ALL {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // Bias toward passing so the Allowed side is reached often.
            let r = (x >> 24) % 8;
            let legacy = (id as usize) < 6;
            match r {
                0 => {
                    sup.set_check(id, false);
                    want |= 1 << (id as usize);
                }
                1 if !legacy => {} // breadth row left undeclared: not failing
                _ => sup.set_check(id, true),
            }
        }
        let mask = sup.prearm_failing_mask();
        assert_eq!(mask, want, "run {run}");
        assert_eq!(mask >> CHECK_COUNT, 0, "no bit above the table");
        assert_eq!(mask == 0, sup.arm_blocked_check().is_none(), "run {run}");
        let first = CheckId::ALL
            .iter()
            .copied()
            .find(|&id| mask & (1 << (id as usize)) != 0);
        assert_eq!(first, sup.arm_blocked_check(), "run {run}: deciding row");
        sup.command(relay_fsm::Event::Arm, true, true);
        let armed = sup.mode() == relay_fsm::Mode::Armed;
        assert_eq!(armed, mask == 0, "run {run}: Arm agrees with the mask");
        if armed {
            allowed_runs += 1;
        } else {
            blocked_runs += 1;
        }
    }
    assert!(
        allowed_runs > 10 && blocked_runs > 10,
        "{allowed_runs}/{blocked_runs}"
    );

    // A never-stepped `FlightSupervisor::new` starts its legacy six PASSING
    // (the bring-up back-compat in `with_core`); the mask says so, in step
    // with the gate. (A host that must not arm before a measured step
    // declares those rows false itself — #609 finding 2.)
    let fresh = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    assert_eq!(fresh.prearm_failing_mask(), 0);
    assert_eq!(fresh.arm_blocked_check(), None);
    // InsideGeofence is bit 20 (its discriminant), not 10 (its gate position).
    let mut s = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    for id in CheckId::ALL {
        s.set_check(id, id != CheckId::InsideGeofence);
    }
    assert_eq!(s.prearm_failing_mask(), 1 << 20);
    assert_eq!(s.arm_blocked_check(), Some(CheckId::InsideGeofence));
}
