//! A host that speaks ONLY the binding can configure, calibrate, arm and take
//! off the supervised vehicle (relay#609 finding 9, #572, #527 — binding v2).
//!
//! WHY THIS FILE EXISTS. TRANSPORT-P01 was `verified` while the binding's
//! frames carried `configure` and `step` and nothing else. The @0.12.0
//! component wraps a `FlightSupervisor`, which writes zero motors until it is
//! configured, calibrated and armed — so a rig driving it "purely through
//! binding-v1 frames" sat `disarmed` for ever. That is the requirement's own
//! falsification clause (an integrator would have to write framing of their
//! own), and nothing in the tree could fail on it, because no test drove a
//! supervisor through bytes.
//!
//! WHAT CROSSES THE BOUNDARY. The host side is the reference `BindingHost`
//! plus a plant (`SimBackend` with a battery). The component side is
//! [`Endpoint`], below: a native `FlightSupervisor` behind `wire::decode_frame`.
//! They exchange `Vec<u8>` and nothing else. The test READS the endpoint's
//! supervisor to NAME the pre-arm row that refused — the binding has no state
//! frame, which the binding document lists as a gap — but the host never
//! touches it, and the primary verdict is the plant's own altitude.
//!
//! WHAT THIS IS NOT. It is not the published wasm component, which has no byte
//! endpoint. But it runs the SAME lifecycle rules: [`Endpoint`] is a
//! field-for-field adapter over `falcon_core::host::SupervisorHost`, the one
//! implementation of the seam's rules (#609 single-source refactor) that the
//! published component and the wasm/native mirror also adapt: lazy
//! construction, the declared-false step-fed rows, `FailsafeConfigured` from
//! the stored supervisor configuration, `ParamsFromNvm` from the stored vehicle
//! tuning, the calibration surviving a rebuild, no rebuild in flight, and an
//! INVALID calibration / supervisor configuration revoking the earlier one.
//! Until then this file carried its own copy of those rules.

use falcon_core::host::{self, SupervisorHost};
use falcon_core::relay_calib::CalParams;
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::relay_preflight::CheckId;
use falcon_core::{FlightBackend, FlightSupervisor, ImuSample, SimBackend};
use falcon_hitl::wire::{
    self, MsgType, SeqTracker, TickVerdict, WireCalibration, WireCommand, WireError,
    WirePilotCommand, WireSupervisorConfig, WireVehicleConfig,
};
use falcon_hitl::{BindingHost, HostReply, sample_sensor_frame};

// ── the component side: a native supervisor behind the binding ──────────────

/// The calibration frame -> the native `CalParams`, field for field.
///
/// All six fields are forwarded, including `mag_offdiag` (payload 60/64/68),
/// which `relay_calib::CalParams` applies as the off-diagonal of the symmetric
/// soft-iron matrix, ordered `[xy, xz, yz]` (relay#611).
fn cal_params(c: &WireCalibration) -> CalParams {
    CalParams {
        gyro_bias: c.gyro_bias,
        accel_bias: c.accel_bias,
        accel_scale: c.accel_scale,
        mag_offset: c.mag_offset,
        mag_scale: c.mag_scale,
        mag_offdiag: c.mag_offdiag,
    }
}

/// The config frame -> the host's `VehicleConfig`, field for field. Its
/// domains are `VehicleConfig::tuning`'s; an out-of-domain record is refused
/// there, not here.
fn vehicle_config(v: &WireVehicleConfig) -> host::VehicleConfig {
    host::VehicleConfig {
        hover_thrust: v.hover_thrust,
        loop_rate_hz: v.loop_rate_hz,
        pos_var: v.pos_var,
        process_floor_vel: v.process_floor_vel,
        process_floor_pos: v.process_floor_pos,
        altitude_kp: v.altitude_kp,
        altitude_kd: v.altitude_kd,
        altitude_ki: v.altitude_ki,
        position_ki: v.position_ki,
    }
}

/// The supervisor-config frame -> the host's `SupervisorConfig`, field for
/// field. Validity is `SupervisorConfig::is_valid`'s.
fn supervisor_config(c: &WireSupervisorConfig) -> host::SupervisorConfig {
    host::SupervisorConfig {
        home: c.home,
        fence_radius: c.fence_radius,
        cruise_alt: c.cruise_alt,
        low_batt_v: c.low_batt_v,
    }
}

fn event(c: WirePilotCommand) -> Event {
    // Exhaustive, no `_` arm: a new wire command must be considered here.
    match c {
        WirePilotCommand::Arm => Event::Arm,
        WirePilotCommand::RequestTakeoff => Event::RequestTakeoff,
        WirePilotCommand::RequestMission => Event::RequestMission,
        WirePilotCommand::RequestLoiter => Event::RequestLoiter,
        WirePilotCommand::RequestRtl => Event::RequestRtl,
        WirePilotCommand::RequestLand => Event::RequestLand,
        WirePilotCommand::RequestDisarm => Event::RequestDisarm,
        WirePilotCommand::Terminate => Event::Terminate,
    }
}

/// The binding has NO mission target (docs/TRANSPORT-BINDING.md, "What this
/// binding does not yet give you"): the `sensor` frame carries no waypoint.
/// `SupervisorHost::step` takes the frame's target, which the supervisor
/// ignores outside Mission and, in Mission, FOLLOWS as a live setpoint (#609
/// finding 4). So the endpoint delivers the seam's own "no target this frame"
/// encoding, a NON-FINITE target, rather than inventing a point: a binding
/// host that sends `request-mission` gets the stale-target HOLD (Loiter where
/// the vehicle is, failsafe bit 8), not a flight to a point nobody sent.
///
/// Until #609's live-follow fix this was `[0, 0, 0]` — home at GROUND LEVEL
/// (`down = 0`) — which under live follow is a FRESH target every frame: Mission
/// would fly the vehicle down to the ground at home and hold it there, never
/// going stale. `request_mission_with_the_old_constant_descends_toward_the_ground`
/// pins that hazard as the negative control.
const NO_TARGET: [f32; 3] = [f32::NAN; 3];

/// The pre-#609-follow endpoint constant, kept only for the negative control.
const OLD_HOME_GROUND_TARGET: [f32; 3] = [0.0, 0.0, 0.0];

/// One tick's sensors from a decoded frame; the motors the supervisor wrote.
struct FrameBackend {
    f: wire::WireSensorFrame,
    motors: [f32; 4],
}

impl FlightBackend for FrameBackend {
    fn read_imu(&mut self) -> ImuSample {
        let i = self.f.imu;
        ImuSample {
            accel: [i[0], i[1], i[2]],
            gyro: [i[3], i[4], i[5]],
        }
    }
    fn read_position(&mut self) -> Option<[f32; 3]> {
        self.f.position_ned
    }
    fn read_mag(&mut self) -> Option<[f32; 3]> {
        self.f.mag_body
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.f.heading_rad
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.f.motor_rpm
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.f.battery_v
    }
    fn write_motors(&mut self, m: &[f32]) {
        for (i, o) in self.motors.iter_mut().enumerate() {
            *o = m.get(i).copied().unwrap_or(0.0);
        }
    }
    fn dt(&self) -> f32 {
        self.f.dt_s.clamp(0.0001, 0.1)
    }
}

/// The component side of the binding, natively: bytes in, `SupervisorHost`
/// calls out. It holds no configuration of its own.
struct Endpoint {
    seam: SupervisorHost,
    inbound: SeqTracker,
    /// What every `step` delivers as the frame target: [`NO_TARGET`], except
    /// in the negative control.
    target: [f32; 3],
    // No outbound counter: every reply echoes the seq of the frame it
    // answers (docs/TRANSPORT-BINDING.md §3, the echo rule).
}

/// The message type a decoder rejection names, if the frame got far enough
/// for the type byte to be read. `TooShort`, `BadMagic`, `BadVersion` and
/// `UnknownType` name none; `BadCommand` is only ever a `Command`.
fn rejected_type(e: &WireError) -> Option<MsgType> {
    match *e {
        WireError::BadLength { msg, .. }
        | WireError::NonFinite { msg, .. }
        | WireError::BadBool { msg, .. } => Some(msg),
        WireError::BadCommand { .. } => Some(MsgType::Command),
        WireError::TooShort { .. }
        | WireError::BadMagic { .. }
        | WireError::BadVersion { .. }
        | WireError::UnknownType { .. } => None,
    }
}

impl Endpoint {
    fn new() -> Self {
        Endpoint {
            seam: SupervisorHost::new(),
            inbound: SeqTracker::new(),
            target: NO_TARGET,
        }
    }

    fn supervisor(&self) -> &FlightSupervisor {
        self.seam
            .supervisor()
            .expect("built by the first command/sensor frame")
    }

    /// THE ECHO RULE for acks: the header seq IS the acknowledged control
    /// frame's seq, and the payload repeats it (layout unchanged). A counter
    /// of the endpoint's own here would interleave a second sequence space
    /// into the host's reply stream and make the next motors read as drops.
    fn reply_ack(&mut self, acked: u32) -> Vec<u8> {
        let mut b = [0u8; falcon_hitl::ACK_FRAME_LEN];
        wire::encode_ack(&mut b, acked, acked);
        b.to_vec()
    }

    /// Bytes in, bytes out (`None` = no reply).
    fn handle(&mut self, frame: &[u8]) -> Option<Vec<u8>> {
        let (h, payload) = match wire::decode_frame(frame) {
            Ok(x) => x,
            Err(e) => {
                // docs/TRANSPORT-BINDING.md: a rejected `calibration` or
                // `supervisor-config` frame says the latest thing the host
                // sent was invalid, whatever the reason the decoder gave
                // (#638 finding 10: this used to revoke on `NonFinite` only,
                // so a `BadLength` kept the old configuration). The rejected
                // frame's values never reach us, so a non-finite record
                // stands in for it and goes through the host AS AN INVALID
                // configuration. relay#611's host rules then decide: a
                // calibration revokes only while Disarmed, a supervisor
                // config rebuilds only while `rebuild_permitted`.
                match rejected_type(&e) {
                    Some(MsgType::Calibration) => {
                        self.seam.configure_calibration(CalParams {
                            gyro_bias: [f32::NAN; 3],
                            ..CalParams::identity()
                        });
                    }
                    Some(MsgType::SupervisorConfig) => {
                        self.seam.configure_supervisor(host::SupervisorConfig {
                            home: [f32::NAN; 3],
                            fence_radius: f32::NAN,
                            cruise_alt: f32::NAN,
                            low_batt_v: f32::NAN,
                        });
                    }
                    // Includes a rejected vehicle config: an invalid tuning is
                    // never installed and does not revoke, so dropping it IS
                    // the host rule. A frame whose type cannot be read (short,
                    // bad magic, version or type byte) names no configuration.
                    _ => {}
                }
                return None;
            }
        };
        let verdict = self.inbound.observe(h.seq);
        let fresh = !matches!(verdict, TickVerdict::Duplicate | TickVerdict::Late { .. });
        match h.msg {
            MsgType::Config => {
                if fresh {
                    let v = wire::decode_config(payload).unwrap();
                    self.seam.configure_vehicle(vehicle_config(&v));
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::SupervisorConfig => {
                if fresh {
                    let c = wire::decode_supervisor_config(payload).unwrap();
                    self.seam.configure_supervisor(supervisor_config(&c));
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Calibration => {
                if fresh {
                    let c = wire::decode_calibration(payload).unwrap();
                    self.seam.configure_calibration(cal_params(&c));
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Command => {
                if fresh {
                    let c = wire::decode_command(payload).unwrap();
                    self.seam.command(event(c.cmd), c.level, c.throttle_low);
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Sensor => {
                if !fresh {
                    return None;
                }
                let f = wire::decode_sensor_frame(payload).unwrap();
                let mut b = FrameBackend {
                    f,
                    motors: [0.0; 4],
                };
                self.seam.step(self.target, &mut b);
                // THE ECHO RULE: the reply carries the seq of the sensor
                // frame it answers, never a counter of the endpoint's own.
                let mut out = [0u8; falcon_hitl::MOTORS_FRAME_LEN];
                wire::encode_motors(&mut out, h.seq, b.motors);
                Some(out.to_vec())
            }
            MsgType::Ack | MsgType::Motors => None,
        }
    }
}

// ── the host side: the reference BindingHost and a plant with a battery ─────

/// `SimBackend` has no ground: stepped with zero thrust it free-falls, which
/// would turn a ground warm-up into a 10 m drop. So the plant RESTS ON THE
/// GROUND — its state is held, and it reports the at-rest gravity reaction —
/// until a positive motor command first arrives, and is a free body from then
/// on. That is the only physics this adds; it makes no claim about touchdown.
struct Plant {
    sim: SimBackend,
    battery_v: Option<f32>,
    on_ground: bool,
}

impl FlightBackend for Plant {
    fn read_imu(&mut self) -> ImuSample {
        self.sim.read_imu()
    }
    fn read_position(&mut self) -> Option<[f32; 3]> {
        self.sim.read_position()
    }
    fn read_mag(&mut self) -> Option<[f32; 3]> {
        self.sim.read_mag()
    }
    fn write_motors(&mut self, m: &[f32]) {
        if self.on_ground && m.iter().all(|&x| x <= 0.0) {
            return;
        }
        self.on_ground = false;
        self.sim.write_motors(m)
    }
    fn dt(&self) -> f32 {
        self.sim.dt()
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.battery_v
    }
}

const DT: f32 = 0.002;
const WARM_TICKS: usize = 1500;
const FLY_TICKS: usize = 8000;

/// Send one control frame and require its ack.
fn control(host: &mut BindingHost, ep: &mut Endpoint, frame: &[u8]) {
    let reply = ep.handle(frame).expect("a control frame is acked");
    assert!(
        matches!(host.on_reply(&reply), Ok(HostReply::Acked { .. })),
        "the ack must close the handshake"
    );
    assert_eq!(host.awaiting_ack(), None);
}

/// One tick: plant -> sensor frame -> endpoint -> motors frame -> plant.
/// Returns the motors the plant was given.
fn tick(host: &mut BindingHost, ep: &mut Endpoint, plant: &mut Plant) -> [f32; 4] {
    let f = sample_sensor_frame(plant);
    let bytes = host.sensor(&f).expect("no control frame outstanding");
    let reply = ep.handle(&bytes).expect("a sensor frame is answered");
    match host.on_reply(&reply) {
        Ok(HostReply::Motors { motors, .. }) => {
            plant.write_motors(&motors);
            motors
        }
        other => panic!("expected motors, got {other:?}"),
    }
}

struct Run {
    supervisor_config: bool,
    calibration: bool,
    battery: bool,
}

struct Outcome {
    blocked: Option<CheckId>,
    mode_after_arm: Mode,
    start_d: f32,
    end_d: f32,
}

fn fly(r: Run) -> Outcome {
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = Plant {
        sim: SimBackend::new(level, DT),
        battery_v: r.battery.then_some(16.4),
        on_ground: true,
    };
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();

    // 1. config
    let f = host
        .configure(&WireVehicleConfig {
            hover_thrust: 0.5,
            loop_rate_hz: 1.0 / DT,
            pos_var: 0.01,
            process_floor_vel: 0.0,
            process_floor_pos: 0.0,
            altitude_kp: 0.05,
            altitude_kd: 0.30,
            altitude_ki: 0.0,
            position_ki: 0.02,
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    // 2. supervisor-config: cruise-alt is metres AGL, POSITIVE (WIT).
    if r.supervisor_config {
        let f = host
            .configure_supervisor(&WireSupervisorConfig {
                home: [0.0, 0.0, 0.0],
                fence_radius: 50.0,
                cruise_alt: 2.0,
                low_batt_v: 14.0,
            })
            .unwrap();
        control(&mut host, &mut ep, &f);
    }
    // 3. calibration: a small non-identity gyro bias (rad/s) — what a
    // gyro-null procedure solves. (An explicit identity would also arm: it
    // means "calibrated upstream"; withholding the frame is what refuses.)
    if r.calibration {
        let f = host
            .configure_calibration(&WireCalibration {
                gyro_bias: [0.001, 0.0, 0.0],
                accel_bias: [0.0; 3],
                accel_scale: [1.0; 3],
                mag_offset: [0.0; 3],
                mag_scale: [1.0; 3],
                mag_offdiag: [0.0; 3],
            })
            .unwrap();
        control(&mut host, &mut ep, &f);
    }
    // 4. warm the estimator on the ground — arming AFTER stepping, the way a
    // GCS does, so this does not lean on the arm-before-first-step defaults
    // path that relay#611 closes.
    for _ in 0..WARM_TICKS {
        tick(&mut host, &mut ep, &mut plant);
    }
    let start_d = plant.sim.pos[2];
    // 5. arm, interlocks held
    let f = host
        .command(&WireCommand {
            cmd: WirePilotCommand::Arm,
            level: true,
            throttle_low: true,
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    let blocked = ep.supervisor().arm_blocked_check();
    let mode_after_arm = ep.supervisor().mode();
    // 6. take off
    let f = host
        .command(&WireCommand {
            cmd: WirePilotCommand::RequestTakeoff,
            level: true,
            throttle_low: true,
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    for _ in 0..FLY_TICKS {
        tick(&mut host, &mut ep, &mut plant);
    }
    Outcome {
        blocked,
        mode_after_arm,
        start_d,
        end_d: plant.sim.pos[2],
    }
}

/// THE CLAIM: configure, calibrate, arm and take off with nothing but frames.
#[test]
fn a_binding_only_host_configures_calibrates_arms_and_takes_off() {
    let o = fly(Run {
        supervisor_config: true,
        calibration: true,
        battery: true,
    });
    assert_eq!(
        o.blocked, None,
        "the pre-arm table must pass: {:?}",
        o.blocked
    );
    assert_eq!(o.mode_after_arm, Mode::Armed);
    // NED: climbing means pos-d DECREASES. cruise-alt 2.0 m AGL.
    assert!(
        o.end_d < o.start_d - 1.0,
        "the vehicle must climb: pos-d {} -> {}",
        o.start_d,
        o.end_d
    );
    assert!(
        (o.end_d - (-2.0)).abs() < 0.5,
        "and hold near 2 m AGL (-2.0 NED): {}",
        o.end_d
    );
}

/// CONTROL: withhold the `calibration` frame and the identical run is refused
/// on `CalibrationPresent` — the row the v1 binding could never satisfy.
#[test]
fn without_the_calibration_frame_arming_is_refused_on_calibration_present() {
    let o = fly(Run {
        supervisor_config: true,
        calibration: false,
        battery: true,
    });
    assert_eq!(o.blocked, Some(CheckId::CalibrationPresent));
    assert_eq!(o.mode_after_arm, Mode::Disarmed);
    assert!(
        o.end_d >= o.start_d - 0.05,
        "it must not climb: {} -> {}",
        o.start_d,
        o.end_d
    );
}

/// CONTROL (#572): a host whose battery element says ABSENT is refused on
/// `BatteryOk` — absence is never read as a healthy pack.
#[test]
fn with_the_battery_element_absent_arming_is_refused_on_battery_ok() {
    let o = fly(Run {
        supervisor_config: true,
        calibration: true,
        battery: false,
    });
    assert_eq!(o.blocked, Some(CheckId::BatteryOk));
    assert_eq!(o.mode_after_arm, Mode::Disarmed);
    assert!(
        o.end_d >= o.start_d - 0.05,
        "it must not climb: {} -> {}",
        o.start_d,
        o.end_d
    );
}

/// CONTROL: withhold `supervisor-config` and arming is refused on
/// `FailsafeConfigured` — a supervisor on defaults must not fly.
#[test]
fn without_the_supervisor_config_frame_arming_is_refused_on_failsafe_configured() {
    let o = fly(Run {
        supervisor_config: false,
        calibration: true,
        battery: true,
    });
    assert_eq!(o.blocked, Some(CheckId::FailsafeConfigured));
    assert_eq!(o.mode_after_arm, Mode::Disarmed);
    assert!(
        o.end_d >= o.start_d - 0.05,
        "it must not climb: {} -> {}",
        o.start_d,
        o.end_d
    );
}

/// #527 x relay#611: a `calibration` frame carrying NaN is REJECTED by the
/// decoder (no ack) — and, because the latest calibration the host sent was
/// invalid, the endpoint drops to the identity rather than keep the earlier
/// valid one, so arming is refused on `CalibrationPresent`.
#[test]
fn a_nan_calibration_frame_is_not_acked_and_revokes_the_earlier_calibration() {
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();
    let good = WireCalibration {
        gyro_bias: [0.001, 0.0, 0.0],
        accel_bias: [0.0; 3],
        accel_scale: [1.0; 3],
        mag_offset: [0.0; 3],
        mag_scale: [1.0; 3],
        mag_offdiag: [0.0; 3],
    };
    let f = host.configure_calibration(&good).unwrap();
    control(&mut host, &mut ep, &f);
    assert!(ep.seam.calibration().is_some());

    let bad = WireCalibration {
        accel_scale: [1.0, f32::NAN, 1.0],
        ..good
    };
    let f = host.configure_calibration(&bad).unwrap();
    assert_eq!(
        wire::decode_frame(&f).err(),
        Some(WireError::NonFinite {
            msg: MsgType::Calibration,
            offset: 28
        })
    );
    assert_eq!(ep.handle(&f), None, "a rejected frame is not acked");
    assert_eq!(
        ep.seam.calibration(),
        None,
        "the earlier calibration must not survive an invalid one"
    );
    // Build a supervisor now. Not through the binding: the host is still
    // awaiting the ack the rejected frame never got. A harmless command
    // straight to the endpoint's `SupervisorHost` builds it the same way.
    ep.seam.command(Event::RequestDisarm, true, true);
    assert!(
        !ep.supervisor().calibration_configured(),
        "and a supervisor built now must be uncalibrated, so CalibrationPresent refuses"
    );
}

/// #638 finding 10: the revocation follows from the REJECTION, not from its
/// reason. A `calibration` frame one byte short is `BadLength`, and must
/// revoke the earlier calibration exactly as a NaN one does; the same holds
/// for `supervisor-config`.
#[test]
fn a_wrong_length_calibration_or_supervisor_frame_also_revokes() {
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();
    let good = WireCalibration {
        gyro_bias: [0.001, 0.0, 0.0],
        accel_bias: [0.0; 3],
        accel_scale: [1.0; 3],
        mag_offset: [0.0; 3],
        mag_scale: [1.0; 3],
        mag_offdiag: [0.0; 3],
    };
    let f = host.configure_calibration(&good).unwrap();
    control(&mut host, &mut ep, &f);
    assert!(ep.seam.calibration().is_some());

    let f = host.configure_calibration(&good).unwrap();
    let short = &f[..f.len() - 1];
    assert!(matches!(
        wire::decode_frame(short),
        Err(WireError::BadLength {
            msg: MsgType::Calibration,
            ..
        })
    ));
    assert_eq!(ep.handle(short), None, "a rejected frame is not acked");
    assert_eq!(
        ep.seam.calibration(),
        None,
        "a wrong-length calibration must revoke the earlier one, like a NaN one"
    );

    // supervisor-config: a valid one builds the configuration, a short one
    // then counts as none.
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();
    let sc = wire::WireSupervisorConfig {
        home: [0.0; 3],
        fence_radius: 50.0,
        cruise_alt: 2.0,
        low_batt_v: 14.0,
    };
    let f = host.configure_supervisor(&sc).unwrap();
    control(&mut host, &mut ep, &f);
    assert!(ep.seam.failsafe_configured());
    let f = host.configure_supervisor(&sc).unwrap();
    let short = &f[..f.len() - 1];
    assert_eq!(ep.handle(short), None, "a rejected frame is not acked");
    assert!(
        !ep.seam.failsafe_configured(),
        "a wrong-length supervisor config must count as none"
    );
}

/// THE ECHO RULE (jess#167, #614): every component -> host reply's header
/// sequence is the sequence of the host frame it answers — a `motors` reply
/// carries its `sensor` frame's seq, an `ack` carries its control frame's seq
/// (in the header AND, unchanged, in the payload). So a host can index a reply
/// to its tick under loss instead of guessing by arrival order.
///
/// The run DROPS one `sensor` frame mid-stream (the host sent it; the
/// component never saw it) and interleaves a control frame later. On every
/// reply: the header seq equals the request's seq, and the host's reply-stream
/// verdicts are `InOrder` everywhere except exactly one `Dropped { lost: 1 }`
/// on the first reply after the drop — still `InOrder` after the interleaved
/// control frame, which is what catches an endpoint that echoes motors but
/// numbers its acks from a counter of its own.
#[test]
fn every_reply_echoes_the_seq_of_the_frame_it_answers_across_a_dropped_sensor_frame() {
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = Plant {
        sim: SimBackend::new(level, DT),
        battery_v: Some(16.4),
        on_ground: true,
    };
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();

    // A control frame, checked for the echo in the ack's HEADER and payload.
    let send_control = |host: &mut BindingHost, ep: &mut Endpoint, frame: &[u8]| {
        let (req, _) = wire::decode_frame(frame).unwrap();
        let reply = ep.handle(frame).expect("a control frame is acked");
        let (h, p) = wire::decode_frame(&reply).unwrap();
        assert_eq!(h.msg, MsgType::Ack);
        assert_eq!(h.seq, req.seq, "the ack's header echoes the control seq");
        assert_eq!(wire::decode_ack(p).unwrap(), req.seq, "and its payload");
        assert_eq!(host.on_reply(&reply), Ok(HostReply::Acked { seq: req.seq }));
    };

    let f = host
        .configure_supervisor(&WireSupervisorConfig {
            home: [0.0; 3],
            fence_radius: 50.0,
            cruise_alt: 2.0,
            low_batt_v: 14.0,
        })
        .unwrap();
    send_control(&mut host, &mut ep, &f);

    const DROP_AT: usize = 7;
    const CONTROL_AT: usize = 12;
    let mut dropped_seen = 0;
    let mut answered = 0;
    for i in 0..20 {
        if i == CONTROL_AT {
            let f = host
                .command(&WireCommand {
                    cmd: WirePilotCommand::RequestDisarm,
                    level: true,
                    throttle_low: true,
                })
                .unwrap();
            send_control(&mut host, &mut ep, &f);
        }
        let sensors = sample_sensor_frame(&mut plant);
        let bytes = host.sensor(&sensors).unwrap();
        let (req, _) = wire::decode_frame(&bytes).unwrap();
        if i == DROP_AT {
            continue; // lost on the way to the component
        }
        let reply = ep.handle(&bytes).expect("a fresh sensor frame is answered");
        let (h, _) = wire::decode_frame(&reply).unwrap();
        assert_eq!(h.msg, MsgType::Motors);
        assert_eq!(
            h.seq, req.seq,
            "tick {i}: the motors reply must carry the seq of the sensor frame it answers"
        );
        match host.on_reply(&reply) {
            Ok(HostReply::Motors { seq, verdict, .. }) => {
                assert_eq!(seq, req.seq, "tick {i}: HostReply names the answered tick");
                if i == DROP_AT + 1 {
                    assert_eq!(verdict, TickVerdict::Dropped { lost: 1 }, "tick {i}");
                    dropped_seen += 1;
                } else {
                    assert_eq!(verdict, TickVerdict::InOrder, "tick {i}");
                }
            }
            other => panic!("tick {i}: expected motors, got {other:?}"),
        }
        answered += 1;
    }
    assert_eq!(dropped_seen, 1, "exactly one drop reported");
    assert_eq!(answered, 19, "every delivered sensor frame was answered");
}

// ── the frame target over a binding that carries none (#609 finding 4) ──────

/// Send `cmd` over the binding.
fn pilot(host: &mut BindingHost, ep: &mut Endpoint, cmd: WirePilotCommand) {
    let f = host
        .command(&WireCommand {
            cmd,
            level: true,
            throttle_low: true,
        })
        .unwrap();
    control(host, ep, &f);
}

/// Configure, calibrate, warm, arm and take off over the binding, with the
/// endpoint delivering `target` every frame, then fly until the supervisor
/// settles into Loiter. Returns the rig and the motors of every tick.
fn to_loiter(target: [f32; 3]) -> (BindingHost, Endpoint, Plant, Vec<[u32; 4]>) {
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut plant = Plant {
        sim: SimBackend::new(level, DT),
        battery_v: Some(16.4),
        on_ground: true,
    };
    let mut host = BindingHost::new();
    let mut ep = Endpoint::new();
    ep.target = target;
    let f = host
        .configure(&WireVehicleConfig {
            hover_thrust: 0.5,
            loop_rate_hz: 1.0 / DT,
            pos_var: 0.01,
            process_floor_vel: 0.0,
            process_floor_pos: 0.0,
            altitude_kp: 0.05,
            altitude_kd: 0.30,
            altitude_ki: 0.0,
            position_ki: 0.02,
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    let f = host
        .configure_supervisor(&WireSupervisorConfig {
            home: [0.0, 0.0, 0.0],
            fence_radius: 50.0,
            cruise_alt: 2.0,
            low_batt_v: 14.0,
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    let f = host
        .configure_calibration(&WireCalibration {
            gyro_bias: [0.001, 0.0, 0.0],
            accel_bias: [0.0; 3],
            accel_scale: [1.0; 3],
            mag_offset: [0.0; 3],
            mag_scale: [1.0; 3],
            mag_offdiag: [0.0; 3],
        })
        .unwrap();
    control(&mut host, &mut ep, &f);
    let mut log = Vec::new();
    let mut step = |host: &mut BindingHost, ep: &mut Endpoint, plant: &mut Plant| {
        let m = tick(host, ep, plant);
        log.push(m.map(f32::to_bits));
    };
    for _ in 0..WARM_TICKS {
        step(&mut host, &mut ep, &mut plant);
    }
    pilot(&mut host, &mut ep, WirePilotCommand::Arm);
    assert_eq!(
        ep.supervisor().mode(),
        Mode::Armed,
        "{:?}",
        ep.supervisor().arm_blocked_check()
    );
    pilot(&mut host, &mut ep, WirePilotCommand::RequestTakeoff);
    for _ in 0..FLY_TICKS {
        step(&mut host, &mut ep, &mut plant);
        if ep.supervisor().mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(
        ep.supervisor().mode(),
        Mode::Loiter,
        "never settled into Loiter"
    );
    // A settled hover before anything else is asked.
    for _ in 0..1000 {
        step(&mut host, &mut ep, &mut plant);
    }
    (host, ep, plant, log)
}

/// OUTSIDE MISSION THE FRAME TARGET IS IGNORED, so the endpoint's switch to
/// the non-finite "no target" changes nothing a binding host flies today:
/// warm-up, arm, takeoff and a settled Loiter command the SAME motors, bit
/// for bit, as with the old `[0, 0, 0]`. What it does change is the live
/// failsafe bit 8, which is now up for the whole run, because no target IS
/// in force (the binding has no state frame, so no binding host sees it).
#[test]
fn outside_mission_the_non_finite_target_flies_bit_identically_to_the_old_constant() {
    let (_, ep_new, plant_new, new) = to_loiter(NO_TARGET);
    let (_, ep_old, plant_old, old) = to_loiter(OLD_HOME_GROUND_TARGET);
    assert_eq!(
        new.len(),
        old.len(),
        "the two runs reached Loiter at different ticks"
    );
    if let Some(i) = (0..new.len()).find(|&i| new[i] != old[i]) {
        panic!("motors differ at tick {i}: {:?} vs {:?}", new[i], old[i]);
    }
    assert_eq!(
        plant_new.sim.pos.map(f32::to_bits),
        plant_old.sim.pos.map(f32::to_bits)
    );
    let stale = falcon_core::failsafe_flags::TARGET_STALE;
    assert_ne!(
        ep_new.supervisor().failsafe_flags() & stale,
        0,
        "no target is in force"
    );
    assert_eq!(ep_old.supervisor().failsafe_flags() & stale, 0);
    // And nothing else in the published cause differs.
    assert_eq!(
        ep_new.supervisor().failsafe_flags() & !stale,
        ep_old.supervisor().failsafe_flags() & !stale
    );
}

/// THE DECISION: a binding host that sends `request-mission` has given no
/// target, so the supervisor takes the stale-target HOLD on the first Mission
/// tick — Loiter, latched where the vehicle is — and the vehicle stays at its
/// 2 m hover.
#[test]
fn request_mission_over_the_binding_takes_the_stale_target_hold() {
    let (mut host, mut ep, mut plant, _) = to_loiter(NO_TARGET);
    let at = plant.sim.pos;
    pilot(&mut host, &mut ep, WirePilotCommand::RequestMission);
    // The command is accepted (Mission), and the first step takes the HOLD.
    assert_eq!(ep.supervisor().mode(), Mode::Mission);
    tick(&mut host, &mut ep, &mut plant);
    assert_eq!(
        ep.supervisor().mode(),
        Mode::Loiter,
        "the HOLD on the first Mission tick"
    );
    let stale = falcon_core::failsafe_flags::TARGET_STALE;
    assert_ne!(
        ep.supervisor().failsafe_flags() & stale,
        0,
        "with its cause, bit 8"
    );
    let mut worst = 0.0f32;
    for _ in 0..(20.0 / DT) as usize {
        tick(&mut host, &mut ep, &mut plant);
        assert_eq!(ep.supervisor().mode(), Mode::Loiter);
        let p = plant.sim.pos;
        worst = worst
            .max((p[0] - at[0]).hypot(p[1] - at[1]))
            .max((p[2] - at[2]).abs());
    }
    eprintln!("binding request-mission HOLD: worst deviation {worst:.3} m over 20 s");
    assert!(worst < 0.5, "the HOLD stays where it was asked: {worst} m");
}

/// NEGATIVE CONTROL: with the old `[0, 0, 0]` (home at GROUND level) the
/// same `request-mission` is a FRESH live target every frame. The vehicle
/// stays in Mission, never goes stale, and descends toward the ground.
#[test]
fn request_mission_with_the_old_constant_descends_toward_the_ground() {
    let (mut host, mut ep, mut plant, _) = to_loiter(OLD_HOME_GROUND_TARGET);
    let start_d = plant.sim.pos[2];
    pilot(&mut host, &mut ep, WirePilotCommand::RequestMission);
    for _ in 0..(20.0 / DT) as usize {
        tick(&mut host, &mut ep, &mut plant);
        assert_eq!(
            ep.supervisor().mode(),
            Mode::Mission,
            "the old constant never goes stale"
        );
    }
    let end_d = plant.sim.pos[2];
    eprintln!("old constant in Mission: pos-d {start_d:.3} -> {end_d:.3} m in 20 s");
    assert!(
        end_d > start_d + 1.0,
        "the old constant must be seen descending (pos-d {start_d} -> {end_d})"
    );
}
