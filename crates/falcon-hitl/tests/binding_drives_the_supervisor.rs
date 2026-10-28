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
//! WHAT THIS IS NOT. It is not the published wasm component. That component
//! gains `pilot.configure-calibration` only in relay#611, and it has no byte
//! endpoint at all; the WIT call it would forward the `Calibration` frame to
//! does not exist on `main` yet. `Endpoint` therefore calls `FlightSupervisor`
//! directly, mirroring the published component only where this test depends
//! on it (lazy construction, `FailsafeConfigured` declared from whether a
//! supervisor configuration arrived and re-declared after every step, the
//! calibration surviving a rebuild), and following relay#611's documented
//! semantics for an INVALID calibration / supervisor configuration.

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

/// The calibration frame -> the native `CalParams` on THIS base.
///
/// WHAT WAITS ON relay#611's AMENDMENT: the frame carries `mag_offdiag`
/// (payload 60/64/68), but `relay_calib::CalParams` has no off-diagonal field
/// yet, so it is NOT forwarded here — the five diagonal-model fields are. Until
/// `CalParams` gains it, this endpoint (a) does not apply soft-iron
/// off-diagonal correction, and (b) judges "calibrated" without it: a
/// calibration that differs from the identity ONLY in `mag_offdiag` reads as
/// not calibrated (`CalibrationPresent` refuses arming).
fn cal_params(c: &WireCalibration) -> CalParams {
    CalParams {
        gyro_bias: c.gyro_bias,
        accel_bias: c.accel_bias,
        accel_scale: c.accel_scale,
        mag_offset: c.mag_offset,
        mag_scale: c.mag_scale,
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

/// The component side of the binding, natively.
struct Endpoint {
    sup: Option<Box<FlightSupervisor>>,
    vcfg: Option<WireVehicleConfig>,
    scfg: Option<WireSupervisorConfig>,
    cal: CalParams,
    inbound: SeqTracker,
    // No outbound counter: every reply echoes the seq of the frame it
    // answers (docs/TRANSPORT-BINDING.md §3, the echo rule).
}

impl Endpoint {
    fn new() -> Self {
        Endpoint {
            sup: None,
            vcfg: None,
            scfg: None,
            cal: CalParams::identity(),
            inbound: SeqTracker::new(),
        }
    }

    fn supervisor(&self) -> &FlightSupervisor {
        self.sup
            .as_deref()
            .expect("built by the first command/sensor frame")
    }

    fn ensure(&mut self) -> &mut FlightSupervisor {
        if self.sup.is_none() {
            // Unconfigured: the published component's inactive defaults.
            let (home, fence, cruise, low) = match self.scfg {
                Some(c) => (c.home, c.fence_radius, c.cruise_alt, c.low_batt_v),
                None => ([0.0; 3], 1.0e9, 2.0, 0.0),
            };
            let mut s = Box::new(FlightSupervisor::new(home, fence, cruise, low));
            if let Some(v) = self.vcfg {
                s.set_hover_thrust(v.hover_thrust);
                let core = s.core_mut();
                core.set_pos_var(v.pos_var);
                core.set_process_floor(v.process_floor_vel, v.process_floor_pos);
                core.set_altitude_gains(v.altitude_kp, v.altitude_kd);
                core.set_altitude_integral_gain(v.altitude_ki);
                core.set_position_integral_gain(v.position_ki);
            }
            // The calibration survives a rebuild (relay#611).
            s.set_calibration(self.cal);
            self.sup = Some(s);
        }
        let configured = self.scfg.is_some();
        let s = self.sup.as_deref_mut().unwrap();
        s.set_check(CheckId::FailsafeConfigured, configured);
        s
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
            // relay#611: the latest calibration the host sent was invalid, so
            // the vehicle must not keep flying on an older one — the identity
            // is installed and `CalibrationPresent` refuses arming.
            Err(WireError::NonFinite {
                msg: MsgType::Calibration,
                ..
            }) => {
                self.cal = CalParams::identity();
                if let Some(s) = self.sup.as_deref_mut() {
                    s.set_calibration(self.cal);
                }
                return None;
            }
            // relay#611: an invalid supervisor configuration is treated as
            // never configured — `FailsafeConfigured` refuses arming.
            Err(WireError::NonFinite {
                msg: MsgType::SupervisorConfig,
                ..
            }) => {
                self.scfg = None;
                self.sup = None;
                return None;
            }
            Err(_) => return None,
        };
        let verdict = self.inbound.observe(h.seq);
        let fresh = !matches!(verdict, TickVerdict::Duplicate | TickVerdict::Late { .. });
        match h.msg {
            MsgType::Config => {
                if fresh {
                    self.vcfg = Some(wire::decode_config(payload).unwrap());
                    self.sup = None;
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::SupervisorConfig => {
                if fresh {
                    self.scfg = Some(wire::decode_supervisor_config(payload).unwrap());
                    self.sup = None;
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Calibration => {
                if fresh {
                    self.cal = cal_params(&wire::decode_calibration(payload).unwrap());
                    let cal = self.cal;
                    if let Some(s) = self.sup.as_deref_mut() {
                        s.set_calibration(cal);
                    }
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Command => {
                if fresh {
                    let c = wire::decode_command(payload).unwrap();
                    self.ensure().command(event(c.cmd), c.level, c.throttle_low);
                }
                Some(self.reply_ack(h.seq))
            }
            MsgType::Sensor => {
                if !fresh {
                    return None;
                }
                let f = wire::decode_sensor_frame(payload).unwrap();
                let configured = self.scfg.is_some();
                let mut b = FrameBackend {
                    f,
                    motors: [0.0; 4],
                };
                let s = self.ensure();
                s.step(&mut b);
                // `update_preflight` inside `step` clobbers this row from a
                // default-true field; re-declare it, as the component does.
                s.set_check(CheckId::FailsafeConfigured, configured);
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
fn tick(host: &mut BindingHost, ep: &mut Endpoint, plant: &mut Plant) {
    let f = sample_sensor_frame(plant);
    let bytes = host.sensor(&f).expect("no control frame outstanding");
    let reply = ep.handle(&bytes).expect("a sensor frame is answered");
    match host.on_reply(&reply) {
        Ok(HostReply::Motors { motors, .. }) => plant.write_motors(&motors),
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
    // gyro-null procedure solves; identity would read as "not calibrated".
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
    assert_ne!(ep.cal, CalParams::identity());

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
        ep.cal,
        CalParams::identity(),
        "the earlier calibration must not survive an invalid one"
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
