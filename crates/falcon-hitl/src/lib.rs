//! # falcon-hitl — the hardware-in-the-loop link backend (v1.14.0)
//!
//! The third member of the backend family, and the one that realises your
//! "**hardware with the simulation as the backend**" topology:
//!
//! | backend | where the sensors/actuators come from |
//! |---|---|
//! | `SimBackend` (v1.1) | simulation, in-process |
//! | `HardwareBackend` (v1.11) | real sensors, via driver traits |
//! | **`LinkBackend` (v1.14)** | **a remote simulator (or real vehicle) over a byte-framed link** |
//!
//! The flight computer runs the SAME verified [`FlightCore`]/`FlightSupervisor`
//! with a [`LinkBackend`]; each control step it sends a 16-byte actuator frame
//! (the four motor commands) and receives a 54-byte sensor frame (accel, gyro,
//! position+valid, mag+valid, battery). A [`SimServer`] on the other end
//! decodes the actuator frame, steps a `SimBackend`, and encodes the sensor
//! frame back. Put the flight computer on a real board and the simulator on a
//! host PC, and this is HITL; point the link at a real vehicle's telemetry
//! bridge instead and the same code flies the aircraft.
//!
//! The wire format is fixed-layout little-endian `f32`s — no `serde`, no
//! `alloc`, `no_std`-clean — so it runs unchanged on the MCU.
//!
//! ## Honest scope
//!
//! The [`Transport`] here is exercised by an in-process loopback (the closed
//! loop flies over the real frame encode/decode). The documented GAP is the
//! *physical* transport — a real UART/USB/UDP link and its latency, jitter,
//! framing-error and reconnect handling. The protocol and the closed-loop
//! contract are real; the wire underneath them is the integration step.

#![no_std]

/// The published `pulseengine:falcon-cascade` transport binding
/// (SWREQ-FALCON-TRANSPORT-P01) — framing, the configuration handshake, the
/// tick contract and the sequence semantics.
///
/// The legacy 16/54-byte `encode_actuator`/`encode_sensor` frames below are
/// SUPERSEDED by this module and kept only so existing callers keep building.
/// They cannot carry heading, rotor RPM, or a battery that says ABSENT (#466) —
/// the exact fields the seam learned it needed at v0.8 and v0.10 by being
/// driven. New hosts use `wire`.
pub mod wire;

// Kani harnesses for the wire decoder, in a PLAIN SIBLING rather than inside
// `wire` itself: `cargo build` and `cargo test` cannot see a break in
// `cfg(kani)` code, so these are exercised with `cargo kani`, and the crate is
// in .github/workflows/kani.yml so the proofs are ENFORCED rather than merely
// authored — the orphaned-verified-leaf pattern this release line keeps finding.
#[cfg(kani)]
mod kani_proofs;

use falcon_core::{FlightBackend, ImuSample};

/// `[f32; 3]`, matching `falcon-core`'s vector type.
pub type Vec3 = [f32; 3];

/// Bytes in a `Motors` frame on the wire: the 8-byte binding header plus four
/// `f32` commands.
///
/// DELIBERATELY NOT CALLED `ACTUATOR_FRAME_LEN`. That name meant 16 and now the
/// frame is 24; re-using it with a new value is the kind of change that fails
/// SILENTLY in a caller that happens to compile. A new name fails loudly.
pub const MOTORS_FRAME_LEN: usize = wire::HEADER_LEN + wire::MOTORS_PAYLOAD_LEN;
/// Bytes in a `Sensor` frame on the wire: header plus the payload (81 B at
/// binding v2, which appended `battery-v`; 76 B at v1).
///
/// Same reasoning — the retired `SENSOR_FRAME_LEN` meant 54.
pub const SENSOR_WIRE_FRAME_LEN: usize = wire::HEADER_LEN + wire::SENSOR_PAYLOAD_LEN;
/// Bytes in a `Config` frame (`vehicle-config`).
pub const CONFIG_FRAME_LEN: usize = wire::HEADER_LEN + wire::CONFIG_PAYLOAD_LEN;
/// Bytes in an `Ack` frame.
pub const ACK_FRAME_LEN: usize = wire::HEADER_LEN + wire::ACK_PAYLOAD_LEN;
/// Bytes in a `SupervisorConfig` frame (binding v2).
pub const SUPERVISOR_CONFIG_FRAME_LEN: usize =
    wire::HEADER_LEN + wire::SUPERVISOR_CONFIG_PAYLOAD_LEN;
/// Bytes in a `Calibration` frame (binding v2).
pub const CALIBRATION_FRAME_LEN: usize = wire::HEADER_LEN + wire::CALIBRATION_PAYLOAD_LEN;
/// Bytes in a `Command` frame (binding v2).
pub const COMMAND_FRAME_LEN: usize = wire::HEADER_LEN + wire::COMMAND_PAYLOAD_LEN;
/// The longest frame the binding defines — a receive buffer of this size holds
/// any of them.
pub const MAX_FRAME_LEN: usize = SENSOR_WIRE_FRAME_LEN;

/// The link transport: send one `Motors` frame, receive one `Sensor` frame.
/// Your impl wraps a UART/USB/UDP socket; the request/response shape models the
/// one-exchange-per-control-step HITL cadence.
///
/// Both buffers are now BINDING frames (`docs/TRANSPORT-BINDING.md`), not this
/// crate's private 16/54-byte pair — that is TRANSPORT-P01 element 5, "the
/// reference host". A host written against the published document now drives
/// this backend without inventing a framing.
pub trait Transport {
    fn exchange(&mut self, out: &[u8; MOTORS_FRAME_LEN], reply: &mut [u8; SENSOR_WIRE_FRAME_LEN]);
}

/// A [`FlightBackend`] that sources its sensors and sinks its actuators over a
/// [`Transport`] to a remote simulator (or vehicle). The verified `FlightCore`
/// is identical to the sim/hardware cases — only the backend differs.
pub struct LinkBackend<T> {
    transport: T,
    cache: wire::WireSensorFrame,
    seq: u32,
    tracker: wire::SeqTracker,
    /// `None` when the last reply was NOT a decodable `Sensor` frame.
    ///
    /// This started as a bare `TickVerdict` defaulting to `First` on a failed
    /// decode, which made GARBAGE look like a fresh stream start — and the
    /// tick contract is exactly the thing TRANSPORT-P01's falsification
    /// condition turns on, so a field that quietly reports the wrong verdict is
    /// worse here than no field.
    last_verdict: Option<wire::TickVerdict>,
    rejected: u32,
    misechoed: u32,
    dt: f32,
}

impl<T: Transport> LinkBackend<T> {
    /// Open the link with control period `dt`. Primes the sensor cache with one
    /// neutral exchange so the first `read_*` has live data.
    pub fn new(mut transport: T, dt: f32) -> Self {
        let mut out = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut out, 0, [0.0; 4]);
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];
        transport.exchange(&out, &mut reply);
        // Seeded one BEFORE the first request (seq 0, by wrap), so a lost
        // init exchange is counted by the next good reply instead of that
        // reply reading as a fresh `First`.
        let mut tracker = wire::SeqTracker::starting_at(u32::MAX);
        let r = Self::read_reply(&reply, 0, &mut tracker);
        LinkBackend {
            transport,
            cache: r.frame.unwrap_or_default(),
            seq: 0,
            tracker,
            rejected: u32::from(r.rejected),
            misechoed: u32::from(r.misechoed),
            last_verdict: r.verdict,
            dt,
        }
    }

    /// Decode one reply to the `Motors` frame numbered `sent`, classify its
    /// sequence, and hand back both.
    ///
    /// A frame that does not decode leaves the cache ALONE rather than zeroing
    /// it: the last good sensors are a better estimate for one tick than a
    /// synthetic zero reading, and zeroed accelerometers read as free-fall.
    ///
    /// THE ECHO RULE IS ENFORCED HERE (#614). The exchange is synchronous —
    /// one `Motors` out, one reply back — so the only reply that answers it is
    /// a `Sensor` frame whose header `seq` is `sent`. Any other seq is a frame
    /// this exchange never asked for (a stale datagram, another session, a
    /// plant numbering its replies from a counter of its own): it is REFUSED —
    /// not cached, not classified, counted in
    /// [`misechoed_replies`](Self::misechoed_replies). Before this, the seq
    /// went straight into the tracker, so a reply carrying seq 1000 to a
    /// request numbered 0 was cached and classified as a tick. Because only
    /// `sent` is accepted and `sent` advances by one per exchange, the
    /// tracker's `Dropped { lost }` counts exactly the exchanges since the last
    /// accepted reply whose own reply was refused or undecodable.
    fn read_reply(
        reply: &[u8; SENSOR_WIRE_FRAME_LEN],
        sent: u32,
        tracker: &mut wire::SeqTracker,
    ) -> ReplyRead {
        match wire::decode_frame(reply) {
            Ok((h, _)) if h.msg == wire::MsgType::Sensor && h.seq != sent => ReplyRead {
                frame: None,
                verdict: None,
                rejected: false,
                misechoed: true,
            },
            Ok((h, payload)) if h.msg == wire::MsgType::Sensor => {
                let verdict = tracker.observe(h.seq);
                // The header decoded and the sequence was classified; if the
                // payload did not, report the verdict and no frame.
                let frame = wire::decode_sensor_frame(payload).ok();
                ReplyRead {
                    frame,
                    verdict: Some(verdict),
                    rejected: false,
                    misechoed: false,
                }
            }
            // Not a frame, or not a Sensor frame. There is no sequence to
            // classify, so there is no verdict — NOT `First`.
            _ => ReplyRead {
                frame: None,
                verdict: None,
                rejected: true,
                misechoed: false,
            },
        }
    }

    /// The latest sensor frame (telemetry / tests).
    pub fn last(&self) -> wire::WireSensorFrame {
        self.cache
    }

    /// What the binding's tick contract says happened to the most recent reply.
    ///
    /// Exposed because TRANSPORT-P01's falsification condition is that a host
    /// "cannot tell a dropped tick from a late one". A backend that silently
    /// swallowed the distinction would satisfy the codec and fail the
    /// requirement.
    pub fn last_verdict(&self) -> Option<wire::TickVerdict> {
        self.last_verdict
    }

    /// How many replies were not decodable `Sensor` frames.
    ///
    /// Counted separately from the verdicts because a rejected reply is not a
    /// tick that went wrong — it is a tick that was never classified, and
    /// collapsing the two is how a link reports a healthy stream while
    /// dropping every frame.
    pub fn rejected_replies(&self) -> u32 {
        self.rejected
    }

    /// How many replies WERE decodable `Sensor` frames but did not echo the
    /// seq of the `Motors` frame they came back to (the echo rule, #614).
    ///
    /// Separate from [`rejected_replies`](Self::rejected_replies) for the same
    /// reason that one is separate from the verdicts: a well-formed frame that
    /// answers a different request is a different fault from line noise, and
    /// a cross-wired link should not look like a noisy one.
    pub fn misechoed_replies(&self) -> u32 {
        self.misechoed
    }
}

/// What [`LinkBackend::read_reply`] made of one reply.
struct ReplyRead {
    frame: Option<wire::WireSensorFrame>,
    verdict: Option<wire::TickVerdict>,
    /// Not a decodable `Sensor` frame header.
    rejected: bool,
    /// A `Sensor` frame whose seq is not the request's.
    misechoed: bool,
}

impl<T: Transport> FlightBackend for LinkBackend<T> {
    fn read_imu(&mut self) -> ImuSample {
        let i = self.cache.imu;
        ImuSample {
            accel: [i[0], i[1], i[2]],
            gyro: [i[3], i[4], i[5]],
        }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.cache.position_ned
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.cache.mag_body
    }
    fn write_motors(&mut self, motors: &[f32]) {
        // the HITL round-trip: actuate, then receive the resulting sensors.
        self.seq = self.seq.wrapping_add(1);
        let mut m = [0.0f32; 4];
        for (i, mi) in m.iter_mut().enumerate() {
            *mi = motors.get(i).copied().unwrap_or(0.0);
        }
        let mut out = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut out, self.seq, m);
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];
        self.transport.exchange(&out, &mut reply);
        let r = Self::read_reply(&reply, self.seq, &mut self.tracker);
        if r.rejected {
            self.rejected = self.rejected.saturating_add(1);
        }
        if r.misechoed {
            self.misechoed = self.misechoed.saturating_add(1);
        }
        self.last_verdict = r.verdict;
        if let Some(f) = r.frame {
            self.cache = f;
        }
    }
    fn dt(&self) -> f32 {
        // THE HOST'S ACTUAL PERIOD, which is what the binding's tick contract
        // says `dt-s` carries — "not its nominal one". The configured `dt` is
        // the fallback for a frame that has not arrived yet or reports a value
        // the component would clamp away anyway (the binding's own range is
        // [0.0001, 0.1] s).
        let d = self.cache.dt_s;
        if d.is_finite() && (0.0001..=0.1).contains(&d) {
            d
        } else {
            self.dt
        }
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        // WHAT THE FRAME SAID, INCLUDING "ABSENT" (binding v2, #572).
        //
        // The retired 54-byte frame carried a bare `battery: f32` with no
        // validity flag, so a host with no battery sense had to send a NUMBER
        // (it sent 0.0 V — a field that can only lie, #466). v1 of the binding
        // carried no battery at all, so this returned `None` unconditionally
        // and a supervised vehicle behind it could never arm. v2 carries
        // `battery-v` with its own presence byte, so `None` here now means the
        // HOST SAID absent, and `Some(0.0)` means it measured a dead pack —
        // which the battery failsafe must see rather than read as "no sense".
        self.cache.battery_v
    }
}

/// The simulator side of the link: decode a `Motors` frame, step the backing
/// `FlightBackend` (a `SimBackend`), and encode the resulting sensors. Generic
/// over the backend so the same server can wrap an injected-pathology sim.
///
/// This is TRANSPORT-P01's "reference host": it speaks the published binding,
/// so it doubles as the executable example of the document.
pub struct SimServer<B> {
    backend: B,
    tracker: wire::SeqTracker,
}

impl<B: FlightBackend> SimServer<B> {
    pub fn new(backend: B) -> Self {
        SimServer {
            backend,
            tracker: wire::SeqTracker::default(),
        }
    }

    /// Borrow the backing backend (to read its ground-truth state in tests).
    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Answer one exchange: apply the `Motors` frame (steps the plant), then
    /// sample the sensors into the reply frame.
    ///
    /// RETURNS A `Result` NOW, where the old 16/54-byte version could not fail.
    /// Framing is real: the bytes may not be a frame at all, and a reference
    /// host that silently treated garbage as zero thrust would be a worse
    /// example than one that refuses. The returned verdict is the tick contract.
    pub fn serve(
        &mut self,
        motors_frame: &[u8; MOTORS_FRAME_LEN],
        reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
    ) -> Result<wire::TickVerdict, wire::WireError> {
        let (h, payload) = wire::decode_frame(motors_frame)?;
        if h.msg != wire::MsgType::Motors {
            return Err(wire::WireError::UnknownType { got: h.msg as u8 });
        }
        let motors = wire::decode_motors(payload)?;
        let verdict = self.tracker.observe(h.seq);

        // NEITHER A DUPLICATE NOR A LATE FRAME MAY STEP THE PLANT, and the
        // binding states both in its own table rather than leaving it to a
        // host's judgement (docs/TRANSPORT-BINDING.md, "Late, dropped and
        // duplicated"):
        //
        //   seq == last          Duplicate   accept the bytes, DO NOT step the
        //                                    controller twice
        //   seq < last backward  Late{by}    DISCARD; a newer frame already
        //                                    superseded it
        //
        // Duplicate: a retransmitted datagram is the ordinary way this happens
        // on UDP, and stepping twice advances the simulation by two control
        // periods for one commanded tick — downstream that reads as a plant
        // moving faster than the loop flying it.
        //
        // Late: the commands in it are ALREADY SUPERSEDED. Stepping the plant
        // with them applies stale thrust after a newer command has been acted
        // on, which is worse than dropping the frame. This one is easy to miss
        // because the sequence tracker's doc only spells out the duplicate
        // obligation; the table in the document is the authority.
        //
        // Both are still ANSWERED, re-reporting the CURRENT sensors, so the
        // host is not left waiting on a frame it will never get.
        let stepped = !matches!(
            verdict,
            wire::TickVerdict::Duplicate | wire::TickVerdict::Late { .. }
        );
        if stepped {
            self.backend.write_motors(&motors); // steps the plant
        }

        // THE ECHO RULE, in this pair's INVERTED topology: here the flight
        // computer ([`LinkBackend`]) leads with `Motors` and the plant
        // answers with `Sensor`, so it is the `Sensor` reply that carries the
        // seq of the frame it answers. Same rule as §3 — a reply echoes its
        // request — with the roles swapped; `LinkBackend` numbers its own
        // requests because it is the side that asks.
        let frame = sample_sensor_frame(&mut self.backend);
        wire::encode_sensor_frame(reply, h.seq, &frame);
        Ok(verdict)
    }
}

/// Sample a plant's sensors into one binding `Sensor` payload.
///
/// The ONE place the reference host turns a [`FlightBackend`] into wire
/// fields, shared by [`SimServer`] and a [`BindingHost`] driving a plant, so
/// the two cannot disagree about what the plant reported.
pub fn sample_sensor_frame<B: FlightBackend>(backend: &mut B) -> wire::WireSensorFrame {
    let imu = backend.read_imu();
    wire::WireSensorFrame {
        imu: [
            imu.accel[0],
            imu.accel[1],
            imu.accel[2],
            imu.gyro[0],
            imu.gyro[1],
            imu.gyro[2],
        ],
        dt_s: backend.dt(),
        position_ned: backend.read_position(),
        mag_body: backend.read_mag(),
        // The seam's optional aiding fields. `SimBackend` resolves neither a
        // heading nor rotor RPM, and the binding's own aiding-rate table
        // records what degrades when each is absent — withholding
        // `motor-rpm` leaves the FDI inert. Sending `none` is the truthful
        // answer for this plant; a host that has them sends them.
        heading_rad: None,
        motor_rpm: None,
        // binding v2 (#572): whatever the plant reports, `None` included. A
        // plant with no battery sense says ABSENT; it is never rounded to a
        // number.
        battery_v: backend.read_battery_v(),
    }
}

/// Why a [`BindingHost`] refused to send or to accept a frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HostError {
    /// A control frame (`Config`, `SupervisorConfig`, `Calibration`,
    /// `Command`) with this sequence is still unacknowledged. The handshake
    /// is stop-and-wait: the host must not send `Sensor` frames — or another
    /// control frame — until its ack arrives (docs/TRANSPORT-BINDING.md §2).
    AwaitingAck { seq: u32 },
    /// The reply was not a frame of this binding.
    Wire(wire::WireError),
    /// An `Ack` for a sequence this host is not waiting on (and not a repeat
    /// of the ack it accepted last).
    UnexpectedAck { acked: u32 },
    /// An `Ack` whose header `seq` is not its `acked-seq` payload. The echo
    /// rule (docs/TRANSPORT-BINDING.md §3) puts the acknowledged control
    /// frame's seq in BOTH places; a frame where they disagree answers no
    /// request coherently, so neither value is trusted (#614).
    AckSeqMismatch { header: u32, acked: u32 },
    /// A `Motors` reply that does not answer the host's outstanding `sensor`
    /// frame — the echo rule, enforced (#614). `why` says which way it fails.
    /// Its motor commands must not be applied, and the host's state is
    /// unchanged by it.
    StrayMotors { seq: u32, why: Stray },
    /// A frame type the component never sends (anything but `Ack`/`Motors`).
    UnexpectedType { msg: wire::MsgType },
}

/// Why a `Motors` reply was refused as [`HostError::StrayMotors`].
///
/// Only ONE `sensor` frame is ever answerable: the newest one the host sent,
/// until its reply is accepted (see [`BindingHost`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Stray {
    /// No `sensor` frame is awaiting an answer — none was sent since the last
    /// accepted `Motors`, or none was ever sent.
    Unsolicited,
    /// A `sensor` frame IS outstanding, but this seq is one the host never
    /// put on the wire at all.
    NeverSent,
    /// The host did send this seq, but it is not the outstanding tick: a
    /// `sensor` frame the host has since SUPERSEDED with a newer one (its
    /// commands are stale, and the tick is counted lost when the newer reply
    /// lands), or a control frame, which a `Motors` frame never answers.
    Superseded { outstanding: u32 },
}

impl From<wire::WireError> for HostError {
    fn from(e: wire::WireError) -> Self {
        HostError::Wire(e)
    }
}

/// What one component -> host frame said.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum HostReply {
    /// The control frame with this sequence was RECEIVED. Not "applied": the
    /// component may discard a configuration it is not in a state to take
    /// (e.g. relay#611 discards `configure-*` while armed), and the binding has
    /// no state frame to report that — see the binding document's gaps.
    Acked { seq: u32 },
    /// A repeat of the ack for the control frame acknowledged last — a
    /// duplicated datagram. Nothing changed; in particular it does NOT close a
    /// later handshake.
    DuplicateAck { seq: u32 },
    /// One tick's motor commands, and what the tick contract says about the
    /// reply's sequence.
    Motors {
        motors: [f32; 4],
        /// The reply's header sequence — by the binding's ECHO RULE
        /// (docs/TRANSPORT-BINDING.md §3), the sequence of the `sensor` frame
        /// these motors answer. A host indexes its per-tick record by this,
        /// not by arrival order, so a lost frame cannot shift every later
        /// reply onto the wrong tick (jess#167).
        ///
        /// ENFORCED, not assumed (#614): `on_reply` returns this variant only
        /// when `seq` is the host's outstanding `sensor` frame (or, with
        /// `verdict: Duplicate`, the `Motors` it accepted last); any other
        /// seq is [`HostError::StrayMotors`].
        seq: u32,
        verdict: wire::TickVerdict,
    },
}

/// THE HOST SIDE OF THE BINDING (TRANSPORT-P01 element 5, binding v2) — what a
/// GCS / simulator / HITL rig needs to drive the supervised component with
/// nothing but binding frames.
///
/// It owns the host's outbound sequence (one counter for EVERY message it
/// sends, per §3) and the stop-and-wait configuration handshake, and it
/// classifies the component's replies with the same [`wire::SeqTracker`],
/// seeded at the host's own starting sequence.
///
/// THE ECHO RULE IS ENFORCED, not assumed (#614). A reply's header `seq` must
/// be the seq of a frame this host sent and is still waiting on, or it is
/// refused and changes nothing:
///
/// | reply | accepted when | otherwise |
/// |---|---|---|
/// | `Motors` seq `s` | `s` is the OUTSTANDING `sensor` frame: the newest one sent, not yet answered | [`HostError::StrayMotors`] |
/// | `Motors` seq `s` | `s` is the `Motors` accepted last | `Ok(Motors { verdict: Duplicate })` — a duplicated datagram; do not act twice |
/// | `Ack` | header `seq == acked-seq`, and that is the awaited control frame | [`HostError::AckSeqMismatch`] / [`HostError::UnexpectedAck`] |
/// | `Ack` | header `seq` == `acked-seq` == the ack accepted last | `Ok(DuplicateAck)` |
///
/// Only the newest `sensor` frame is answerable. A host that sends tick `n+1`
/// before tick `n`'s reply arrived has superseded `n`: a late reply to `n` is
/// refused (`Stray::Superseded`), and `n` is counted in the `Dropped { lost }`
/// of `n+1`'s reply. So `lost` counts frames this host SENT whose replies were
/// never accepted — it can no longer be inflated by a reply carrying a seq
/// the host never sent (an integrator's probe on jess#167 got
/// `Dropped { lost: 3 }` for three ticks that never existed). Because the
/// tracker is seeded at the host's starting seq, loss before the first
/// accepted reply is reported too: Sensor 1, 2, 3 out and only Motors 3 back
/// reads `Dropped { lost: 2 }`, where an unseeded tracker said `First`. An
/// accepted `Ack` advances the same baseline, so an acknowledged control
/// frame between two ticks is not counted lost.
///
/// It does not own a socket: each method returns the exact bytes to send, and
/// [`on_reply`](Self::on_reply) takes the bytes that came back.
///
/// Before v2 the binding could carry `configure` and `step` and nothing else,
/// so a host driving the @0.12.0 supervised component over it could never
/// configure the supervisor, never install a calibration and never send `arm`
/// — the vehicle stayed `disarmed` and the binding's only supported path
/// could not fly it (#609 finding 9).
#[derive(Clone, Copy, Debug)]
pub struct BindingHost {
    /// The seq of the last frame sent (the starting value before any).
    seq: u32,
    /// How many frames this host has sent, saturating. Only used to say
    /// WHICH way a stray reply fails (`NeverSent` vs `Superseded`); the
    /// acceptance itself is an equality on `outstanding_sensor`.
    sent: u32,
    awaiting: Option<u32>,
    last_acked: Option<u32>,
    /// The newest `sensor` frame sent and not yet answered — the ONLY seq a
    /// `Motors` reply may carry to be accepted as a tick.
    outstanding_sensor: Option<u32>,
    /// The seq of the `Motors` reply accepted last (its duplicate is benign).
    last_motors: Option<u32>,
    tracker: wire::SeqTracker,
}

impl Default for BindingHost {
    /// Not derived: a derived default would leave the reply tracker UNSEEDED,
    /// and an unseeded tracker reads its first reply as `First` whatever was
    /// lost before it.
    fn default() -> Self {
        Self::new()
    }
}

/// A component -> host frame after decoding, before the echo rule is applied.
#[derive(Clone, Copy, Debug)]
enum ReplyBody {
    Ack { acked: u32 },
    Motors([f32; 4]),
}

impl BindingHost {
    pub const fn new() -> Self {
        BindingHost {
            seq: 0,
            sent: 0,
            awaiting: None,
            last_acked: None,
            outstanding_sensor: None,
            last_motors: None,
            tracker: wire::SeqTracker::starting_at(0),
        }
    }

    /// The `sensor` frame a `Motors` reply would be accepted for, if any.
    pub const fn outstanding_sensor(&self) -> Option<u32> {
        self.outstanding_sensor
    }

    /// Number the next outbound frame.
    fn next_seq(&mut self) -> u32 {
        self.seq = self.seq.wrapping_add(1);
        self.sent = self.sent.saturating_add(1);
        self.seq
    }

    /// Did this host put `s` on the wire (within the last `sent` frames)?
    fn was_sent(&self, s: u32) -> bool {
        self.seq.wrapping_sub(s) < self.sent
    }

    /// The control frame whose ack this host is waiting on, if any.
    pub const fn awaiting_ack(&self) -> Option<u32> {
        self.awaiting
    }

    /// Give up on the outstanding ack (a host's timeout path). The next
    /// control frame goes out with a NEW sequence; re-sending the old bytes
    /// would read as a `Duplicate` at the component.
    pub fn cancel_pending(&mut self) {
        self.awaiting = None;
    }

    fn next_control(&mut self) -> Result<u32, HostError> {
        if let Some(seq) = self.awaiting {
            return Err(HostError::AwaitingAck { seq });
        }
        let seq = self.next_seq();
        self.awaiting = Some(seq);
        Ok(seq)
    }

    /// Number the next `sensor` frame; it becomes the outstanding tick,
    /// superseding any earlier unanswered one.
    fn next_sensor(&mut self) -> Result<u32, HostError> {
        if let Some(seq) = self.awaiting {
            return Err(HostError::AwaitingAck { seq });
        }
        let seq = self.next_seq();
        self.outstanding_sensor = Some(seq);
        Ok(seq)
    }

    /// `controller.configure` as a `Config` frame.
    pub fn configure(
        &mut self,
        cfg: &wire::WireVehicleConfig,
    ) -> Result<[u8; CONFIG_FRAME_LEN], HostError> {
        let seq = self.next_control()?;
        let mut out = [0u8; CONFIG_FRAME_LEN];
        wire::encode_config(&mut out, seq, cfg);
        Ok(out)
    }

    /// `pilot.configure-supervisor` as a `SupervisorConfig` frame.
    pub fn configure_supervisor(
        &mut self,
        cfg: &wire::WireSupervisorConfig,
    ) -> Result<[u8; SUPERVISOR_CONFIG_FRAME_LEN], HostError> {
        let seq = self.next_control()?;
        let mut out = [0u8; SUPERVISOR_CONFIG_FRAME_LEN];
        wire::encode_supervisor_config(&mut out, seq, cfg);
        Ok(out)
    }

    /// `pilot.configure-calibration` as a `Calibration` frame.
    pub fn configure_calibration(
        &mut self,
        cal: &wire::WireCalibration,
    ) -> Result<[u8; CALIBRATION_FRAME_LEN], HostError> {
        let seq = self.next_control()?;
        let mut out = [0u8; CALIBRATION_FRAME_LEN];
        wire::encode_calibration(&mut out, seq, cal);
        Ok(out)
    }

    /// `pilot.command` as a `Command` frame.
    pub fn command(
        &mut self,
        cmd: &wire::WireCommand,
    ) -> Result<[u8; COMMAND_FRAME_LEN], HostError> {
        let seq = self.next_control()?;
        let mut out = [0u8; COMMAND_FRAME_LEN];
        wire::encode_command(&mut out, seq, cmd);
        Ok(out)
    }

    /// One tick's `sensor-frame` (the `step` input). Refused while a control
    /// frame is unacknowledged — the handshake's rule 3.
    pub fn sensor(
        &mut self,
        f: &wire::WireSensorFrame,
    ) -> Result<[u8; SENSOR_WIRE_FRAME_LEN], HostError> {
        let seq = self.next_sensor()?;
        let mut out = [0u8; SENSOR_WIRE_FRAME_LEN];
        wire::encode_sensor_frame(&mut out, seq, f);
        Ok(out)
    }

    /// Classify one component -> host frame.
    ///
    /// A frame that fails [`wire::decode_frame`] — including a `Motors` frame
    /// carrying a NaN command (#527) — is an error, never a reply: a host must
    /// not apply motor commands it could not decode. A frame that decodes is
    /// then held to the echo rule (see the table on [`BindingHost`]); every
    /// refusal leaves the host's state exactly as it was.
    pub fn on_reply(&mut self, frame: &[u8]) -> Result<HostReply, HostError> {
        let (h, payload) = wire::decode_frame(frame)?;
        let body = match h.msg {
            wire::MsgType::Ack => ReplyBody::Ack {
                acked: wire::decode_ack(payload)?,
            },
            wire::MsgType::Motors => ReplyBody::Motors(wire::decode_motors(payload)?),
            other => return Err(HostError::UnexpectedType { msg: other }),
        };
        self.classify(h.seq, body)
    }

    /// The echo rule over a decoded reply whose header carried `seq`. Split
    /// from the byte decoding so the Kani harness can drive it directly.
    fn classify(&mut self, seq: u32, body: ReplyBody) -> Result<HostReply, HostError> {
        match body {
            ReplyBody::Ack { acked } => {
                if seq != acked {
                    return Err(HostError::AckSeqMismatch { header: seq, acked });
                }
                if self.awaiting == Some(acked) {
                    self.tracker.observe(seq);
                    self.awaiting = None;
                    self.last_acked = Some(acked);
                    return Ok(HostReply::Acked { seq: acked });
                }
                // A network-duplicated ack of the frame ALREADY acknowledged:
                // accept the bytes and change nothing (§3, `Duplicate`). It
                // must not close a LATER handshake, and it is not an error.
                if self.last_acked == Some(acked) {
                    return Ok(HostReply::DuplicateAck { seq: acked });
                }
                Err(HostError::UnexpectedAck { acked })
            }
            ReplyBody::Motors(motors) => {
                // THE SENT-CHECK: only the outstanding tick is answerable.
                if self.outstanding_sensor == Some(seq) {
                    let verdict = self.tracker.observe(seq);
                    self.outstanding_sensor = None;
                    self.last_motors = Some(seq);
                    return Ok(HostReply::Motors {
                        motors,
                        seq,
                        verdict,
                    });
                }
                // A duplicated datagram of the reply accepted last.
                if self.last_motors == Some(seq) {
                    return Ok(HostReply::Motors {
                        motors,
                        seq,
                        verdict: wire::TickVerdict::Duplicate,
                    });
                }
                let why = match self.outstanding_sensor {
                    None => Stray::Unsolicited,
                    Some(_) if !self.was_sent(seq) => Stray::NeverSent,
                    Some(outstanding) => Stray::Superseded { outstanding },
                };
                Err(HostError::StrayMotors { seq, why })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use falcon_core::{FlightCore, SimBackend};

    /// An in-process loopback transport: the flight computer's frames are
    /// answered directly by a `SimServer`. Stands in for a real UART/UDP link.
    struct Loopback<B> {
        server: SimServer<B>,
    }
    impl<B: FlightBackend> Transport for Loopback<B> {
        fn exchange(
            &mut self,
            out: &[u8; MOTORS_FRAME_LEN],
            reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
        ) {
            // A reference host answers or refuses; it never flies on garbage.
            self.server
                .serve(out, reply)
                .expect("loopback frames are well formed");
        }
    }

    /// Counts how many times the plant was stepped, so "did not step twice" is
    /// a measurement rather than an inference from downstream state.
    #[derive(Default)]
    struct CountingBackend {
        steps: u32,
    }
    impl FlightBackend for CountingBackend {
        fn read_imu(&mut self) -> ImuSample {
            ImuSample {
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0; 3],
            }
        }
        fn read_position(&mut self) -> Option<Vec3> {
            None
        }
        fn read_mag(&mut self) -> Option<Vec3> {
            None
        }
        fn write_motors(&mut self, _m: &[f32]) {
            self.steps += 1;
        }
        fn dt(&self) -> f32 {
            0.002
        }
    }

    /// One exchange crosses the PUBLISHED binding and comes back decodable.
    #[test]
    fn the_loopback_carries_the_published_binding() {
        let mut server = SimServer::new(CountingBackend::default());
        let mut out = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut out, 1, [0.1, 0.2, 0.3, 0.4]);
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];
        let verdict = server.serve(&out, &mut reply).expect("a well-formed frame");
        assert!(matches!(verdict, wire::TickVerdict::First));

        let (h, payload) = wire::decode_frame(&reply).expect("the reply is a frame");
        assert_eq!(h.msg, wire::MsgType::Sensor);
        assert_eq!(h.seq, 1, "the reply carries the sequence it answers");
        let f = wire::decode_sensor_frame(payload).expect("the payload decodes");
        assert_eq!(f.imu[2], -9.81);
        assert_eq!(server.backend().steps, 1);
    }

    /// A retransmitted frame must NOT advance the plant a second time.
    ///
    /// The binding states the obligation — "The caller must NOT step the
    /// controller twice on one tick" — and before element 5 this server had no
    /// sequence at all, so it could not have honoured it. Stepping twice would
    /// advance the simulation by two control periods for one commanded tick.
    #[test]
    fn a_duplicate_frame_does_not_step_the_plant_twice() {
        let mut server = SimServer::new(CountingBackend::default());
        let mut out = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut out, 7, [0.5; 4]);
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];

        server.serve(&out, &mut reply).unwrap();
        assert_eq!(server.backend().steps, 1);

        // the SAME sequence again
        let v = server.serve(&out, &mut reply).unwrap();
        assert!(
            matches!(v, wire::TickVerdict::Duplicate),
            "verdict was {v:?}"
        );
        assert_eq!(
            server.backend().steps,
            1,
            "a duplicate stepped the plant twice"
        );
        // ...and the host is still answered rather than left waiting.
        assert!(wire::decode_frame(&reply).is_ok());
    }

    /// A LATE frame must not step the plant either — the binding's table says
    /// "discard; a newer frame already superseded it".
    ///
    /// Easy to miss: `SeqTracker`'s own doc spells out only the DUPLICATE
    /// obligation, so an implementer reading the code rather than the document
    /// gates one case and ships the other. Stepping on a late frame applies
    /// stale thrust after a newer command has already been acted on.
    #[test]
    fn a_late_frame_does_not_step_the_plant_with_superseded_commands() {
        let mut server = SimServer::new(CountingBackend::default());
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];
        let mut out = [0u8; MOTORS_FRAME_LEN];

        wire::encode_motors(&mut out, 5, [0.5; 4]);
        server.serve(&out, &mut reply).unwrap();
        wire::encode_motors(&mut out, 9, [0.5; 4]);
        let v = server.serve(&out, &mut reply).unwrap();
        assert!(matches!(v, wire::TickVerdict::Dropped { lost: 3 }), "{v:?}");
        assert_eq!(server.backend().steps, 2);

        // seq 6 arrives after 9 — superseded, not lost.
        wire::encode_motors(&mut out, 6, [0.9; 4]);
        let v = server.serve(&out, &mut reply).unwrap();
        assert!(matches!(v, wire::TickVerdict::Late { by: 3 }), "{v:?}");
        assert_eq!(
            server.backend().steps,
            2,
            "a late frame stepped the plant with superseded commands"
        );
    }

    /// A reply that is not a decodable `Sensor` frame reports NO verdict —
    /// never `First`, which would make garbage look like a fresh stream.
    #[test]
    fn an_undecodable_reply_reports_no_verdict_rather_than_a_fresh_stream() {
        struct GarbageLink;
        impl Transport for GarbageLink {
            fn exchange(
                &mut self,
                _out: &[u8; MOTORS_FRAME_LEN],
                reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
            ) {
                reply.fill(0xAB); // not a frame at all
            }
        }
        let mut backend = LinkBackend::new(GarbageLink, 0.002);
        assert_eq!(
            backend.last_verdict(),
            None,
            "garbage must not be classified as a tick"
        );
        assert_eq!(backend.rejected_replies(), 1);
        backend.write_motors(&[0.1; 4]);
        assert_eq!(backend.last_verdict(), None);
        assert_eq!(backend.rejected_replies(), 2);
        // ...and the cache was NOT zeroed into a free-fall reading.
        assert!(backend.read_imu().accel.iter().all(|v| v.is_finite()));
    }

    /// Garbage is refused, not flown. The old 16/54-byte `serve` could not
    /// fail, so a desynchronised stream read as a zero-thrust command.
    #[test]
    fn garbage_bytes_are_refused_rather_than_flown() {
        let mut server = SimServer::new(CountingBackend::default());
        let junk = [0xABu8; MOTORS_FRAME_LEN];
        let mut reply = [0u8; SENSOR_WIRE_FRAME_LEN];
        assert!(server.serve(&junk, &mut reply).is_err());
        assert_eq!(
            server.backend().steps,
            0,
            "the plant was stepped on garbage"
        );
    }

    /// A host with no battery sense reports ABSENT, not a plausible voltage.
    ///
    /// The retired frame carried a bare `battery: f32` and sent 0.0 V for
    /// "none" because the format had no way to say absent (#466). The binding
    /// has no battery field at @0.11.0, so `None` is the honest answer.
    #[test]
    fn a_host_with_no_battery_sense_reports_absent_not_zero_volts() {
        let server = SimServer::new(CountingBackend::default());
        let mut backend = LinkBackend::new(Loopback { server }, 0.002);
        assert_eq!(
            backend.read_battery_v(),
            None,
            "absent must not be reported as a number"
        );
    }

    /// A plant WITH battery sense: the voltage crosses the binding (v2, #572),
    /// and a measured 0.0 V — a dead pack — arrives as `Some(0.0)`, not as
    /// ABSENT. On binding v1 `read_battery_v` returned `None` unconditionally,
    /// so this fails there on the first assertion.
    #[test]
    fn a_host_with_battery_sense_reports_it_and_zero_volts_stays_present() {
        struct Pack(f32);
        impl FlightBackend for Pack {
            fn read_imu(&mut self) -> ImuSample {
                ImuSample {
                    accel: [0.0, 0.0, -9.81],
                    gyro: [0.0; 3],
                }
            }
            fn read_position(&mut self) -> Option<Vec3> {
                None
            }
            fn read_mag(&mut self) -> Option<Vec3> {
                None
            }
            fn write_motors(&mut self, _m: &[f32]) {}
            fn dt(&self) -> f32 {
                0.002
            }
            fn read_battery_v(&mut self) -> Option<f32> {
                Some(self.0)
            }
        }
        let mut backend = LinkBackend::new(
            Loopback {
                server: SimServer::new(Pack(16.4)),
            },
            0.002,
        );
        assert_eq!(backend.read_battery_v(), Some(16.4));

        let mut dead = LinkBackend::new(
            Loopback {
                server: SimServer::new(Pack(0.0)),
            },
            0.002,
        );
        assert_eq!(
            dead.read_battery_v(),
            Some(0.0),
            "a measured 0.0 V is a dead pack, not an absent sensor"
        );
    }

    /// The host's stop-and-wait handshake: no `Sensor` frame and no second
    /// control frame while a control frame is unacknowledged, and an ack for
    /// a sequence it did not send is refused.
    #[test]
    fn the_binding_host_waits_for_each_ack_before_sending_on() {
        let mut host = BindingHost::new();
        let f = host
            .configure_supervisor(&wire::WireSupervisorConfig {
                home: [0.0; 3],
                fence_radius: 50.0,
                cruise_alt: 2.0,
                low_batt_v: 14.0,
            })
            .unwrap();
        let (h, _) = wire::decode_frame(&f).unwrap();
        assert_eq!(host.awaiting_ack(), Some(h.seq));
        assert_eq!(
            host.sensor(&wire::WireSensorFrame::default()),
            Err(HostError::AwaitingAck { seq: h.seq })
        );
        assert!(matches!(
            host.command(&wire::WireCommand {
                cmd: wire::WirePilotCommand::Arm,
                level: true,
                throttle_low: true,
            }),
            Err(HostError::AwaitingAck { .. })
        ));

        let mut ack = [0u8; ACK_FRAME_LEN];
        // Header and payload agree (the echo rule) but name a frame this host
        // is not waiting on.
        let stray = h.seq.wrapping_add(5);
        wire::encode_ack(&mut ack, stray, stray);
        assert_eq!(
            host.on_reply(&ack),
            Err(HostError::UnexpectedAck { acked: stray })
        );
        wire::encode_ack(&mut ack, h.seq, h.seq);
        assert_eq!(host.on_reply(&ack), Ok(HostReply::Acked { seq: h.seq }));
        assert!(host.sensor(&wire::WireSensorFrame::default()).is_ok());

        // A duplicated datagram of that same ack is accepted and changes
        // nothing — even while a LATER control frame is outstanding.
        let f2 = host
            .command(&wire::WireCommand {
                cmd: wire::WirePilotCommand::Arm,
                level: true,
                throttle_low: true,
            })
            .unwrap();
        let (h2, _) = wire::decode_frame(&f2).unwrap();
        assert_eq!(
            host.on_reply(&ack),
            Ok(HostReply::DuplicateAck { seq: h.seq })
        );
        assert_eq!(
            host.awaiting_ack(),
            Some(h2.seq),
            "a duplicate of an old ack must not close the new handshake"
        );
    }

    /// A `Motors` reply carrying a NaN command is an ERROR at the host, never
    /// a set of motor commands to apply (#527).
    #[test]
    fn the_binding_host_refuses_a_nan_motor_command() {
        let mut host = BindingHost::new();
        let mut m = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut m, 1, [0.5, f32::NAN, 0.5, 0.5]);
        assert_eq!(
            host.on_reply(&m),
            Err(HostError::Wire(wire::WireError::NonFinite {
                msg: wire::MsgType::Motors,
                offset: 4
            }))
        );
    }

    // ── the echo rule, ENFORCED at the host (#614; provenance jess#167) ──────
    //
    // An integrator's probe drove `BindingHost` with replies it never sent and
    // it accepted every one: `on_reply` only fed the header seq to a tracker.
    // These four reproduce the probe's rows.

    fn motors_frame(seq: u32) -> [u8; MOTORS_FRAME_LEN] {
        let mut m = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut m, seq, [0.5; 4]);
        m
    }

    fn sensor_seq(host: &mut BindingHost) -> u32 {
        let f = host.sensor(&wire::WireSensorFrame::default()).unwrap();
        wire::decode_frame(&f).unwrap().0.seq
    }

    fn supervisor_seq(host: &mut BindingHost) -> u32 {
        let f = host
            .configure_supervisor(&wire::WireSupervisorConfig {
                home: [0.0; 3],
                fence_radius: 50.0,
                cruise_alt: 2.0,
                low_batt_v: 14.0,
            })
            .unwrap();
        wire::decode_frame(&f).unwrap().0.seq
    }

    /// Probe row 1, the harmful one: Sensor 1 and 2 out, Motors 5 back. It
    /// was accepted as `Dropped { lost: 3 }` — three ticks that were never
    /// sent, reported lost.
    #[test]
    fn echo_rule_a_motors_seq_the_host_never_sent_is_refused() {
        let mut host = BindingHost::new();
        assert_eq!(sensor_seq(&mut host), 1);
        assert!(host.on_reply(&motors_frame(1)).is_ok(), "the genuine reply");
        assert_eq!(sensor_seq(&mut host), 2);
        let r = host.on_reply(&motors_frame(5));
        assert_eq!(
            r,
            Err(HostError::StrayMotors {
                seq: 5,
                why: Stray::NeverSent
            }),
            "a reply to seq 5, never sent, must be refused — not read as 3 lost ticks"
        );
        assert_eq!(
            host.outstanding_sensor(),
            Some(2),
            "tick 2 is still answerable"
        );
    }

    /// Probe row 2: a fresh host sent Sensor 1; Motors 1000 came back.
    #[test]
    fn echo_rule_a_far_future_motors_seq_on_a_fresh_host_is_refused() {
        let mut host = BindingHost::new();
        assert_eq!(sensor_seq(&mut host), 1);
        let r = host.on_reply(&motors_frame(1000));
        assert_eq!(
            r,
            Err(HostError::StrayMotors {
                seq: 1000,
                why: Stray::NeverSent
            })
        );
    }

    /// Probe row 3: nothing sent at all, and a Motors arrives.
    #[test]
    fn echo_rule_an_unsolicited_motors_is_refused() {
        let mut host = BindingHost::new();
        let r = host.on_reply(&motors_frame(5));
        assert_eq!(
            r,
            Err(HostError::StrayMotors {
                seq: 5,
                why: Stray::Unsolicited
            })
        );
    }

    /// Probe row 4: a control frame seq 1 out; an Ack whose HEADER says 8 and
    /// whose `acked-seq` payload says 1. The two must agree (§3).
    #[test]
    fn echo_rule_an_ack_whose_header_disagrees_with_its_payload_is_refused() {
        let mut host = BindingHost::new();
        let c = supervisor_seq(&mut host);
        assert_eq!(c, 1);
        let mut ack = [0u8; ACK_FRAME_LEN];
        wire::encode_ack(&mut ack, 8, c);
        let r = host.on_reply(&ack);
        assert_eq!(
            r,
            Err(HostError::AckSeqMismatch {
                header: 8,
                acked: 1
            })
        );
        assert_eq!(
            host.awaiting_ack(),
            Some(c),
            "a refused ack closed the handshake"
        );
    }

    /// GENUINE loss is still reported, and exactly: a fresh host sends Sensor
    /// 1, 2, 3 and only 3's Motors arrives — two ticks have no answer.
    #[test]
    fn echo_rule_real_loss_is_reported_exactly() {
        let mut host = BindingHost::new();
        for want in 1..=3 {
            assert_eq!(sensor_seq(&mut host), want);
        }
        let r = host.on_reply(&motors_frame(3));
        assert!(
            matches!(
                r,
                Ok(HostReply::Motors {
                    seq: 3,
                    verdict: wire::TickVerdict::Dropped { lost: 2 },
                    ..
                })
            ),
            "{r:?}"
        );
        // ...and across an interleaved, acked control frame no loss is
        // invented: Sensor 4 -> Motors 4, Config 5 -> Ack 5, Sensor 6 -> Motors 6.
        assert_eq!(sensor_seq(&mut host), 4);
        assert!(matches!(
            host.on_reply(&motors_frame(4)),
            Ok(HostReply::Motors {
                seq: 4,
                verdict: wire::TickVerdict::InOrder,
                ..
            })
        ));
        let c = supervisor_seq(&mut host);
        let mut ack = [0u8; ACK_FRAME_LEN];
        wire::encode_ack(&mut ack, c, c);
        assert_eq!(host.on_reply(&ack), Ok(HostReply::Acked { seq: c }));
        let s = sensor_seq(&mut host);
        assert!(matches!(
            host.on_reply(&motors_frame(s)),
            Ok(HostReply::Motors {
                verdict: wire::TickVerdict::InOrder,
                ..
            })
        ));
    }

    /// A refused reply changes NOTHING: after a stray Motors 5 is refused, the
    /// genuine reply to the outstanding tick still reads exactly as before.
    #[test]
    fn echo_rule_a_refused_reply_leaves_the_host_state_untouched() {
        let mut host = BindingHost::new();
        assert_eq!(sensor_seq(&mut host), 1);
        assert!(matches!(
            host.on_reply(&motors_frame(1)),
            Ok(HostReply::Motors { seq: 1, .. })
        ));
        assert_eq!(sensor_seq(&mut host), 2);
        assert_eq!(
            host.on_reply(&motors_frame(5)),
            Err(HostError::StrayMotors {
                seq: 5,
                why: Stray::NeverSent
            })
        );
        assert!(
            matches!(
                host.on_reply(&motors_frame(2)),
                Ok(HostReply::Motors {
                    seq: 2,
                    verdict: wire::TickVerdict::InOrder,
                    ..
                })
            ),
            "a refused reply must not move the host's sequence baseline"
        );
    }

    /// Reordering and duplication. A duplicated datagram of the reply just
    /// accepted reads `Duplicate` (accept the bytes, do not act twice). A reply
    /// to a SUPERSEDED tick — Sensor 3 then Sensor 4 sent, Motors 3 arriving
    /// after — is refused: only the newest tick is answerable, so 3 counts as
    /// lost when 4's reply lands.
    #[test]
    fn echo_rule_a_duplicate_reads_duplicate_and_a_superseded_reply_is_refused() {
        let mut host = BindingHost::new();
        assert_eq!(sensor_seq(&mut host), 1);
        assert!(host.on_reply(&motors_frame(1)).is_ok());
        assert_eq!(sensor_seq(&mut host), 2);
        assert!(host.on_reply(&motors_frame(2)).is_ok());
        assert!(
            matches!(
                host.on_reply(&motors_frame(2)),
                Ok(HostReply::Motors {
                    seq: 2,
                    verdict: wire::TickVerdict::Duplicate,
                    ..
                })
            ),
            "a duplicated datagram of the last reply"
        );
        assert_eq!(sensor_seq(&mut host), 3);
        assert_eq!(sensor_seq(&mut host), 4);
        let r = host.on_reply(&motors_frame(3));
        assert_eq!(
            r,
            Err(HostError::StrayMotors {
                seq: 3,
                why: Stray::Superseded { outstanding: 4 }
            }),
            "a reply to the superseded tick 3"
        );
        let r = host.on_reply(&motors_frame(4));
        assert!(
            matches!(
                r,
                Ok(HostReply::Motors {
                    seq: 4,
                    verdict: wire::TickVerdict::Dropped { lost: 1 },
                    ..
                })
            ),
            "{r:?}"
        );
        // A Motors carrying the seq of a CONTROL frame answers nothing —
        // whether or not a tick is outstanding.
        let c = supervisor_seq(&mut host);
        assert_eq!(
            host.on_reply(&motors_frame(c)),
            Err(HostError::StrayMotors {
                seq: c,
                why: Stray::Unsolicited
            })
        );
        let mut ack = [0u8; ACK_FRAME_LEN];
        wire::encode_ack(&mut ack, c, c);
        assert!(host.on_reply(&ack).is_ok());
        let s = sensor_seq(&mut host);
        assert_eq!(
            host.on_reply(&motors_frame(c)),
            Err(HostError::StrayMotors {
                seq: c,
                why: Stray::Superseded { outstanding: s }
            })
        );
        // ...and the Motors frame of tick 2, long answered, is not a duplicate
        // of anything now: refused.
        assert!(host.on_reply(&motors_frame(2)).is_err());
    }

    /// The init exchange (seq 0, inside `new`) mis-echoed: the next good
    /// reply must still count it lost, not read as a fresh `First`.
    #[test]
    fn echo_rule_link_backend_counts_a_lost_init_exchange() {
        struct BadFirst {
            n: u32,
        }
        impl Transport for BadFirst {
            fn exchange(
                &mut self,
                out: &[u8; MOTORS_FRAME_LEN],
                reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
            ) {
                let (h, _) = wire::decode_frame(out).unwrap();
                self.n += 1;
                let seq = if self.n == 1 {
                    h.seq.wrapping_add(9)
                } else {
                    h.seq
                };
                wire::encode_sensor_frame(reply, seq, &wire::WireSensorFrame::default());
            }
        }
        let mut backend = LinkBackend::new(BadFirst { n: 0 }, 0.002);
        assert_eq!(backend.last_verdict(), None);
        backend.write_motors(&[0.1; 4]);
        assert_eq!(
            backend.last_verdict(),
            Some(wire::TickVerdict::Dropped { lost: 1 })
        );
    }

    /// The documented edge: a control frame sent while a tick is outstanding,
    /// acked BEFORE that tick's reply arrives. The reply still answers the
    /// outstanding tick, so it is accepted — and reads `Late`, which the
    /// binding's table says to discard. Pinned so the doc's sentence stays true.
    #[test]
    fn echo_rule_a_tick_reply_overtaken_by_a_later_ack_reads_late() {
        let mut host = BindingHost::new();
        let s = sensor_seq(&mut host);
        let c = supervisor_seq(&mut host);
        let mut ack = [0u8; ACK_FRAME_LEN];
        wire::encode_ack(&mut ack, c, c);
        assert_eq!(host.on_reply(&ack), Ok(HostReply::Acked { seq: c }));
        assert!(matches!(
            host.on_reply(&motors_frame(s)),
            Ok(HostReply::Motors {
                verdict: wire::TickVerdict::Late { by: 1 },
                ..
            })
        ));
    }

    /// THE INVERSE PAIR: `LinkBackend` leads with `Motors` and must refuse a
    /// `Sensor` reply that does not echo the seq it sent. A reply echoing a
    /// different seq is not this exchange's answer, so it must neither update
    /// the cache nor be classified as a tick.
    #[test]
    fn echo_rule_link_backend_refuses_a_sensor_reply_with_the_wrong_seq() {
        struct Misecho(u32);
        impl Transport for Misecho {
            fn exchange(
                &mut self,
                out: &[u8; MOTORS_FRAME_LEN],
                reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
            ) {
                let (h, _) = wire::decode_frame(out).unwrap();
                let f = wire::WireSensorFrame {
                    battery_v: Some(16.4),
                    ..Default::default()
                };
                wire::encode_sensor_frame(reply, h.seq.wrapping_add(self.0), &f);
            }
        }
        let mut backend = LinkBackend::new(Misecho(1000), 0.002);
        assert_eq!(
            backend.last_verdict(),
            None,
            "a Sensor reply to a seq never sent was classified as a tick"
        );
        assert_eq!(backend.read_battery_v(), None, "and its payload was cached");
        assert_eq!(backend.misechoed_replies(), 1);
        assert_eq!(
            backend.rejected_replies(),
            0,
            "well-formed, so not 'rejected'"
        );
        backend.write_motors(&[0.1; 4]);
        assert_eq!(backend.last_verdict(), None);
        assert_eq!(backend.read_battery_v(), None);
        assert_eq!(backend.misechoed_replies(), 2);
    }

    /// The inverse pair still reports GENUINE loss exactly: when one
    /// exchange's reply is mis-echoed, the next correctly echoed reply reads
    /// `Dropped { lost: 1 }` — one exchange went unanswered, no more.
    #[test]
    fn echo_rule_link_backend_counts_a_refused_reply_as_one_lost_exchange() {
        struct OneBad {
            n: u32,
        }
        impl Transport for OneBad {
            fn exchange(
                &mut self,
                out: &[u8; MOTORS_FRAME_LEN],
                reply: &mut [u8; SENSOR_WIRE_FRAME_LEN],
            ) {
                let (h, _) = wire::decode_frame(out).unwrap();
                self.n += 1;
                let seq = if self.n == 3 {
                    h.seq.wrapping_add(7)
                } else {
                    h.seq
                };
                wire::encode_sensor_frame(reply, seq, &wire::WireSensorFrame::default());
            }
        }
        let mut backend = LinkBackend::new(OneBad { n: 0 }, 0.002);
        assert_eq!(backend.last_verdict(), Some(wire::TickVerdict::InOrder));
        backend.write_motors(&[0.1; 4]);
        assert_eq!(backend.last_verdict(), Some(wire::TickVerdict::InOrder));
        backend.write_motors(&[0.1; 4]); // the mis-echoed one
        assert_eq!(backend.last_verdict(), None);
        backend.write_motors(&[0.1; 4]);
        assert_eq!(
            backend.last_verdict(),
            Some(wire::TickVerdict::Dropped { lost: 1 })
        );
        assert_eq!(backend.misechoed_replies(), 1);
    }

    /// The verified core stabilises a tilted vehicle **over the HITL link** —
    /// every sensor read and motor write crosses the published binding to a
    /// `SimServer`, proving the link backend carries the real closed loop.
    #[test]
    fn flight_core_stabilizes_over_the_hitl_link() {
        let dt = 0.002f32;
        let th = 0.4f32;
        let (c, s) = (relay_math::cosf(th), relay_math::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let server = SimServer::new(SimBackend::new(r0, dt));
        let mut backend = LinkBackend::new(Loopback { server }, dt);
        let mut core = FlightCore::new(0.5, 1.0 / dt);

        for _ in 0..6000 {
            core.step(&mut backend); // sensors + motors cross the wire each step
        }
        // recompute tilt from the gravity-reaction accel the sim reported:
        // z_frac ≈ 1 when level (accel ≈ [0,0,±g]).
        let i = backend.last().imu;
        let a = [i[0], i[1], i[2]];
        let mag = relay_math::sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).max(1e-6);
        let z_frac = a[2].abs() / mag;
        assert!(
            z_frac > 0.99,
            "core must level the vehicle over the HITL link: z_frac {z_frac}, accel {a:?}"
        );
        // Every tick was in order over a loopback, so the tick contract is
        // reporting rather than merely compiling.
        assert!(matches!(
            backend.last_verdict(),
            Some(wire::TickVerdict::InOrder)
        ));
        assert_eq!(backend.rejected_replies(), 0);
    }
}
