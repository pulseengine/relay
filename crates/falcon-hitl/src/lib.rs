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
/// Bytes in a `Sensor` frame on the wire: header plus the 76-byte payload.
///
/// Same reasoning — the retired `SENSOR_FRAME_LEN` meant 54.
pub const SENSOR_WIRE_FRAME_LEN: usize = wire::HEADER_LEN + wire::SENSOR_PAYLOAD_LEN;

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
        let mut tracker = wire::SeqTracker::default();
        let (cache, last_verdict) = Self::read_reply(&reply, &mut tracker);
        LinkBackend {
            transport,
            cache: cache.unwrap_or_default(),
            seq: 0,
            tracker,
            rejected: u32::from(last_verdict.is_none()),
            last_verdict,
            dt,
        }
    }

    /// Decode one reply, classify its sequence, and hand back both.
    ///
    /// A frame that does not decode leaves the cache ALONE rather than zeroing
    /// it: the last good sensors are a better estimate for one tick than a
    /// synthetic zero reading, and zeroed accelerometers read as free-fall.
    fn read_reply(
        reply: &[u8; SENSOR_WIRE_FRAME_LEN],
        tracker: &mut wire::SeqTracker,
    ) -> (Option<wire::WireSensorFrame>, Option<wire::TickVerdict>) {
        match wire::decode_frame(reply) {
            Ok((h, payload)) if h.msg == wire::MsgType::Sensor => {
                let verdict = tracker.observe(h.seq);
                match wire::decode_sensor_frame(payload) {
                    // The header decoded and the sequence was classified, but
                    // the payload did not: report the verdict and no frame.
                    Err(_) => (None, Some(verdict)),
                    Ok(f) => (Some(f), Some(verdict)),
                }
            }
            // Not a frame, or not a Sensor frame. There is no sequence to
            // classify, so there is no verdict — NOT `First`.
            _ => (None, None),
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
        let (frame, verdict) = Self::read_reply(&reply, &mut self.tracker);
        if verdict.is_none() {
            self.rejected = self.rejected.saturating_add(1);
        }
        self.last_verdict = verdict;
        if let Some(f) = frame {
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
        // NONE, AND THAT IS THE POINT RATHER THAN A REGRESSION.
        //
        // The retired 54-byte frame carried a bare `battery: f32` with no
        // validity flag, so a host with no battery sense had to send a NUMBER.
        // The old code sent 0.0 V and said why: "adding one changes the HITL
        // wire format — so until that protocol changes, the fallback is chosen
        // to be fail-SAFE". That is a field that can only lie, and this crate's
        // own module note named it: those frames "cannot carry heading, rotor
        // RPM, or a battery that says ABSENT (#466)".
        //
        // The binding's `sensor-frame` has no battery field at all, so ABSENT is
        // now representable honestly, and `FlightBackend`'s own default for this
        // method is `None` too.
        //
        // WHAT IS DEFERRED, stated so it is not mistaken for complete: a host
        // that genuinely HAS a battery cannot report it over this binding at
        // @0.11.0. `battery-v: option<f32>` is drafted for @0.12.0 ("Pack
        // voltage (V), or `none` when the host has no battery sense"), which is
        // parked behind the same seam decision as the supervisor's mode and
        // failsafe fields. Until it lands, a HITL battery failsafe is
        // unreachable through this path — which is already true of the seam
        // generally, not something introduced here.
        None
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

        let imu = self.backend.read_imu();
        let frame = wire::WireSensorFrame {
            imu: [
                imu.accel[0],
                imu.accel[1],
                imu.accel[2],
                imu.gyro[0],
                imu.gyro[1],
                imu.gyro[2],
            ],
            dt_s: self.backend.dt(),
            position_ned: self.backend.read_position(),
            mag_body: self.backend.read_mag(),
            // The seam's optional aiding fields. `SimBackend` resolves neither a
            // heading nor rotor RPM, and the binding's own aiding-rate table
            // records what degrades when each is absent — withholding
            // `motor-rpm` leaves the FDI inert. Sending `none` is the truthful
            // answer for this plant; a host that has them sends them.
            heading_rad: None,
            motor_rpm: None,
        };
        wire::encode_sensor_frame(reply, h.seq, &frame);
        Ok(verdict)
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
