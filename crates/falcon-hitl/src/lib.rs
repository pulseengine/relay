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

use falcon_core::{FlightBackend, ImuSample};

/// `[f32; 3]`, matching `falcon-core`'s vector type.
pub type Vec3 = [f32; 3];

/// Bytes in an actuator frame: four `f32` motor commands.
pub const ACTUATOR_FRAME_LEN: usize = 16;
/// Bytes in a sensor frame: accel(3) gyro(3) pos(3) + pos_valid + mag(3) +
/// mag_valid + battery = 12+12+12+1+12+1+4.
pub const SENSOR_FRAME_LEN: usize = 54;

/// One sensor exchange from the simulator/vehicle to the flight computer.
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorFrame {
    pub accel: Vec3,
    pub gyro: Vec3,
    pub pos: Vec3,
    pub pos_valid: bool,
    pub mag: Vec3,
    pub mag_valid: bool,
    pub battery: f32,
}

// ── fixed-layout little-endian codec ─────────────────────────────────────

fn put_f32(buf: &mut [u8], at: usize, v: f32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
fn get_f32(buf: &[u8], at: usize) -> f32 {
    let mut b = [0u8; 4];
    b.copy_from_slice(&buf[at..at + 4]);
    f32::from_le_bytes(b)
}

/// Encode the four motor commands into an actuator frame.
pub fn encode_actuator(motors: &[f32], out: &mut [u8; ACTUATOR_FRAME_LEN]) {
    for i in 0..4 {
        put_f32(out, i * 4, motors.get(i).copied().unwrap_or(0.0));
    }
}

/// Decode an actuator frame into four motor commands.
pub fn decode_actuator(buf: &[u8; ACTUATOR_FRAME_LEN]) -> [f32; 4] {
    let mut m = [0.0f32; 4];
    for (i, mi) in m.iter_mut().enumerate() {
        *mi = get_f32(buf, i * 4);
    }
    m
}

/// Encode a sensor frame.
pub fn encode_sensor(f: &SensorFrame, out: &mut [u8; SENSOR_FRAME_LEN]) {
    for i in 0..3 {
        put_f32(out, i * 4, f.accel[i]);
        put_f32(out, 12 + i * 4, f.gyro[i]);
        put_f32(out, 24 + i * 4, f.pos[i]);
    }
    out[36] = f.pos_valid as u8;
    for i in 0..3 {
        put_f32(out, 37 + i * 4, f.mag[i]);
    }
    out[49] = f.mag_valid as u8;
    put_f32(out, 50, f.battery);
}

/// Decode a sensor frame.
pub fn decode_sensor(buf: &[u8; SENSOR_FRAME_LEN]) -> SensorFrame {
    let mut f = SensorFrame::default();
    for i in 0..3 {
        f.accel[i] = get_f32(buf, i * 4);
        f.gyro[i] = get_f32(buf, 12 + i * 4);
        f.pos[i] = get_f32(buf, 24 + i * 4);
        f.mag[i] = get_f32(buf, 37 + i * 4);
    }
    f.pos_valid = buf[36] != 0;
    f.mag_valid = buf[49] != 0;
    f.battery = get_f32(buf, 50);
    f
}

/// The link transport: send one actuator frame, receive one sensor frame. Your
/// impl wraps a UART/USB/UDP socket; the request/response shape models the
/// one-exchange-per-control-step HITL cadence.
pub trait Transport {
    fn exchange(&mut self, out: &[u8; ACTUATOR_FRAME_LEN], reply: &mut [u8; SENSOR_FRAME_LEN]);
}

/// A [`FlightBackend`] that sources its sensors and sinks its actuators over a
/// [`Transport`] to a remote simulator (or vehicle). The verified `FlightCore`
/// is identical to the sim/hardware cases — only the backend differs.
pub struct LinkBackend<T> {
    transport: T,
    cache: SensorFrame,
    dt: f32,
}

impl<T: Transport> LinkBackend<T> {
    /// Open the link with control period `dt`. Primes the sensor cache with one
    /// neutral exchange so the first `read_*` has live data.
    pub fn new(mut transport: T, dt: f32) -> Self {
        let zero = [0u8; ACTUATOR_FRAME_LEN];
        let mut reply = [0u8; SENSOR_FRAME_LEN];
        transport.exchange(&zero, &mut reply);
        LinkBackend { transport, cache: decode_sensor(&reply), dt }
    }

    /// The latest sensor frame (telemetry / tests).
    pub fn last(&self) -> SensorFrame {
        self.cache
    }
}

impl<T: Transport> FlightBackend for LinkBackend<T> {
    fn read_imu(&mut self) -> ImuSample {
        ImuSample { accel: self.cache.accel, gyro: self.cache.gyro }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.cache.pos_valid.then_some(self.cache.pos)
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.cache.mag_valid.then_some(self.cache.mag)
    }
    fn write_motors(&mut self, motors: &[f32]) {
        // the HITL round-trip: actuate, then receive the resulting sensors.
        let mut out = [0u8; ACTUATOR_FRAME_LEN];
        encode_actuator(motors, &mut out);
        let mut reply = [0u8; SENSOR_FRAME_LEN];
        self.transport.exchange(&out, &mut reply);
        self.cache = decode_sensor(&reply);
    }
    fn dt(&self) -> f32 {
        self.dt
    }
    fn read_battery_v(&mut self) -> f32 {
        self.cache.battery
    }
}

/// The simulator side of the link: decode an actuator frame, step the backing
/// `FlightBackend` (a `SimBackend`), and encode the resulting sensors. Generic
/// over the backend so the same server can wrap an injected-pathology sim.
pub struct SimServer<B> {
    backend: B,
}

impl<B: FlightBackend> SimServer<B> {
    pub fn new(backend: B) -> Self {
        SimServer { backend }
    }

    /// Borrow the backing backend (to read its ground-truth state in tests).
    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Answer one exchange: apply the actuator frame (steps the plant), then
    /// sample the sensors into the reply frame.
    pub fn serve(
        &mut self,
        actuator: &[u8; ACTUATOR_FRAME_LEN],
        reply: &mut [u8; SENSOR_FRAME_LEN],
    ) {
        let motors = decode_actuator(actuator);
        self.backend.write_motors(&motors); // steps the plant
        let imu = self.backend.read_imu();
        let pos = self.backend.read_position();
        let mag = self.backend.read_mag();
        let frame = SensorFrame {
            accel: imu.accel,
            gyro: imu.gyro,
            pos: pos.unwrap_or([0.0; 3]),
            pos_valid: pos.is_some(),
            mag: mag.unwrap_or([0.0; 3]),
            mag_valid: mag.is_some(),
            battery: self.backend.read_battery_v(),
        };
        encode_sensor(&frame, reply);
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
            out: &[u8; ACTUATOR_FRAME_LEN],
            reply: &mut [u8; SENSOR_FRAME_LEN],
        ) {
            self.server.serve(out, reply);
        }
    }

    /// The codec round-trips exactly.
    #[test]
    fn frames_round_trip() {
        let mut ab = [0u8; ACTUATOR_FRAME_LEN];
        encode_actuator(&[0.1, 0.2, 0.3, 0.4], &mut ab);
        assert_eq!(decode_actuator(&ab), [0.1, 0.2, 0.3, 0.4]);

        let f = SensorFrame {
            accel: [1.0, -2.0, 9.81],
            gyro: [0.1, 0.2, -0.3],
            pos: [5.0, 6.0, -7.0],
            pos_valid: true,
            mag: [1.0, 0.0, 0.0],
            mag_valid: false,
            battery: 15.2,
        };
        let mut sb = [0u8; SENSOR_FRAME_LEN];
        encode_sensor(&f, &mut sb);
        let g = decode_sensor(&sb);
        assert_eq!(g.accel, f.accel);
        assert_eq!(g.gyro, f.gyro);
        assert_eq!(g.pos, f.pos);
        assert!(g.pos_valid && !g.mag_valid);
        assert_eq!(g.battery, 15.2);
    }

    /// The verified core stabilises a tilted vehicle **over the HITL link** —
    /// every sensor read and motor write crosses the byte-framed protocol to a
    /// `SimServer`, proving the link backend carries the real closed loop.
    #[test]
    fn flight_core_stabilizes_over_the_hitl_link() {
        let dt = 0.002f32;
        let th = 0.4f32;
        let (c, s) = (libm::cosf(th), libm::sinf(th));
        let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let server = SimServer::new(SimBackend::new(r0, dt));
        let link = LinkBackend::new(Loopback { server }, dt);
        let mut backend = link;
        let mut core = FlightCore::new(0.5, 1.0 / dt);

        for _ in 0..6000 {
            core.step(&mut backend); // sensors + motors cross the wire each step
        }
        // ground truth from the sim behind the server, read back over the link
        let tilt = backend.last(); // last sensor frame
        // recompute tilt from the gravity-reaction accel the sim reported:
        // at rest the body-frame accel points along body −z·(−g) … simplest is
        // to assert the vehicle is near level via the reported accel z-dominance.
        let a = tilt.accel;
        let mag = libm::sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).max(1e-6);
        let z_frac = a[2].abs() / mag; // ≈1 when level (accel ≈ [0,0,±g])
        assert!(
            z_frac > 0.99,
            "core must level the vehicle over the HITL link: z_frac {z_frac}, accel {a:?}"
        );
    }
}
