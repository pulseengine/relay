//! Lockstep tick interface on the falcon-hitl wire (SWREQ-FALCON-PLANT-P01).
//!
//! A harness outside relay (jess's lockstep HIL rung, jess#317) advances the
//! plant one tick at a time and compares the motor words a target produces
//! across runs. For that comparison to mean anything the plant must be
//! deterministic: the same seed and the same motor-word stream give the same
//! sensor bytes on the same host, every run. The tests below hold that, with
//! negative controls showing the byte comparison does see the plant.
//!
//! The exchange is whole frames of the published binding
//! (`falcon_hitl::wire`, the version the harness's link already carries):
//!
//! ```text
//!   start(dt)            -> Sensor frame, seq 0
//!   step(Motors seq n,dt) -> Sensor frame, seq n+1   (Motors must echo seq n)
//! ```
//!
//! The Motors frame must echo the seq of the Sensor frame it answers (the
//! binding's echo rule, docs/TRANSPORT-BINDING.md §3). A mismatch is a lost or
//! reordered tick, and is refused rather than stepped: a lockstep run that
//! skips a tick is not the run it claims to be.
//!
//! Out of scope: identity across hosts (x86 vs aarch64 libm). The rung compares
//! motor words produced on the target, and the plant runs on one host per run.

use core::fmt;

use falcon_hitl::wire::{
    self, HEADER_LEN, MOTORS_PAYLOAD_LEN, MsgType, SENSOR_PAYLOAD_LEN, WireError, WireSensorFrame,
};

use crate::physics::{MockPhysics, Physics};

/// Bytes in one Sensor frame (header + payload).
pub const SENSOR_FRAME_LEN: usize = HEADER_LEN + SENSOR_PAYLOAD_LEN;
/// Bytes in one Motors frame (header + payload).
pub const MOTORS_FRAME_LEN: usize = HEADER_LEN + MOTORS_PAYLOAD_LEN;

/// One encoded Sensor frame.
pub type SensorFrame = [u8; SENSOR_FRAME_LEN];

/// Why a lockstep call was refused. Every variant leaves the plant unstepped.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LockstepError {
    /// Seed 0 is the xorshift generator's fixed point: the noise would be a
    /// constant, not a stream.
    ZeroSeed,
    /// `noise_std` is negative or not finite.
    BadNoise(f32),
    /// `dt` is not a finite positive number of seconds.
    BadDt(f32),
    /// The frame did not decode under the binding.
    Wire(WireError),
    /// A well-formed frame of a type other than Motors.
    NotMotors(MsgType),
    /// The Motors frame does not echo the seq of the last Sensor frame.
    SeqMismatch { got: u32, want: u32 },
    /// A motor command outside [0, 1].
    MotorOutOfRange { index: usize, value: f32 },
    /// `step` before `start`: there is no Sensor frame to answer.
    NotStarted,
}

impl fmt::Display for LockstepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LockstepError::ZeroSeed => write!(f, "seed 0 is the noise generator's fixed point"),
            LockstepError::BadNoise(v) => write!(f, "noise_std {v} is not finite and >= 0"),
            LockstepError::BadDt(v) => write!(f, "dt {v} s is not finite and > 0"),
            LockstepError::Wire(e) => write!(f, "frame refused by the binding: {e}"),
            LockstepError::NotMotors(m) => write!(f, "expected a Motors frame, got {m:?}"),
            LockstepError::SeqMismatch { got, want } => {
                write!(
                    f,
                    "Motors seq {got} does not echo the last Sensor seq {want}"
                )
            }
            LockstepError::MotorOutOfRange { index, value } => {
                write!(f, "motor {index} command {value} is outside [0, 1]")
            }
            LockstepError::NotStarted => write!(f, "step before start"),
        }
    }
}

impl std::error::Error for LockstepError {}

/// The deterministic plant behind a lockstep harness.
pub struct LockstepPlant {
    plant: MockPhysics,
    noise_std: f32,
    /// Seq of the last Sensor frame sent, or `None` before `start`.
    last_seq: Option<u32>,
}

impl LockstepPlant {
    /// A plant at rest at the origin whose IMU noise stream is seeded by
    /// `seed`. `noise_std` is the per-axis IMU noise (0 for a noiseless plant).
    pub fn reset(seed: u64, noise_std: f32) -> Result<Self, LockstepError> {
        if seed == 0 {
            return Err(LockstepError::ZeroSeed);
        }
        if !(noise_std.is_finite() && noise_std >= 0.0) {
            return Err(LockstepError::BadNoise(noise_std));
        }
        let mut plant = MockPhysics::at_rest();
        plant.rng = seed;
        Ok(Self {
            plant,
            noise_std,
            last_seq: None,
        })
    }

    /// The first Sensor frame (seq 0): the plant as it is, unstepped. `dt` is
    /// reported in the frame as this tick's period.
    pub fn start(&mut self, dt: f32) -> Result<SensorFrame, LockstepError> {
        check_dt(dt)?;
        Ok(self.sense(0, dt))
    }

    /// Apply one Motors frame for `dt` seconds and return the next Sensor
    /// frame. The Motors frame must echo the last Sensor frame's seq.
    pub fn step(&mut self, motors_frame: &[u8], dt: f32) -> Result<SensorFrame, LockstepError> {
        let want = self.last_seq.ok_or(LockstepError::NotStarted)?;
        check_dt(dt)?;
        let (hdr, payload) = wire::decode_frame(motors_frame).map_err(LockstepError::Wire)?;
        if hdr.msg != MsgType::Motors {
            return Err(LockstepError::NotMotors(hdr.msg));
        }
        if hdr.seq != want {
            return Err(LockstepError::SeqMismatch { got: hdr.seq, want });
        }
        let m = wire::decode_motors(payload).map_err(LockstepError::Wire)?;
        for (index, &value) in m.iter().enumerate() {
            if !(0.0..=1.0).contains(&value) {
                return Err(LockstepError::MotorOutOfRange { index, value });
            }
        }
        self.plant.step(m, dt);
        Ok(self.sense(want.wrapping_add(1), dt))
    }

    /// The underlying plant, for scenario setup (ground contact, a failed
    /// rotor) and for negative controls that move a parameter.
    pub fn plant_mut(&mut self) -> &mut MockPhysics {
        &mut self.plant
    }

    fn sense(&mut self, seq: u32, dt: f32) -> SensorFrame {
        let (imu, pos) = self.plant.measure(self.noise_std);
        let f = WireSensorFrame {
            imu: [
                imu.accel_body[0],
                imu.accel_body[1],
                imu.accel_body[2],
                imu.gyro_body[0],
                imu.gyro_body[1],
                imu.gyro_body[2],
            ],
            dt_s: dt,
            position_ned: Some(pos),
            mag_body: None,
            heading_rad: None,
            motor_rpm: self.plant.motor_rpm(),
            battery_v: None,
        };
        let mut out = [0u8; SENSOR_FRAME_LEN];
        wire::encode_sensor_frame(&mut out, seq, &f);
        self.last_seq = Some(seq);
        out
    }
}

fn check_dt(dt: f32) -> Result<(), LockstepError> {
    if dt.is_finite() && dt > 0.0 {
        Ok(())
    } else {
        Err(LockstepError::BadDt(dt))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICKS: u32 = 2000;
    const DT: f32 = 0.004;
    const NOISE: f32 = 0.01;
    const SEEDS: [u64; 3] = [0xCAFE_BABE_DEAD_BEEF, 1, 0x0123_4567_89AB_CDEF];

    /// A motor stream that is a pure function of the tick: hover plus a small
    /// differential pattern, so attitude, position and RPM all move.
    fn motors_at(tick: u32) -> [f32; 4] {
        let base = 0.59_f32;
        let d = ((tick % 50) as f32 - 25.0) * 0.0004;
        [base + d, base - d, base + 0.5 * d, base - 0.5 * d]
    }

    fn motors_frame(seq: u32, m: [f32; 4]) -> [u8; MOTORS_FRAME_LEN] {
        let mut b = [0u8; MOTORS_FRAME_LEN];
        wire::encode_motors(&mut b, seq, m);
        b
    }

    /// Every Sensor frame of one run, concatenated. `mutate` may change the
    /// plant before the first frame, and `motor` the command at a tick.
    fn run(
        seed: u64,
        dt: f32,
        mutate: impl Fn(&mut MockPhysics),
        motor: impl Fn(u32) -> [f32; 4],
    ) -> Vec<u8> {
        let mut p = LockstepPlant::reset(seed, NOISE).unwrap();
        mutate(p.plant_mut());
        let mut out = Vec::with_capacity((TICKS as usize + 1) * SENSOR_FRAME_LEN);
        out.extend_from_slice(&p.start(dt).unwrap());
        for tick in 0..TICKS {
            let f = p.step(&motors_frame(tick, motor(tick)), dt).unwrap();
            out.extend_from_slice(&f);
        }
        out
    }

    fn nominal(seed: u64) -> Vec<u8> {
        run(seed, DT, |_| {}, motors_at)
    }

    fn first_diff_frame(a: &[u8], b: &[u8]) -> Option<usize> {
        a.iter()
            .zip(b)
            .position(|(x, y)| x != y)
            .map(|i| i / SENSOR_FRAME_LEN)
    }

    // rivet: verifies SWREQ-FALCON-PLANT-P01
    #[test]
    fn same_seed_same_motor_stream_gives_identical_bytes() {
        for seed in SEEDS {
            let a = nominal(seed);
            for _ in 0..2 {
                assert_eq!(a, nominal(seed), "seed {seed:#x}: two runs differ");
            }
            assert_eq!(a.len(), (TICKS as usize + 1) * SENSOR_FRAME_LEN);
        }
    }

    /// Without this, a plant that ignored its inputs would pass the identity
    /// test above. Each mutation must change the bytes.
    #[test]
    fn controls_seed_dt_thrust_and_motor_word_each_change_the_bytes() {
        let a = nominal(SEEDS[0]);
        assert_ne!(a, nominal(SEEDS[1]), "seed is not reaching the noise");

        let dt_up = f32::from_bits(DT.to_bits() + 1);
        assert_ne!(a, run(SEEDS[0], dt_up, |_| {}, motors_at), "dt +1 ULP");

        let thrust = run(
            SEEDS[0],
            DT,
            |p| p.thrust_scale = f32::from_bits(p.thrust_scale.to_bits() + 1),
            motors_at,
        );
        assert_ne!(a, thrust, "thrust_scale +1 ULP did not reach the bytes");

        // One motor word, one ULP, at tick 1000: the frames before it are
        // untouched and the frame it produces is the first to differ.
        let one_word = run(
            SEEDS[0],
            DT,
            |_| {},
            |t| {
                let mut m = motors_at(t);
                if t == 1000 {
                    m[2] = f32::from_bits(m[2].to_bits() + 1);
                }
                m
            },
        );
        assert_eq!(first_diff_frame(&a, &one_word), Some(1001));
    }

    /// The lockstep wrapper adds nothing: the decoded frames carry exactly what
    /// MockPhysics, driven directly, measures.
    #[test]
    fn frames_carry_exactly_what_mock_physics_measures() {
        let mut direct = MockPhysics::at_rest();
        let mut p = LockstepPlant::reset(direct.rng, NOISE).unwrap();
        let mut frame = p.start(DT).unwrap();
        let mut expect = direct.measure(NOISE);
        for tick in 0..200 {
            let (hdr, payload) = wire::decode_frame(&frame).unwrap();
            assert_eq!(hdr.msg, MsgType::Sensor);
            assert_eq!(hdr.seq, tick);
            let f = wire::decode_sensor_frame(payload).unwrap();
            let (imu, pos) = expect;
            let want = [
                imu.accel_body[0],
                imu.accel_body[1],
                imu.accel_body[2],
                imu.gyro_body[0],
                imu.gyro_body[1],
                imu.gyro_body[2],
            ];
            assert_eq!(f.imu.map(f32::to_bits), want.map(f32::to_bits));
            assert_eq!(
                f.position_ned.unwrap().map(f32::to_bits),
                pos.map(f32::to_bits)
            );
            assert_eq!(f.motor_rpm, direct.motor_rpm());
            assert_eq!(f.dt_s.to_bits(), DT.to_bits());

            let m = motors_at(tick);
            direct.step(m, DT);
            expect = direct.measure(NOISE);
            frame = p.step(&motors_frame(tick, m), DT).unwrap();
        }
    }

    #[test]
    fn refusals_leave_the_plant_unstepped() {
        assert_eq!(
            LockstepPlant::reset(0, NOISE).err(),
            Some(LockstepError::ZeroSeed)
        );
        assert!(matches!(
            LockstepPlant::reset(1, -1.0),
            Err(LockstepError::BadNoise(_))
        ));
        assert!(matches!(
            LockstepPlant::reset(1, f32::NAN),
            Err(LockstepError::BadNoise(_))
        ));

        let mut p = LockstepPlant::reset(1, NOISE).unwrap();
        let hover = motors_frame(0, [0.59; 4]);
        assert_eq!(p.step(&hover, DT).err(), Some(LockstepError::NotStarted));
        assert!(matches!(p.start(0.0), Err(LockstepError::BadDt(_))));
        assert!(matches!(
            p.start(f32::INFINITY),
            Err(LockstepError::BadDt(_))
        ));
        let f0 = p.start(DT).unwrap();

        // Each refused step must leave the next good step's output unchanged.
        let mut q = LockstepPlant::reset(1, NOISE).unwrap();
        assert_eq!(q.start(DT).unwrap(), f0);
        let want = q.step(&hover, DT).unwrap();

        assert_eq!(
            p.step(&motors_frame(5, [0.59; 4]), DT).err(),
            Some(LockstepError::SeqMismatch { got: 5, want: 0 })
        );
        assert!(matches!(
            p.step(&motors_frame(0, [1.5, 0.5, 0.5, 0.5]), DT),
            Err(LockstepError::MotorOutOfRange { index: 0, .. })
        ));
        assert!(matches!(
            p.step(&f0, DT),
            Err(LockstepError::NotMotors(MsgType::Sensor))
        ));
        let mut bad_magic = hover;
        bad_magic[0] ^= 0xff;
        assert!(matches!(
            p.step(&bad_magic, DT),
            Err(LockstepError::Wire(_))
        ));
        assert!(matches!(
            p.step(&hover[..10], DT),
            Err(LockstepError::Wire(_))
        ));
        assert!(matches!(p.step(&hover, -DT), Err(LockstepError::BadDt(_))));

        assert_eq!(p.step(&hover, DT).unwrap(), want);
    }

    /// The harness binds to the binding version its link carries. A version
    /// bump changes every frame this plant emits, so it must be announced on
    /// the seam (jess#167) before it ships; this test is where it surfaces.
    #[test]
    fn frames_are_binding_version_3() {
        let mut p = LockstepPlant::reset(1, 0.0).unwrap();
        let f = p.start(DT).unwrap();
        assert_eq!(f[2], 3, "lockstep consumers bind to wire VERSION 3");
        assert_eq!(f.len(), 89);
    }
}
