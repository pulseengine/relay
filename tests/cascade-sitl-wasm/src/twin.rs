//! TWO COMPONENTS, ONE INPUT STREAM, BIT-EXACT (#575).
//!
//! `DIFF_AGAINST=<b.wasm> cascade-sitl-wasm <a.wasm> [ticks] [dt]`
//!
//! Instantiates two components that export the `cascade` surface (`controller`,
//! `observer`, `pilot` @0.12.0), drives them with the IDENTICAL call sequence,
//! and fails on the first tick where any motor command or any published
//! `flight-state` field differs IN ITS BIT PATTERN. The plant is advanced by
//! side A's output, so side B is a pure observer of A's input stream.
//!
//! What it is for: `a` = the monolithic `cascade`, `b` = the `wac plug`-composed
//! `cascade-split`. The split's equivalence claim is "the same vehicle", and the
//! only honest form of that claim is "the same bits, every tick, through every
//! mode". It is equally the refactor-neutrality check for falcon-core changes:
//! `a` = `cascade` built before, `b` = `cascade` built after.
//!
//! SUPERVISED ON BOTH SIDES, BY CONSTRUCTION. Both sides are components
//! exporting `pilot`, i.e. both hold a `FlightSupervisor`; there is no native
//! mirror in this mode to fall out of step with. (The native mirror is exactly
//! where #571's differential compared a supervised side against an unsupervised
//! one and reported a real, meaningless 0.069364190.)
//!
//! UNDER #609's LIFECYCLE RULES, ARMING IS EARNED. Both sides refuse an `arm`
//! that arrives before a freshly built supervisor has stepped, refuse one on an
//! invalid or missing supervisor configuration, and allow one only on the
//! measured pre-arm table — which needs a real calibration (#586). So the rig
//! injects a known gyro bias (`GYRO_BIAS`), configures the calibration that
//! cancels it, and arms the way a GCS does: `arm` every tick while disarmed,
//! starting BEFORE the first step after every rebuild. Because the arm is
//! re-sent every tick, both sides must accept it on the SAME tick. And the
//! termination segment starts with a DEAD ACCELEROMETER, under which
//! `EstimatorConverged` must refuse every arm: a split whose tilt-uncertainty
//! forward were stubbed arms there and diverges in the published mode. That is
//! the stage #585 had to park behind `TWIN_PENDING_586`; with the calibration
//! crossing the seam (so `CalibrationPresent` no longer masks the estimator
//! rows) it runs always. (Measured: on a HEALTHY sensor the accepted-arm tick
//! is NOT decided by the tilt row — stubbing it changed nothing there — which
//! is why the dead-accelerometer stage exists.)
//!
//! #609 ALSO MADE SOME STATES TERMINAL IN-PROCESS: a configure is REJECTED in
//! `Terminated`, with an isolated rotor, and with a latched low battery. So the
//! scenario is three SEGMENTS, each on a FRESH pair of instances and a fresh
//! plant, and each ends by sending the configure that must be rejected and
//! checking (on both sides, bit for bit) that it was:
//!
//!   1. LIFECYCLE — the pre-tick state; an unconfigured supervisor refusing to
//!      arm; `configure-supervisor` and `configure` rebuilds on the ground, each
//!      followed by an `arm` before the first step that must be refused; an
//!      ARMED hold with a noisy gyro and a takeoff with no reset (`keep_warm`);
//!      takeoff, loiter, a mission leg, the mission-complete RTL, the landing
//!      and the touchdown disarm; estimate-only ticks after touchdown (the
//!      estimator's in-flight flag deliberately left stale, #452); a
//!      low-battery failsafe; a refused re-arm; a silent pack; and a
//!      `configure-supervisor` the battery latch must REJECT.
//!   2. TERMINATION — a dead accelerometer refusing every arm; then a flight
//!      TERMINATED IN THE AIR (estimate-only ticks with
//!      the in-flight flag still set), the fall, and a `configure-supervisor`
//!      the absorbing `Terminated` mode must REJECT.
//!   3. ROTOR-OUT — a dead rotor, FDI isolation, land-in-place, and a
//!      `configure` the isolated rotor must REJECT.
//!
//! At the end it ASSERTS that the run actually visited those modes, raised
//! those failsafe bits and saw those refusals, so a scenario that silently
//! stopped exercising them fails rather than passing on a hover.

use crate::exports::pulseengine::falcon_cascade::pilot::{
    PilotCommand, SensorCalibration, SupervisorConfig,
};
use crate::physics::{MockPhysics, Physics};
use crate::pulseengine::falcon_cascade::types::{
    FlightState, ImuSample as WitImu, MotorPwm, RotorRpm, SensorFrame, Vec3, VehicleConfig,
    Waypoint,
};
use crate::ComposedCascade;
use anyhow::{bail, Context, Result};
use wasmtime::component::{Component, Linker};
use wasmtime::{Engine, Store};

/// Published `mode` values (relay-fsm discriminants, pinned on the seam).
const DISARMED: u8 = 0;
const ARMED: u8 = 1;
const TAKEOFF: u8 = 2;
const LOITER: u8 = 3;
const MISSION: u8 = 4;
const LAND: u8 = 5;
const RTL: u8 = 6;
const TERMINATED: u8 = 7;

/// Published `failsafe` bits (pinned on the seam).
const FS_RTL_LATCHED: u32 = 1 << 0;
const FS_BATT_LOW: u32 = 1 << 1;
const FS_BATT_CRITICAL: u32 = 1 << 2;
const FS_BATT_SILENT: u32 = 1 << 3;
const FS_ARM_BLOCKED: u32 = 1 << 7;

/// Ticks flight 1 is held ARMED before takeoff, and the gyro noise (rad/s)
/// during the hold — see `Stage::F1ArmedHold`.
const ARMED_HOLD_TICKS: u32 = 400;
const ARMED_HOLD_GYRO_NOISE: f32 = 0.02;

/// A KNOWN gyro zero-rate bias (rad/s) added to every gyro sample both sides
/// receive, and the calibration both are configured with to cancel it — the
/// same value and rationale as main.rs's `SITL_GYRO_BIAS`. A non-identity
/// calibration is what lets the measured pre-arm table pass
/// (`CalibrationPresent`), and in the split it has to reach BOTH partitions:
/// the control half de-biases its own copy of the gyro.
const GYRO_BIAS: [f32; 3] = [0.002, -0.0015, 0.001];

/// Ticks the Termination segment starts with a dead accelerometer — see
/// `Stage::DeadAccel`.
const DEAD_ACCEL_TICKS: u32 = 400;

/// How long a side may sit disarmed, stepping, with `arm` sent every tick,
/// before the run FAILS (as main.rs's `ARM_DEADLINE_S`).
const ARM_DEADLINE_S: f32 = 10.0;

const PACK_V: f32 = 16.8;
const SAG_V: f32 = 13.0;
const LOW_BATT_V: f32 = 14.0;
const CRUISE_M: f32 = 2.0;

struct Side {
    store: Store<()>,
    b: ComposedCascade,
}

impl Side {
    fn load(engine: &Engine, path: &str) -> Result<Self> {
        let component =
            Component::from_file(engine, path).with_context(|| format!("loading {path}"))?;
        // Empty linker, as in the main harness: neither side may need WASI or
        // any host function. Both import only the records-only `types`.
        let linker: Linker<()> = Linker::new(engine);
        let mut store = Store::new(engine, ());
        let b = ComposedCascade::instantiate(&mut store, &component, &linker)
            .with_context(|| format!("instantiating {path} (empty linker, no WASI)"))?;
        Ok(Side { store, b })
    }
    fn configure(&mut self, c: VehicleConfig) -> Result<()> {
        self.b
            .pulseengine_falcon_cascade_controller()
            .call_configure(&mut self.store, c)
    }
    fn configure_supervisor(&mut self, c: SupervisorConfig) -> Result<()> {
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_configure_supervisor(&mut self.store, c)
    }
    fn configure_calibration(&mut self, c: SensorCalibration) -> Result<()> {
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_configure_calibration(&mut self.store, c)
    }
    fn command(&mut self, c: PilotCommand) -> Result<()> {
        self.b
            .pulseengine_falcon_cascade_pilot()
            .call_command(&mut self.store, c, true, true)
    }
    fn step(&mut self, f: SensorFrame, t: Waypoint) -> Result<MotorPwm> {
        self.b
            .pulseengine_falcon_cascade_controller()
            .call_step(&mut self.store, f, t)
    }
    fn read_state(&mut self) -> Result<FlightState> {
        self.b
            .pulseengine_falcon_cascade_observer()
            .call_read_state(&mut self.store)
    }
}

/// Every field, by bit pattern. Exhaustive by construction: the destructuring
/// below fails to compile if `flight-state` gains a field nobody compares.
fn diff_state(a: &FlightState, b: &FlightState) -> Option<String> {
    let FlightState {
        tick_head,
        qw,
        qx,
        qy,
        qz,
        pos_n,
        pos_e,
        pos_d,
        vel_n,
        vel_e,
        vel_d,
        wx,
        wy,
        wz,
        bgx,
        bgy,
        bgz,
        innovation,
        failed_rotor,
        mode,
        failsafe,
        valid,
        tick_tail,
    } = *a;
    let f = |name: &str, x: f32, y: f32| {
        (x.to_bits() != y.to_bits()).then(|| {
            format!(
                "{name}: {x:e} ({:#010x}) vs {y:e} ({:#010x})",
                x.to_bits(),
                y.to_bits()
            )
        })
    };
    let u = |name: &str, x: u32, y: u32| (x != y).then(|| format!("{name}: {x:#x} vs {y:#x}"));
    None.or(u("tick-head", tick_head, b.tick_head))
        .or(f("qw", qw, b.qw))
        .or(f("qx", qx, b.qx))
        .or(f("qy", qy, b.qy))
        .or(f("qz", qz, b.qz))
        .or(f("pos-n", pos_n, b.pos_n))
        .or(f("pos-e", pos_e, b.pos_e))
        .or(f("pos-d", pos_d, b.pos_d))
        .or(f("vel-n", vel_n, b.vel_n))
        .or(f("vel-e", vel_e, b.vel_e))
        .or(f("vel-d", vel_d, b.vel_d))
        .or(f("wx", wx, b.wx))
        .or(f("wy", wy, b.wy))
        .or(f("wz", wz, b.wz))
        .or(f("bgx", bgx, b.bgx))
        .or(f("bgy", bgy, b.bgy))
        .or(f("bgz", bgz, b.bgz))
        .or(f("innovation", innovation, b.innovation))
        .or(u(
            "failed-rotor",
            failed_rotor as u32,
            b.failed_rotor as u32,
        ))
        .or(u("mode", mode as u32, b.mode as u32))
        .or(u("failsafe", failsafe, b.failsafe))
        .or(u("valid", valid, b.valid))
        .or(u("tick-tail", tick_tail, b.tick_tail))
}

fn diff_motors(a: &MotorPwm, b: &MotorPwm) -> Option<String> {
    let MotorPwm { m1, m2, m3, m4 } = *a;
    for (i, (x, y)) in [(m1, b.m1), (m2, b.m2), (m3, b.m3), (m4, b.m4)]
        .into_iter()
        .enumerate()
    {
        if x.to_bits() != y.to_bits() {
            return Some(format!("m{}: {x:e} vs {y:e}", i + 1));
        }
    }
    None
}

/// The three segments, each on fresh instances (see the module docs).
#[derive(Clone, Copy, PartialEq, Debug)]
enum Segment {
    Lifecycle,
    Termination,
    RotorOut,
}

/// The scenario's stages. Each advances on side A's PUBLISHED state.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Stage {
    /// Supervisor never configured: an `arm` must be refused.
    Unconfigured,
    /// A DEAD ACCELEROMETER (zero specific force) on the ground, `arm` sent
    /// every tick: every one must be refused, by `EstimatorConverged` (the
    /// tilt uncertainty cannot shrink). This is the stage that makes the split's
    /// tilt-uncertainty forward DECISIVE here: a split with that forward
    /// stubbed passes every other row and arms within a few ticks.
    DeadAccel,
    /// A supervisor was just (re)built: `arm` every tick while disarmed, the
    /// FIRST one before any step (must be refused, #609 finding 2), until the
    /// measured table accepts it. Then `after_arm`.
    Arming,
    /// Flight 1 ARMED but held on the ground: estimate-only ticks with a NOISY
    /// gyro, so the control half's `keep_warm` has something to remember, then
    /// a takeoff with NO reset in between. DECISIVE for `keep_warm` only at the
    /// 1000 Hz design rate — at any other rate the first control tick rebuilds
    /// the gyro filter (`resync_loop_rate`) — so scripts/split-differential.sh
    /// runs the twin at dt=0.001 as well.
    F1ArmedHold,
    /// Flight 1 airborne, waiting for a settled loiter.
    F1Climb,
    /// Flight 1 on its mission leg; ends in mission-complete RTL, land, disarm.
    F1Mission,
    /// On the ground after flight 1: estimate-only ticks, in-flight flag stale;
    /// then a `controller.configure` rebuild.
    F1Ground,
    /// Flight 2, waiting for loiter.
    F2Climb,
    /// Flight 2 with the pack sagged below LOW: failsafe RTL, land, disarm.
    F2Sag,
    /// On the ground: a refused re-arm, a silent pack, then a
    /// `configure-supervisor` the battery latch must reject.
    F2Ground,
    /// Climb to loiter in the termination / rotor-out segments.
    Climb,
    /// TERMINATED IN THE AIR, then the fall and the ground; then a
    /// `configure-supervisor` that `Terminated` must reject. This is the case
    /// that tells "estimate-only leaves the estimator's in-flight flag alone"
    /// apart from "estimate-only clears it": the fused core keeps the flag the
    /// last control tick set — TRUE, mid-hover — through every Terminated tick.
    Terminated,
    /// A dead rotor: isolate, land in place, disarm.
    RotorOut,
    /// On the ground after the rotor-out; then a `configure` the isolated
    /// rotor must reject.
    RotorGround,
    /// The segment is over.
    Done,
}

/// What the whole run must have exercised (the coverage verdict).
#[derive(Default)]
struct Coverage {
    ticks: u32,
    modes_seen: [u32; 8],
    failsafe_seen: u32,
    rotor_isolated: bool,
    armed_hold_ticks: u32,
    /// `arm` before the first step of a fresh supervisor, refused.
    fresh_refusals: u32,
    /// Other `arm`s that had to be refused (unconfigured, latched battery).
    other_refusals: u32,
    /// `arm`s refused while the accelerometer read zero.
    dead_accel_refusals: u32,
    /// Configures that #609 says must be REJECTED, and were.
    rejected_configures: Vec<&'static str>,
    /// Ticks from each rebuild to the accepted `arm` (the estimator-health
    /// rows decide this number).
    arm_ticks: Vec<u32>,
    segments: Vec<String>,
}

fn v3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

pub fn run(a_path: &str, b_path: &str, max_ticks: u32, dt: f32) -> Result<()> {
    let mut cfg = wasmtime::Config::new();
    cfg.wasm_component_model(true);
    let engine = Engine::new(&cfg)?;

    println!("=== twin differential (bit-exact, supervised on both sides) ===");
    println!("A (drives the plant): {a_path}");
    println!("B                   : {b_path}");
    println!("budget              : {max_ticks} ticks per segment @ dt={dt}s");

    let mut cov = Coverage::default();
    for seg in [Segment::Lifecycle, Segment::Termination, Segment::RotorOut] {
        segment(&engine, a_path, b_path, seg, max_ticks, dt, &mut cov)?;
    }

    println!();
    println!(
        "ticks compared      : {} (motors + every flight-state field, bit-exact)",
        cov.ticks
    );
    for s in &cov.segments {
        println!("segment             : {s}");
    }
    let m = &cov.modes_seen;
    println!(
        "ticks per mode      : disarmed {} armed {} takeoff {} loiter {} mission {} land {} rtl {} terminated {}",
        m[DISARMED as usize],
        m[ARMED as usize],
        m[TAKEOFF as usize],
        m[LOITER as usize],
        m[MISSION as usize],
        m[LAND as usize],
        m[RTL as usize],
        m[TERMINATED as usize]
    );
    println!("failsafe bits seen  : {:#010b}", cov.failsafe_seen);
    println!("rotor isolated      : {}", cov.rotor_isolated);
    println!(
        "armed hold          : {} ticks (noisy gyro, then takeoff, no reset)",
        cov.armed_hold_ticks
    );
    println!(
        "arm refusals        : {} before a fresh supervisor's first step, {} others",
        cov.fresh_refusals, cov.other_refusals
    );
    println!(
        "dead accelerometer  : {} consecutive arms refused (the deciding row, per the mirror test: EstimatorConverged)",
        cov.dead_accel_refusals
    );
    println!(
        "arm accepted after  : {:?} ticks from each (re)build or recovery",
        cov.arm_ticks
    );
    println!("rejected configures : {:?}", cov.rejected_configures);
    // `resync_loop_rate` rebuilds the gyro filter past 2% off the 1000 Hz
    // design rate, which erases what `keep_warm` kept during the hold.
    let keep_warm_decisive = ((1.0 / dt) - 1000.0).abs() <= 0.02 * 1000.0;
    println!(
        "keep_warm           : {}",
        if keep_warm_decisive {
            "DECISIVE (design rate: the hold's filter state reaches the motors)"
        } else {
            "not decisive at this rate (first control tick rebuilds the filter) — run at dt=0.001"
        }
    );

    // COVERAGE IS PART OF THE VERDICT. Bit-equality over a run that never left
    // the ground would be true and worthless.
    let mut missing = Vec::new();
    for (md, name) in [
        (ARMED, "armed"),
        (TAKEOFF, "takeoff"),
        (LOITER, "loiter"),
        (MISSION, "mission"),
        (RTL, "rtl"),
        (LAND, "land"),
        (TERMINATED, "terminated"),
    ] {
        if cov.modes_seen[md as usize] == 0 {
            missing.push(format!("mode {name} never published"));
        }
    }
    for (bit, name) in [
        (FS_ARM_BLOCKED, "arm-blocked"),
        (FS_BATT_LOW, "batt-low"),
        (FS_BATT_CRITICAL, "batt-critical"),
        (FS_RTL_LATCHED, "rtl-latched"),
        (FS_BATT_SILENT, "batt-silent"),
    ] {
        if cov.failsafe_seen & bit == 0 {
            missing.push(format!("failsafe bit {name} never raised"));
        }
    }
    if cov.armed_hold_ticks < ARMED_HOLD_TICKS {
        missing.push(format!(
            "flight 1 was held armed for {} ticks, not {ARMED_HOLD_TICKS}",
            cov.armed_hold_ticks
        ));
    }
    if !cov.rotor_isolated {
        missing.push("no rotor was ever isolated".into());
    }
    // Lifecycle: 2 rebuilds; termination and rotor-out: 1 each.
    if cov.fresh_refusals != 4 {
        missing.push(format!(
            "{} fresh-supervisor arm refusals, expected 4",
            cov.fresh_refusals
        ));
    }
    if cov.other_refusals != 2 {
        missing.push(format!(
            "{} unconfigured/latched arm refusals, expected 2",
            cov.other_refusals
        ));
    }
    if cov.dead_accel_refusals != DEAD_ACCEL_TICKS - 1 {
        missing.push(format!(
            "{} dead-accelerometer arm refusals, expected {}",
            cov.dead_accel_refusals,
            DEAD_ACCEL_TICKS - 1
        ));
    }
    if cov.rejected_configures.len() != 3 {
        missing.push(format!(
            "rejected configures {:?}, expected battery latch, terminated, isolated rotor",
            cov.rejected_configures
        ));
    }
    if !missing.is_empty() {
        bail!(
            "NO MISMATCH, BUT THE SCENARIO DID NOT COVER: {}",
            missing.join("; ")
        );
    }
    println!();
    println!("PASS: A and B agree bit-for-bit on every tick, through every mode above");
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn segment(
    engine: &Engine,
    a_path: &str,
    b_path: &str,
    seg: Segment,
    max_ticks: u32,
    dt: f32,
    cov: &mut Coverage,
) -> Result<()> {
    // FRESH INSTANCES per segment: #609 makes the end state of each one
    // terminal for an in-process configure.
    let mut a = Side::load(engine, a_path)?;
    let mut b = Side::load(engine, b_path)?;

    // The pre-tick contract: nothing measured, `mode` 0 NOT meaning disarmed.
    let (sa, sb) = (a.read_state()?, b.read_state()?);
    if let Some(d) = diff_state(&sa, &sb) {
        bail!("{seg:?}: MISMATCH before the first step: {d}");
    }
    if sa.valid != 0 {
        bail!(
            "{seg:?}: pre-tick state marks fields valid ({:#x})",
            sa.valid
        );
    }

    // Mock-plant tuning — the same numbers the main harness gives it.
    let calib = VehicleConfig {
        hover_thrust: 0.49,
        loop_rate_hz: 1.0 / dt,
        pos_var: 0.01,
        process_floor_vel: 0.0,
        process_floor_pos: 0.0,
        altitude_kp: 0.05,
        altitude_kd: 0.30,
        altitude_ki: 0.0,
        position_ki: 0.02,
    };
    let supcfg = SupervisorConfig {
        home: v3(0.0, 0.0, 0.0),
        fence_radius: 100.0,
        cruise_alt: CRUISE_M,
        low_batt_v: LOW_BATT_V,
    };
    let cal = SensorCalibration {
        gyro_bias: v3(GYRO_BIAS[0], GYRO_BIAS[1], GYRO_BIAS[2]),
        accel_bias: v3(0.0, 0.0, 0.0),
        accel_scale: v3(1.0, 1.0, 1.0),
        mag_offset: v3(0.0, 0.0, 0.0),
        mag_scale: v3(1.0, 1.0, 1.0),
    };
    macro_rules! both {
        ($($call:tt)*) => {{
            a.$($call)*?;
            b.$($call)*?;
        }};
    }
    // Same order main.rs uses: tuning, calibration, then (where the segment
    // wants one) the supervisor configuration.
    both!(configure(calib));
    both!(configure_calibration(cal));

    let (mut stage, mut after_arm) = match seg {
        Segment::Lifecycle => (Stage::Unconfigured, Stage::F1ArmedHold),
        Segment::Termination => {
            both!(configure_supervisor(supcfg));
            (Stage::DeadAccel, Stage::Climb)
        }
        Segment::RotorOut => {
            both!(configure_supervisor(supcfg));
            (Stage::Arming, Stage::Climb)
        }
    };

    let mut plant = MockPhysics::at_rest();
    // This scenario LANDS (and falls), so the plant needs a floor.
    plant.set_ground_contact(true);

    let gnss_div = ((1.0 / dt) / 5.0).round().max(1.0) as u32;
    let home_target = Waypoint {
        north: 0.0,
        east: 0.0,
        down: -CRUISE_M,
        yaw: 0.0,
    };
    let mut target = home_target;
    let mut battery: Option<f32> = Some(PACK_V);
    let mut stage_ticks: u32 = 0;
    let mut loiter_ticks: u32 = 0;
    // The pack goes silent at ground tick 300 and the configure comes 1.75 s
    // later, whatever the rate: `batt-silent` needs > 1 s of silence.
    let f2_reset_at: u32 = 300 + (1.75 / dt).ceil() as u32;
    let arm_deadline = (ARM_DEADLINE_S / dt) as u32;
    // A check to run on the published state AFTER the next step.
    #[derive(Clone, Copy, PartialEq)]
    enum Expect {
        None,
        /// An `arm` was just sent and must have been refused.
        Refused(&'static str),
        /// A configure was just sent and must have been rejected.
        Rejected(&'static str),
    }
    let mut expect = Expect::None;
    let mut prev_mode: u8 = DISARMED;
    let mut stages_done: Vec<Stage> = Vec::new();
    let mut seg_ticks: u32 = 0;
    // TWIN_TRACE=1 prints every published mode change of side A.
    let trace = std::env::var("TWIN_TRACE").is_ok_and(|v| v != "0");
    macro_rules! advance {
        ($next:expr) => {{
            stages_done.push(stage);
            stage = $next;
            stage_ticks = 0;
            loiter_ticks = 0;
        }};
    }

    for tick in 0..max_ticks {
        if stage == Stage::Done {
            break;
        }
        // ── scripted commands, applied to BOTH sides at the same point ──
        match stage {
            Stage::Unconfigured if stage_ticks == 100 => {
                both!(command(PilotCommand::Arm));
                expect = Expect::Refused("unconfigured supervisor");
            }
            Stage::Unconfigured if stage_ticks == 400 => {
                // REBUILD PATH 1, on the ground: permitted.
                both!(configure_supervisor(supcfg));
                after_arm = Stage::F1ArmedHold;
                // The FIRST `arm`, before the rebuilt supervisor's first step.
                both!(command(PilotCommand::Arm));
                expect = Expect::Refused("fresh supervisor, no step yet");
                advance!(Stage::Arming);
            }
            Stage::DeadAccel if stage_ticks == DEAD_ACCEL_TICKS => advance!(Stage::Arming),
            Stage::DeadAccel => {
                both!(command(PilotCommand::Arm));
                expect = Expect::Refused(if seg_ticks == 0 {
                    "fresh supervisor, no step yet"
                } else {
                    "dead accelerometer"
                });
            }
            Stage::Arming => {
                if stage_ticks > arm_deadline {
                    bail!(
                        "{seg:?}: never ARMED within {ARM_DEADLINE_S} s of a rebuild — the \
                         measured pre-arm table refused every `arm`"
                    );
                }
                // A rebuild sends its first `arm` itself, before the step it
                // happens on (see above); a segment's initial build is first
                // stepped here, so the arm at `stage_ticks == 0` is the one
                // that precedes it.
                if prev_mode == DISARMED {
                    both!(command(PilotCommand::Arm));
                    if seg_ticks == 0 {
                        expect = Expect::Refused("fresh supervisor, no step yet");
                    }
                }
            }
            Stage::F1ArmedHold if stage_ticks == ARMED_HOLD_TICKS => {
                both!(command(PilotCommand::RequestTakeoff));
                advance!(Stage::F1Climb);
            }
            // Entered from `Arming` (which advances AFTER a step, so this is
            // the first command point): take off.
            Stage::F2Climb | Stage::Climb if stage_ticks == 0 && prev_mode == ARMED => {
                both!(command(PilotCommand::RequestTakeoff));
            }
            Stage::F1Climb if loiter_ticks >= 400 => {
                target = Waypoint {
                    north: 3.0,
                    east: 0.0,
                    down: -CRUISE_M,
                    yaw: 0.0,
                };
                both!(command(PilotCommand::RequestMission));
                advance!(Stage::F1Mission);
            }
            Stage::F1Ground if stage_ticks == 400 => {
                // REBUILD PATH 2: `controller.configure`, on the ground with
                // nothing latched — permitted.
                target = home_target;
                both!(configure(calib));
                after_arm = Stage::F2Climb;
                // The FIRST `arm`, before the rebuilt supervisor's first step.
                both!(command(PilotCommand::Arm));
                expect = Expect::Refused("fresh supervisor, no step yet");
                advance!(Stage::Arming);
            }
            Stage::F2Climb if loiter_ticks >= 400 => {
                battery = Some(SAG_V);
                advance!(Stage::F2Sag);
            }
            Stage::F2Ground if stage_ticks == 200 => {
                both!(command(PilotCommand::Arm));
                expect = Expect::Refused("latched low battery");
            }
            Stage::F2Ground if stage_ticks == 300 => battery = None,
            Stage::F2Ground if stage_ticks == f2_reset_at => {
                // A healthy pack again, but the LOW latch holds: #609 finding 1
                // says this configure must be REJECTED (a rebuild would forget
                // the latch).
                battery = Some(PACK_V);
                both!(configure_supervisor(supcfg));
                expect = Expect::Rejected("configure-supervisor with a latched low battery");
            }
            Stage::F2Ground if stage_ticks == f2_reset_at + 400 => advance!(Stage::Done),
            Stage::Climb if loiter_ticks >= 400 => match seg {
                Segment::Termination => {
                    both!(command(PilotCommand::Terminate));
                    advance!(Stage::Terminated);
                }
                _ => {
                    plant.fail_rotor(0);
                    advance!(Stage::RotorOut);
                }
            },
            Stage::Terminated if stage_ticks == 800 => {
                both!(configure_supervisor(supcfg));
                expect = Expect::Rejected("configure-supervisor in Terminated");
            }
            Stage::Terminated if stage_ticks == 1200 => advance!(Stage::Done),
            Stage::RotorGround if stage_ticks == 200 => {
                both!(configure(calib));
                expect = Expect::Rejected("configure with an isolated rotor");
            }
            Stage::RotorGround if stage_ticks == 400 => advance!(Stage::Done),
            _ => {}
        }
        if stage == Stage::Done {
            break;
        }

        // ── one frame, offered to both ──
        // Noise ONLY while held armed (see `F1ArmedHold`); everywhere else the
        // plant is noiseless, as in the main harness. The gyro carries the
        // known bias the configured calibration removes.
        let noise = if stage == Stage::F1ArmedHold {
            ARMED_HOLD_GYRO_NOISE
        } else {
            0.0
        };
        let (mut s, true_pos) = plant.measure(noise);
        if stage == Stage::DeadAccel {
            s.accel_body = [0.0; 3];
        }
        let position_ned =
            (tick % gnss_div == 0).then(|| v3(true_pos[0], true_pos[1], true_pos[2]));
        let frame = SensorFrame {
            imu: WitImu {
                ax: s.accel_body[0],
                ay: s.accel_body[1],
                az: s.accel_body[2],
                gx: s.gyro_body[0] + GYRO_BIAS[0],
                gy: s.gyro_body[1] + GYRO_BIAS[1],
                gz: s.gyro_body[2] + GYRO_BIAS[2],
            },
            dt_s: dt,
            position_ned,
            mag_body: plant.mag_body_ned().map(|m| v3(m[0], m[1], m[2])),
            heading_rad: plant.heading_ned(),
            motor_rpm: plant.motor_rpm().map(|r| RotorRpm {
                m1: r[0],
                m2: r[1],
                m3: r[2],
                m4: r[3],
            }),
            battery_v: battery,
        };
        let ma = a.step(frame, target)?;
        let mb = b.step(frame, target)?;
        if let Some(d) = diff_motors(&ma, &mb) {
            bail!("{seg:?}: MISMATCH at tick {tick} ({stage:?}), motor command: {d}");
        }
        let (sa, sb) = (a.read_state()?, b.read_state()?);
        if let Some(d) = diff_state(&sa, &sb) {
            bail!("{seg:?}: MISMATCH at tick {tick} ({stage:?}), published state: {d}");
        }
        plant.step([ma.m1, ma.m2, ma.m3, ma.m4], dt);
        seg_ticks += 1;
        stage_ticks += 1;

        // ── the expectation set by this tick's command ──
        let mode = sa.mode;
        match expect {
            Expect::None => {}
            Expect::Refused(why) => {
                if mode != DISARMED {
                    bail!("{seg:?}: an `arm` that must be refused ({why}) armed: mode {mode}");
                }
                if why.starts_with("fresh") {
                    cov.fresh_refusals += 1;
                } else if why.starts_with("dead") {
                    cov.dead_accel_refusals += 1;
                } else {
                    cov.other_refusals += 1;
                }
            }
            Expect::Rejected(what) => {
                // A rebuild would publish Disarmed with no failsafe and no
                // isolated rotor. Each rejection is checked on the state the
                // rebuild would have erased.
                let kept = match stage {
                    Stage::F2Ground => sa.failsafe & FS_BATT_LOW != 0,
                    Stage::Terminated => mode == TERMINATED,
                    Stage::RotorGround => sa.failed_rotor != 0xFF,
                    _ => false,
                };
                if !kept {
                    bail!(
                        "{seg:?}: {what} was NOT rejected — the state a rebuild erases is gone \
                         (mode {mode}, failsafe {:#x}, failed-rotor {:#x})",
                        sa.failsafe,
                        sa.failed_rotor
                    );
                }
                cov.rejected_configures.push(what);
            }
        }
        expect = Expect::None;

        // ── coverage + stage transitions, from A's published state ──
        cov.modes_seen[(mode & 7) as usize] += 1;
        if stage == Stage::F1ArmedHold && mode == ARMED {
            cov.armed_hold_ticks += 1;
        }
        cov.failsafe_seen |= sa.failsafe;
        if sa.failed_rotor != 0xFF {
            cov.rotor_isolated = true;
        }
        loiter_ticks = if mode == LOITER { loiter_ticks + 1 } else { 0 };
        let touched_down = prev_mode == LAND && mode == DISARMED;
        if trace && mode != prev_mode {
            println!("  {seg:?} tick {tick} ({stage:?}+{stage_ticks}): mode {prev_mode} -> {mode}, failsafe {:#x}", sa.failsafe);
        }
        prev_mode = mode;
        match stage {
            Stage::Arming if mode == ARMED => {
                cov.arm_ticks.push(stage_ticks);
                advance!(after_arm);
            }
            Stage::F1Mission if touched_down => advance!(Stage::F1Ground),
            Stage::F2Sag if touched_down => advance!(Stage::F2Ground),
            Stage::RotorOut if touched_down => advance!(Stage::RotorGround),
            _ => {}
        }
    }
    if stage != Stage::Done {
        bail!("{seg:?}: scenario stopped at {stage:?} after {seg_ticks} ticks (budget too small?)");
    }
    cov.ticks += seg_ticks;
    cov.segments.push(format!(
        "{seg:?}: {seg_ticks} ticks, stages {stages_done:?}"
    ));
    Ok(())
}
