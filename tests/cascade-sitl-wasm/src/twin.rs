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
//! A HOLD IS NOT ENOUGH. A plain altitude hold keeps the mode and the failsafe
//! word constant, so a differential over it would be blind to most of what the
//! split has to preserve. The scenario below is event-driven (stages advance on
//! side A's PUBLISHED mode) and walks through: the pre-tick state; an
//! unconfigured supervisor refusing to arm; `configure-supervisor` and
//! `configure` resets mid-run; arming through `pilot`; takeoff, loiter, a
//! mission leg, the mission-complete RTL, the landing and the touchdown disarm;
//! estimate-only ticks after touchdown (where the estimator's in-flight flag is
//! deliberately left stale, #452); a low-battery failsafe; a refused re-arm; a
//! silent pack; a flight termination IN THE AIR (estimate-only ticks with the
//! in-flight flag still set); and a rotor-out with FDI isolation and
//! land-in-place. At the end it ASSERTS that the run actually
//! visited those modes and raised those failsafe bits, so a scenario that
//! silently stopped exercising them fails rather than passing on a hover.

use crate::exports::pulseengine::falcon_cascade::pilot::{PilotCommand, SupervisorConfig};
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

/// PENDING relay#586 — the stage that would make the ESTIMATOR-HEALTH forwards
/// decisive at this seam. Off by default; `TWIN_PENDING_586=1` turns it on.
///
/// WHY IT CANNOT RUN GREEN TODAY. `tilt_uncertainty`, `nav_compromised` and
/// `gnss_diverged` reach the outside world ONLY through the pre-arm gate (the
/// `EstimatorConverged` / `SensorsHealthy` / `EstimatorInnovation` /
/// `GnssAgreement` rows, and `arm-blocked` = bit 7). From the first refreshed
/// tick on, `CalibrationPresent` fails on every tick — no seam carries a
/// calibration (relay#586) — so arming is refused and bit 7 is set WHATEVER
/// the estimator says. A stubbed forward therefore changes nothing a host can
/// observe, and the only arm that succeeds is the one judged on the
/// constructor's bring-up defaults, before any row is refreshed. (In addition,
/// `nav_compromised` / `gnss_diverged` are constant `false` through this seam
/// by construction: both latches are fed only on the dual-GNSS path, which the
/// seam does not carry.)
///
/// WHAT THE STAGE DOES once #586 lets a calibration cross: after a reset it
/// steps ONE tick (fresh estimator, tilt NOT converged) and arms — which must
/// be REFUSED — then warms the estimator and arms again, which must be
/// ACCEPTED. A split whose tilt forward were stubbed to 0 would accept the
/// first arm and diverge in the published mode. Enabled today it stops with a
/// `PENDING relay#586` error at the second arm. The forward IS measured
/// directly, in CI, by crates/falcon-core/tests/supervised_core_split.rs.
const PENDING_586_ENV: &str = "TWIN_PENDING_586";

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

/// The scenario's stages. Each advances on side A's PUBLISHED state.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Stage {
    /// Supervisor never configured: an `arm` must be refused.
    Unconfigured,
    /// PENDING relay#586 (see `PENDING_586_ENV`): arm before and after the
    /// estimator converges, on the refreshed pre-arm table.
    Pending586,
    /// Flight 1 ARMED but held on the ground: estimate-only ticks with a NOISY
    /// gyro, so the control half's `keep_warm` has something to remember, and
    /// then a takeoff with NO reset in between. This is the only reset-free
    /// estimate-only -> flight path a host can reach today: re-arming from
    /// Disarmed after the first tick is refused through this seam (relay#586,
    /// `CalibrationPresent`). It is DECISIVE for `keep_warm` only at the
    /// 1000 Hz design rate — at any other rate the first control tick rebuilds
    /// the gyro filter (`resync_loop_rate`) and erases what `keep_warm` kept —
    /// so scripts/split-differential.sh runs the twin at dt=0.001 as well.
    F1ArmedHold,
    /// Flight 1 airborne, waiting for a settled loiter.
    F1Climb,
    /// Flight 1 on its mission leg; ends in mission-complete RTL, land, disarm.
    F1Mission,
    /// On the ground after flight 1: estimate-only ticks, in-flight flag stale.
    F1Ground,
    /// Flight 2 (after a `controller.configure` reset), waiting for loiter.
    F2Climb,
    /// Flight 2 with the pack sagged below LOW: failsafe RTL, land, disarm.
    F2Sag,
    /// On the ground: a refused re-arm, then the pack goes silent.
    F2Ground,
    /// Flight 3 (after a `configure-supervisor` reset), waiting for loiter.
    F3Climb,
    /// Flight 3 TERMINATED IN THE AIR, then the fall and the ground. This is
    /// the case that tells "estimate-only leaves the estimator's in-flight flag
    /// alone" apart from "estimate-only clears it": the fused core keeps the
    /// flag the last control tick set — TRUE, mid-hover — so the gravity update
    /// stays off through every Terminated tick.
    F3Terminated,
    /// Flight 4 (after another `configure-supervisor` reset), waiting for loiter.
    F4Climb,
    /// Flight 4 with a dead rotor: isolate, land in place, disarm.
    F4RotorOut,
    /// On the ground after flight 4; then the run ends.
    Done,
}

pub fn run(a_path: &str, b_path: &str, max_ticks: u32, dt: f32) -> Result<()> {
    let mut cfg = wasmtime::Config::new();
    cfg.wasm_component_model(true);
    let engine = Engine::new(&cfg)?;
    let mut a = Side::load(&engine, a_path)?;
    let mut b = Side::load(&engine, b_path)?;

    println!("=== twin differential (bit-exact, supervised on both sides) ===");
    println!("A (drives the plant): {a_path}");
    println!("B                   : {b_path}");
    println!("budget              : {max_ticks} ticks @ dt={dt}s");

    // The pre-tick contract: nothing measured, `mode` 0 NOT meaning disarmed.
    let (sa, sb) = (a.read_state()?, b.read_state()?);
    if let Some(d) = diff_state(&sa, &sb) {
        bail!("MISMATCH before the first step: {d}");
    }
    if sa.valid != 0 {
        bail!("pre-tick state marks fields valid ({:#x})", sa.valid);
    }

    // Mock-plant calibration — the same numbers the main harness gives it.
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
        home: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        fence_radius: 100.0,
        cruise_alt: CRUISE_M,
        low_batt_v: LOW_BATT_V,
    };
    macro_rules! both {
        ($($call:tt)*) => {{
            a.$($call)*?;
            b.$($call)*?;
        }};
    }
    both!(configure(calib));

    let mut plant = MockPhysics::at_rest();
    // This scenario LANDS (and falls), four times, so the plant needs a floor.
    plant.set_ground_contact(true);

    let gnss_div = ((1.0 / dt) / 5.0).round().max(1.0) as u32;
    let mut target = Waypoint {
        north: 0.0,
        east: 0.0,
        down: -CRUISE_M,
        yaw: 0.0,
    };
    let mut battery: Option<f32> = Some(PACK_V);
    let mut stage = Stage::Unconfigured;
    let mut stage_ticks: u32 = 0;
    let mut loiter_ticks: u32 = 0;
    let mut modes_seen = [0u32; 8];
    let mut armed_hold_ticks: u32 = 0;
    // The pack goes silent at ground tick 300 and the reset comes 1.75 s later,
    // whatever the rate: `batt-silent` needs > BATT_SILENCE_S (1 s) of silence.
    let f2_reset_at: u32 = 300 + (1.75 / dt).ceil() as u32;
    let pending_586 = std::env::var(PENDING_586_ENV).is_ok_and(|v| v != "0");
    let mut failsafe_seen: u32 = 0;
    let mut rotor_isolated = false;
    let mut prev_mode: u8 = DISARMED;
    let mut ticks_run: u32 = 0;
    let mut stages_done: Vec<Stage> = Vec::new();

    for tick in 0..max_ticks {
        // ── scripted commands, applied to BOTH sides at the same point ──
        match stage {
            Stage::Unconfigured if stage_ticks == 100 => both!(command(PilotCommand::Arm)),
            Stage::Unconfigured if stage_ticks == 400 => {
                // RESET PATH 1: drops the supervisor (and, in the split, both
                // partitions), then arms on the fresh supervisor.
                both!(configure_supervisor(supcfg));
                stages_done.push(stage);
                stage_ticks = 0;
                if pending_586 {
                    stage = Stage::Pending586;
                } else {
                    both!(command(PilotCommand::Arm));
                    stage = Stage::F1ArmedHold;
                }
            }
            // One tick after the reset: the estimator has NOT converged.
            Stage::Pending586 if stage_ticks == 1 => both!(command(PilotCommand::Arm)),
            Stage::Pending586 if stage_ticks == 2 && prev_mode == ARMED => {
                bail!("PENDING-586 stage: an UNCONVERGED estimator was allowed to arm")
            }
            Stage::Pending586 if stage_ticks == 800 => both!(command(PilotCommand::Arm)),
            Stage::Pending586 if stage_ticks == 801 => {
                if prev_mode != ARMED {
                    bail!(
                        "PENDING relay#586: a warmed estimator is still refused on the \
                         refreshed pre-arm table — CalibrationPresent cannot be satisfied \
                         through this seam, so the estimator-health rows are not decisive"
                    );
                }
                stages_done.push(stage);
                stage = Stage::F1ArmedHold;
                stage_ticks = 0;
            }
            Stage::F1ArmedHold if stage_ticks == ARMED_HOLD_TICKS => {
                both!(command(PilotCommand::RequestTakeoff));
                stages_done.push(stage);
                stage = Stage::F1Climb;
                stage_ticks = 0;
            }
            Stage::F1Climb if loiter_ticks >= 400 => {
                target = Waypoint {
                    north: 3.0,
                    east: 0.0,
                    down: -CRUISE_M,
                    yaw: 0.0,
                };
                both!(command(PilotCommand::RequestMission));
                stages_done.push(stage);
                stage = Stage::F1Mission;
                stage_ticks = 0;
            }
            Stage::F1Ground if stage_ticks == 400 => {
                // RESET PATH 2: `controller.configure` mid-run.
                target = Waypoint {
                    north: 0.0,
                    east: 0.0,
                    down: -CRUISE_M,
                    yaw: 0.0,
                };
                both!(configure(calib));
                both!(command(PilotCommand::Arm));
                both!(command(PilotCommand::RequestTakeoff));
                stages_done.push(stage);
                stage = Stage::F2Climb;
                stage_ticks = 0;
                loiter_ticks = 0;
            }
            Stage::F2Climb if loiter_ticks >= 400 => {
                battery = Some(SAG_V);
                stages_done.push(stage);
                stage = Stage::F2Sag;
                stage_ticks = 0;
            }
            Stage::F2Ground if stage_ticks == 200 => {
                // Refused: the pack is latched low.
                both!(command(PilotCommand::Arm));
            }
            Stage::F2Ground if stage_ticks == 300 => battery = None,
            Stage::F2Ground if stage_ticks == f2_reset_at => {
                // RESET PATH 1 again, with a healthy pack.
                battery = Some(PACK_V);
                both!(configure_supervisor(supcfg));
                both!(command(PilotCommand::Arm));
                both!(command(PilotCommand::RequestTakeoff));
                stages_done.push(stage);
                stage = Stage::F3Climb;
                stage_ticks = 0;
                loiter_ticks = 0;
            }
            Stage::F3Climb if loiter_ticks >= 400 => {
                // FLIGHT TERMINATION, IN THE AIR.
                both!(command(PilotCommand::Terminate));
                stages_done.push(stage);
                stage = Stage::F3Terminated;
                stage_ticks = 0;
            }
            Stage::F3Terminated if stage_ticks == 800 => {
                // Terminated is left only by an on-ground reset.
                both!(configure_supervisor(supcfg));
                both!(command(PilotCommand::Arm));
                both!(command(PilotCommand::RequestTakeoff));
                stages_done.push(stage);
                stage = Stage::F4Climb;
                stage_ticks = 0;
                loiter_ticks = 0;
            }
            Stage::F4Climb if loiter_ticks >= 400 => {
                plant.fail_rotor(0);
                stages_done.push(stage);
                stage = Stage::F4RotorOut;
                stage_ticks = 0;
            }
            Stage::Done if stage_ticks == 400 => break,
            _ => {}
        }

        // ── one frame, offered to both ──
        // Noise ONLY while held armed (see `F1ArmedHold`); everywhere else the
        // plant is noiseless, as in the main harness.
        let noise = if stage == Stage::F1ArmedHold {
            ARMED_HOLD_GYRO_NOISE
        } else {
            0.0
        };
        let (s, true_pos) = plant.measure(noise);
        let position_ned = (tick % gnss_div == 0).then(|| Vec3 {
            x: true_pos[0],
            y: true_pos[1],
            z: true_pos[2],
        });
        let frame = SensorFrame {
            imu: WitImu {
                ax: s.accel_body[0],
                ay: s.accel_body[1],
                az: s.accel_body[2],
                gx: s.gyro_body[0],
                gy: s.gyro_body[1],
                gz: s.gyro_body[2],
            },
            dt_s: dt,
            position_ned,
            mag_body: plant.mag_body_ned().map(|m| Vec3 {
                x: m[0],
                y: m[1],
                z: m[2],
            }),
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
            bail!("MISMATCH at tick {tick} ({stage:?}), motor command: {d}");
        }
        let (sa, sb) = (a.read_state()?, b.read_state()?);
        if let Some(d) = diff_state(&sa, &sb) {
            bail!("MISMATCH at tick {tick} ({stage:?}), published state: {d}");
        }
        plant.step([ma.m1, ma.m2, ma.m3, ma.m4], dt);
        ticks_run += 1;
        stage_ticks += 1;

        // ── coverage + stage transitions, from A's published state ──
        let mode = sa.mode;
        modes_seen[(mode & 7) as usize] += 1;
        if stage == Stage::F1ArmedHold && mode == ARMED {
            armed_hold_ticks += 1;
        }
        failsafe_seen |= sa.failsafe;
        if sa.failed_rotor != 0xFF {
            rotor_isolated = true;
        }
        loiter_ticks = if mode == LOITER { loiter_ticks + 1 } else { 0 };
        let touched_down = prev_mode == LAND && mode == DISARMED;
        prev_mode = mode;
        match stage {
            Stage::F1Mission if touched_down => {
                stages_done.push(stage);
                stage = Stage::F1Ground;
                stage_ticks = 0;
            }
            Stage::F2Sag if touched_down => {
                stages_done.push(stage);
                stage = Stage::F2Ground;
                stage_ticks = 0;
            }
            Stage::F4RotorOut if touched_down => {
                stages_done.push(stage);
                stage = Stage::Done;
                stage_ticks = 0;
            }
            _ => {}
        }
    }

    println!();
    println!("ticks compared      : {ticks_run} (motors + every flight-state field, bit-exact)");
    println!("stages completed    : {stages_done:?}; final {stage:?}");
    println!(
        "ticks per mode      : disarmed {} takeoff {} loiter {} mission {} land {} rtl {} terminated {}",
        modes_seen[DISARMED as usize],
        modes_seen[TAKEOFF as usize],
        modes_seen[LOITER as usize],
        modes_seen[MISSION as usize],
        modes_seen[LAND as usize],
        modes_seen[RTL as usize],
        modes_seen[TERMINATED as usize]
    );
    println!("failsafe bits seen  : {failsafe_seen:#010b}");
    println!("rotor isolated      : {rotor_isolated}");
    println!("armed hold          : {armed_hold_ticks} ticks (noisy gyro, then takeoff, no reset)");
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
    println!(
        "estimator health    : {}",
        if pending_586 {
            "PENDING-586 stage RAN"
        } else {
            "PENDING relay#586 — masked by CalibrationPresent at this seam (see PENDING_586_ENV); measured directly in falcon-core tests/supervised_core_split.rs"
        }
    );

    // COVERAGE IS PART OF THE VERDICT. Bit-equality over a run that never left
    // the ground would be true and worthless.
    let mut missing = Vec::new();
    if stage != Stage::Done {
        missing.push(format!("scenario stopped at {stage:?} (budget too small?)"));
    }
    for (m, name) in [
        (TAKEOFF, "takeoff"),
        (LOITER, "loiter"),
        (MISSION, "mission"),
        (RTL, "rtl"),
        (LAND, "land"),
        (TERMINATED, "terminated"),
    ] {
        if modes_seen[m as usize] == 0 {
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
        if failsafe_seen & bit == 0 {
            missing.push(format!("failsafe bit {name} never raised"));
        }
    }
    if armed_hold_ticks < ARMED_HOLD_TICKS {
        missing.push(format!(
            "flight 1 was held armed for {armed_hold_ticks} ticks, not {ARMED_HOLD_TICKS}"
        ));
    }
    if !rotor_isolated {
        missing.push("no rotor was ever isolated".into());
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
