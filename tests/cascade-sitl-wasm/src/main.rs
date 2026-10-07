//! Fly the SHIPPED wasm cascade through the SITL plant.
//!
//! WHY THIS EXISTS. Every Gazebo/SITL result falcon has — hover, the
//! Monte-Carlo campaigns, rotor-out recovery, touchdown — was produced by
//! `examples/falcon-sitl-gz`, which links the NATIVE crates (relay-iekf,
//! relay-pos, relay-att, relay-rate, relay-mix-quad). It contains no wasmtime
//! and loads no `.wasm`. So the flight evidence is about the native binary,
//! and the published wasm components had never flown anything.
//!
//! The nearest prior art, `tests/rate-loop-proof`, does close a loop across the
//! wasm seam — but one stage against a single-axis rigid body with a constant
//! quaternion and no gravity. A real loop, not a plant, and not the cascade.
//!
//! This closes the actual gap: the SAME `MockPhysics` plant the SITL bench
//! flies (included by `#[path]` so it is the same code, not a copy), driven by
//! the composed cascade across the Component Model seam.
//!
//! The component under test is `wac plug`-composed from the five PUBLISHED
//! stage components into the published cascade socket — the artifacts the
//! release actually ships, not the bazel `falcon-cascade-composed`, which is a
//! different build path (std-linked, 15.7 MB, 18 wasi imports, 6 memory.grow
//! against the published set's 0/0).
//!
//! Usage:
//!   cascade-sitl-wasm <composed.wasm> [ticks] [dt]

// The SITL plant itself. `gz_real` inside it is `#[cfg(feature = "gazebo")]`,
// so without that feature this pulls exactly `Physics` + `MockPhysics`.
#[path = "../../../examples/falcon-sitl-gz/src/physics.rs"]
mod physics;
mod native_mirror;
// The SAME pacing logic the native bench uses, included by path rather than
// copied — for the same reason physics.rs is: a copy would let the two drift,
// and the entire point is that the wasm harness and the native bench pace the
// plant identically.
#[path = "../../../examples/falcon-sitl-gz/src/pace.rs"]
mod pace;

use anyhow::{bail, Context, Result};
use physics::{MockPhysics, Physics};
#[cfg(feature = "gazebo")]
use physics::GazeboPhysics;
use wasmtime::component::{Component, Linker};
use wasmtime::{Config, Engine, Store};

// Declaring the host's own view inline keeps this harness pinned to exactly the
// surface it drives, independent of the other worlds in the package.
//
// (The comment that stood here said the WIT is spar-derived and hand-editing it
// would trip the drift gate. That is false and was worth correcting rather than
// deleting: spar.yml enumerates three roots — relay-transport, dronecan and
// param — and falcon-cascade is not one of them. The claim would have deterred
// exactly the seam change v0.9 needed.)
wasmtime::component::bindgen!({
    inline: r#"
        package host:sitl;
        world composed-cascade {
            export pulseengine:falcon-cascade/controller@0.11.0;
            export pulseengine:falcon-cascade/observer@0.11.0;
        }
    "#,
    path: "../../wit/falcon-cascade",
});

use pulseengine::falcon_cascade::types::{
    ImuSample as WitImu, RotorRpm, SensorFrame, Vec3, VehicleConfig, Waypoint,
};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let wasm = args.next().unwrap_or_else(|| {
        eprintln!("usage: cascade-sitl-wasm <composed.wasm> [ticks] [dt]");
        std::process::exit(2);
    });
    let ticks: u32 = args.next().unwrap_or_else(|| "4000".into()).parse()?;
    let dt: f32 = args.next().unwrap_or_else(|| "0.0025".into()).parse()?;

    let mut cfg = Config::new();
    cfg.wasm_component_model(true);
    let engine = Engine::new(&cfg)?;
    let component = Component::from_file(&engine, &wasm)
        .with_context(|| format!("loading {wasm}"))?;

    // Empty linker: the composed component imports only `types`, which is
    // records-only — no functions for a host to supply, and no WASI. If this
    // instantiation ever needs a WASI context, the artifact under test is not
    // the no_std one we ship.
    let linker: Linker<()> = Linker::new(&engine);
    let mut store = Store::new(&engine, ());
    let bindings = ComposedCascade::instantiate(&mut store, &component, &linker)
        .context("instantiating the composed cascade (empty linker, no WASI)")?;
    let controller = bindings.pulseengine_falcon_cascade_controller();
    // THE RETURN DIRECTION (SWREQ-FALCON-TRANSPORT-P01, seam @0.11.0). Exercised
    // here rather than merely compiled against: this is the only harness that
    // drives the PUBLISHED component through a real physics engine, so it is the
    // only place the seqlock and the monotonic tick can be checked across the
    // actual Component Model ABI instead of in-process.
    let observer = bindings.pulseengine_falcon_cascade_observer();
    let mut last_tick: u32 = 0;
    let mut state_reads: u32 = 0;

    // Hold 2 m above the launch point. NED: down is negative up.
    let down: f32 = std::env::var("TARGET_DOWN").ok().and_then(|v| v.parse().ok()).unwrap_or(-2.0);
    let target = Waypoint { north: 0.0, east: 0.0, down, yaw: 0.0 };
    let noise: f32 = std::env::var("IMU_NOISE").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    // BACKEND. `mock` is the analytic plant every number in #380 came from;
    // `gazebo` is the same real bridge the native bench flies, so a wasm result
    // and a native result are comparable rather than merely adjacent.
    let backend = std::env::var("BACKEND").unwrap_or_else(|_| "mock".into());
    let mut boxed: Box<dyn Physics> = match backend.as_str() {
        "mock" => Box::new(MockPhysics::at_rest()),
        #[cfg(feature = "gazebo")]
        "gazebo" => {
            let world = std::env::var("GZ_WORLD").unwrap_or_else(|_| "falcon".into());
            let model = std::env::var("GZ_MODEL").unwrap_or_else(|_| "quad".into());
            match GazeboPhysics::connect(&world, &model) {
                Some(p) => Box::new(p),
                None => bail!(
                    "could not connect to gz world '{world}' model '{model}'. Is `gz sim` \
                     running with worlds/falcon-quad.sdf, and gz-transport13 on the \
                     library path?"
                ),
            }
        }
        #[cfg(not(feature = "gazebo"))]
        "gazebo" => bail!("rebuild with --features gazebo to use the real gz bridge"),
        other => bail!("unknown BACKEND '{other}' (expected: mock | gazebo)"),
    };
    let plant = &mut *boxed;

    println!("=== wasm cascade in the SITL loop ===");
    println!("component : {wasm}");
    println!("plant     : {} (BACKEND={backend})", plant.name());
    println!("target    : hold N=0 E=0 D={down} m, yaw 0");
    println!("schedule  : {ticks} ticks @ dt={dt}s ({:.1}s, {:.0} Hz)", ticks as f32 * dt, 1.0 / dt);
    println!();

    // ── VEHICLE CALIBRATION (v0.9 seam) ──────────────────────────────────
    // These are the SAME numbers examples/falcon-sitl-gz installs on its own
    // FlightCore, so "the wasm matches the native reference" is a statement
    // about the control law rather than about two different tunings.
    //
    // Before this seam existed the component could fly nothing but its
    // defaults, and on gz those defaults do not leave the ground: measured
    // 1.91 m hold error with est_z -0.09 m, against 0.16 m for the tuned
    // native run. hover-thrust is the dominant term — at 0.5 the airframe
    // cannot lift and the altitude estimate diverges to -58.8 m.
    //
    // The mock plant deliberately gets ONLY its hover thrust: the native
    // scenario applies its tuning under `if name != "mock"`, so mirroring it
    // means leaving every other knob at the falcon-core default.
    let gz = plant.name() != "mock";
    let calib = VehicleConfig {
        // mock hovers at ~0.49 (THRUST_SCALE 20 m/s² vs g); the gz falcon-quad
        // at ~0.585 (ω_hover≈757 of maxRotVel 1000 through the √pwm map).
        hover_thrust: if gz { 0.585 } else { 0.49 },
        loop_rate_hz: 1.0 / dt,
        pos_var: if gz { 0.25 } else { 0.01 },
        process_floor_vel: if gz { 0.30 } else { 0.0 },
        process_floor_pos: if gz { 0.05 } else { 0.0 },
        altitude_kp: if gz { 0.15 } else { 0.05 },
        altitude_kd: if gz { 1.00 } else { 0.30 },
        altitude_ki: if gz { 0.03 } else { 0.0 },
        // Not touched by the native scenario in either mode — so it must carry
        // falcon-core's default (0.02), not 0.
        position_ki: 0.02,
    };
    println!(
        "calibration: hover={:.3} rate={:.0}Hz pos_var={:.2} alt=({:.2},{:.2},{:.2})",
        calib.hover_thrust, calib.loop_rate_hz, calib.pos_var,
        calib.altitude_kp, calib.altitude_kd, calib.altitude_ki
    );
    controller.call_configure(&mut store, calib)?;
    println!();

    let gnss_div: u32 = std::env::var("GNSS_DIV").ok().and_then(|v| v.parse().ok())
        .unwrap_or_else(|| ((1.0 / dt) / 5.0).round().max(1.0) as u32);
    let mut fixes = 0u32;

    // DIFFERENTIAL=1 runs the SAME flight core natively alongside and reports
    // the worst per-motor disagreement. This is "develop in wasm, deploy that
    // wasm unchanged" made falsifiable: identical Rust reached two ways must
    // agree, and if it does not, the Component Model is not transparent.
    let mut differential = std::env::var("DIFFERENTIAL")
        .ok().filter(|v| v != "0")
        .map(|_| {
            let mut n = native_mirror::NativeCascade::new();
            // Derived from the SAME `calib` the component was given, so the two
            // sides cannot drift apart by someone editing one literal.
            n.configure(native_mirror::Calib {
                hover_thrust: calib.hover_thrust,
                loop_rate_hz: calib.loop_rate_hz,
                pos_var: calib.pos_var,
                process_floor_vel: calib.process_floor_vel,
                process_floor_pos: calib.process_floor_pos,
                altitude_kp: calib.altitude_kp,
                altitude_kd: calib.altitude_kd,
                altitude_ki: calib.altitude_ki,
                position_ki: calib.position_ki,
            });
            (n, 0.0f32, 0u32)
        });

    // ── PACING (found by cpetig) ──────────────────────────────────────────
    // The loop had NONE. measure() -> call_step() -> step() ran flat out, and
    // the gz bridge is fully non-blocking: `step` is a fire-and-forget publish
    // and `measure` drains a channel and returns the cached latest sample. So
    // 2000 ticks burned ~0.5 s of wall clock, gz advanced ~0.5 s of sim, the
    // controller re-consumed the SAME stale IMU sample roughly 4x per fresh
    // one, and motor commands flooded out faster than physics stepped.
    //
    // The dt this harness passed to the component was therefore FICTION — which
    // also means every gz number it produced was measuring a desynchronised
    // loop rather than the controller.
    //
    // The native bench never had this: it paces on `counters().is_some()`. The
    // wasm harness simply never got the same treatment.
    let tick_period = std::time::Duration::from_secs_f32(dt);
    let pace_real_time = plant.counters().is_some();
    let sim_lock = pace_real_time && std::env::var("NO_SIM_LOCK").is_err();
    let pace_deadline_us = (tick_period.as_micros() as u64).saturating_mul(8).max(2000);
    let mut last_imu = plant.counters().map(|c| c.0).unwrap_or(0);
    let run_start = std::time::Instant::now();

    // NOT TILT. This is sqrt(ax^2 + ay^2) -- lateral SPECIFIC FORCE. Independent
    // review found it published as `peak_tilt` while FV-FALCON-FAULT-005's wasm
    // half was being judged on it (#499). Renamed, and a real tilt measurement
    // added below from the plant's own attitude.
    let mut peak_lat_accel = 0.0f32;
    // ROTOR-OUT STATE (#398 wasm half). Tracked only after the fault, because a
    // three-rotor quad legitimately departs from its pre-fault attitude.
    let mut peak_true_tilt_after = 0.0f32;
    let mut peak_horiz_after = 0.0f32;
    let mut isolated_rotor: Option<u8> = None;
    let mut saw_true_tilt = false;
    let (mut pace_fresh, mut pace_deadline) = (0u32, 0u32);
    // v0.10 seam exercise. FAIL_ROTOR=<idx>@<seconds> kills a rotor mid-flight;
    // NO_RPM=1 withholds the ESC telemetry. Together they make the new field
    // FALSIFIABLE rather than merely present: with telemetry the rotor-out FDI
    // inside the component can see the loss, without it the FDI is inert, and
    // the two runs must not produce the same flight.
    let fail_rotor: Option<(usize, u32)> = std::env::var("FAIL_ROTOR").ok().and_then(|v| {
        let (idx, at) = v.split_once('@')?;
        Some((idx.parse().ok()?, (at.parse::<f32>().ok()? / dt) as u32))
    });
    let no_rpm = std::env::var("NO_RPM").is_ok();
    // A FLOOR, for the scenario that comes down. The native rotor-out scenario
    // does exactly this ("This scenario LANDS, so the plant needs a floor to
    // land on", examples/falcon-sitl-gz/src/main.rs:1484) and this harness did
    // not, so the airframe sank 1.01 m THROUGH the ground plane and kept
    // flying -- the same defect the mock plant's own ground comment records
    // ("a rotor-out scenario ended 359 m below its launch point while the
    // verdict called it landed").
    //
    // GATED on the fault, matching native, which enables it only in the
    // landing scenario. It is not free: the clamp feeds the specific-force
    // derivation (#485), so switching it on unconditionally would change the
    // nominal hold this harness is the CI oracle for.
    if fail_rotor.is_some() {
        plant.set_ground_contact(true);
    }
    let rotor_trace = std::env::var("ROTOR_TRACE").is_ok();
    // The judged window (fault -> first touchdown) and the full-run figure.
    let mut touchdown_tick: Option<u32> = None;
    let mut airborne_ticks: u32 = 0;
    let mut peak_horiz_full: f32 = 0.0;
    // PEAK HORIZONTAL OVER THE WHOLE RUN, tracked UNCONDITIONALLY — the nominal
    // hold needs it and `peak_horiz_full` above cannot serve: that one is only
    // updated inside `if let Some((r, at)) = fail_rotor`, so on a nominal flight
    // it stays 0.0 forever. Kept as a separate variable rather than widening
    // that one, because its rotor-out meaning is "peak SINCE THE LOSS" and the
    // verified FAULT-005 record quotes it.
    //
    // This is plant TRUTH (`plant.measure` returns `(sensors, true_pos)`), not
    // the estimate. Judging the hold on the estimate is the #403 /
    // accelerometer-as-gravity failure mode: a diverging estimator reports its
    // own setpoint back and the bar never fires.
    let mut peak_horiz_all: f32 = 0.0;
    let mut rpm_frames = 0u32;
    for tick in 0..ticks {
        let tick_start = std::time::Instant::now();
        let (s, true_pos) = plant.measure(noise);
        {
            let h = (true_pos[0] * true_pos[0] + true_pos[1] * true_pos[1]).sqrt();
            if h > peak_horiz_all {
                peak_horiz_all = h;
            }
        }
        let imu = WitImu {
            ax: s.accel_body[0], ay: s.accel_body[1], az: s.accel_body[2],
            gx: s.gyro_body[0],  gy: s.gyro_body[1],  gz: s.gyro_body[2],
        };
        // The tick that matters: one full estimate -> position -> attitude ->
        // rate -> mixer pass, executed inside the wasm component.
        // v0.8: the host states its own period and offers what it has. A 5 Hz
        // position fix matches the native bench's gnss_div=50 @250 Hz.
        let position_ned = if gnss_div > 0 && tick % gnss_div == 0 {
            fixes += 1;
            Some(Vec3 { x: true_pos[0], y: true_pos[1], z: true_pos[2] })
        } else {
            None
        };
        // Aiding measurements, offered exactly as the native SitlBackend offers
        // them. Passing `None` here was a harness defect, not a seam limit: the
        // v0.8 seam already carries both fields and the gz plant already exposes
        // both. Without `heading` yaw is UNOBSERVABLE — the estimate drifts, the
        // geometric SE(3) attitude error is computed against a wrong yaw, the
        // thrust axis tilts off vertical and the vehicle climbs and then falls.
        // Measured with them absent: reached 1.08 m at 1.2 s, then -0.54 m by
        // 8 s, on a calibration that holds 2.00 m with them present.
        let mag_body = plant.mag_body_ned().map(|m| Vec3 { x: m[0], y: m[1], z: m[2] });
        let heading_rad = plant.heading_ned();
        // v0.10: ESC telemetry, so the rotor-out FDI is live inside the
        // component rather than inert. The native backend has always fed this.
        let rpm = if no_rpm { None } else { plant.motor_rpm() };
        if rpm.is_some() {
            rpm_frames += 1;
        }
        let motor_rpm = rpm.map(|r| RotorRpm { m1: r[0], m2: r[1], m3: r[2], m4: r[3] });
        let frame = SensorFrame {
            imu,
            dt_s: dt,
            position_ned,
            mag_body,
            heading_rad,
            motor_rpm,
        };
        let m = controller.call_step(&mut store, frame, target)?;

        // READ THE PUBLISHED STATE, every tick, and hold it to its contract.
        // Three properties, each of which has a real failure mode:
        //   head == tail   -> the body corresponds to ONE tick (seqlock)
        //   tick advances  -> `step` actually republished; a frozen tick means
        //                     an observer would read stale numbers forever with
        //                     no error, which is the absence-looks-like-data
        //                     shape this project keeps finding
        //   valid != 0     -> fields are marked measured, not silently zero
        let st = observer.call_read_state(&mut store)?;
        state_reads += 1;
        if st.tick_head != st.tick_tail {
            anyhow::bail!(
                "seqlock torn at tick {tick}: head {} != tail {}",
                st.tick_head,
                st.tick_tail
            );
        }
        if tick > 0 && st.tick_head != last_tick.wrapping_add(1) {
            anyhow::bail!(
                "published tick did not advance by 1 at tick {tick}: {} -> {}",
                last_tick,
                st.tick_head
            );
        }
        if st.valid == 0 {
            anyhow::bail!("published state at tick {tick} marks NOTHING valid");
        }
        last_tick = st.tick_head;

        // Fed the IDENTICAL frame — both sides must see one input sequence or
        // they diverge for reasons that say nothing about the boundary. The
        // plant is advanced by the WASM output, so the native side is a pure
        // observer of the artifact under test.
        if let Some((nat, worst, worst_tick)) = differential.as_mut() {
            let n = nat.step(
                s.accel_body,
                s.gyro_body,
                [target.north, target.east, target.down],
                position_ned.map(|p| [p.x, p.y, p.z]),
                mag_body.map(|m| [m.x, m.y, m.z]),
                heading_rad,
                rpm,
                dt,
            );
            let dmax = [
                (n[0] - m.m1).abs(), (n[1] - m.m2).abs(),
                (n[2] - m.m3).abs(), (n[3] - m.m4).abs(),
            ].iter().copied().fold(0.0f32, f32::max);
            if dmax > *worst { *worst = dmax; *worst_tick = tick; }
        }
        let lat = (s.accel_body[0].powi(2) + s.accel_body[1].powi(2)).sqrt();
        peak_lat_accel = peak_lat_accel.max(lat);

        // AFTER THE FAULT, judge the properties FAULT-P02 actually claims:
        // the FDI isolates the dead rotor and the body stays upright. Altitude
        // hold is NOT one of them -- a rank-deficient quad relinquishes yaw and
        // cannot hold altitude (Mueller & D'Andrea), and judging the wasm leg on
        // an altitude bar is the exact error the native leg already corrected.
        if let Some((r, at)) = fail_rotor {
            if tick >= at {
                // `true_pos` comes from this tick's single measure() call -- calling
                // measure() again here would draw from the noise RNG and perturb
                // the very run being judged.
                let p = true_pos;
                let horiz = (p[0] * p[0] + p[1] * p[1]).sqrt();

                // THE JUDGED WINDOW ENDS AT FIRST TOUCHDOWN, and that rule is
                // written here before the number it produces was looked at.
                //
                // WHY A WINDOW IS NEEDED AT ALL. Nothing in this harness
                // terminates the flight: the component wraps FlightCore, so
                // there is no failsafe to disarm it (#414). So the run keeps
                // flying a three-rotor airframe for as long as the tick budget
                // lasts, and `peak_horiz` over the whole run is a function of
                // the tick count, not of the control law.
                //
                // WHY TOUCHDOWN IS THE DEFENSIBLE END, and not a duration:
                //   - It is defined by the PHYSICS, not chosen by me. A window
                //     picked to make a number fit is the bar-fitting this
                //     repo keeps catching.
                //   - It is what the NATIVE leg actually measured. There the
                //     FlightSupervisor disarms, and its 0.95-1.00 m came from
                //     the 1.26 s between the loss at 10.000 s and landing at
                //     11.264 s -- not from a full run.
                //   - After touchdown the analytic floor clamps only `p_ned[2]`
                //     and `v_ned[2]`; HORIZONTAL velocity is untouched, so a
                //     still-powered airframe slides on a frictionless plane and
                //     the excursion grows without bound. Measured before this
                //     window existed: 24.91 m, of which 21.65 m was accumulated
                //     AFTER the vehicle had already reached the ground -- and
                //     1.01 m BELOW it, because this harness never enabled the
                //     floor at all.
                // The full-run figure is still printed, labelled, rather than
                // dropped -- it is the measure of the missing failsafe.
                if touchdown_tick.is_none() && p[2] >= 0.0 {
                    touchdown_tick = Some(tick);
                    println!(
                        "  touchdown at tick {tick} ({:.2}s, {:.2}s after the loss) \
                         -- the judged window ends here",
                        tick as f32 * dt,
                        (tick - at) as f32 * dt
                    );
                }
                peak_horiz_full = peak_horiz_full.max(horiz);
                if touchdown_tick.is_none() {
                    airborne_ticks += 1;
                    peak_horiz_after = peak_horiz_after.max(horiz);
                    // TRUE tilt from the plant, not the estimate -- the estimate
                    // is part of what a rotor-out verdict is testing.
                    if let Some(tt) = plant.true_tilt_rad() {
                        peak_true_tilt_after = peak_true_tilt_after.max(tt);
                        saw_true_tilt = true;
                    }
                }
                // ROTOR_TRACE: the excursion's SHAPE, not just its peak. A peak
                // alone cannot distinguish a vehicle that drifts under control
                // from one that has sunk to the floor and is sliding on a
                // frictionless analytic ground plane — and those call for
                // opposite verdicts.
                if rotor_trace && tick % 250 == 0 {
                    println!(
                        "  ROTOR t={:.2} horiz={:.2}m down={:.2}m v=({:.2},{:.2},{:.2}) tilt={:.3}",
                        tick as f32 * dt,
                        (p[0] * p[0] + p[1] * p[1]).sqrt(),
                        p[2],
                        plant.velocity_ned().map(|v| v[0]).unwrap_or(f32::NAN),
                        plant.velocity_ned().map(|v| v[1]).unwrap_or(f32::NAN),
                        plant.velocity_ned().map(|v| v[2]).unwrap_or(f32::NAN),
                        plant.true_tilt_rad().unwrap_or(f32::NAN),
                    );
                }
                // ISOLATION, read through the PUBLISHED seam -- the observer
                // export (@0.11.0) is what makes this judgeable at all. Before
                // the state return there was no way to ask the component which
                // rotor it had isolated, which is why this leg had no verdict.
                if isolated_rotor.is_none() {
                    let st = observer.call_read_state(&mut store)?;
                    if st.failed_rotor != 0xFF {
                        isolated_rotor = Some(st.failed_rotor);
                        println!(
                            "  FDI isolated rotor {} at tick {tick} ({:.2}s)",
                            st.failed_rotor,
                            tick as f32 * dt
                        );
                    }
                }
                let _ = r;
            }
        }
        if let Some((r, at)) = fail_rotor {
            if tick == at {
                plant.fail_rotor(r);
                println!("  !! rotor {r} FAILED at tick {tick} ({:.2}s)", tick as f32 * dt);
            }
        }
        plant.step([m.m1, m.m2, m.m3, m.m4], dt);

        // Mirrors examples/falcon-sitl-gz two-stage pacing exactly.
        if sim_lock {
            // Stage 1 (anti-burst): hold a uniform real-time control period so
            // the inner loop never bursts through buffered IMU samples.
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
            // Stage 2 (anti-stale): if the sim has fallen behind (RTF < 1) the
            // IMU can still be stale after a full period — wait for a fresh
            // sample so we pace to PHYSICS rather than over-driving it.
            // Bounded, so a stalled publisher cannot hang the run.
            loop {
                let now_imu = plant.counters().map(|c| c.0).unwrap_or(last_imu + 1);
                let waited_us = tick_start.elapsed().as_micros() as u64;
                match pace::pace_decision(last_imu, now_imu, waited_us, pace_deadline_us) {
                    // Counted separately. Collapsing these two into one arm made
                    // "pacing: gyro-sync" unfalsifiable: a run that hit the
                    // 8x-period deadline on EVERY tick — i.e. never actually
                    // synced to a fresh gyro sample — printed exactly the same
                    // line as a clean one. A deadline hit means the loop gave up
                    // waiting and ran on a STALE sample, which is the failure
                    // this stage exists to prevent.
                    pace::Pace::Fresh(w) => {
                        pace_fresh += 1;
                        last_imu = w;
                        break;
                    }
                    pace::Pace::Deadline(w) => {
                        pace_deadline += 1;
                        last_imu = w;
                        break;
                    }
                    pace::Pace::Wait => std::thread::sleep(std::time::Duration::from_micros(150)),
                }
            }
        } else if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }

    let (_s, p) = plant.measure(0.0);
    let alt = -p[2];
    let horiz = (p[0] * p[0] + p[1] * p[1]).sqrt();
    println!("final NED  : n={:.3} e={:.3} d={:.3}  (altitude {:.3} m)", p[0], p[1], p[2], alt);
    println!("horizontal : {horiz:.3} m from launch  (peak {peak_horiz_all:.3} m over the run)");
    println!("peak |a_xy|: {peak_lat_accel:.3} m/s^2  (lateral specific force, NOT tilt)");
    println!();

    // TWO SEPARATE VERDICTS, because they have different answers and merging
    // them would hide the second.
    //
    // (1) Does the shipped component close the loop at all? This is what was
    //     never demonstrated before: wasmtime instantiates it with an EMPTY
    //     linker (no WASI), and 4000 ticks of estimate -> position -> attitude
    //     -> rate -> mixer execute inside the component against the same plant
    //     the SITL bench flies.
    if !alt.is_finite() || !horiz.is_finite() {
        bail!("FAIL: diverged to non-finite state — the loop does not close");
    }
    // Wall-vs-sim, printed always: the failure this fixes was INVISIBLE — the
    // run completed, reported a plausible-looking altitude, and nothing said the
    // clock had come apart. A desync should never again be silent.
    // Reported unconditionally: "the field exists" and "the field carried data
    // on every tick" are different claims, and only the second means the FDI
    // inside the component could see anything at all.
    println!("esc telem : {rpm_frames}/{ticks} ticks carried per-rotor RPM");
    let wall = run_start.elapsed().as_secs_f32();
    let scheduled = ticks as f32 * dt;
    println!(
        "wall/sim   : {wall:.2}s wall vs {scheduled:.2}s scheduled (RTF {:.2}, pacing: {})",
        if wall > 0.0 { scheduled / wall } else { 0.0 },
        if sim_lock { "gyro-sync" } else if pace_real_time { "wall-clock" } else { "free-running" }
    );
    if sim_lock {
        // RTF alone cannot distinguish "synced to physics" from "gave up and ran
        // stale"; both look like a slow run. These two numbers can.
        println!(
            "pace       : {pace_fresh} fresh gyro samples, {pace_deadline} deadline hits              ({:.1}% stale)",
            100.0 * pace_deadline as f32 / (pace_fresh + pace_deadline).max(1) as f32
        );
    }
    println!("LOOP CLOSES: {ticks} ticks executed through the Component Model seam.");
    println!(
        "STATE RETURN: {state_reads} read-state calls, seqlock coherent every tick, \
         final published tick {last_tick}, valid mask 0x{:x}.",
        observer.call_read_state(&mut store)?.valid
    );

    if let Some((_, worst, worst_tick)) = differential.as_ref() {
        println!("DIFFERENTIAL: worst per-motor |wasm - native| = {worst:.9} at tick {worst_tick}");
        // BIT-EXACT, no tolerance: both sides run the same f32 code on the same
        // inputs under shared IEEE-754 semantics, so any delta means something
        // structural differs — a different constant, call order, or lost update.
        if *worst != 0.0 {
            bail!("FAIL: wasm and native disagree by {worst:.9} (tick {worst_tick}). The same \
                   Rust reached two ways must compute the same answer.");
        }
        println!("PASS: wasm and native are BIT-IDENTICAL across {ticks} ticks.");
    }

    // (2) Does it HOLD the commanded altitude? The answer depends on the tick
    //     rate, and that dependency is itself the first defect.
    //
    //     DEFECT 1 — the estimator hardcodes its integration step:
    //         wasm/cm/iekf/src/lib.rs:  f.propagate(RImu { gyro, accel }, 0.001);
    //     1 kHz, always. No interface lets a host declare its rate and the
    //     component cannot detect one, so a host that is not exactly 1 kHz gets
    //     a confidently wrong answer with no error:
    //         1000 Hz -> 2.00 m   |err| 0.00     (what it assumes)
    //          400 Hz -> 12.43 m  |err| 10.43
    //          250 Hz -> 29.17 m  |err| 27.17
    //     Pass dt=0.001 to see the component behave as designed; pass anything
    //     else to see the bug.
    //
    //     DEFECT 2 — IMU-only. `ekf.estimate: func(imu) -> vehicle-state` takes
    //     no position fix, baro, mag or heading, while the native FlightBackend
    //     supplies all four. At 1 kHz this survives a quiet plant and fails
    //     hard once the accelerometer is realistically noisy (IMU_NOISE, m/s^2):
    //         0.00 -> 2.00 m      0.01 -> 2.01 m
    //         0.05 -> 2.01 m      0.20 -> -121.46 m   (dead reckoning gone)
    //     So the seam gap costs nothing in a noiseless bench and everything on
    //     a real MEMS IMU. That is why a quiet PASS here is not evidence of
    //     flightworthiness.
    //
    //     A NOTE ON HOW THIS WAS FIRST MIS-DIAGNOSED, kept because the mistake
    //     is instructive: the 29 m divergence was originally attributed to
    //     DEFECT 2 on the strength of reading the WIT. The structural argument
    //     was correct and the attribution was wrong — it was DEFECT 1 all
    //     along, eleven lines into the component being tested. An explanation
    //     that fits the symptom is not the same as the one that caused it;
    //     vary the parameter and watch the number move.
    let want = -down;
    let err = (alt - want).abs();
    // ── ROTOR-OUT VERDICT (#398 wasm half, FV-FALCON-FAULT-005) ──────────
    // Judged ONLY when a rotor was actually killed, and judged on what
    // SWREQ-FALCON-FAULT-P02 claims: "an injected rotor loss is isolated and the
    // body settles upright (no tumble)". NOT on altitude -- a three-rotor quad
    // is rank-deficient, relinquishes yaw and CANNOT hold altitude, so an
    // altitude bar is the wrong oracle. Judging this leg on the hold bar was
    // what made it unjudgeable, and it is the same error the native leg already
    // corrected (#474).
    //
    // Before the @0.11.0 observer export there was no way to ask the component
    // which rotor it had isolated, so this verdict could not exist at all.
    // There is deliberately NO disarm term: the published component wraps
    // FlightCore, not FlightSupervisor, so it has no mode machine to disarm
    // (#414). That absence is stated, not silently dropped.
    if let Some((want_rotor, _)) = fail_rotor {
        println!();
        println!(
            "ROTOR-OUT  : isolated={:?} peak_true_tilt={:.3}rad (measured={}) \
             peak_horiz={:.2}m  [judged window: {} ticks, {:.2}s, ends {}]",
            isolated_rotor, peak_true_tilt_after, saw_true_tilt, peak_horiz_after,
            airborne_ticks, airborne_ticks as f32 * dt,
            match touchdown_tick {
                Some(t) => format!("at touchdown, tick {t}"),
                None => "AT THE TICK BUDGET -- never touched down".into(),
            }
        );
        println!(
            "ROTOR-OUT  : full-run peak_horiz={peak_horiz_full:.2}m -- NOT judged: past \
             touchdown the airframe is still powered (no failsafe, #414) on a floor \
             that clamps only vertical velocity, so this figure measures the missing \
             failsafe and the tick budget, not the control law."
        );
        // AN EMPTY WINDOW MUST FAIL, NOT PASS. If the vehicle is already on the
        // ground when the rotor is killed there is no airborne behaviour to
        // judge, and every bar below would be vacuously met at 0.0 -- the
        // empty-scope-equals-pass shape this repo keeps finding.
        if airborne_ticks < 25 {
            bail!(
                "FAIL: only {airborne_ticks} airborne tick(s) after the loss -- too \
                 short a window to judge an isolate-and-stay-upright claim. Was the \
                 vehicle airborne when the rotor was killed?"
            );
        }
        if !saw_true_tilt {
            bail!(
                "FAIL: the plant reported no TRUE tilt, so an upright-vs-tumbled \
                 verdict cannot be rendered. A verdict that cannot see tilt must \
                 fail rather than assume."
            );
        }
        if isolated_rotor != Some(want_rotor as u8) {
            bail!(
                "FAIL: the FDI did not isolate the dead rotor through the published \
                 seam. wanted Some({want_rotor}), got {isolated_rotor:?}."
            );
        }
        if peak_true_tilt_after >= 0.5 {
            bail!(
                "FAIL: the airframe did not stay upright after the loss. peak true \
                 tilt {peak_true_tilt_after:.3} rad >= 0.5 rad. The native leg holds \
                 0.176-0.220 rad on the same bar."
            );
        }
        if peak_horiz_after >= 10.0 {
            bail!(
                "FAIL: the airframe travelled {peak_horiz_after:.2} m between the loss \
                 and touchdown (bar 10.0 m)."
            );
        }
        println!("ROTOR-OUT  : PASS (isolated, stayed upright, bounded excursion)");
        // A rotor-out run is NOT judged on the hold, so stop here.
        return Ok(());
    }

    println!("HOLD ERROR : commanded {want:.2} m, reached {alt:.2} m  (|err| = {err:.2} m)");
    if err > 0.5 {
        bail!(
            "FAIL: the wasm cascade did not hold altitude. |err| = {err:.2} m > 0.5 m.\n\
             \n\
             This is now a REAL failure. It used to be the expected outcome, and the \
             message here used to say so — the seam could accept no position measurement \
             and no calibration, so the component provably could not hold and the harness \
             existed to demonstrate that. Both gaps are closed (v0.9 `configure` + the \
             aiding fields), and the same component now holds 2.00 m on gz to within \
             0.14 m, bit-identical to native. So if you are reading this, something \
             regressed.\n\
             \n\
             Check, in order: (1) is `configure` being called at all — an unconfigured \
             component keeps mock-plant defaults and will not lift a gz airframe; \
             (2) are `mag-body`/`heading-rad` being offered — without them yaw is \
             unobservable and the vehicle climbs and then falls; (3) has the plant or \
             its hover point changed under the calibration."
        );
    }
    // THE HORIZONTAL HALF, which this harness computed and printed but never
    // judged. ENDURANCE-P01 asks for "within 1.0 m horizontally and 0.5 m
    // vertically"; only the vertical half was barred, so a component could
    // drift away sideways while holding 2.00 m altitude and every rung of the
    // endurance ladder printed PASS. Found 2026-10-07 while recording the
    // ladder's own result — the altitude number was mistaken for the hold.
    //
    // THE BARS AND THE WINDOW ARE THE NATIVE LEG'S, not new ones:
    // `final_horiz < 1.0 && peak_horiz < 2.0` over the whole run
    // (examples/falcon-sitl-gz/src/main.rs, the `None =>` nominal arm). The
    // peak bar is deliberately looser because the window includes the climb-out,
    // where the airframe legitimately swings wide on its way to 2 m.
    if horiz >= 1.0 {
        bail!(
            "FAIL: the wasm cascade held altitude but DRIFTED. horizontal {horiz:.3} m from the setpoint >= 1.0 m (peak {peak_horiz_all:.3} m).\n\
             \n\
             Altitude is NOT the hold. ENDURANCE-P01 wants 1.0 m horizontally \
             and 0.5 m vertically, and #403 is a HORIZONTAL divergence — the \
             native leg holds 0.02 m final / 0.05 m peak for a full hour on the \
             same plant, so this is a seam defect rather than a plant limit. \
             Measured on plant truth, so a drifting estimator cannot hide it."
        );
    }
    if peak_horiz_all >= 2.0 {
        bail!(
            "FAIL: the wasm cascade ended near the setpoint ({horiz:.3} m) but EXCURSIONED to {peak_horiz_all:.3} m during the run (bar 2.0 m). A \
             hold that wanders and comes back is not a hold; the final-value \
             check alone would have passed this."
        );
    }
    println!("PASS: closed the loop, held the commanded altitude AND the horizontal hold.");
    Ok(())
}
