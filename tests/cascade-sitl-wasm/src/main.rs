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
            export pulseengine:falcon-cascade/controller@0.9.0;
        }
    "#,
    path: "../../wit/falcon-cascade",
});

use pulseengine::falcon_cascade::types::{
    ImuSample as WitImu, SensorFrame, Vec3, VehicleConfig, Waypoint,
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

    let mut peak_tilt = 0.0f32;
    for tick in 0..ticks {
        let tick_start = std::time::Instant::now();
        let (s, true_pos) = plant.measure(noise);
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
        let frame = SensorFrame {
            imu,
            dt_s: dt,
            position_ned,
            mag_body,
            heading_rad,
        };
        let m = controller.call_step(&mut store, frame, target)?;

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
                dt,
            );
            let dmax = [
                (n[0] - m.m1).abs(), (n[1] - m.m2).abs(),
                (n[2] - m.m3).abs(), (n[3] - m.m4).abs(),
            ].iter().copied().fold(0.0f32, f32::max);
            if dmax > *worst { *worst = dmax; *worst_tick = tick; }
        }
        let tilt = (s.accel_body[0].powi(2) + s.accel_body[1].powi(2)).sqrt();
        peak_tilt = peak_tilt.max(tilt);
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
                    pace::Pace::Fresh(w) | pace::Pace::Deadline(w) => {
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
    println!("horizontal : {horiz:.3} m from launch");
    println!("peak |a_xy|: {peak_tilt:.3} m/s^2");
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
    let wall = run_start.elapsed().as_secs_f32();
    let scheduled = ticks as f32 * dt;
    println!(
        "wall/sim   : {wall:.2}s wall vs {scheduled:.2}s scheduled (RTF {:.2}, pacing: {})",
        if wall > 0.0 { scheduled / wall } else { 0.0 },
        if sim_lock { "gyro-sync" } else if pace_real_time { "wall-clock" } else { "free-running" }
    );
    println!("LOOP CLOSES: {ticks} ticks executed through the Component Model seam.");

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
    println!("PASS: closed the loop AND held the commanded altitude.");
    Ok(())
}
