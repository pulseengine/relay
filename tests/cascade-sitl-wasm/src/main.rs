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

use anyhow::{bail, Context, Result};
use physics::{MockPhysics, Physics};
use wasmtime::component::{Component, Linker};
use wasmtime::{Config, Engine, Store};

// The composed component's world is `import types; export controller` — it does
// NOT import the five stage interfaces, because wac already satisfied them. The
// repo's `cascade` world still declares those imports, so it cannot describe
// this artifact. Declaring the host's own view inline keeps the generated WIT
// untouched (it is spar-derived; hand-editing it would trip the drift gate).
wasmtime::component::bindgen!({
    inline: r#"
        package host:sitl;
        world composed-cascade {
            export pulseengine:falcon-cascade/controller@0.8.0;
        }
    "#,
    path: "../../wit/falcon-cascade",
});

use pulseengine::falcon_cascade::types::{ImuSample as WitImu, SensorFrame, Vec3, Waypoint};

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
    let mut plant = MockPhysics::at_rest();

    println!("=== wasm cascade in the SITL loop ===");
    println!("component : {wasm}");
    println!("plant     : MockPhysics (the same module falcon-sitl-gz flies)");
    println!("target    : hold N=0 E=0 D={down} m, yaw 0");
    println!("schedule  : {ticks} ticks @ dt={dt}s ({:.1}s, {:.0} Hz)", ticks as f32 * dt, 1.0 / dt);
    println!("imu noise : {noise} m/s^2");
    println!();

    // GNSS divisor: a fix every `gnss_div` ticks. 5 Hz at any rate, matching
    // the native bench's `SitlBackend::new(physics, dt, noise, 50)` at 250 Hz.
    // Set GNSS_DIV=0 to withhold position entirely and reproduce the v0.7
    // IMU-only behaviour through the v0.8 interface.
    let gnss_div: u32 = std::env::var("GNSS_DIV").ok().and_then(|v| v.parse().ok())
        .unwrap_or_else(|| ((1.0 / dt) / 5.0).round().max(1.0) as u32);

    let mut peak_tilt = 0.0f32;
    let mut fixes = 0u32;
    for tick in 0..ticks {
        let (s, true_pos) = plant.measure(noise);
        let imu = WitImu {
            ax: s.accel_body[0], ay: s.accel_body[1], az: s.accel_body[2],
            gx: s.gyro_body[0],  gy: s.gyro_body[1],  gz: s.gyro_body[2],
        };
        // v0.8: the host now states its own period and offers what it has.
        // MockPhysics has no magnetometer and no heading reference (the gz
        // bridge supplies both), so those stay `none` here — which is the
        // honest frame for this plant rather than a synthesised one.
        let position_ned = if gnss_div > 0 && tick % gnss_div == 0 {
            fixes += 1;
            Some(Vec3 { x: true_pos[0], y: true_pos[1], z: true_pos[2] })
        } else {
            None
        };
        let frame = SensorFrame {
            imu,
            dt_s: dt,
            position_ned,
            mag_body: None,
            heading_rad: None,
        };
        // The tick that matters: one full estimate -> position -> attitude ->
        // rate -> mixer pass, executed inside the wasm component.
        let m = controller.call_step(&mut store, frame, target)?;
        let tilt = (s.accel_body[0].powi(2) + s.accel_body[1].powi(2)).sqrt();
        peak_tilt = peak_tilt.max(tilt);
        plant.step([m.m1, m.m2, m.m3, m.m4], dt);
    }

    let (_s, p) = plant.measure(0.0);
    let alt = -p[2];
    let horiz = (p[0] * p[0] + p[1] * p[1]).sqrt();
    println!("final NED  : n={:.3} e={:.3} d={:.3}  (altitude {:.3} m)", p[0], p[1], p[2], alt);
    println!("horizontal : {horiz:.3} m from launch");
    println!("peak |a_xy|: {peak_tilt:.3} m/s^2");
    println!("gnss fixes : {fixes} (every {gnss_div} ticks = {:.1} Hz)", if gnss_div > 0 { 1.0 / (dt * gnss_div as f32) } else { 0.0 });
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
    println!("LOOP CLOSES: {ticks} ticks executed through the Component Model seam.");

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
            "FAIL (expected, and the point of this harness): the published wasm cascade \
             cannot hold altitude. |err| = {err:.2} m > 0.5 m. The seam accepts no position \
             measurement — see the note above. This is an interface gap, not a gain-tuning \
             problem, and no amount of retuning fixes it from outside the component."
        );
    }
    println!("PASS: closed the loop AND held the commanded altitude.");
    Ok(())
}
