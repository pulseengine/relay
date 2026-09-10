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
            export pulseengine:falcon-cascade/controller@0.7.0;
        }
    "#,
    path: "../../wit/falcon-cascade",
});

use pulseengine::falcon_cascade::types::{ImuSample as WitImu, Waypoint};

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
    let mut plant = MockPhysics::at_rest();

    println!("=== wasm cascade in the SITL loop ===");
    println!("component : {wasm}");
    println!("plant     : MockPhysics (the same module falcon-sitl-gz flies)");
    println!("target    : hold N=0 E=0 D={down} m, yaw 0");
    println!("schedule  : {ticks} ticks @ dt={dt}s ({:.1}s, {:.0} Hz)", ticks as f32 * dt, 1.0 / dt);
    println!();

    let mut peak_tilt = 0.0f32;
    for _ in 0..ticks {
        let (s, _true_pos) = plant.measure(0.0);
        let imu = WitImu {
            ax: s.accel_body[0], ay: s.accel_body[1], az: s.accel_body[2],
            gx: s.gyro_body[0],  gy: s.gyro_body[1],  gz: s.gyro_body[2],
        };
        // The tick that matters: one full estimate -> position -> attitude ->
        // rate -> mixer pass, executed inside the wasm component.
        let m = controller.call_step(&mut store, imu, target)?;
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

    // (2) Does it HOLD the commanded altitude? It does not, and the reason is
    //     structural rather than a tuning problem, so this is reported as a
    //     finding instead of being quietly tolerated by a loose threshold.
    //
    //     The shipped WIT seam is `ekf.estimate: func(imu: imu-sample) ->
    //     vehicle-state`. IMU ONLY. There is no interface on the published
    //     cascade that accepts a position fix, a barometer, a magnetometer or a
    //     heading — while the native `FlightBackend` this bench normally flies
    //     supplies read_position, read_mag, read_heading and read_motor_rpm.
    //
    //     So altitude is unobservable across this seam: the estimator can only
    //     dead-reckon the accelerometer, and the altitude loop chases an
    //     estimate that drifts. Commanding a deeper target climbs FURTHER
    //     (-2 m -> 12.4 m, -5 m -> 29.9 m), which is the signature of a loop
    //     responding to its setpoint while its feedback diverges — not of a
    //     controller ignoring the command.
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
