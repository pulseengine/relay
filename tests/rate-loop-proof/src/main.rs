//! Closed-loop proof THROUGH the WASI-free falcon-rate wasm component.
//!
//! Loads the component, then closes the control loop across the real WIT seam:
//! plant (omega_dot = torque / inertia) -> gyro into vehicle-state -> the
//! component's `rate.tick` -> torque -> plant. Asserts the body rate tracks a
//! 1 rad/s step. No WASI: the component imports only pulseengine:falcon-cascade/types.

use anyhow::{bail, Result};
use wasmtime::component::{Component, Linker};
use wasmtime::{Config, Engine, Store};

wasmtime::component::bindgen!({
    world: "rate-component",
    path: "../../wit/falcon-cascade/cascade.wit",
});

use pulseengine::falcon_cascade::types::{RateSetpoint, VehicleState};

fn main() -> Result<()> {
    let wasm = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: rate-loop-proof <falcon_rate_cm.wasm>");
        std::process::exit(2);
    });

    let mut cfg = Config::new();
    cfg.wasm_component_model(true);
    let engine = Engine::new(&cfg)?;
    let component = Component::from_file(&engine, &wasm)?;
    let linker: Linker<()> = Linker::new(&engine);
    let mut store = Store::new(&engine, ());
    let bindings = RateComponent::instantiate(&mut store, &component, &linker)?;
    let rate = bindings.pulseengine_falcon_cascade_rate();

    // Plant: 5 g·m² roll inertia (500 g, 10-inch quad), no friction.
    let inertia = 0.005_f32;
    let dt = 1.0 / 1000.0;
    let mut omega = [0.0_f32; 3];
    let target = 1.0_f32; // rad/s step about x

    let mut converged_at: Option<f32> = None;
    for step in 0..3000 {
        let state = VehicleState {
            qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
            pos_n: 0.0, pos_e: 0.0, pos_d: 0.0,
            vel_n: 0.0, vel_e: 0.0, vel_d: 0.0,
            wx: omega[0], wy: omega[1], wz: omega[2],
            innovation: 0.0,
        };
        let sp = RateSetpoint { rx: target, ry: 0.0, rz: 0.0, thrust: 0.5 };
        let torque = rate.call_tick(&mut store, state, sp)?; // <-- across the wasm seam
        omega[0] += torque.tx / inertia * dt;
        omega[1] += torque.ty / inertia * dt;
        omega[2] += torque.tz / inertia * dt;
        if (omega[0] - target).abs() < 0.01 && converged_at.is_none() {
            converged_at = Some(step as f32 * dt);
        }
    }

    let err = (omega[0] - target).abs();
    println!("through-wasm closed loop: omega_x -> {:.4} rad/s (target {:.1}), |err| = {:.4}", omega[0], target, err);
    match converged_at {
        Some(t) if t < 2.0 && err < 0.01 => {
            println!("PASS: converged at {:.3}s, steady-state |err| {:.4} < 0.01 — loop closes through the component", t, err);
            Ok(())
        }
        _ => bail!("FAIL: did not close the loop (converged_at={:?}, err={:.4})", converged_at, err),
    }
}
