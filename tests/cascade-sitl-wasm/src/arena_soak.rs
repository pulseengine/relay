//! Arena soak tests for the built leaf components (#574).
//!
//! `position#tick` (18 flat params) and `attitude#tick` (19) exceed the
//! canonical ABI's MAX_FLAT_PARAMS of 16, so every call makes the host
//! `cabi_realloc` a 72 B / 76 B argument area in the component's 8 KiB bump
//! arena. Before #574 neither export reclaimed it, so the arena ran out after
//! 8192 / 72 = 113 calls.
//!
//! WHY TWO LEVELS. Through the Component Model API the exhaustion is
//! INVISIBLE: the old `cabi_realloc` returned null on exhaustion, wasmtime
//! accepts 0 as an aligned in-bounds address, and lowers the arguments at the
//! bottom of the shadow stack. The outputs stay plausible. So:
//!
//! * the CORE-LEVEL tests emulate the host exactly as the canonical ABI does
//!   it — `cabi_realloc(0, 0, 4, N)`, write the arguments at the returned
//!   pointer, call the core export with it — and assert the pointer the host
//!   would be handed is never null and never moves. This is the oracle that
//!   fails on a component without reclamation;
//! * the COMPONENT-LEVEL tests drive the same exports through wasmtime's
//!   component API for 10 000 calls, and, given a reference build, assert the
//!   outputs are bit-identical to it call for call. iekf and falcon-mixer pass
//!   their parameters flat (6 and 4) and never spilled, but they open the
//!   same scope now, so they are run the same way to show that changed
//!   nothing.
//!
//! All are `#[ignore]`: they need components built by
//! `scripts/build-components.sh` (or the same `cargo component build` flags).
//!
//!   FALCON_POSITION_WASM=.. FALCON_ATTITUDE_WASM=.. FALCON_RATE_WASM=.. \
//!   FALCON_IEKF_WASM=.. FALCON_MIXER_WASM=.. \
//!     cargo test --release -- --ignored arena_soak
//!
//! Optional `FALCON_<STAGE>_REF_WASM` name a reference build for the
//! bit-identity comparison. A missing required variable FAILS the test rather
//! than skipping it. `legacy_ekf_component_10k_calls` (FALCON_EKF_WASM) covers
//! the unpublished Mahony filter the same way.

use wasmtime::component::{Component, Linker};
use wasmtime::{Engine, Instance, Module, Store};

const CALLS: u32 = 10_000;

fn wasm_path(var: &str) -> String {
    std::env::var(var).unwrap_or_else(|_| panic!("{var} must name a built component"))
}

/// The first core module embedded in a component's encoding (section id 1).
fn core_module(component: &[u8]) -> &[u8] {
    assert_eq!(&component[..8], b"\0asm\x0d\x00\x01\x00", "not a component");
    let mut i = 8;
    while i < component.len() {
        let id = component[i];
        i += 1;
        let (mut size, mut shift) = (0usize, 0);
        loop {
            let b = component[i];
            i += 1;
            size |= ((b & 0x7f) as usize) << shift;
            shift += 7;
            if b & 0x80 == 0 {
                break;
            }
        }
        if id == 1 {
            return &component[i..i + size];
        }
        i += size;
    }
    panic!("no core module in component");
}

/// Emulate the host for `calls` calls of a core export taking a spilled
/// argument area of `n_args` f32s. Returns the outputs (`n_out` f32s read from
/// the returned pointer) of the last call.
fn core_soak(var: &str, export_suffix: &str, n_args: usize, n_out: usize) -> Vec<f32> {
    let bytes = std::fs::read(wasm_path(var)).unwrap();
    let engine = Engine::default();
    let module = Module::new(&engine, core_module(&bytes)).unwrap();
    assert_eq!(
        module.imports().len(),
        0,
        "leaf core module should import nothing"
    );
    let mut store = Store::new(&engine, ());
    let inst = Instance::new(&mut store, &module, &[]).unwrap();
    let memory = inst.get_memory(&mut store, "memory").unwrap();
    let realloc = inst
        .get_typed_func::<(i32, i32, i32, i32), i32>(&mut store, "cabi_realloc")
        .unwrap();
    let name = module
        .exports()
        .map(|e| e.name().to_string())
        .find(|n| n.ends_with(export_suffix))
        .unwrap_or_else(|| panic!("no export ending {export_suffix}"));
    let tick = inst.get_typed_func::<i32, i32>(&mut store, &name).unwrap();

    let len = (n_args * 4) as i32;
    let mut first: Option<i32> = None;
    let (mut moved_at, mut null_at): (Option<u32>, Option<u32>) = (None, None);
    let mut out = vec![0.0f32; n_out];
    for call in 1..=CALLS {
        let p = realloc
            .call(&mut store, (0, 0, 4, len))
            .unwrap_or_else(|e| panic!("call {call}: cabi_realloc trapped: {e}"));
        // Record rather than stop at the first anomaly, so a failure reports
        // both how fast the area leaks and when the arena runs out. (Carrying
        // on after a NULL reproduces the old host behaviour: lower at 0.)
        if p != *first.get_or_insert(p) && moved_at.is_none() {
            moved_at = Some(call);
        }
        if p == 0 && null_at.is_none() {
            null_at = Some(call);
        }
        // A level vehicle at hover, drifting slowly so the controller state
        // evolves; quaternion first, as in vehicle-state.
        let mut args = vec![0.0f32; n_args];
        args[0] = 1.0;
        args[6] = -2.0 + call as f32 * 1e-4;
        args[n_args - 1] = 0.5;
        let raw: Vec<u8> = args.iter().flat_map(|v| v.to_le_bytes()).collect();
        memory.write(&mut store, p as usize, &raw).unwrap();
        let r = tick
            .call(&mut store, p)
            .unwrap_or_else(|e| panic!("call {call}: export trapped: {e}"));
        let mut buf = vec![0u8; n_out * 4];
        memory.read(&store, r as usize, &mut buf).unwrap();
        for (o, c) in out.iter_mut().zip(buf.chunks_exact(4)) {
            *o = f32::from_le_bytes(c.try_into().unwrap());
        }
        assert!(
            out.iter().all(|v| v.is_finite()),
            "call {call}: non-finite output"
        );
    }
    assert!(
        moved_at.is_none() && null_at.is_none(),
        "{var}: the {len} B argument area is not reclaimed — it first moved at call \
         {moved_at:?}, and cabi_realloc returned NULL (arena exhausted) at call {null_at:?}"
    );
    println!(
        "{var}: {CALLS} calls, argument area pinned at {:#x}",
        first.unwrap()
    );
    out
}

#[test]
#[ignore = "needs a built falcon-position component (FALCON_POSITION_WASM)"]
fn arena_soak_position_core_never_exhausts() {
    // vehicle-state (14) + waypoint (4) = 18 flat -> 72 B; attitude-setpoint 5 out.
    core_soak("FALCON_POSITION_WASM", "/position@0.12.0#tick", 18, 5);
}

#[test]
#[ignore = "needs a built falcon-attitude component (FALCON_ATTITUDE_WASM)"]
fn arena_soak_attitude_core_never_exhausts() {
    // vehicle-state (14) + attitude-setpoint (5) = 19 flat -> 76 B; rate-setpoint 4 out.
    core_soak("FALCON_ATTITUDE_WASM", "/attitude@0.12.0#tick", 19, 4);
}

#[test]
#[ignore = "needs a built falcon-rate component (FALCON_RATE_WASM)"]
fn arena_soak_rate_core_never_exhausts() {
    // Regression: rate reclaimed before #574 (reset-at-entry) and must still.
    core_soak("FALCON_RATE_WASM", "/rate@0.12.0#tick", 18, 4);
}

mod pos {
    wasmtime::component::bindgen!({ world: "position-component", path: "../../wit/falcon-cascade" });
}
mod att {
    wasmtime::component::bindgen!({ world: "attitude-component", path: "../../wit/falcon-cascade" });
}
mod est {
    wasmtime::component::bindgen!({ world: "ekf-component", path: "../../wit/falcon-cascade" });
}
mod mix {
    wasmtime::component::bindgen!({ world: "mixer-component", path: "../../wit/falcon-cascade" });
}

fn instantiate(engine: &Engine, path: &str) -> (Store<()>, Component) {
    let c = Component::from_file(engine, path).unwrap_or_else(|e| panic!("loading {path}: {e}"));
    (Store::new(engine, ()), c)
}

fn bits(v: &[f32]) -> Vec<u32> {
    v.iter().map(|x| x.to_bits()).collect()
}

/// Drive `position#tick` through the component API; if a reference build is
/// given, compare every call's output bit for bit.
#[test]
#[ignore = "needs a built falcon-position component (FALCON_POSITION_WASM)"]
fn arena_soak_position_component_10k_calls() {
    use pos::pulseengine::falcon_cascade::types::{VehicleState, Waypoint};
    let engine = Engine::default();
    let linker: Linker<()> = Linker::new(&engine);
    let run = |path: &str| -> Vec<Vec<u32>> {
        let (mut store, c) = instantiate(&engine, path);
        let b = pos::PositionComponent::instantiate(&mut store, &c, &linker).unwrap();
        let p = b.pulseengine_falcon_cascade_position();
        (0..CALLS)
            .map(|i| {
                let st = VehicleState {
                    qw: 1.0,
                    qx: 0.0,
                    qy: 0.0,
                    qz: 0.0,
                    pos_n: 0.01 * i as f32,
                    pos_e: 0.0,
                    pos_d: -1.5,
                    vel_n: 0.0,
                    vel_e: 0.0,
                    vel_d: 0.0,
                    wx: 0.0,
                    wy: 0.0,
                    wz: 0.0,
                    innovation: 0.0,
                };
                let wp = Waypoint {
                    north: 5.0,
                    east: -1.0,
                    down: -2.0,
                    yaw: 0.3,
                };
                let a = p
                    .call_tick(&mut store, st, wp)
                    .unwrap_or_else(|e| panic!("call {i}: {e}"));
                bits(&[a.qw, a.qx, a.qy, a.qz, a.thrust])
            })
            .collect()
    };
    let got = run(&wasm_path("FALCON_POSITION_WASM"));
    assert_eq!(got.len(), CALLS as usize);
    if let Ok(r) = std::env::var("FALCON_POSITION_REF_WASM") {
        let want = run(&r);
        let first = got.iter().zip(&want).position(|(a, b)| a != b);
        assert_eq!(first, None, "position output diverged from the reference");
        println!("position: {CALLS} calls BIT-IDENTICAL to {r}");
    }
}

#[test]
#[ignore = "needs a built falcon-attitude component (FALCON_ATTITUDE_WASM)"]
fn arena_soak_attitude_component_10k_calls() {
    use att::pulseengine::falcon_cascade::types::{AttitudeSetpoint, VehicleState};
    let engine = Engine::default();
    let linker: Linker<()> = Linker::new(&engine);
    let run = |path: &str| -> Vec<Vec<u32>> {
        let (mut store, c) = instantiate(&engine, path);
        let b = att::AttitudeComponent::instantiate(&mut store, &c, &linker).unwrap();
        let a = b.pulseengine_falcon_cascade_attitude();
        (0..CALLS)
            .map(|i| {
                let th = 0.2 * ((i % 400) as f32 / 400.0);
                let st = VehicleState {
                    qw: (th / 2.0).cos(),
                    qx: (th / 2.0).sin(),
                    qy: 0.0,
                    qz: 0.0,
                    pos_n: 0.0,
                    pos_e: 0.0,
                    pos_d: -2.0,
                    vel_n: 0.0,
                    vel_e: 0.0,
                    vel_d: 0.0,
                    wx: 0.1,
                    wy: 0.0,
                    wz: 0.0,
                    innovation: 0.0,
                };
                let sp = AttitudeSetpoint {
                    qw: 1.0,
                    qx: 0.0,
                    qy: 0.0,
                    qz: 0.0,
                    thrust: 0.5,
                };
                let r = a
                    .call_tick(&mut store, st, sp)
                    .unwrap_or_else(|e| panic!("call {i}: {e}"));
                bits(&[r.rx, r.ry, r.rz, r.thrust])
            })
            .collect()
    };
    let got = run(&wasm_path("FALCON_ATTITUDE_WASM"));
    assert_eq!(got.len(), CALLS as usize);
    if let Ok(r) = std::env::var("FALCON_ATTITUDE_REF_WASM") {
        let want = run(&r);
        let first = got.iter().zip(&want).position(|(a, b)| a != b);
        assert_eq!(first, None, "attitude output diverged from the reference");
        println!("attitude: {CALLS} calls BIT-IDENTICAL to {r}");
    }
}

/// Run `CALLS` of one export on a fresh instance, collecting each output's
/// bits; then, if `ref_var` names a reference build, compare call for call.
fn compare_to_reference(var: &str, ref_var: &str, what: &str, run: impl Fn(&str) -> Vec<Vec<u32>>) {
    let got = run(&wasm_path(var));
    assert_eq!(got.len(), CALLS as usize);
    if let Ok(r) = std::env::var(ref_var) {
        let want = run(&r);
        let first = got.iter().zip(&want).position(|(a, b)| a != b);
        assert_eq!(first, None, "{what} output diverged from the reference");
        println!("{what}: {CALLS} calls BIT-IDENTICAL to {r}");
    }
}

fn estimator_soak(var: &str, ref_var: &str, what: &str) {
    use est::pulseengine::falcon_cascade::types::ImuSample;
    let engine = Engine::default();
    let linker: Linker<()> = Linker::new(&engine);
    compare_to_reference(var, ref_var, what, |path| {
        let (mut store, c) = instantiate(&engine, path);
        let b = est::EkfComponent::instantiate(&mut store, &c, &linker).unwrap();
        let e = b.pulseengine_falcon_cascade_ekf();
        (0..CALLS)
            .map(|i| {
                let t = i as f32 * 1e-3;
                let imu = ImuSample {
                    ax: 0.05 * t.sin(),
                    ay: -0.03,
                    az: -9.81,
                    gx: 0.01 * t.cos(),
                    gy: 0.0,
                    gz: 0.002,
                };
                let v = e
                    .call_estimate(&mut store, imu)
                    .unwrap_or_else(|err| panic!("call {i}: {err}"));
                bits(&[
                    v.qw,
                    v.qx,
                    v.qy,
                    v.qz,
                    v.pos_n,
                    v.pos_e,
                    v.pos_d,
                    v.vel_n,
                    v.vel_e,
                    v.vel_d,
                    v.wx,
                    v.wy,
                    v.wz,
                    v.innovation,
                ])
            })
            .collect()
    });
}

#[test]
#[ignore = "needs a built falcon-iekf component (FALCON_IEKF_WASM)"]
fn arena_soak_iekf_component_10k_calls() {
    estimator_soak("FALCON_IEKF_WASM", "FALCON_IEKF_REF_WASM", "iekf");
}

#[test]
#[ignore = "needs a built (unpublished) falcon-ekf component (FALCON_EKF_WASM)"]
fn legacy_ekf_component_10k_calls() {
    estimator_soak("FALCON_EKF_WASM", "FALCON_EKF_REF_WASM", "ekf");
}

#[test]
#[ignore = "needs a built falcon-mixer component (FALCON_MIXER_WASM)"]
fn arena_soak_mixer_component_10k_calls() {
    use mix::pulseengine::falcon_cascade::types::TorqueSetpoint;
    let engine = Engine::default();
    let linker: Linker<()> = Linker::new(&engine);
    compare_to_reference(
        "FALCON_MIXER_WASM",
        "FALCON_MIXER_REF_WASM",
        "mixer",
        |path| {
            let (mut store, c) = instantiate(&engine, path);
            let b = mix::MixerComponent::instantiate(&mut store, &c, &linker).unwrap();
            let m = b.pulseengine_falcon_cascade_mixer();
            (0..CALLS)
                .map(|i| {
                    let k = (i % 200) as f32 / 200.0 - 0.5;
                    let tq = TorqueSetpoint {
                        tx: 0.02 * k,
                        ty: -0.01 * k,
                        tz: 0.005,
                        thrust: 0.5 + 0.2 * k,
                    };
                    let o = m
                        .call_mix(&mut store, tq)
                        .unwrap_or_else(|err| panic!("call {i}: {err}"));
                    bits(&[o.m1, o.m2, o.m3, o.m4])
                })
                .collect()
        },
    );
}
