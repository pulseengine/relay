//! Equivalence driver for the falcon-rate-stream 0.3 async component.
//!
//! wasm32-wasip3: feeds N rate-inputs into the imported `monitor` stream,
//! collects the torque stream and prints one line per output.
//! native: computes the same lines from relay-rate's RatePid directly.
//!
//! Exit codes (fleet rule): 0 = N outputs produced, 1 = wrong output count,
//! 2 = could not run. Equality is judged by diffing the two outputs.

const N: u32 = 200;

/// Deterministic inputs: body rates and rate setpoints on a coarse grid.
fn input(i: u32) -> ([f32; 3], [f32; 3], f32) {
    let g = |k: u32, m: u32| ((i * k) % m) as f32 / m as f32 - 0.5;
    (
        [g(37, 101), g(53, 97), g(29, 89)],
        [g(17, 83), g(41, 79), g(23, 73)],
        0.25 + 0.5 * (i % 7) as f32 / 7.0,
    )
}

fn line(i: u32, t: [f32; 4]) -> String {
    format!(
        "{i} {:08x} {:08x} {:08x} {:08x}",
        t[0].to_bits(),
        t[1].to_bits(),
        t[2].to_bits(),
        t[3].to_bits()
    )
}

#[cfg(not(target_family = "wasm"))]
fn main() {
    use relay_rate::{RatePid, Timestamp};
    let mut pid = RatePid::new();
    for i in 0..N {
        // Same timestamp sequence as stream.rs: 1 ms per input from 0.
        let ms = i as u64;
        let ts = Timestamp {
            seconds: ms / 1000,
            fraction: ((ms % 1000) * (1u64 << 32) / 1000) as u32,
        };
        let (w, sp, thrust) = input(i);
        let t = pid.tick(ts, w, sp);
        println!("{}", line(i, [t[0], t[1], t[2], thrust]));
    }
}

#[cfg(target_family = "wasm")]
mod bindings {
    wit_bindgen::generate!({
        inline: "package relay:p3-driver; world driver { import pulseengine:falcon-cascade-stream/rate-stream@0.1.0; }",
        path: "../../../wit/falcon-cascade-stream",
        world: "relay:p3-driver/driver",
        generate_all,
    });
}

#[cfg(target_family = "wasm")]
fn main() {
    use bindings::pulseengine::falcon_cascade_stream::rate_stream::{monitor, RateInput};
    use bindings::pulseengine::falcon_cascade_stream::types::{RateSetpoint, VehicleState};

    let inputs: Vec<RateInput> = (0..N)
        .map(|i| {
            let (w, sp, thrust) = input(i);
            RateInput {
                state: VehicleState {
                    qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0,
                    pos_n: 0.0, pos_e: 0.0, pos_d: 0.0,
                    vel_n: 0.0, vel_e: 0.0, vel_d: 0.0,
                    wx: w[0], wy: w[1], wz: w[2],
                    innovation: 0.0,
                },
                sp: RateSetpoint { rx: sp[0], ry: sp[1], rz: sp[2], thrust },
            }
        })
        .collect();

    let outs = wit_bindgen::block_on(async move {
        let (mut tx, rx) = bindings::wit_stream::new::<RateInput>();
        wit_bindgen::spawn_local(async move {
            let rest = tx.write_all(inputs).await;
            if !rest.is_empty() {
                eprintln!("p3-rate-driver: component closed its input after {} unread", rest.len());
            }
        });
        monitor(rx).await.collect().await
    });

    for (i, t) in outs.iter().enumerate() {
        println!("{}", line(i as u32, [t.tx, t.ty, t.tz, t.thrust]));
    }
    if outs.len() != N as usize {
        eprintln!("p3-rate-driver: {} outputs for {} inputs", outs.len(), N);
        std::process::exit(1);
    }
}
