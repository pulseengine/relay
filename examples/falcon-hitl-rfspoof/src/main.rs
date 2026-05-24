//! Falcon v0.11 — HITL harness CLI.
//!
//! ```text
//!   $ falcon-hitl-rfspoof --backend=stub --duration=5
//!   $ falcon-hitl-rfspoof --backend=hackrf --iq=/tmp/spoof.iq --duration=30
//! ```
//!
//! The harness binary is intentionally tiny: it picks a backend,
//! constructs the geofence + relay-sc state, and runs the scenario.
//! See `harness.rs` for the driver and `stub.rs` / `hackrf.rs` for
//! backends.

mod hackrf;
mod harness;
mod stub;

use harness::{load_rtl_rts, run_scenario};
use relay_lc::engine::Geofence;
use relay_sc::engine::CommandStore;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let backend = arg(&args, "--backend").unwrap_or_else(|| "stub".into());
    let duration_s: f32 = arg(&args, "--duration").and_then(|s| s.parse().ok()).unwrap_or(5.0);

    println!("falcon-hitl-rfspoof: backend={backend} duration={duration_s}s");
    println!("  fence: ±100 m × ±100 m × ±100 m (NED, centred on home)");

    // 100 m fence centred on origin.
    let mut fence = Geofence::new(-10_000, 10_000, -10_000, 10_000, -10_000, 10_000);
    let mut sc = CommandStore::new();
    load_rtl_rts(&mut sc, 0, 0xA17C);

    let verdict = match backend.as_str() {
        "stub" => {
            let mut b = stub::StubBench::new(0, 0, -500, 0, 20_000, -500, 2.0);
            run_scenario(&mut b, &mut fence, &mut sc, 0.01, duration_s, 0, 1.0)
        }
        "hackrf" => {
            // 200 m east of the fence boundary — well outside.
            let mut b = hackrf::HackRfBench::new(2.0, 0, 0, -500, 0, 20_000, -500);
            run_scenario(&mut b, &mut fence, &mut sc, 0.01, duration_s, 0, 1.0)
        }
        other => {
            eprintln!("unknown backend: {other}  (expected: stub | hackrf)");
            std::process::exit(2);
        }
    };

    println!("verdict = {verdict:#?}");
    if verdict.pass() {
        println!("PASS");
    } else {
        println!("FAIL");
        std::process::exit(1);
    }
}

fn arg(args: &[String], key: &str) -> Option<String> {
    let prefix = format!("{key}=");
    for a in args {
        if let Some(v) = a.strip_prefix(&prefix) {
            return Some(v.into());
        }
    }
    None
}
