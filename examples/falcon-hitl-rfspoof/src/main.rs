//! Falcon HITL harness CLI.
//!
//! ```text
//!   $ falcon-hitl-rfspoof --backend=stub --duration=5
//!   $ falcon-hitl-rfspoof --backend=hackrf --iq=/tmp/spoof.iq --duration=30
//!   $ falcon-hitl-rfspoof --preset=px4-sitl   # convenience preset (v0.14)
//! ```
//!
//! `--preset=` short-circuits the per-flag wiring for canonical
//! benches. `px4-sitl` → mavlink backend, UDP :14550, PX4 stock
//! home coord (Zürich / ETH). Override individual fields by passing
//! `--backend=`, `--listen=`, `--home=`, `--duration=` after the
//! preset.
//!
//! The harness binary is intentionally tiny: it picks a backend,
//! constructs the geofence + relay-sc state, and runs the scenario.
//! See `harness.rs` for the driver and `stub.rs` / `hackrf.rs` /
//! `mavlink.rs` for backends.

mod hackrf;
mod harness;
mod stub;
pub mod mavlink;

use harness::{load_rtl_rts, run_scenario};
use relay_lc::engine::Geofence;
use relay_sc::engine::CommandStore;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    // Presets fill in default flag values; explicit --foo= after the
    // preset still wins (presets only set defaults).
    let preset = arg(&args, "--preset");
    let defaults = match preset.as_deref() {
        Some("px4-sitl") => Defaults {
            backend: "mavlink",
            duration_s: 30.0,
            listen: Some("0.0.0.0:14550"),
            home: Some("47.3977,8.5456,488"),
        },
        Some(other) => {
            eprintln!("unknown preset: {other}  (expected: px4-sitl)");
            std::process::exit(2);
        }
        None => Defaults::EMPTY,
    };

    let backend = arg(&args, "--backend").unwrap_or_else(|| defaults.backend.into());
    let duration_s: f32 = arg(&args, "--duration")
        .and_then(|s| s.parse().ok())
        .unwrap_or(defaults.duration_s);

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
        "mavlink" => {
            // Bind UDP to whatever port the FC sends to (PX4 default 14550).
            let bind_addr = arg(&args, "--listen")
                .or_else(|| defaults.listen.map(String::from))
                .unwrap_or_else(|| "0.0.0.0:14550".into());
            let sock = std::net::UdpSocket::bind(&bind_addr).unwrap_or_else(|e| {
                eprintln!("could not bind {bind_addr}: {e}"); std::process::exit(3);
            });
            sock.set_nonblocking(true).expect("set_nonblocking");
            println!("  mavlink: listening on {bind_addr}");
            // Default home = Budapest centre — override with --home=lat,lon,alt_m.
            let home = match arg(&args, "--home").or_else(|| defaults.home.map(String::from)) {
                Some(s) => parse_home(&s).expect("--home=lat,lon,alt_m"),
                None => mavlink::Home { lat_e7: 475_023_456, lon_e7: 190_401_234, alt_mm: 120_000 },
            };
            let src = mavlink::UdpFrameSource::new(sock);
            let mut b = mavlink::MavlinkBench::new(src, home, (0, 0, -500));
            // Real link: 10 Hz GLOBAL_POSITION_INT rate is typical;
            // 100 Hz harness tick is fine because drain_frames is
            // non-blocking and tolerates "no new frame this tick".
            run_scenario(&mut b, &mut fence, &mut sc, 0.01, duration_s, 0, 5.0)
        }
        other => {
            eprintln!("unknown backend: {other}  (expected: stub | hackrf | mavlink)");
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

/// CLI defaults a preset can fill in. Empty by default; presets override
/// fields they care about; explicit `--foo=` flags still trump everything.
struct Defaults {
    backend: &'static str,
    duration_s: f32,
    listen: Option<&'static str>,
    home: Option<&'static str>,
}

impl Defaults {
    const EMPTY: Self = Self {
        backend: "stub",
        duration_s: 5.0,
        listen: None,
        home: None,
    };
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

fn parse_home(s: &str) -> Option<mavlink::Home> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 { return None; }
    let lat: f64 = parts[0].parse().ok()?;
    let lon: f64 = parts[1].parse().ok()?;
    let alt_m: f64 = parts[2].parse().ok()?;
    Some(mavlink::Home {
        lat_e7: (lat * 1e7) as i32,
        lon_e7: (lon * 1e7) as i32,
        alt_mm: (alt_m * 1000.0) as i32,
    })
}
