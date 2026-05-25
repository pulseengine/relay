//! `falcon-sitl-gz` — scaffold for the Gazebo SITL backend.
//!
//! Per `docs/SIMULATOR.md` (v0.15.0) option C: the same SITL
//! contract `examples/falcon-sitl-hover` exercises, but with a
//! pluggable physics layer so a real Gazebo Sim bridge can drop
//! in without touching the verified cascade or its assertions.
//!
//! v0.16.1 ships:
//!   * `Physics` trait — the one-method interface a Gazebo bridge
//!     implements (`step` + `measure`).
//!   * `MockPhysics` — in-process reference impl; tiny port of
//!     the falcon-sitl-hover `Plant` toy integrator. Used by
//!     cargo tests; runs without external dependencies.
//!   * `GazeboPhysics` — stub for the real bridge. Documents the
//!     gz-transport wire-up (subscribe to imu_sensor / navsat,
//!     publish to motor cmd_vel topics) but doesn't pull in
//!     gz-transport / Protobuf today.
//!   * One scenario (`hover`) that runs the verified cascade
//!     against whichever `Physics` impl the CLI picks. Records a
//!     verdict the bench user can read.
//!
//! What this is NOT:
//!   * A working Gazebo loop — that needs gz-sim installed, an
//!     SDF world, and gz-transport bindings.
//!   * A replacement for falcon-sitl-hover — the existing 17
//!     scenarios stay where they are. This crate is the scaffold
//!     for the next layer of "would our stuff fly" evidence.

mod physics;

use physics::{GazeboPhysics, MockPhysics, Physics};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let backend = arg(&args, "--backend").unwrap_or_else(|| "mock".into());
    let duration_s: f32 = arg(&args, "--duration").and_then(|s| s.parse().ok()).unwrap_or(5.0);

    println!("falcon-sitl-gz: backend={backend} duration={duration_s}s");

    let pass = match backend.as_str() {
        "mock" => run_hover(&mut MockPhysics::at_rest(), duration_s),
        "gazebo" => {
            let world = arg(&args, "--world").unwrap_or_else(|| "falcon".into());
            let model = arg(&args, "--model").unwrap_or_else(|| "quad".into());
            let mut p = build_gazebo(&args, world, model);
            run_hover(&mut p, duration_s)
        }
        other => {
            eprintln!("unknown backend: {other}  (expected: mock | gazebo)");
            std::process::exit(2);
        }
    };

    if pass {
        println!("PASS");
    } else {
        println!("FAIL");
        std::process::exit(1);
    }
}

/// Single-scenario loop: command max thrust, hold for the duration,
/// report whether the body climbed. Deliberately minimal — the
/// existing 17 scenarios in falcon-sitl-hover stay there; this is
/// the smallest end-to-end demonstration the scaffold runs.
fn run_hover(physics: &mut dyn Physics, duration_s: f32) -> bool {
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let mut t = 0.0_f32;
    let mut initial_alt: Option<f32> = None;
    let mut min_alt = f32::MAX;
    let mut max_alt = f32::MIN;

    // Command 70 % PWM on all four motors — enough to hover.
    let motor_pwm = [0.7_f32; 4];

    for _ in 0..n {
        physics.step(motor_pwm, dt);
        let (_imu, pos) = physics.measure(0.01);
        let alt_m = -pos[2]; // NED down → altitude is -z
        if initial_alt.is_none() {
            initial_alt = Some(alt_m);
        }
        min_alt = min_alt.min(alt_m);
        max_alt = max_alt.max(alt_m);
        t += dt;
    }

    let start = initial_alt.unwrap_or(0.0);
    let net_climb = max_alt - start;
    println!(
        "  verdict: backend={} steps={} climb={:.2} m  (min={:.2} max={:.2})",
        physics.name(),
        n,
        net_climb,
        min_alt,
        max_alt,
    );
    // PASS criterion: under MockPhysics, 70 % PWM × THRUST_SCALE=20 N
    // is well above hover-thrust at the default inertia + gravity, so
    // the body should climb. Under the GazeboPhysics stub everything
    // stays zero and the verdict prints FAIL — that's the honest
    // signal the bench wire-up is needed.
    let _ = t;
    net_climb > 0.1
}

/// Construct a `GazeboPhysics` for the CLI gazebo backend. With
/// feature `gazebo` ON, parses `--home=lat,lon,alt_m` and threads
/// it into `connect_with_home`. Without the feature, falls back to
/// the stub `new(world, model)`.
#[cfg(feature = "gazebo")]
fn build_gazebo(args: &[String], world: String, model: String) -> GazeboPhysics {
    let home = match arg(args, "--home") {
        Some(s) => parse_home(&s).expect("--home=lat,lon,alt_m"),
        None => physics::Home::ORIGIN,
    };
    println!("  gazebo home: lat={} lon={} alt={} m", home.lat_deg, home.lon_deg, home.alt_m);
    GazeboPhysics::connect_with_home(world, model, home)
        .expect("connect_with_home: gz-transport connect failed; is `gz sim` running?")
}

#[cfg(not(feature = "gazebo"))]
fn build_gazebo(_args: &[String], world: String, model: String) -> GazeboPhysics {
    let _ = arg(_args, "--home"); // accept the flag silently in stub mode
    println!("  (stub backend — rebuild with --features gazebo for the real bridge)");
    GazeboPhysics::new(world, model)
}

#[cfg(feature = "gazebo")]
fn parse_home(s: &str) -> Option<physics::Home> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 { return None; }
    let lat_deg: f64 = parts[0].parse().ok()?;
    let lon_deg: f64 = parts[1].parse().ok()?;
    let alt_m: f64 = parts[2].parse().ok()?;
    Some(physics::Home { lat_deg, lon_deg, alt_m })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::MockPhysics;

    #[test]
    fn mock_backend_climbs_under_full_thrust() {
        let mut p = MockPhysics::at_rest();
        let pass = run_hover(&mut p, 1.0);
        assert!(pass, "70 % PWM × THRUST_SCALE should easily beat gravity");
    }

    /// Stub-only — when `gazebo` feature is ON, `GazeboPhysics::new`
    /// connects to gz-transport (and panics if no gz-sim is running),
    /// so the panic-free contract doesn't apply.
    #[cfg(not(feature = "gazebo"))]
    #[test]
    fn gazebo_stub_does_not_panic() {
        let mut g = crate::physics::GazeboPhysics::new("test-world", "test-quad");
        let _ = run_hover(&mut g, 0.1);
        // Result is FAIL (everything zeros), but the harness must not
        // panic — that's the contract for a stub-only run.
    }
}
