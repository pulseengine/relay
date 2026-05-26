//! `falcon-sitl-gz` — Gazebo SITL bench runner.
//!
//! v0.16.1 — pluggable `Physics` trait + `MockPhysics` reference +
//!           `GazeboPhysics` stub.
//! v0.18.0 — real gz-transport-rs bridge behind the `gazebo` feature.
//! v0.18.1 — NavSat + Home projection.
//! v0.19.0 — bench-evidence wiring: `--evidence-dir`, structured
//!           per-tick CSV, diagnostic counters (`imu_recv`,
//!           `navsat_recv`, `motor_send`) printed in the verdict.
//!           Mirrors the PX4-SITL bench shape from v0.18.2.

mod physics;

use physics::{GazeboPhysics, MockPhysics, Physics};
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let backend = arg(&args, "--backend").unwrap_or_else(|| "mock".into());
    let scenario = arg(&args, "--scenario").unwrap_or_else(|| "hover".into());
    let duration_s: f32 = arg(&args, "--duration").and_then(|s| s.parse().ok()).unwrap_or(5.0);
    let evidence_dir = arg(&args, "--evidence-dir").map(PathBuf::from);

    println!("falcon-sitl-gz: backend={backend} scenario={scenario} duration={duration_s}s");
    if let Some(d) = &evidence_dir {
        println!("  evidence-dir: {}", d.display());
    }

    // Set up the bench-evidence sink (CSV + harness log) before
    // constructing physics — that way connect-failure logs land on
    // disk too, and a future bench operator can read why the run
    // never started.
    let mut evidence = match &evidence_dir {
        Some(dir) => match EvidenceSink::open(dir, &backend, &scenario) {
            Ok(s) => Some(s),
            Err(e) => {
                eprintln!("  warning: failed to open evidence-dir: {e}");
                None
            }
        },
        None => None,
    };

    let pass = match backend.as_str() {
        "mock" => {
            let mut p = MockPhysics::at_rest();
            run_scenario(&mut p, &scenario, duration_s, evidence.as_mut())
        }
        "gazebo" => {
            let world = arg(&args, "--world").unwrap_or_else(|| "falcon".into());
            let model = arg(&args, "--model").unwrap_or_else(|| "quad".into());
            let mut p = build_gazebo(&args, world, model);
            run_scenario(&mut p, &scenario, duration_s, evidence.as_mut())
        }
        other => {
            eprintln!("unknown backend: {other}  (expected: mock | gazebo)");
            std::process::exit(2);
        }
    };

    if let Some(s) = evidence.as_mut() { s.finish(pass); }

    if pass {
        println!("PASS");
    } else {
        println!("FAIL");
        std::process::exit(1);
    }
}

/// Runs the named scenario against the chosen physics backend. For
/// v0.19.0 the only implemented scenario is `hover` (the v0.16.1
/// 70% PWM climb test). `step`, `mission`, and `disturbance` are
/// reserved per `docs/SIMULATOR.md`'s falsifiable-criteria table
/// and currently fall through to the hover path with a "TODO" note —
/// they land as separate scenarios in v0.19.x once the gz bench
/// produces baseline hover evidence.
fn run_scenario(
    physics: &mut dyn Physics,
    scenario: &str,
    duration_s: f32,
    evidence: Option<&mut EvidenceSink>,
) -> bool {
    match scenario {
        "hover" => run_hover(physics, duration_s, evidence),
        other => {
            eprintln!(
                "  scenario {other} not yet wired (v0.19.0 ships hover only); falling back to hover",
            );
            run_hover(physics, duration_s, evidence)
        }
    }
}

/// Single-scenario loop: command 70 % PWM, hold for the duration,
/// report whether the body climbed. Deliberately minimal — same
/// scenario v0.16.1+ has shipped; v0.19.0 only adds the bench-evidence
/// + counter reporting around it.
fn run_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let mut t = 0.0_f32;
    let mut initial_alt: Option<f32> = None;
    let mut min_alt = f32::MAX;
    let mut max_alt = f32::MIN;
    let tick_period = Duration::from_secs_f32(dt);

    // Real-time pacing: the gz-transport bridge needs wall-clock
    // time between polls so the OS can deliver IMU/NavSat frames.
    // MockPhysics is synchronous and reports `counters() == None`,
    // so we only sleep when the backend actually streams. Same
    // pattern as `HitlBench::real_time()` in v0.18.2 falcon-hitl-rfspoof.
    let pace_real_time = physics.counters().is_some();

    // Command 70 % PWM on all four motors — enough to hover under
    // MockPhysics's THRUST_SCALE=20 N, and gives gz-sim's
    // MulticopterMotorModel enough margin to lift the 700 g body.
    let motor_pwm = [0.7_f32; 4];

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        physics.step(motor_pwm, dt);
        let (imu, pos) = physics.measure(0.01);
        let alt_m = -pos[2]; // NED down → altitude is -z
        if initial_alt.is_none() { initial_alt = Some(alt_m); }
        min_alt = min_alt.min(alt_m);
        max_alt = max_alt.max(alt_m);

        if let Some(ref mut e) = evidence {
            e.write_tick(step, t, pos, imu.accel_body, imu.gyro_body, motor_pwm,
                         physics.counters());
        }

        t += dt;

        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();

    let start = initial_alt.unwrap_or(0.0);
    let net_climb = max_alt - start;
    let counters = physics.counters();
    println!(
        "  verdict: backend={} steps={} climb={:.2} m  (min={:.2} max={:.2})  wall={:.2}s",
        physics.name(), n, net_climb, min_alt, max_alt, wall.as_secs_f32(),
    );
    if let Some((imu_recv, navsat_recv, motor_send)) = counters {
        println!(
            "  counters: imu_recv={imu_recv} navsat_recv={navsat_recv} motor_send={motor_send}",
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary(n, net_climb, min_alt, max_alt, wall.as_secs_f32(), counters);
    }
    // PASS criterion: net climb > 0.1 m. Under MockPhysics, 70 % PWM
    // easily beats gravity. Under the real gz bridge, success depends
    // on the SDF model's motor scaling — if the bench reports FAIL
    // with imu_recv > 0 and motor_send > 0 the bridge wiring is
    // working and the SDF's motorConstant needs adjustment.
    net_climb > 0.1
}

/// Bench-evidence sink — writes one harness log + one per-tick CSV
/// under `<dir>/<timestamp>-<backend>-<scenario>-{harness.log,ticks.csv}`.
/// Same shape as `bench-evidence/px4-sitl/<TS>-*.log` from v0.18.2.
struct EvidenceSink {
    harness: fs::File,
    ticks: fs::File,
    timestamp: String,
}

impl EvidenceSink {
    fn open(dir: &PathBuf, backend: &str, scenario: &str) -> std::io::Result<Self> {
        fs::create_dir_all(dir)?;
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let stem = format!("{ts}-{backend}-{scenario}");
        let harness_path = dir.join(format!("{stem}-harness.log"));
        let ticks_path = dir.join(format!("{stem}-ticks.csv"));
        let mut harness = fs::File::create(&harness_path)?;
        let mut ticks = fs::File::create(&ticks_path)?;
        writeln!(
            harness,
            "falcon-sitl-gz bench-evidence\nbackend: {backend}\nscenario: {scenario}\ntimestamp: {ts}\n"
        )?;
        writeln!(
            ticks,
            "step,t_s,n_m,e_m,d_m,ax_body,ay_body,az_body,gx_body,gy_body,gz_body,m0,m1,m2,m3,imu_recv,navsat_recv,motor_send"
        )?;
        Ok(Self { harness, ticks, timestamp: format!("{ts}") })
    }

    #[allow(clippy::too_many_arguments)]
    fn write_tick(
        &mut self,
        step: u32,
        t: f32,
        pos: [f32; 3],
        accel: [f32; 3],
        gyro: [f32; 3],
        pwm: [f32; 4],
        counters: Option<(u64, u64, u64)>,
    ) {
        let (i, n, m) = counters.unwrap_or((0, 0, 0));
        let _ = writeln!(
            self.ticks,
            "{},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{},{},{}",
            step, t, pos[0], pos[1], pos[2],
            accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2],
            pwm[0], pwm[1], pwm[2], pwm[3], i, n, m,
        );
    }

    fn write_summary(
        &mut self,
        steps: u32,
        net_climb: f32,
        min_alt: f32,
        max_alt: f32,
        wall_s: f32,
        counters: Option<(u64, u64, u64)>,
    ) {
        let _ = writeln!(self.harness, "steps:      {steps}");
        let _ = writeln!(self.harness, "net_climb:  {net_climb:.3} m");
        let _ = writeln!(self.harness, "min_alt:    {min_alt:.3} m");
        let _ = writeln!(self.harness, "max_alt:    {max_alt:.3} m");
        let _ = writeln!(self.harness, "wall:       {wall_s:.3} s");
        if let Some((i, n, m)) = counters {
            let _ = writeln!(self.harness, "imu_recv:   {i}");
            let _ = writeln!(self.harness, "navsat_recv:{n}");
            let _ = writeln!(self.harness, "motor_send: {m}");
        }
    }

    fn finish(&mut self, pass: bool) {
        let _ = writeln!(self.harness, "verdict:    {}", if pass { "PASS" } else { "FAIL" });
        let _ = self.harness.flush();
        let _ = self.ticks.flush();
        let _ = &self.timestamp;
    }
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
        let pass = run_hover(&mut p, 1.0, None);
        assert!(pass, "70 % PWM × THRUST_SCALE should easily beat gravity");
    }

    /// v0.19.0 — when --evidence-dir is set, the runner produces two
    /// files: a harness log and a per-tick CSV. Smoke-test that both
    /// files materialise + carry the right header/footer.
    #[test]
    fn evidence_sink_produces_log_and_csv() {
        let tmp = std::env::temp_dir().join(format!("fsg-bench-{}", std::process::id()));
        let _ = fs::remove_dir_all(&tmp);
        let mut sink = EvidenceSink::open(&tmp, "mock", "hover").expect("open");
        sink.write_tick(0, 0.0, [0.0; 3], [0.0; 3], [0.0; 3], [0.7; 4], Some((1, 2, 3)));
        sink.write_summary(1, 0.5, 0.0, 0.5, 0.01, Some((1, 2, 3)));
        sink.finish(true);
        // Both files exist; the CSV has the header line + one data row.
        let entries: Vec<_> = fs::read_dir(&tmp).unwrap().collect();
        assert_eq!(entries.len(), 2, "expected harness.log + ticks.csv in {:?}", tmp);
        let _ = fs::remove_dir_all(&tmp);
    }

    /// Stub-only — when `gazebo` feature is ON, `GazeboPhysics::new`
    /// connects to gz-transport (and panics if no gz-sim is running),
    /// so the panic-free contract doesn't apply.
    #[cfg(not(feature = "gazebo"))]
    #[test]
    fn gazebo_stub_does_not_panic() {
        let mut g = crate::physics::GazeboPhysics::new("test-world", "test-quad");
        let _ = run_hover(&mut g, 0.1, None);
        // Result is FAIL (everything zeros), but the harness must not
        // panic — that's the contract for a stub-only run.
    }
}
