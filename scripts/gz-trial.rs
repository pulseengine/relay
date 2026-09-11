#!/usr/bin/env rust-script
//! Run one Gazebo SITL trial against a PRISTINE world, with a settled vehicle.
//!
//! WHY THIS EXISTS. Two things make an ad-hoc `gz sim &` + run-the-binary loop
//! produce numbers that are not comparable to each other, and both cost real
//! time to rediscover:
//!
//!   1. THE WORLD DOES NOT RESET. `/world/<w>/control` does not answer a reset
//!      request on this setup (`gz service` times out), so a second trial starts
//!      wherever the first one left the vehicle — 15 m downrange, in one case.
//!      The only reliable pristine pose is a fresh server.
//!
//!   2. THE VEHICLE IS STILL FALLING. The model spawns at z=0.80 m and drops
//!      onto its landing gear (resting at z=0.019 m). Starting a trial mid-drop
//!      referenced the whole flight to a zero ~0.5 m above the ground. Measured
//!      before this was understood: the SAME binary on the SAME world returned
//!      0.20 m, 0.19 m, and one run that never left the ground at -0.54 m.
//!
//! The backend now waits for a settled datum itself, so (2) is defence in depth
//! rather than the only guard — but a trial driver that does not prove the
//! vehicle is at rest cannot tell a regression from a bad start.
//!
//! It exists as a script because the evidence it produced (which calibration
//! knob actually carries the gz hold) is the kind a reviewer should be able to
//! reproduce without reconstructing the rig from a commit message.
//!
//! Usage:
//!   scripts/gz-trial.rs --label A -- ./target/release/falcon-sitl-gz --backend=gazebo ...
//!   scripts/gz-trial.rs --label B --env HOVER_THRUST=0.5 -- <cmd>...
//!
//! Options:
//!   --label <s>    prefix for every reported line (default "trial")
//!   --env K=V      set one variable for the trial command; repeatable
//!   --world <s>    gz world name (default "falcon")
//!   --model <s>    gz model name (default "quad")
//!   --no-settle    skip the at-rest wait (to REPRODUCE the flaky start)
//!   --keep         leave the server running after the trial
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! ```

use anyhow::{bail, Context, Result};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

const WORLD_SDF: &str = "examples/falcon-sitl-gz/worlds/falcon-quad.sdf";
/// Two consecutive pose reads within this many metres means "at rest".
const SETTLE_EPS: f64 = 0.001;
/// Lines worth echoing from a trial. Anything else is noise at this level.
const KEEP: &[&str] = &[
    "verdict", "counters", "PASS", "FAIL", "datum", "calibration", "HOLD ERROR",
    "DIFFERENTIAL", "final NED", "wall/sim", "pace ", "BIT-IDENT", "LOOP CLOSES",
];

struct Args {
    label: String,
    envs: Vec<(String, String)>,
    world: String,
    model: String,
    settle: bool,
    keep: bool,
    cmd: Vec<String>,
}

fn parse() -> Result<Args> {
    let mut a = Args {
        label: "trial".into(),
        envs: vec![],
        world: "falcon".into(),
        model: "quad".into(),
        settle: true,
        keep: false,
        cmd: vec![],
    };
    let mut it = std::env::args().skip(1);
    while let Some(t) = it.next() {
        match t.as_str() {
            "--label" => a.label = it.next().context("--label needs a value")?,
            "--world" => a.world = it.next().context("--world needs a value")?,
            "--model" => a.model = it.next().context("--model needs a value")?,
            "--no-settle" => a.settle = false,
            "--keep" => a.keep = true,
            "--env" => {
                let kv = it.next().context("--env needs K=V")?;
                let (k, v) = kv.split_once('=').context("--env expects K=V")?;
                a.envs.push((k.into(), v.into()));
            }
            "--" => {
                a.cmd = it.collect();
                break;
            }
            other => bail!("unknown option {other}; the trial command goes after `--`"),
        }
    }
    if a.cmd.is_empty() {
        bail!("no trial command. Usage: gz-trial.rs --label A -- <cmd>...");
    }
    Ok(a)
}

/// `gz topic -e -t <t> -n 1`, which BLOCKS until one message arrives. Used as
/// the readiness probe: a `gz topic -l` poll returns instantly before discovery
/// completes, so a retry loop built on it burns every attempt in under a second
/// and reports a server that is merely slow as a server that never came up.
fn one_message(topic: &str, secs: u64) -> Option<String> {
    let out = Command::new("timeout")
        .args([&secs.to_string(), "gz", "topic", "-e", "-t", topic, "-n", "1"])
        .stderr(Stdio::null())
        .output()
        .ok()?;
    let s = String::from_utf8_lossy(&out.stdout).to_string();
    (!s.trim().is_empty()).then_some(s)
}

fn pose_z(model: &str) -> Option<f64> {
    let s = one_message(&format!("/model/{model}/pose"), 5)?;
    // Protobuf omits zero-valued fields, so `x:`/`y:` may be absent entirely --
    // read `z:` from the first `position` block rather than assuming three.
    let pos = s.split("position").nth(1)?;
    pos.lines()
        .find_map(|l| l.trim().strip_prefix("z:")?.trim().parse::<f64>().ok())
}

fn start_server(sdf: &str) -> Result<Child> {
    Command::new("gz")
        .args(["sim", "-s", "-r", "-v2", sdf])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .context("spawning `gz sim` -- is gz-harmonic installed and on PATH?")
}

fn main() -> Result<()> {
    let a = parse()?;
    let say = |m: &str| println!("[{}] {m}", a.label);

    let _ = Command::new("pkill").args(["-f", "gz sim -s -r"]).status();
    let mut srv = start_server(WORLD_SDF)?;

    let imu = format!(
        "/world/{}/model/{}/link/base_link/sensor/imu_sensor/imu",
        a.world, a.model
    );
    let up = Instant::now();
    let mut ready = false;
    for _ in 0..15 {
        if one_message(&imu, 4).is_some() {
            ready = true;
            break;
        }
    }
    if !ready {
        let _ = srv.kill();
        bail!("gz published no IMU sample within ~60 s -- server did not come up");
    }
    say(&format!("gz up after {:.1}s", up.elapsed().as_secs_f32()));

    if a.settle {
        let mut prev: Option<f64> = None;
        let mut settled = None;
        for _ in 0..20 {
            let z = pose_z(&a.model);
            if let (Some(z), Some(p)) = (z, prev) {
                if (z - p).abs() < SETTLE_EPS {
                    settled = Some(z);
                    break;
                }
            }
            prev = z;
        }
        match settled {
            Some(z) => say(&format!("vehicle at rest: z={z:.4} m")),
            // Not fatal: report it and let the trial run, so the numbers carry
            // their own caveat instead of the driver silently deciding for you.
            None => say("WARNING: vehicle never settled -- treat this trial as suspect"),
        }
    }

    let mut c = Command::new(&a.cmd[0]);
    c.args(&a.cmd[1..]);
    for (k, v) in &a.envs {
        c.env(k, v);
    }
    let out = c.output().with_context(|| format!("running {:?}", a.cmd))?;
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    for line in text.lines() {
        if KEEP.iter().any(|k| line.contains(k)) {
            say(line.trim_end());
        }
    }

    if !a.keep {
        let _ = srv.kill();
        let _ = srv.wait();
        let _ = Command::new("pkill").args(["-f", "gz sim -s -r"]).status();
    }
    // The trial's own exit status is the verdict; propagate it so a caller can
    // chain trials without re-parsing text.
    std::process::exit(out.status.code().unwrap_or(1));
}
