//! chain-walk — lower relay's shipped flight core through meld, loom and synth,
//! and report synth's per-target reach (SWREQ-RELAY-CHAIN-P01, North Star O1).
//!
//! Given the release candidate's `falcon-cascade` component, it runs:
//!
//!   meld fuse <component> --memory shared --pack-rebase --reproducible
//!   loom optimize <fused>
//!   synth compile <optimized> --all-exports --allow-skipped-exports, per target
//!
//! and parses synth's own two lines per target:
//!
//!   `Found N exported functions`
//!   `warning: K of N functions were skipped (not in output): a, b, ...`
//!
//! Reach is measured on three targets and gated on one. The job fails only when
//! `cortex-m7dp`, the acceptance target, declines any function, exits non-zero,
//! prints no `Found` line, or writes no ELF. `cortex-m7` and `rv32imac` are
//! reported, never gated. Their reach is a number to move, not a bar.
//!
//! A planted negative control runs every time: a module holding one v128
//! function, which synth declines on cortex-m7dp, is judged by the SAME gate
//! and must fail. If it passes, the gate is blind, and the run fails.
//!
//! It writes `<out>/reach.json` and `<out>/north-star-input.O1.json` (schema
//! pulseengine.eu data/north-star/input.schema.json, v1). Execution match on
//! the lowered ELF and a WCET bound are reported `ran: false`, which the North
//! Star counts as failed, never as 0. An emulator replay is not on-target
//! evidence. No translation-validation share is reported.
//!
//! The tools come from PATH, or from MELD, LOOM and SYNTH. Run it under the
//! varve layer pinned in varve.toml, so `varve which` names the toolchain.
//! It is native, not a WASI component, because it spawns other tools.
//!
//! Build:  rustc --edition 2021 -O scripts/chain-walk.rs -o chain-walk
//! Run:    ./chain-walk --component falcon-cascade-vX.wasm --out DIR --version X.Y.Z
//! Tests:  rustc --edition 2021 --test scripts/chain-walk.rs -o cw-test && ./cw-test
//! Exit:   0 gate holds and control failed as planted; 1 gate fails or control
//!         passed; 2 a tool could not run or an argument is missing.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

/// The acceptance target. Only this target gates.
const GATE_TARGET: &str = "cortex-m7dp";

/// (name, synth arguments). The first entry is the gate target.
const TARGETS: &[(&str, &[&str])] = &[
    ("cortex-m7dp", &["-t", "cortex-m7dp"]),
    ("cortex-m7", &["-t", "cortex-m7"]),
    ("rv32imac", &["-b", "riscv", "-t", "rv32imac"]),
];

/// One function synth declines on cortex-m7dp (v128 is not lowered). The
/// control for the gate. `ok` keeps the module non-trivial.
const CONTROL_WAT: &str = r#"(module
  (memory 1)
  (func (export "declined") (param v128) (result v128) local.get 0)
  (func (export "ok") (param i32) (result i32) local.get 0 i32.const 1 i32.add))
"#;

#[derive(Debug, Clone, PartialEq)]
struct Reach {
    total: Option<u32>,
    declined: Vec<String>,
}

impl Reach {
    fn reached(&self) -> Option<u32> {
        self.total
            .map(|t| t.saturating_sub(self.declined.len() as u32))
    }
}

#[derive(Debug)]
struct TargetRun {
    target: String,
    exit_ok: bool,
    elf_written: bool,
    reach: Reach,
}

fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut it = s.chars().peekable();
    while let Some(c) = it.next() {
        if c == '\x1b' {
            if it.peek() == Some(&'[') {
                it.next();
                for d in it.by_ref() {
                    if d.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        out.push(c);
    }
    out
}

/// Parse synth's log. A missing `Found` line leaves `total` None, which the
/// gate treats as "not measured", never as zero.
fn parse_reach(log: &str) -> Reach {
    let log = strip_ansi(log);
    let mut total = None;
    let mut declined = Vec::new();
    for line in log.lines() {
        if let Some(i) = line.find("Found ") {
            let rest = &line[i + 6..];
            if let Some(n) = rest.strip_suffix(" exported functions:").or_else(|| rest.strip_suffix(" exported functions")) {
                if let Ok(n) = n.trim().parse::<u32>() {
                    total = Some(n);
                }
            }
        }
        if let Some(i) = line.find("were skipped (not in output):") {
            let list = &line[i + "were skipped (not in output):".len()..];
            declined = list
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
        }
    }
    Reach { total, declined }
}

/// The gate. Every way of not having measured is a fail.
fn gate(run: &TargetRun) -> Result<(), String> {
    let total = run.reach.total.ok_or("no `Found N exported functions` line: not measured")?;
    if total == 0 {
        return Err("synth found 0 functions: empty is not a pass".into());
    }
    if !run.reach.declined.is_empty() {
        return Err(format!(
            "{} of {} declined: {}",
            run.reach.declined.len(),
            total,
            run.reach.declined.join(", ")
        ));
    }
    if !run.exit_ok {
        return Err("synth exited non-zero".into());
    }
    if !run.elf_written {
        return Err("no ELF written".into());
    }
    Ok(())
}

fn is_elf(p: &Path) -> bool {
    std::fs::read(p).map(|b| b.starts_with(b"\x7fELF")).unwrap_or(false)
}

fn tool(env: &str, default: &str) -> String {
    std::env::var(env).unwrap_or_else(|_| default.to_string())
}

fn run_logged(cmd: &mut Command, log: &Path) -> Result<bool, String> {
    let out = cmd
        .output()
        .map_err(|e| format!("cannot run {:?}: {e}", cmd.get_program()))?;
    let mut text = String::from_utf8_lossy(&out.stdout).into_owned();
    text.push_str(&String::from_utf8_lossy(&out.stderr));
    std::fs::write(log, &text).map_err(|e| format!("write {}: {e}", log.display()))?;
    Ok(out.status.success())
}

fn synth_target(input: &Path, name: &str, args: &[&str], out: &Path) -> Result<TargetRun, String> {
    let elf = out.join(format!("falcon-cascade-{name}.elf"));
    let _ = std::fs::remove_file(&elf);
    let log = out.join(format!("synth-{name}.log"));
    let exit_ok = run_logged(
        Command::new(tool("SYNTH", "synth"))
            .arg("compile")
            .arg(input)
            .arg("--all-exports")
            .args(args)
            .arg("--allow-skipped-exports")
            .arg("-o")
            .arg(&elf),
        &log,
    )?;
    let text = std::fs::read_to_string(&log).unwrap_or_default();
    Ok(TargetRun {
        target: name.to_string(),
        exit_ok,
        elf_written: is_elf(&elf),
        reach: parse_reach(&text),
    })
}

fn json_str(s: &str) -> String {
    let mut o = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn opt_num(n: Option<u32>) -> String {
    n.map(|v| v.to_string()).unwrap_or_else(|| "null".into())
}

/// RFC 3339 UTC from the system clock, no dependencies.
fn now_rfc3339() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Civil-from-days (Howard Hinnant).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

fn reach_fact(run: &TargetRun, gated: bool, control_failed: Option<bool>, evidence: &str) -> String {
    let pass = if gated {
        gate(run).is_ok()
    } else {
        run.reach.total.is_some() && run.reach.declined.is_empty() && run.exit_ok && run.elf_written
    };
    let declined: Vec<String> = run.reach.declined.iter().map(|s| json_str(s)).collect();
    let mut f = format!(
        "{{\"id\":{},\"ran\":{},\"pass\":{}",
        json_str(&format!("falcon-cascade/{}/synth-reach", run.target)),
        run.reach.total.is_some(),
        pass
    );
    if let Some(c) = control_failed {
        let _ = write!(f, ",\"control_failed_as_expected\":{c}");
    }
    let _ = write!(
        f,
        ",\"detail\":{{\"target\":{},\"rung\":\"synth-reach\",\"gated\":{},\"total\":{},\"reached\":{},\"declined\":[{}],\"object_written\":{}}}",
        json_str(&run.target),
        gated,
        opt_num(run.reach.total),
        opt_num(run.reach.reached()),
        declined.join(","),
        run.elf_written
    );
    if !evidence.is_empty() {
        let _ = write!(f, ",\"evidence\":{}", json_str(evidence));
    }
    f.push('}');
    f
}

fn not_run_fact(id: &str, target: &str, rung: &str, why: &str) -> String {
    format!(
        "{{\"id\":{},\"ran\":false,\"pass\":false,\"detail\":{{\"target\":{},\"rung\":{},\"why\":{}}}}}",
        json_str(id),
        json_str(target),
        json_str(rung),
        json_str(why)
    )
}

struct Args {
    component: PathBuf,
    out: PathBuf,
    version: String,
    evidence: String,
}

fn parse_args() -> Result<Args, String> {
    let mut component = None;
    let mut out = None;
    let mut version = None;
    let mut evidence = String::new();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut val = |name: &str| it.next().ok_or(format!("{name} needs a value"));
        match a.as_str() {
            "--component" => component = Some(PathBuf::from(val("--component")?)),
            "--out" => out = Some(PathBuf::from(val("--out")?)),
            "--version" => version = Some(val("--version")?),
            "--evidence" => evidence = val("--evidence")?,
            "-h" | "--help" => {
                println!("usage: chain-walk --component <falcon-cascade.wasm> --out <dir> --version <X.Y.Z> [--evidence <run-url>]");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(Args {
        component: component.ok_or("--component is required")?,
        out: out.ok_or("--out is required")?,
        version: version.ok_or("--version is required")?,
        evidence,
    })
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("chain-walk: {e}");
            return ExitCode::from(2);
        }
    };
    match walk(&args) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("chain-walk: {e}");
            ExitCode::from(2)
        }
    }
}

fn walk(a: &Args) -> Result<bool, String> {
    std::fs::create_dir_all(&a.out).map_err(|e| format!("mkdir {}: {e}", a.out.display()))?;
    let fused = a.out.join("falcon-cascade-fused.wasm");
    let opt = a.out.join("falcon-cascade-opt.wasm");

    if !run_logged(
        Command::new(tool("MELD", "meld"))
            .arg("fuse")
            .arg(&a.component)
            .arg("-o")
            .arg(&fused)
            .args(["--memory", "shared", "--pack-rebase", "--reproducible"]),
        &a.out.join("meld.log"),
    )? {
        return Err("meld fuse failed; see meld.log".into());
    }
    if !run_logged(
        Command::new(tool("LOOM", "loom")).arg("optimize").arg(&fused).arg("-o").arg(&opt),
        &a.out.join("loom.log"),
    )? {
        return Err("loom optimize failed; see loom.log".into());
    }

    let mut runs = Vec::new();
    for (name, targs) in TARGETS {
        runs.push(synth_target(&opt, name, targs, &a.out)?);
    }

    // Planted control: the same gate on a module synth declines on the gate target.
    let cdir = a.out.join("control");
    std::fs::create_dir_all(&cdir).map_err(|e| format!("mkdir control: {e}"))?;
    let cwat = cdir.join("declined.wat");
    std::fs::write(&cwat, CONTROL_WAT).map_err(|e| format!("write control: {e}"))?;
    let gate_args = TARGETS[0].1;
    let control = synth_target(&cwat, GATE_TARGET, gate_args, &cdir)?;
    let control_failed = gate(&control).is_err();

    let gate_run = &runs[0];
    let verdict = gate(gate_run);

    // reach.json: the plain record.
    let mut reach = String::from("{\"component\":");
    reach.push_str(&json_str(&a.component.display().to_string()));
    reach.push_str(",\"targets\":[");
    for (i, r) in runs.iter().enumerate() {
        if i > 0 {
            reach.push(',');
        }
        let _ = write!(
            reach,
            "{{\"target\":{},\"gated\":{},\"total\":{},\"reached\":{},\"declined\":[{}],\"exit_ok\":{},\"object_written\":{}}}",
            json_str(&r.target),
            r.target == GATE_TARGET,
            opt_num(r.reach.total),
            opt_num(r.reach.reached()),
            r.reach.declined.iter().map(|s| json_str(s)).collect::<Vec<_>>().join(","),
            r.exit_ok,
            r.elf_written
        );
    }
    let _ = write!(reach, "],\"control_failed_as_expected\":{control_failed}}}\n");
    std::fs::write(a.out.join("reach.json"), reach).map_err(|e| format!("write reach.json: {e}"))?;

    // north-star-input.O1.json, schema v1.
    let mut facts = Vec::new();
    for r in &runs {
        let gated = r.target == GATE_TARGET;
        facts.push(reach_fact(r, gated, gated.then_some(control_failed), &a.evidence));
    }
    facts.push(not_run_fact(
        "falcon-cascade/cortex-m7dp/execution-match",
        GATE_TARGET,
        "execution-match",
        "the lowered ELF has not been executed; a Renode replay of SWREQ-FALCON-REFVEC-P01 is planned for falcon-v1.145.0 and is emulator evidence",
    ));
    facts.push(not_run_fact(
        "falcon-cascade/cortex-m7dp/wcet",
        GATE_TARGET,
        "wcet",
        "no WCET bound is computed",
    ));
    let input = format!(
        "{{\"schema_version\":1,\"objective\":\"O1\",\"repo\":\"pulseengine/relay\",\"version\":{},\"generated_at\":{},\"generator\":{{\"tool\":\"relay scripts/chain-walk.rs\",\"version\":\"1\"}},\"facts\":[{}],\"not_measured\":[{}]}}\n",
        json_str(&a.version),
        json_str(&now_rfc3339()),
        facts.join(","),
        [
            "execution of the lowered ELF on any target or emulator",
            "WCET",
            "whether the lowered code computes the same floats as the module (synth#1691 class)",
            "translation validation (none reported, by rule)",
            "jess's ladder rungs, which jess reports",
        ]
        .iter()
        .map(|s| json_str(s))
        .collect::<Vec<_>>()
        .join(",")
    );
    std::fs::write(a.out.join("north-star-input.O1.json"), input)
        .map_err(|e| format!("write north-star-input.O1.json: {e}"))?;

    for r in &runs {
        println!(
            "chain-walk: {:<12} reach {}/{} object={} {}",
            r.target,
            opt_num(r.reach.reached()),
            opt_num(r.reach.total),
            r.elf_written,
            if r.target == GATE_TARGET { "(gate)" } else { "(reported)" }
        );
    }
    println!(
        "chain-walk: control (v128 on {GATE_TARGET}) {}",
        if control_failed { "failed as planted" } else { "PASSED: the gate is blind" }
    );
    match (&verdict, control_failed) {
        (Ok(()), true) => {
            println!("chain-walk: PASS");
            Ok(true)
        }
        (Err(e), _) => {
            println!("chain-walk: FAIL {GATE_TARGET}: {e}");
            Ok(false)
        }
        (Ok(()), false) => {
            println!("chain-walk: FAIL the control passed the gate");
            Ok(false)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(log: &str, exit_ok: bool, elf: bool) -> TargetRun {
        TargetRun { target: "cortex-m7dp".into(), exit_ok, elf_written: elf, reach: parse_reach(log) }
    }

    const FULL: &str = "\x1b[2m2026\x1b[0m \x1b[32m INFO\x1b[0m Found 37 exported functions:\nCompiled 37 functions to x.elf\n";
    const PARTIAL: &str = "INFO Found 37 exported functions:\nwarning: 4 of 37 functions were skipped (not in output): func_7, pulseengine:falcon-cascade/controller@0.13.0#step, func_20, func_29\n";

    #[test]
    fn full_reach_passes() {
        let r = run(FULL, true, true);
        assert_eq!(r.reach.total, Some(37));
        assert_eq!(r.reach.reached(), Some(37));
        assert!(gate(&r).is_ok());
    }

    #[test]
    fn declined_function_fails() {
        let r = run(PARTIAL, true, true);
        assert_eq!(r.reach.reached(), Some(33));
        assert_eq!(r.reach.declined[1], "pulseengine:falcon-cascade/controller@0.13.0#step");
        assert!(gate(&r).is_err());
    }

    #[test]
    fn missing_found_line_fails_not_zero() {
        let r = run("Compiled 37 functions to x.elf\n", true, true);
        assert_eq!(r.reach.total, None);
        assert!(gate(&r).is_err());
    }

    #[test]
    fn empty_log_fails() {
        assert!(gate(&run("", true, true)).is_err());
    }

    #[test]
    fn zero_functions_fails() {
        assert!(gate(&run("Found 0 exported functions:\n", true, true)).is_err());
    }

    #[test]
    fn nonzero_exit_fails_even_at_full_reach() {
        assert!(gate(&run(FULL, false, true)).is_err());
    }

    #[test]
    fn missing_elf_fails_even_at_full_reach() {
        assert!(gate(&run(FULL, true, false)).is_err());
    }

    #[test]
    fn rfc3339_shape() {
        let t = now_rfc3339();
        assert_eq!(t.len(), 20);
        assert!(t.ends_with('Z') && &t[4..5] == "-" && &t[10..11] == "T");
    }

    #[test]
    fn json_escapes() {
        assert_eq!(json_str("a\"b\\c\n"), "\"a\\\"b\\\\c\\n\"");
    }

    #[test]
    fn not_run_fact_is_a_fail() {
        let f = not_run_fact("x", "t", "wcet", "w");
        assert!(f.contains("\"ran\":false") && f.contains("\"pass\":false"));
    }
}
