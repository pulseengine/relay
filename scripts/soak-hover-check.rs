//! soak-hover-check — judge SWREQ-FALCON-RATE-P05's own bars on the soak's
//! unfaulted supervised hover.
//!
//! Owner decision 2026-11-19 (S3): the nightly soak judges RATE-P05's bars,
//! not the 5x looser ring bar. `falcon-sitl-gz --scenario=supervised-hover`
//! flies the production FlightSupervisor with no rotor kill and prints
//!
//!   `  hover: window=5-10s ticks=N rp_rate_rms=Xrad/s motor_out_of_band_frac=Y fdi_gate_open_frac=Z`
//!
//! and this check passes only when, over that window:
//!   - the window is RATE-P05's settled hover, 5-10 s, with at least
//!     MIN_TICKS ticks (an empty or truncated window is not a measurement);
//!   - roll/pitch body-rate RMS is strictly below 0.1 rad/s, and not exactly
//!     0 (the gz adapter zero-fills a silent IMU: a dead IMU is not a quiet one);
//!   - no tick has a motor command outside [0.02, 0.98] (fraction exactly 0);
//!   - the FDI steady gate is open on at least 95% of ticks.
//! Every number must parse as a finite decimal; NaN, inf, an absent field or a
//! missing line fail closed.
//!
//! scripts/soak-ring-check.sh stays as the #270 ring detector on the faulted
//! run's pre-kill window (0.5 rad/s); this check is RATE-P05's oracle. It is a
//! new checker, so it is Rust (owner rule 2026-11-18: no new Python or shell
//! tooling).
//!
//! Build and run:  rustc --edition 2021 -O scripts/soak-hover-check.rs -o shc && ./shc <harness-output>
//! Tests (the planted bad inputs): rustc --edition 2021 --test scripts/soak-hover-check.rs -o shc-test && ./shc-test
//! Exit: 0 all bars hold, 1 a bar fails or the window was not measured, 2 could not read.

use std::process::ExitCode;

/// RATE-P05's settled hover window, as the harness prints it.
const WINDOW: &str = "5-10s";
/// 5 s at 250 Hz is 1250 ticks; fewer than this means ticks were lost.
const MIN_TICKS: u64 = 1000;
const RMS_BAR: f64 = 0.1;
const GATE_OPEN_MIN: f64 = 0.95;

#[derive(Debug, PartialEq)]
struct Hover {
    ticks: u64,
    rms: f64,
    oob: f64,
    gate: f64,
}

fn field<'a>(line: &'a str, name: &str) -> Option<&'a str> {
    line.split_whitespace().find_map(|t| t.strip_prefix(name)?.strip_prefix('='))
}

/// A finite, non-negative decimal written as digits (no NaN/inf, no sign).
fn decimal(s: &str) -> Result<f64, String> {
    let ok = !s.is_empty()
        && s.chars().all(|c| c.is_ascii_digit() || c == '.')
        && s.chars().filter(|&c| c == '.').count() <= 1
        && !s.starts_with('.')
        && !s.ends_with('.');
    if !ok {
        return Err(format!("'{s}' is not a finite decimal"));
    }
    s.parse::<f64>().map_err(|e| format!("'{s}': {e}"))
}

fn parse(text: &str) -> Result<Hover, String> {
    let line = text
        .lines()
        .find(|l| l.starts_with("  hover: "))
        .ok_or("no 'hover:' line: the harness did not report RATE-P05's window, so this check verified NOTHING")?;
    let get = |n: &str| field(line, n).ok_or(format!("hover line has no {n}= field: {line}"));
    let window = get("window")?;
    if window != WINDOW {
        return Err(format!("window {window} is not RATE-P05's settled hover {WINDOW}"));
    }
    let ticks_s = get("ticks")?;
    if ticks_s.is_empty() || !ticks_s.chars().all(|c| c.is_ascii_digit()) {
        return Err(format!("ticks '{ticks_s}' is not a count"));
    }
    let ticks: u64 = ticks_s.parse().map_err(|e| format!("ticks: {e}"))?;
    let rms_s = get("rp_rate_rms")?
        .strip_suffix("rad/s")
        .ok_or("rp_rate_rms has no rad/s unit")?;
    Ok(Hover {
        ticks,
        rms: decimal(rms_s).map_err(|e| format!("rp_rate_rms {e}"))?,
        oob: decimal(get("motor_out_of_band_frac")?).map_err(|e| format!("motor_out_of_band_frac {e}"))?,
        gate: decimal(get("fdi_gate_open_frac")?).map_err(|e| format!("fdi_gate_open_frac {e}"))?,
    })
}

/// The verdict: Ok(summary) when every bar holds, Err(reasons) otherwise.
fn judge(h: &Hover) -> Result<String, Vec<String>> {
    let mut fails = Vec::new();
    if h.ticks < MIN_TICKS {
        fails.push(format!("only {} ticks in the window (need >= {MIN_TICKS}); the bars would pass vacuously", h.ticks));
    }
    if h.rms == 0.0 {
        fails.push("roll/pitch RMS is exactly 0: a SILENT (zero-filled) IMU, not a quiet hover".into());
    } else if h.rms >= RMS_BAR {
        fails.push(format!("roll/pitch body-rate RMS {} rad/s >= {RMS_BAR}", h.rms));
    }
    if h.oob > 0.0 {
        fails.push(format!("{:.2}% of ticks had a motor outside [0.02, 0.98]", h.oob * 100.0));
    }
    if h.gate < GATE_OPEN_MIN {
        fails.push(format!("FDI steady gate open on {:.2}% of ticks (need >= {:.0}%)", h.gate * 100.0, GATE_OPEN_MIN * 100.0));
    }
    if fails.is_empty() {
        Ok(format!(
            "ticks={} rp_rate_rms={} rad/s (< {RMS_BAR}), motors in band on every tick, FDI gate open {:.2}%",
            h.ticks,
            h.rms,
            h.gate * 100.0
        ))
    } else {
        Err(fails)
    }
}

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: soak-hover-check <harness-output-file>");
        return ExitCode::from(2);
    };
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read {path}: {e}");
            return ExitCode::from(2);
        }
    };
    match parse(&text).map_err(|e| vec![e]).and_then(|h| judge(&h)) {
        Ok(s) => {
            println!("::notice::RATE-P05 hover bars hold: {s}");
            ExitCode::SUCCESS
        }
        Err(fails) => {
            for f in &fails {
                println!("::error::RATE-P05 HOVER BAR FAILED: {f}");
            }
            ExitCode::from(1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: &str = "  ring: prekill_ticks=2500 prekill_rp_rate_rms=0.0015rad/s prekill_saturated_frac=0.0000\n  hover: window=5-10s ticks=1250 rp_rate_rms=0.0016rad/s motor_out_of_band_frac=0.0000 fdi_gate_open_frac=1.0000\n";

    fn verdict(text: &str) -> Result<String, Vec<String>> {
        parse(text).map_err(|e| vec![e]).and_then(|h| judge(&h))
    }

    fn with(field: &str, value: &str) -> String {
        GOOD.lines()
            .map(|l| {
                if l.starts_with("  hover: ") {
                    l.split(' ')
                        .map(|t| if t.starts_with(&format!("{field}=")) { format!("{field}={value}") } else { t.to_string() })
                        .collect::<Vec<_>>()
                        .join(" ")
                } else {
                    l.to_string()
                }
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn a_measured_quiet_hover_passes() {
        assert!(verdict(GOOD).is_ok(), "{:?}", verdict(GOOD));
    }

    #[test]
    fn each_bar_fails_on_its_planted_bad_input() {
        for (f, v) in [
            ("rp_rate_rms", "0.1000rad/s"),  // at the bar: strictly below is required
            ("rp_rate_rms", "2.4674rad/s"),  // the #270 ring
            ("rp_rate_rms", "0.0000rad/s"),  // silent IMU
            ("motor_out_of_band_frac", "0.0008"), // one tick out of band
            ("fdi_gate_open_frac", "0.9499"),
            ("fdi_gate_open_frac", "0.6620"), // the 33.8%-shut era
            ("ticks", "999"),
            ("ticks", "0"),
        ] {
            assert!(verdict(&with(f, v)).is_err(), "{f}={v} passed");
        }
    }

    #[test]
    fn the_edges_that_must_pass_do() {
        assert!(verdict(&with("rp_rate_rms", "0.0999rad/s")).is_ok());
        assert!(verdict(&with("fdi_gate_open_frac", "0.9500")).is_ok());
        assert!(verdict(&with("ticks", "1000")).is_ok());
    }

    #[test]
    fn unmeasured_or_malformed_fails_closed() {
        for (f, v) in [
            ("rp_rate_rms", "NaNrad/s"),
            ("rp_rate_rms", "infrad/s"),
            ("rp_rate_rms", "0.0016"), // unit missing
            ("motor_out_of_band_frac", "NaN"),
            ("fdi_gate_open_frac", "-1.0"),
            ("fdi_gate_open_frac", ""),
            ("ticks", "12x"),
            ("window", "0-4s"),
        ] {
            assert!(verdict(&with(f, v)).is_err(), "{f}={v} passed");
        }
        assert!(verdict("  ring: prekill_ticks=2500\n").is_err(), "no hover line passed");
        assert!(verdict(&GOOD.replace(" fdi_gate_open_frac=1.0000", "")).is_err(), "missing field passed");
        // The faulted run prints the line with an empty window.
        assert!(verdict("  hover: window=5-10s ticks=0 rp_rate_rms=NaNrad/s motor_out_of_band_frac=NaN fdi_gate_open_frac=NaN\n").is_err());
    }
}
