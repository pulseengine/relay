#!/usr/bin/env rust-script
//! Checks the TRACE-P02 promotion census against the tree (FV-FALCON-TRACE-002).
//!
//! SWREQ-FALCON-TRACE-P02 records a census of the `implemented` sw-reqs: a
//! PROMOTED list (sw-reqs moved to `verified` with their verifiers) and a
//! RESIDUAL list (sw-reqs left at `implemented`, one line per reason, each line
//! starting with its count). A census written in prose drifts silently; this
//! script re-derives every number in it from the artifact YAML.
//!
//! Checks, all on `artifacts/**/*.yaml`:
//!   promoted  every PROMOTED id is a sw-req at `verified`, and at least one
//!             artifact at `verified` links `verifies` to it (criterion a).
//!             The PROMOTED header count equals the number of ids found.
//!   residual  every RESIDUAL id resolves to an existing sw-req (a typo is a
//!             failure, not a silent drop). Each reason line's leading count
//!             equals the ids on that line; the RESIDUAL header count equals
//!             their sum (criterion b, c).
//!   complete  every sw-req at `implemented` whose release is
//!             falcon-v1.143.0 or EARLIER is named in RESIDUAL, except
//!             TRACE-P02 itself (criterion c).
//!
//! Blind spot, by design: an `implemented` sw-req targeted to a release after
//! v1.143 is not required to appear. New work lands there every week; requiring
//! it in a v1.143 census would turn every implementation PR red. Those are
//! counted and printed, not judged.
//!
//! Exit codes: 0 every check passed; 1 a check failed; 2 could not evaluate
//! (unparseable artifact file, TRACE-P02 missing, a section header missing).
//!
//! Usage: scripts/trace-census.rs --check
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_yaml = "0.9"
//! serde = { version = "1", features = ["derive"] }
//! walkdir = "2"
//! ```

use anyhow::{bail, Context, Result};
use std::collections::{BTreeMap, BTreeSet};

const CENSUS: &str = "SWREQ-FALCON-TRACE-P02";
const SCOPE_RELEASE: (u32, u32, u32) = (1, 143, 0);

#[derive(Debug, serde::Deserialize)]
struct Doc {
    #[serde(default)]
    artifacts: Vec<Artifact>,
}

#[derive(Debug, serde::Deserialize)]
struct Link {
    #[serde(default, rename = "type")]
    kind: String,
    #[serde(default)]
    target: String,
}

#[derive(Debug, serde::Deserialize)]
struct Artifact {
    id: String,
    #[serde(default, rename = "type")]
    kind: String,
    #[serde(default)]
    status: String,
    #[serde(default)]
    release: String,
    #[serde(default)]
    description: String,
    #[serde(default)]
    links: Vec<Link>,
}

fn main() -> std::process::ExitCode {
    if !std::env::args().skip(1).any(|a| a == "--check") {
        eprintln!("usage: scripts/trace-census.rs --check");
        return std::process::ExitCode::from(2);
    }
    match run(std::path::Path::new(".")) {
        Ok(true) => std::process::ExitCode::from(0),
        Ok(false) => std::process::ExitCode::from(1),
        Err(e) => {
            eprintln!("trace-census: could not evaluate: {e:#}");
            std::process::ExitCode::from(2)
        }
    }
}

fn load(root: &std::path::Path) -> Result<BTreeMap<String, Artifact>> {
    let mut all = BTreeMap::new();
    for e in walkdir::WalkDir::new(root.join("artifacts")) {
        let e = e.context("walking artifacts/")?;
        if !e.path().extension().is_some_and(|x| x == "yaml") {
            continue;
        }
        let text = std::fs::read_to_string(e.path())
            .with_context(|| format!("reading {}", e.path().display()))?;
        // An unparseable file would shrink the population being checked, so it
        // is an evaluation failure, never a skip.
        let doc: Doc = serde_yaml::from_str(&text)
            .with_context(|| format!("parsing {}", e.path().display()))?;
        for a in doc.artifacts {
            all.insert(a.id.clone(), a);
        }
    }
    Ok(all)
}

/// `falcon-v1.143.0` -> (1, 143, 0).
fn version(release: &str) -> Option<(u32, u32, u32)> {
    let v = release.strip_prefix("falcon-v")?;
    let mut it = v.split('.').map(|p| p.parse::<u32>().ok());
    Some((it.next()??, it.next()??, it.next()??))
}

/// Census ids are written without the `SWREQ-` prefix (`FALCON-FOLLOW-P01`,
/// `CCSDS-P01`). A token counts when it ends in `-P<digits>`.
fn ids_in(text: &str) -> Vec<String> {
    text.split(|c: char| !(c.is_ascii_uppercase() || c.is_ascii_digit() || c == '-'))
        .filter(|t| {
            t.rsplit_once("-P").is_some_and(|(head, n)| {
                !head.is_empty() && !n.is_empty() && n.chars().all(|c| c.is_ascii_digit())
            })
        })
        .map(|t| format!("SWREQ-{t}"))
        .collect()
}

/// The number after `<header>, ` (e.g. `RESIDUAL, 57`).
fn header_count(section: &str, header: &str) -> Result<usize> {
    let rest = section
        .strip_prefix(header)
        .and_then(|s| s.strip_prefix(", "))
        .with_context(|| format!("`{header}, <n>` header"))?;
    let n: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
    n.parse()
        .with_context(|| format!("count after `{header}, `"))
}

/// Split the RESIDUAL section into reason entries, each starting `<n>, `.
fn reasons(text: &str) -> Vec<&str> {
    let b = text.as_bytes();
    let mut starts = Vec::new();
    for i in 2..b.len() {
        let after_sep = (b[i - 2] == b':' || b[i - 2] == b'.') && b[i - 1].is_ascii_whitespace();
        if !after_sep || !b[i].is_ascii_digit() {
            continue;
        }
        let mut j = i;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if text[j..].starts_with(", ") {
            starts.push(i);
        }
    }
    starts
        .iter()
        .enumerate()
        .map(|(k, &s)| text[s..starts.get(k + 1).copied().unwrap_or(text.len())].trim())
        .collect()
}

fn run(root: &std::path::Path) -> Result<bool> {
    let all = load(root)?;
    let census = all
        .get(CENSUS)
        .with_context(|| format!("{CENSUS} not found"))?;
    let d = &census.description;
    let p = d.find("PROMOTED, ").context("no `PROMOTED, ` section")?;
    let r = d.find("RESIDUAL, ").context("no `RESIDUAL, ` section")?;
    if r < p {
        bail!("RESIDUAL section precedes PROMOTED");
    }
    let promoted_text = &d[p..r];
    let residual_text = &d[r..];
    let mut ok = true;
    let mut fail = |msg: String| {
        println!("FAIL {msg}");
        ok = false;
    };

    // promoted
    let promoted: BTreeSet<String> = ids_in(promoted_text).into_iter().collect();
    let stated = header_count(promoted_text, "PROMOTED")?;
    if promoted.is_empty() {
        fail("PROMOTED names no sw-req ids".into());
    }
    if promoted.len() != stated {
        fail(format!("PROMOTED says {stated}, names {}", promoted.len()));
    }
    for id in &promoted {
        match all.get(id) {
            None => fail(format!("PROMOTED {id}: no such artifact")),
            Some(a) if a.kind != "sw-req" => fail(format!("PROMOTED {id}: type {}", a.kind)),
            Some(a) if a.status != "verified" => {
                fail(format!("PROMOTED {id}: status {}", a.status))
            }
            Some(_) => {
                let verifiers: Vec<&str> = all
                    .values()
                    .filter(|v| v.status == "verified")
                    .filter(|v| {
                        v.links
                            .iter()
                            .any(|l| l.kind == "verifies" && &l.target == id)
                    })
                    .map(|v| v.id.as_str())
                    .collect();
                if verifiers.is_empty() {
                    fail(format!("PROMOTED {id}: no verified artifact verifies it"));
                }
            }
        }
    }

    // residual: a folded YAML scalar joins the reason lines into one paragraph,
    // so a reason starts wherever `<n>, ` follows `: ` or `. `.
    let mut residual: BTreeSet<String> = BTreeSet::new();
    let mut line_sum = 0usize;
    for line in reasons(residual_text) {
        let n: String = line.chars().take_while(|c| c.is_ascii_digit()).collect();
        let n: usize = n.parse()?;
        let ids: BTreeSet<String> = ids_in(line).into_iter().collect();
        if ids.len() != n {
            fail(format!(
                "residual line says {n}, names {}: {}",
                ids.len(),
                &line[..line.len().min(60)]
            ));
        }
        line_sum += n;
        for id in ids {
            if !residual.insert(id.clone()) {
                fail(format!("residual {id} named on two reason lines"));
            }
        }
    }
    let stated = header_count(residual_text, "RESIDUAL")?;
    if line_sum != stated {
        fail(format!(
            "RESIDUAL says {stated}, reason lines sum to {line_sum}"
        ));
    }
    for id in &residual {
        match all.get(id) {
            None => fail(format!("RESIDUAL {id}: no such artifact")),
            Some(a) if a.kind != "sw-req" => fail(format!("RESIDUAL {id}: type {}", a.kind)),
            Some(_) => {}
        }
        if promoted.contains(id) {
            fail(format!("{id} is both PROMOTED and RESIDUAL"));
        }
    }

    // complete
    let mut later = 0usize;
    for a in all
        .values()
        .filter(|a| a.kind == "sw-req" && a.status == "implemented")
    {
        if a.id == CENSUS {
            continue;
        }
        match version(&a.release) {
            Some(v) if v > SCOPE_RELEASE => later += 1,
            Some(_) => {
                if !residual.contains(&a.id) {
                    fail(format!(
                        "{} is implemented ({}) but not named in RESIDUAL",
                        a.id, a.release
                    ));
                }
            }
            None => fail(format!(
                "{} is implemented with no falcon-v release ({:?})",
                a.id, a.release
            )),
        }
    }

    println!(
        "trace-census: promoted {} (all verified, each with a verified verifier: {}), residual {} named, {} implemented sw-reqs targeted after falcon-v1.143.0 not judged",
        promoted.len(),
        if ok { "yes" } else { "see FAIL lines" },
        residual.len(),
        later
    );
    Ok(ok)
}
