#!/usr/bin/env rust-script
//! Which relay/falcon capabilities are actually reachable from something we ship?
//!
//! WHY THIS EXISTS (#422, SWREQ-FALCON-CLAIMS-P01). docs/PX4-PARITY-ASSESSMENT.md
//! credited RC modes (relay-rc), airframe variants (relay-mix-multi), sensor
//! voting (relay-sensvote) and extra flight modes (relay-modextra) as delivered.
//! None of them has a consumer: nothing shipped depends on them. A hand-written
//! capability column drifts; this one is derived from the dependency graph.
//!
//! A crate under crates/ counts as SHIPPED only if it is in the transitive
//! normal-dependency closure of a shipped root:
//!   * the flight core — falcon-core (what the vehicle flies);
//!   * a published wasm component — each wasm/cm/*/Cargo.toml's relay-*/falcon-*
//!     dependencies, plus relay_* imports in Rust sources BUILD.bazel compiles
//!     from wasm/cm that are no crate root (the legacy stream pipeline, #411);
//!   * the released binary — falcon-hello (release.yml builds only that).
//! Anything else under crates/ is NOT SHIPPED. Crates under examples/, tests/,
//! host/ or tools/ are tooling, not flight capabilities, and are listed apart.
//!
//! The output is deterministic (no dates, no versions) so `--check` can fail when
//! the committed docs/CAPABILITY-REACHABILITY.md no longer matches the tree.
//!
//! Exit: 0 ok / up to date · 1 --check found the committed file stale ·
//! 2 could not evaluate.
//!
//! Usage:
//!   scripts/capability-reachability.rs > docs/CAPABILITY-REACHABILITY.md
//!   scripts/capability-reachability.rs --check
//!   rust-script --test scripts/capability-reachability.rs
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_json = "1"
//! toml = "0.8"
//! ```

use anyhow::{bail, Context, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::process::ExitCode;

const OUT: &str = "docs/CAPABILITY-REACHABILITY.md";

/// relay is a SHARED framework: a crate nothing in THIS repo ships can still be a
/// sibling repo's dependency. CI cannot see sibling checkouts, so this is an
/// explicit, dated list rather than a scan (a scan would make --check depend on
/// which repos happen to be cloned). Verified 2026-09-17 against
/// wohl/Cargo.toml path dependencies (`../relay/crates/<crate>`); no sibling
/// repo has a git dependency on relay. Update it when a consumer changes.
const EXTERNAL: &[(&str, &str)] = &[
    ("relay-ccsds", "wohl"),
    ("relay-cs", "wohl"),
    ("relay-ds", "wohl"),
    ("relay-hk", "wohl"),
    ("relay-hs", "wohl"),
    ("relay-lc", "wohl"),
    ("relay-sc", "wohl"),
    ("relay-sch", "wohl"),
    ("relay-tbl", "wohl"),
    ("relay-to", "wohl"),
];

/// name -> (manifest dir relative to the repo, normal-dependency names)
type Graph = BTreeMap<String, (String, BTreeSet<String>)>;

fn closure(graph: &Graph, roots: &BTreeSet<String>) -> BTreeSet<String> {
    let mut seen = BTreeSet::new();
    let mut stack: Vec<String> = roots.iter().filter(|r| graph.contains_key(*r)).cloned().collect();
    while let Some(n) = stack.pop() {
        if !seen.insert(n.clone()) {
            continue;
        }
        if let Some((_, deps)) = graph.get(&n) {
            stack.extend(deps.iter().filter(|d| !seen.contains(*d)).cloned());
        }
    }
    seen
}

fn is_tooling(dir: &str) -> bool {
    ["examples/", "tests/", "host/", "tools/"].iter().any(|p| dir.starts_with(p))
}

fn workspace_graph() -> Result<Graph> {
    let o = std::process::Command::new("cargo")
        .args(["metadata", "--format-version", "1"])
        .output()
        .context("running cargo metadata")?;
    if !o.status.success() {
        bail!("cargo metadata failed: {}", String::from_utf8_lossy(&o.stderr));
    }
    let m: serde_json::Value = serde_json::from_slice(&o.stdout)?;
    let root = m["workspace_root"].as_str().context("workspace_root")?.to_string();
    let members: BTreeSet<&str> = m["workspace_members"].as_array().context("members")?.iter().filter_map(|v| v.as_str()).collect();
    let mut name_of: BTreeMap<String, (String, String)> = BTreeMap::new(); // id -> (name, dir)
    for p in m["packages"].as_array().context("packages")? {
        let id = p["id"].as_str().unwrap_or_default().to_string();
        let manifest = p["manifest_path"].as_str().unwrap_or_default();
        let dir = std::path::Path::new(manifest)
            .parent()
            .and_then(|d| d.strip_prefix(&root).ok())
            .map(|d| d.to_string_lossy().into_owned())
            .unwrap_or_default();
        name_of.insert(id, (p["name"].as_str().unwrap_or_default().to_string(), dir));
    }
    let mut g = Graph::new();
    for node in m["resolve"]["nodes"].as_array().context("resolve.nodes")? {
        let id = node["id"].as_str().unwrap_or_default();
        if !members.contains(id) {
            continue;
        }
        let (name, dir) = name_of[id].clone();
        let deps = node["deps"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|d| {
                d["dep_kinds"].as_array().into_iter().flatten().any(|k| k["kind"].is_null() || k["kind"] == "normal")
            })
            .filter_map(|d| d["pkg"].as_str().and_then(|p| name_of.get(p)).map(|(n, _)| n.clone()))
            .collect();
        g.insert(name, (dir, deps));
    }
    if g.is_empty() {
        bail!("cargo metadata listed no workspace members — refusing to report");
    }
    Ok(g)
}

/// Roots from the published wasm components: manifest deps + Bazel-only imports.
fn component_roots() -> Result<BTreeSet<String>> {
    let mut roots = BTreeSet::new();
    let mut n = 0;
    for e in std::fs::read_dir("wasm/cm")? {
        let dir = e?.path();
        let manifest = dir.join("Cargo.toml");
        if !manifest.is_file() {
            continue;
        }
        n += 1;
        let t: toml::Value = toml::from_str(&std::fs::read_to_string(&manifest)?)?;
        if let Some(deps) = t.get("dependencies").and_then(|d| d.as_table()) {
            roots.extend(deps.keys().filter(|k| k.starts_with("relay-") || k.starts_with("falcon-")).cloned());
        }
    }
    if n == 0 {
        bail!("no components under wasm/cm — refusing to report");
    }
    let build = std::fs::read_to_string("BUILD.bazel").context("reading BUILD.bazel")?;
    for piece in build.split('"') {
        if piece.starts_with("wasm/cm/") && piece.ends_with(".rs") && !piece.ends_with("/lib.rs") {
            let text = std::fs::read_to_string(piece).with_context(|| format!("reading {piece}"))?;
            for (i, _) in text.match_indices("relay_") {
                let rest: String = text[i..].chars().take_while(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || *c == '_').collect();
                if rest.len() > "relay_".len() && (i == 0 || !text.as_bytes()[i - 1].is_ascii_alphanumeric()) {
                    roots.insert(rest.replace('_', "-"));
                }
            }
        }
    }
    Ok(roots)
}

fn render(g: &Graph, core: &BTreeSet<String>, comp: &BTreeSet<String>, bin: &BTreeSet<String>) -> String {
    let mark = |b: bool| if b { "yes" } else { "—" };
    let mut caps = Vec::new();
    let mut tools = Vec::new();
    for (name, (dir, _)) in g {
        if !(name.starts_with("relay-") || name.starts_with("falcon-")) {
            continue;
        }
        if is_tooling(dir) {
            tools.push(name.clone());
            continue;
        }
        let (c, p, b) = (core.contains(name), comp.contains(name), bin.contains(name));
        let external = EXTERNAL.iter().find(|(n, _)| n == name).map(|(_, by)| *by);
        let verdict = if c {
            "flight core".to_string()
        } else if p {
            "shipped in a component".to_string()
        } else if b {
            "shipped in falcon-hello only".to_string()
        } else if let Some(by) = external {
            format!("not shipped here — consumed by {by}")
        } else {
            "**NOT SHIPPED — no consumer**".to_string()
        };
        caps.push((verdict, name.clone(), dir.clone(), mark(c), mark(p), mark(b)));
    }
    const NONE: &str = "**NOT SHIPPED — no consumer**";
    caps.sort_by(|a, b| (a.0 != NONE, &a.1).cmp(&(b.0 != NONE, &b.1)));
    let not_shipped = caps.iter().filter(|c| c.0 == NONE).count();
    let external = caps.iter().filter(|c| c.0.starts_with("not shipped here")).count();
    let mut s = String::new();
    s += "# Capability reachability\n\n";
    s += "<!-- GENERATED by scripts/capability-reachability.rs — do not edit by hand.\n     Regenerate: scripts/capability-reachability.rs > docs/CAPABILITY-REACHABILITY.md\n     CI checks it is current (FV-FALCON-CLAIMS-004). -->\n\n";
    s += "A capability counts as **delivered** only if something we ship depends on it: the flight core (`falcon-core`), a published wasm component (including Bazel-only sources), or the released binary (`falcon-hello`). Derived from the dependency graph, not written by hand (#422).\n\n";
    s += &format!(
        "**{} of {} crates under `crates/` have no consumer at all** — nothing falcon releases depends on them and no sibling repo uses them, whatever any other document says. A further {} are not shipped by falcon but are a sibling repo's dependency (relay is a shared framework).\n\n",
        not_shipped,
        caps.len(),
        external
    );
    s += "**Not shipped is not the same as dead.** Several consumer-less crates are rivet-traced relay framework engines awaiting a consumer, or seams other repos (e.g. jess) integrate against without a code dependency. For *flight* capabilities the disposition — wire, park against a configuration axis, or retire — is SWREQ-FALCON-ORPHAN-P01; nothing here should be retired from this table alone.\n\n";
    s += "| crate | verdict | flight core | component | falcon-hello | path |\n|---|---|---|---|---|---|\n";
    for (v, n, d, c, p, b) in &caps {
        s += &format!("| `{n}` | {v} | {c} | {p} | {b} | `{d}` |\n");
    }
    s += "\n## Tooling (examples, tests, host tools) — not flight capabilities\n\n";
    s += &tools.iter().map(|t| format!("`{t}`")).collect::<Vec<_>>().join(", ");
    s += "\n";
    s
}

fn run(check: bool) -> Result<bool> {
    let g = workspace_graph()?;
    let core = closure(&g, &BTreeSet::from(["falcon-core".to_string()]));
    if core.len() < 2 {
        bail!("falcon-core's closure is empty — refusing to report every capability as unshipped");
    }
    let comp = closure(&g, &component_roots()?);
    let bin = closure(&g, &BTreeSet::from(["falcon-hello".to_string()]));
    let md = render(&g, &core, &comp, &bin);
    if check {
        let committed = std::fs::read_to_string(OUT).with_context(|| format!("reading {OUT}"))?;
        return Ok(committed == md);
    }
    print!("{md}");
    Ok(true)
}

fn main() -> ExitCode {
    let check = std::env::args().any(|a| a == "--check");
    match run(check) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => {
            eprintln!("{OUT} is stale: regenerate with `scripts/capability-reachability.rs > {OUT}`");
            ExitCode::from(1)
        }
        Err(e) => {
            eprintln!("capability-reachability: could not evaluate: {e:#}");
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g(edges: &[(&str, &str, &[&str])]) -> Graph {
        edges
            .iter()
            .map(|(n, d, deps)| (n.to_string(), (d.to_string(), deps.iter().map(|x| x.to_string()).collect())))
            .collect()
    }

    #[test]
    fn closure_is_transitive_and_ignores_unknown_roots() {
        let graph = g(&[("falcon-core", "crates/falcon-core", &["relay-iekf"]), ("relay-iekf", "crates/relay-iekf", &["relay-math"]), ("relay-math", "crates/relay-math", &[]), ("relay-rc", "crates/relay-rc", &[])]);
        let c = closure(&graph, &BTreeSet::from(["falcon-core".to_string(), "nope".to_string()]));
        assert_eq!(c, BTreeSet::from(["falcon-core".into(), "relay-iekf".into(), "relay-math".into()]));
    }

    #[test]
    fn an_unconsumed_capability_is_not_shipped_and_tooling_is_listed_apart() {
        // The #422 shape: relay-rc has no consumer.
        let graph = g(&[("falcon-core", "crates/falcon-core", &[]), ("relay-rc", "crates/relay-rc", &[]), ("falcon-sitl-gz", "examples/falcon-sitl-gz", &["relay-rc"])]);
        let core = BTreeSet::from(["falcon-core".to_string()]);
        let md = render(&graph, &core, &BTreeSet::new(), &BTreeSet::new());
        assert!(md.contains("| `relay-rc` | **NOT SHIPPED — no consumer** |"), "{md}");
        assert!(md.contains("**1 of 2 crates under `crates/` have no consumer"), "{md}");
        assert!(md.contains("`falcon-sitl-gz`") && !md.contains("| `falcon-sitl-gz` |"), "tooling is not a capability row: {md}");
    }

    #[test]
    fn a_sibling_repos_dependency_is_not_reported_as_consumer_less() {
        // relay-sch is wohl's scheduler: not shipped by falcon, but load-bearing elsewhere.
        let graph = g(&[("relay-sch", "crates/relay-sch", &[])]);
        let e = BTreeSet::new();
        let md = render(&graph, &e, &e, &e);
        assert!(md.contains("| `relay-sch` | not shipped here — consumed by wohl |"), "{md}");
        assert!(md.contains("**0 of 1 crates"), "{md}");
    }

    #[test]
    fn output_is_deterministic() {
        let graph = g(&[("relay-b", "crates/relay-b", &[]), ("relay-a", "crates/relay-a", &[])]);
        let e = BTreeSet::new();
        assert_eq!(render(&graph, &e, &e, &e), render(&graph, &e, &e, &e));
    }
}
