#!/usr/bin/env rust-script
//! Compose the five published falcon stage components into the cascade socket,
//! producing a SELF-CONTAINED component a host can instantiate.
//!
//! Replaces the bash version, whose first run had a bug that was the SHELL's,
//! not the logic's:
//!
//!     ls "$DIR"/falcon-cascade-*.wasm | head -1
//!
//! also matches `falcon-cascade-stream-composed` and `falcon-cascade-stream-fused`,
//! and `ls` sorts those FIRST — so it silently plugged the five stages into the
//! wrong socket and wac died with `invalid leading byte (0x66) for component
//! defined type`, a message that names nothing useful. Selecting by an explicit
//! predicate instead of a glob makes that class unrepresentable.
//!
//! Usage:
//!   scripts/compose-cascade.rs <dir-with-published-components> [out.wasm]
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! wasmparser = "0.221"
//! ```

use anyhow::{bail, Context, Result};
use std::path::{Path, PathBuf};
use std::process::Command;

/// The five stages plugged into the socket. `iekf`, NOT `ekf`: BUILD.bazel's
/// wac_plug plugs the VERIFIED IEKF into the ekf socket ("the verified stack is
/// the composed stack"). Composing falcon-ekf here would silently produce a
/// different vehicle than the one we verify, and nothing downstream would say so.
const PLUGS: [&str; 5] = ["iekf", "position", "attitude", "rate", "mixer"];
const SOCKET: &str = "cascade";

/// `falcon-<stage>-v<digit>...wasm` and nothing else.
///
/// Anchored on `-v` followed by a digit precisely because `falcon-cascade-` is a
/// PREFIX of `falcon-cascade-stream-composed-`. Anything other than exactly one
/// match is an error naming the candidates, so an ambiguous bundle fails loudly
/// rather than silently resolving to whichever sorted first.
fn find_one(dir: &Path, stage: &str) -> Result<PathBuf> {
    let prefix = format!("falcon-{stage}-v");
    let mut hits: Vec<PathBuf> = std::fs::read_dir(dir)
        .with_context(|| format!("reading {}", dir.display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            let Some(name) = p.file_name().and_then(|n| n.to_str()) else {
                return false;
            };
            name.ends_with(".wasm")
                && name
                    .strip_prefix(&prefix)
                    .and_then(|rest| rest.chars().next())
                    .is_some_and(|c| c.is_ascii_digit())
        })
        .collect();
    hits.sort();
    match hits.len() {
        1 => Ok(hits.remove(0)),
        0 => bail!("no falcon-{stage}-v<N>.wasm in {}", dir.display()),
        n => bail!(
            "expected exactly 1 falcon-{stage}-v<N>.wasm in {}, found {n}: {hits:?}",
            dir.display()
        ),
    }
}

/// Component vs core module, read from the PAYLOAD rather than a filename or an
/// OCI mediaType. `falcon-rate:1.129.0` shipped as a raw core module past a
/// green check because the manifest claimed "component" and nobody read the bytes.
fn assert_component(path: &Path) -> Result<()> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    match bytes.get(..8).unwrap_or_default() {
        [0x00, 0x61, 0x73, 0x6d, 0x0d, 0x00, 0x01, 0x00] => Ok(()),
        [0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00] => {
            bail!("{} is a CORE MODULE, not a component", path.display())
        }
        head => bail!("{} is not wasm (first 8 bytes: {head:02x?})", path.display()),
    }
}

/// Count `memory.grow` across every code section. The composed cascade must have
/// zero: a growing linear memory cannot be lowered to bare metal, which is the
/// point of the no_std conversion (SWREQ-FALCON-OCI-P02).
fn count_memory_grow(bytes: &[u8]) -> Result<usize> {
    use wasmparser::{Parser, Payload};
    let mut n = 0;
    for payload in Parser::new(0).parse_all(bytes) {
        if let Payload::CodeSectionEntry(body) = payload? {
            let mut ops = body.get_operators_reader()?;
            while !ops.eof() {
                if matches!(ops.read()?, wasmparser::Operator::MemoryGrow { .. }) {
                    n += 1;
                }
            }
        }
    }
    Ok(n)
}

fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let dir = PathBuf::from(
        args.next()
            .context("usage: compose-cascade.rs <dir-with-published-components> [out.wasm]")?,
    );
    let out = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| dir.join("composed-cascade.wasm"));

    let socket = find_one(&dir, SOCKET)?;
    assert_component(&socket)?;
    let plugs: Vec<PathBuf> = PLUGS
        .iter()
        .map(|s| {
            let p = find_one(&dir, s)?;
            assert_component(&p)?;
            Ok(p)
        })
        .collect::<Result<_>>()?;

    println!("socket : {}", socket.display());
    for p in &plugs {
        println!("plug   : {}", p.display());
    }

    // argv array, not a shell string: nothing here can word-split or glob.
    let mut cmd = Command::new("wac");
    cmd.arg("plug").arg(&socket);
    for p in &plugs {
        cmd.arg("--plug").arg(p);
    }
    cmd.arg("-o").arg(&out);
    let status = cmd
        .status()
        .context("running `wac` — is wac-cli installed? (cargo install wac-cli)")?;
    if !status.success() {
        bail!("wac plug failed: {status}");
    }

    // The composed result gets the same scrutiny as its inputs. A composition
    // that silently produced a core module, or reintroduced memory.grow from a
    // plug nobody checked, would otherwise be found by a consumer.
    assert_component(&out)?;
    let bytes = std::fs::read(&out)?;
    let grows = count_memory_grow(&bytes)?;
    println!(
        "composed -> {}  ({} bytes, component, memory.grow = {grows})",
        out.display(),
        bytes.len()
    );
    if grows != 0 {
        bail!("composed cascade contains {grows} memory.grow — it cannot be lowered to bare metal");
    }
    Ok(())
}
