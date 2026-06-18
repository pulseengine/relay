//! Subprocess harness for witness.
//!
//! witness invokes us with three env vars:
//!
//!   WITNESS_MODULE    — path to the instrumented .wasm
//!   WITNESS_MANIFEST  — path to the branch manifest .json
//!   WITNESS_OUTPUT    — path where we must write the snapshot
//!
//! We:
//!   1. Load the instrumented module with wasmtime + wasi-preview2.
//!   2. Invoke every no-arg, non-`__witness_*` export witness's
//!      `--invoke-all` mode would invoke.
//!   3. Read each `__witness_counter_<id>` exported global.
//!   4. Write a `witness-harness-v1` snapshot to WITNESS_OUTPUT.
//!
//! The v1 schema is sufficient for branch-coverage reconstruction.
//! v2 (with per-row brval / brcnt / trace memory) is a future
//! follow-up.
//!
//! See witness's `crates/witness-core/src/run_record.rs` —
//! `HarnessSnapshot` is the on-wire shape.

use anyhow::{anyhow, Context, Result};
use serde::Serialize;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::PathBuf;
use wasmtime::{Config, Engine, Linker, Module, Store, Val};
// wasmtime-wasi 44 renamed `preview1` → `p1`; bound here as `preview1`
// so the rest of the file keeps reading naturally. PR #44 (Dependabot
// 27→45 upgrade) merged with main still importing the old path; v0.19.2
// catches it up.
use wasmtime_wasi::p1::{self as preview1, WasiP1Ctx};
use wasmtime_wasi::WasiCtxBuilder;

/// `witness-harness-v1` schema — keys are decimal branch IDs as
/// strings, values are hit counts. Matches `HarnessSnapshot` in
/// witness-core.
#[derive(Serialize)]
struct HarnessSnapshotV1 {
    schema: &'static str,
    counters: HashMap<String, u64>,
}

fn main() -> Result<()> {
    let module_path: PathBuf = env::var("WITNESS_MODULE")
        .context("WITNESS_MODULE env var not set")?
        .into();
    // WITNESS_MANIFEST is informational here — we discover branch IDs
    // from `__witness_counter_<id>` exports, not from the manifest.
    let _manifest_path: Option<PathBuf> = env::var("WITNESS_MANIFEST").ok().map(Into::into);
    let output_path: PathBuf = env::var("WITNESS_OUTPUT")
        .context("WITNESS_OUTPUT env var not set")?
        .into();

    eprintln!("witness-wasi-harness: module={}", module_path.display());

    let mut config = Config::new();
    config.async_support(false);
    let engine = Engine::new(&config)?;

    let module_bytes = fs::read(&module_path)
        .with_context(|| format!("read {}", module_path.display()))?;
    let module = Module::new(&engine, module_bytes)?;

    // Build a wasi-preview1 ctx — the simplest WASI surface that
    // satisfies modules built with wit-bindgen's default Rust panic
    // glue. (Preview2 component-level imports are a separate path;
    // preview1 is what core modules see after meld_fuse + ABI
    // lowering.)
    let wasi = WasiCtxBuilder::new().inherit_stdio().build_p1();
    let mut store: Store<WasiP1Ctx> = Store::new(&engine, wasi);
    let mut linker = Linker::new(&engine);
    preview1::add_to_linker_sync(&mut linker, |s| s)?;

    // Stub any unresolved imports as no-ops returning zero. Witness-
    // instrumented modules with `wasi:cli/exit::exit` etc. would
    // otherwise abort at instantiation; we don't care about side
    // effects, only branch counters.
    define_stubs_for_unresolved_imports(&mut linker, &module, &engine)?;

    let instance = linker.instantiate(&mut store, &module)?;

    // 1. Discover + invoke exports.
    //
    // `_initialize` is called first if present (component-model
    // init). `_start` is skipped because it implies WASI command-
    // style execution which would consume the rest of the run.
    //
    // After that, two precedence layers:
    //   (a) `WITNESS_HARNESS_INVOKES` env var — semicolon-separated
    //       list of `func_name=val1,val2,…` specs. Lets callers drive
    //       funcs with realistic args without teaching the harness
    //       about each subject. Values are parsed against the export's
    //       declared param types. The name/args separator is `=`
    //       (NOT `:`) because WIT export names are
    //       `namespace:package/iface@ver#func` — they always contain a
    //       colon, so a `:` separator can't address them (#202). A bare
    //       `:` is still accepted as a fallback for colon-free names.
    //   (b) auto-discover — every non-`__witness_*` export gets
    //       called with zero-filled args. Walks at least one path
    //       per export even when no script is supplied.
    let mut invoked: Vec<String> = Vec::new();
    let export_names: Vec<String> = module.exports().map(|e| e.name().to_string()).collect();
    if let Some(init) = instance.get_func(&mut store, "_initialize") {
        let nresults = init.ty(&store).results().len();
        let mut results = vec![Val::I32(0); nresults];
        if let Err(e) = init.call(&mut store, &[], &mut results) {
            eprintln!("witness-wasi-harness: _initialize failed: {e:#}");
        }
    }

    let scripted = env::var("WITNESS_HARNESS_INVOKES").unwrap_or_default();
    if !scripted.is_empty() {
        for spec in scripted.split(';').filter(|s| !s.is_empty()) {
            // Split on `=` so WIT export names (which contain `:`) are
            // addressable; fall back to `:` for colon-free names (#202).
            let (name, args_csv) = match spec.split_once('=') {
                Some((n, a)) => (n.trim(), a),
                None => match spec.split_once(':') {
                    Some((n, a)) => (n.trim(), a),
                    None => (spec.trim(), ""),
                },
            };
            let Some(func) = instance.get_func(&mut store, name) else {
                eprintln!("witness-wasi-harness: scripted invoke {name}: export not found");
                continue;
            };
            let ty = func.ty(&store);
            let parsed = parse_args(&ty, args_csv);
            let params = match parsed {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("witness-wasi-harness: scripted invoke {name}: parse args: {e}");
                    continue;
                }
            };
            let nresults = ty.results().len();
            let mut results = vec![Val::I32(0); nresults];
            match func.call(&mut store, &params, &mut results) {
                Ok(()) => invoked.push(format!("{name} (scripted)")),
                Err(e) => eprintln!("witness-wasi-harness: scripted invoke {name} failed: {e:#}"),
            }
        }
    }
    for name in &export_names {
        if name.starts_with("__witness_") || name == "_start" || name == "_initialize" {
            continue;
        }
        let Some(func) = instance.get_func(&mut store, name) else { continue };
        let ty = func.ty(&store);
        let params: Vec<Val> = ty.params().map(zero_val).collect();
        let nresults = ty.results().len();
        let mut results = vec![Val::I32(0); nresults];
        match func.call(&mut store, &params, &mut results) {
            Ok(()) => invoked.push(name.clone()),
            Err(e) => eprintln!("witness-wasi-harness: invoke {name} failed: {e:#}"),
        }
    }
    eprintln!("witness-wasi-harness: invoked {} exports", invoked.len());

    // 2. Read every `__witness_counter_<id>` global and write the
    //    snapshot. Branch IDs are the decimal-encoded `<id>`.
    let mut counters: HashMap<String, u64> = HashMap::new();
    for name in &export_names {
        let Some(id_str) = name.strip_prefix("__witness_counter_") else { continue };
        let Some(global) = instance.get_global(&mut store, name) else { continue };
        let v = global.get(&mut store);
        let hits = match v {
            Val::I32(x) => x as u32 as u64,
            Val::I64(x) => x as u64,
            other => {
                eprintln!("witness-wasi-harness: counter {name} has non-int type {other:?}");
                continue;
            }
        };
        counters.insert(id_str.to_string(), hits);
    }
    eprintln!("witness-wasi-harness: collected {} counters", counters.len());

    let snapshot = HarnessSnapshotV1 {
        schema: "witness-harness-v1",
        counters,
    };
    fs::write(&output_path, serde_json::to_vec_pretty(&snapshot)?)
        .with_context(|| format!("write {}", output_path.display()))?;
    eprintln!("witness-wasi-harness: wrote {}", output_path.display());
    Ok(())
}

/// Parse a CSV string of arg values against an export's declared
/// param types. Empty string → empty Vec (callable as a no-arg
/// func). Numeric parsing is type-aware: i32 / i64 are signed
/// decimal; f32 / f64 use Rust's default float parser. Mirrors
/// witness's own --invoke-with-args precedent: the spec's type
/// comes from the export, not from annotations on the value.
fn parse_args(ty: &wasmtime::FuncType, csv: &str) -> Result<Vec<Val>> {
    use wasmtime::ValType;
    let expected = ty.params().collect::<Vec<_>>();
    let trimmed = csv.trim();
    let parts: Vec<&str> = if trimmed.is_empty() {
        Vec::new()
    } else {
        trimmed.split(',').map(str::trim).collect()
    };
    if parts.len() != expected.len() {
        return Err(anyhow!(
            "expected {} args, got {} ({:?})",
            expected.len(),
            parts.len(),
            parts
        ));
    }
    expected
        .into_iter()
        .zip(parts)
        .map(|(ty, s)| -> Result<Val> {
            Ok(match ty {
                ValType::I32 => Val::I32(s.parse::<i32>().with_context(|| format!("i32 {s}"))?),
                ValType::I64 => Val::I64(s.parse::<i64>().with_context(|| format!("i64 {s}"))?),
                ValType::F32 => Val::F32(s.parse::<f32>().with_context(|| format!("f32 {s}"))?.to_bits()),
                ValType::F64 => Val::F64(s.parse::<f64>().with_context(|| format!("f64 {s}"))?.to_bits()),
                other => return Err(anyhow!("unsupported arg type {other:?}")),
            })
        })
        .collect()
}

/// Zero value of the given wasm value type — used to fill out an
/// export's argument list when we want to walk *some* path through
/// the cascade without knowing the call semantics.
fn zero_val(ty: wasmtime::ValType) -> Val {
    use wasmtime::ValType;
    match ty {
        ValType::I32 => Val::I32(0),
        ValType::I64 => Val::I64(0),
        ValType::F32 => Val::F32(0),
        ValType::F64 => Val::F64(0),
        ValType::V128 => Val::V128(0u128.into()),
        // Reference types: pass externref/funcref nulls. These are
        // unlikely on Rust-compiled wasm but we cover them defensively.
        ValType::Ref(_) => Val::ExternRef(None),
    }
}

/// For every import on the module that the WASI preview1 linker
/// didn't satisfy, define a no-op stub returning zeros. This lets
/// us run modules that pulled in preview2-shaped imports (like the
/// falcon cascade's `wasi:cli/exit@0.2.6::exit`) without those
/// imports actually being called — instrument hits are still
/// counted, and side-effecting imports go nowhere.
fn define_stubs_for_unresolved_imports(
    linker: &mut Linker<WasiP1Ctx>,
    module: &Module,
    _engine: &Engine,
) -> Result<()> {
    for imp in module.imports() {
        let module_name = imp.module().to_string();
        let name = imp.name().to_string();
        let ty = imp.ty();
        let wasmtime::ExternType::Func(ft) = ty else { continue };

        let result_count = ft.results().len();
        // Try-install the stub. If the WASI preview1 linker already
        // satisfies this import, `func_new` returns an "already
        // defined" error — silently skip those.
        let install = linker.func_new(
            &module_name,
            &name,
            ft.clone(),
            move |_caller, _params, results| {
                for r in results.iter_mut().take(result_count) {
                    *r = Val::I32(0);
                }
                Ok(())
            },
        );
        if let Err(e) = install {
            // "already defined" is the only acceptable error; surface
            // anything else.
            let msg = format!("{e}");
            if !msg.contains("already") && !msg.contains("defined") {
                return Err(anyhow!("install stub for {module_name}::{name}: {e}"));
            }
        }
    }
    Ok(())
}
