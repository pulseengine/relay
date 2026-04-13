// Relay Host Runner — loads and validates P3 WASM components via wasmtime.

use anyhow::Result;
use wasmtime::component::Component;
use wasmtime::{Config, Engine};

fn main() -> Result<()> {
    let wasm_path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("Usage: relay-host-runner <path-to-component.wasm>");
        std::process::exit(1);
    });

    println!("Loading component: {wasm_path}");

    let mut config = Config::new();
    config.wasm_component_model(true);
    config.wasm_component_model_async(true);

    let engine = Engine::new(&config)?;
    let component = Component::from_file(&engine, &wasm_path)?;

    let component_type = component.component_type();
    println!("\nExports:");
    for (name, _) in component_type.exports(&engine) {
        println!("  {name}");
    }

    println!("\nImports:");
    for (name, _) in component_type.imports(&engine) {
        println!("  {name}");
    }

    println!("\nComponent loaded and validated successfully.");
    Ok(())
}
