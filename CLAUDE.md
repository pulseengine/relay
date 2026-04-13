# CLAUDE.md

## Relay — Formally Verified Flight Software Components

Relay is a stream-based component framework for safety-critical real-time systems.
Components are stream transformers: typed input streams in, typed output streams out.
Composition is through stream wiring (P3 async), not centralized message routing.

Part of the [PulseEngine](https://github.com/pulseengine) toolchain.

### Tagline

*Relay routes.*

### Design Principles

1. **Streams, not callbacks** — Components declare typed `stream<T>` inputs/outputs. Composition is stream wiring.
2. **Formally verified at every layer** — Verus (SMT/Z3), Rocq (theorem proving), Lean (scheduling theory). Write to the intersection.
3. **Domain-agnostic core** — The framework works for spacecraft, drones, automotive, industrial. Domain types are separate WIT packages.
4. **P3-native** — Designed for WASM Component Model Preview 3 async. Callback lifting mode for embedded.
5. **Gale-backed** — On embedded targets, every stream is a Gale ring_buf (verified), every wait is a Gale poll (verified).

### Architecture

```
WIT interfaces (typed streams)
    ↓
Guest components (Rust, compiled to WASM)
    ↓
Meld (fuses components, wires streams at build time)
    ↓
Loom (optimizes stream access patterns)
    ↓
Kiln (std path) or Synth → Gale (embedded path)
```

### Build

```bash
cargo build
cargo test
```

### Formal Verification

Triple-track verification following Gale's model:
- **Verus**: SMT verification of component logic. No trait objects, closures, or async in verified code.
- **Rocq**: Theorem proofs for stream routing correctness, scheduling properties.
- **Lean**: Mathematical proofs for timing, priority, fairness.
- **Kani**: Bounded model checking for runtime edge cases.

```bash
bazel test //:verus_test
```

### Traceability

ASPICE V-model traceability via Rivet:
```bash
rivet validate
rivet coverage
```

### Key Rules

- Use `rivet validate` to verify changes to artifact YAML files
- Use `rivet list --format json` for machine-readable artifact queries
- Follow the [PulseEngine Formal Verification Guide](https://pulseengine.eu/guides/VERIFICATION-GUIDE.md)
- All component logic that can be verified MUST be verified across all three tracks
