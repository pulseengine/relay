#!/usr/bin/env bash
# Build the falcon flight core as a WASM Component Model component and RUN it in
# wasmtime — the verified IEKF→geometric→ADRC→mixer cascade executing as a
# portable component, closing the control loop against the analytic SimBackend
# inside the component. The mechanical gate for the v1.26 "verified core is a
# runnable CM component" claim, and the hand-off artifact for the separate
# hardware-integration project (meld → loom → synth → gale).
#
# Requires: cargo-component, wasmtime. (Bench-only — not in the cargo CI, which
# lacks the wasm32-wasip2 component toolchain; like the meld / gz steps.)
set -euo pipefail
cd "$(dirname "$0")/../wasm/cm/flight"
echo "== building falcon flight component (wasm32-wasip2) =="
cargo component build --release >/dev/null 2>&1
W=$(ls target/wasm32-wasip*/release/falcon_flight_component.wasm | head -1)
echo "   component: $W ($(wc -c < "$W") bytes)"
echo "== running the verified core in wasmtime =="
STAB=$(wasmtime run --invoke 'run-stabilization()' "$W" 2>/dev/null)
POS=$(wasmtime run --invoke 'run-position-hold()' "$W" 2>/dev/null)
echo "   run-stabilization -> ${STAB} rad (final tilt)"
echo "   run-position-hold -> ${POS} m (final position error)"
python3 - "$STAB" "$POS" <<'PY'
import sys
stab, pos = float(sys.argv[1]), float(sys.argv[2])
ok = stab < 0.1 and pos < 0.6
print(f"== {'PASS' if ok else 'FAIL'}: stabilization {stab:.4f}<0.1 rad, position {pos:.4f}<0.6 m, in wasmtime ==")
sys.exit(0 if ok else 1)
PY
