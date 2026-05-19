#!/usr/bin/env bash
#
# falcon-hello end-to-end demo runner.
#
# Drives the v0.1 acceptance test from a single command:
#   1. Builds the falcon-hello binary in release mode.
#   2. Spawns the GCS subprocess in the background.
#   3. Spawns the vehicle subprocess for a bounded duration.
#   4. Captures both stdouts.
#   5. Asserts that the GCS received ≥ EXPECTED_HEARTBEATS heartbeats.
#   6. Exits 0 on success, 1 on any failure.
#
# Used as the `run:` step for FV-FALCON-WORLD-001 in the rivet
# verification graph, so spar / a GitHub Action can extract it
# and run as a CI gate.
#
# Override with environment:
#   FALCON_HELLO_DURATION   bounded run length in seconds (default 4)
#   FALCON_HELLO_RATE_HZ    vehicle heartbeat rate           (default 4)
#   FALCON_HELLO_EXPECTED   minimum received count to pass   (default 8)
#   FALCON_HELLO_PORT_BASE  UDP port pair base               (default 14700)

set -euo pipefail

DURATION=${FALCON_HELLO_DURATION:-4}
RATE=${FALCON_HELLO_RATE_HZ:-4}
EXPECTED=${FALCON_HELLO_EXPECTED:-8}
PORT_BASE=${FALCON_HELLO_PORT_BASE:-14700}

GCS_PORT=${PORT_BASE}
VEH_PORT=$((PORT_BASE + 1))

REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO_ROOT"

echo "[falcon-hello-demo] building release binary..."
cargo build --release -p falcon-hello >/dev/null

BIN="$REPO_ROOT/target/release/falcon-hello"
if [[ ! -x "$BIN" ]]; then
    echo "[falcon-hello-demo] error: binary not found at $BIN" >&2
    exit 1
fi

GCS_LOG=$(mktemp)
VEH_LOG=$(mktemp)
cleanup() { rm -f "$GCS_LOG" "$VEH_LOG"; }
trap cleanup EXIT

echo "[falcon-hello-demo] launching gcs on 127.0.0.1:${GCS_PORT}"
"$BIN" --mode gcs \
       --bind  "127.0.0.1:${GCS_PORT}" \
       --remote "127.0.0.1:${VEH_PORT}" \
       --duration "$DURATION" >"$GCS_LOG" 2>&1 &
GCS_PID=$!

# Give the GCS a moment to bind before the vehicle starts blasting.
sleep 0.3

echo "[falcon-hello-demo] launching vehicle (${RATE} Hz × ${DURATION}s)"
"$BIN" --mode vehicle \
       --bind  "127.0.0.1:${VEH_PORT}" \
       --remote "127.0.0.1:${GCS_PORT}" \
       --rate  "$RATE" \
       --duration "$DURATION" >"$VEH_LOG" 2>&1 || true

# Wait for gcs to finish its own duration window.
wait "$GCS_PID" || true

# Count successful heartbeat receptions in the gcs log.
RX_COUNT=$(grep -c "rx heartbeat" "$GCS_LOG" || true)
TX_COUNT=$(grep -c "tx seq="     "$VEH_LOG" || true)

echo "[falcon-hello-demo] vehicle sent ${TX_COUNT} heartbeat(s)"
echo "[falcon-hello-demo] gcs received ${RX_COUNT} heartbeat(s)"

if (( RX_COUNT < EXPECTED )); then
    echo "[falcon-hello-demo] FAIL: expected at least ${EXPECTED} heartbeats, got ${RX_COUNT}" >&2
    echo "--- vehicle log ---" >&2
    cat "$VEH_LOG" >&2
    echo "--- gcs log ---" >&2
    cat "$GCS_LOG" >&2
    exit 1
fi

# Spot-check the decoded fields land where we expect.
if ! grep -q "type=2 autopilot=8 status=3" "$GCS_LOG"; then
    echo "[falcon-hello-demo] FAIL: decoded heartbeat fields do not match falcon-quad standby" >&2
    grep "rx heartbeat" "$GCS_LOG" | head -5 >&2
    exit 1
fi

echo "[falcon-hello-demo] PASS"
