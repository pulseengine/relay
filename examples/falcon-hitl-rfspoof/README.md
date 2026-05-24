# falcon-hitl-rfspoof — HITL harness against an RF GPS-spoofing source

This is the **host-side test driver** for the v0.10 geofence safety
path under real-RF conditions. It deliberately stays small and
delegates everything safety-critical to the formally verified
components it composes:

| Layer                        | Crate            | Coverage                       |
|------------------------------|------------------|--------------------------------|
| Position-bound trip          | `relay-lc`       | Verus LC-P09/P10 + Kani LC-K01..K05 |
| RTL command dispatch         | `relay-sc`       | Verus + cFS-DNA verified path  |
| HITL bench abstraction       | this crate       | unit-tested via `StubBench`    |

## What ships in software

* `harness.rs`  — `HitlBench` trait + `run_scenario()` driver + verdict shape.
* `stub.rs`     — deterministic in-process backend (always available; covered by tests).
* `hackrf.rs`   — HackRF backend with `gps-sdr-sim` / `hackrf_transfer` argv builders.
* `main.rs`     — CLI binary.

The stub backend is the one cargo tests run. It synthesises a fake
"spoofer" that walks the reported position out of the fence at a
configured rate and pins the harness contract: the latch trips, the
RTL RTS dispatches, the verdict reports both.

The HackRF backend currently returns the **planned** trajectory rather
than reading a live FC telemetry feed — that single boundary is the
piece that needs a real bench. See **Lab setup** below.

## Lab setup (real RF)

### Bill of materials

| Item                       | Notes                                          |
|----------------------------|------------------------------------------------|
| HackRF One (or BladeRF)    | TX-capable SDR for L1 C/A                      |
| GPS active patch antenna   | TX side — terminate or shield, do not radiate  |
| ANT500 (or RF shielding)   | Containment between SDR and the GPS receiver   |
| Test flight controller     | PX4 / ArduPilot / your own (FC under test)     |
| Telemetry link             | USB or serial → host running this harness      |

### ⚠️ Legal / safety

Transmitting fake GPS signals over-the-air is illegal in nearly every
jurisdiction. **Always** test inside an RF-shielded enclosure or with a
direct cabled connection (SDR TX → attenuator → splitter → GPS RX).

### Tools

Install:

```bash
brew install hackrf            # macOS; or apt install hackrf on Debian
git clone https://github.com/osqzss/gps-sdr-sim && cd gps-sdr-sim && make
sudo cp gps-sdr-sim /usr/local/bin/
```

Fetch a recent RINEX nav file (any day works for the spoofed
trajectory):

```bash
curl -O https://cddis.nasa.gov/archive/gnss/data/daily/2024/brdc/brdc0010.24n.Z
uncompress brdc0010.24n.Z
```

### Generate the IQ

```bash
gps-sdr-sim -e brdc0010.24n -l 47.5023,19.0401,120 -o /tmp/spoof.iq -d 30
```

### Transmit + run the harness

In one shell:

```bash
hackrf_transfer -t /tmp/spoof.iq -f 1575420000 -s 2600000 -x 0
```

In another:

```bash
cargo run -p falcon-hitl-rfspoof -- --backend=hackrf --duration=30
```

To wire the harness to your FC's telemetry, replace `step()` in
`HackRfBench` (`src/hackrf.rs`) with the parser for your link
(MAVLink `GLOBAL_POSITION_INT`, NMEA `GGA`, etc.) so `position_cm()`
returns the FC's reported NED position.

## Without hardware

```bash
cargo run -p falcon-hitl-rfspoof
# falcon-hitl-rfspoof: backend=stub duration=5s
#   fence: ±100 m × ±100 m × ±100 m (NED, centred on home)
# verdict = HitlVerdict { … latched: true, rtl_dispatched: true, … }
# PASS
```

## Evidence

The harness contract is recorded in
[`artifacts/verification/FV-FALCON-HITL-001.yaml`](../../artifacts/verification/FV-FALCON-HITL-001.yaml).
