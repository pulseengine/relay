# PX4-SITL × falcon-hitl-rfspoof — end-to-end loop against a 3D physics sim

This directory is the wiring for running the verified
`relay-lc → relay-sc` chain against a real PX4 flight-controller
running in software-in-the-loop, with a 3D physics backend
(jMAVSim or Gazebo) supplying the world. It needs **no new code** —
the v0.12 `MavlinkBench` already listens for
`GLOBAL_POSITION_INT` (MAVLink id 33) on UDP. PX4-SITL emits exactly
that on port 14550 by default; the loop closes.

## What you see

```
        ┌─────────────────────────────────────────┐
        │  PX4-SITL  +  jMAVSim / Gazebo          │
        │  (3D physics + PX4 nav stack)           │
        └────────────────┬────────────────────────┘
                         │  GLOBAL_POSITION_INT @ 10 Hz
                         │  via UDP :14550
                         ▼
        ┌─────────────────────────────────────────┐
        │  falcon-hitl-rfspoof --backend=mavlink  │
        │    MavlinkBench → relay-lc::Geofence    │
        │                 → relay-sc::CommandStore│
        └────────────────┬────────────────────────┘
                         │  verdict: HitlVerdict { latched, rtl_dispatched, … }
                         ▼
                       PASS / FAIL
```

The PX4 vehicle flies its mission; falcon's geofence checks the
position estimate PX4 publishes; on the first violation the
verified relay-sc RTL RTS fires.

## Prerequisites

| Tool        | Install                                                                |
|-------------|------------------------------------------------------------------------|
| PX4 + jMAVSim | `git clone --recursive https://github.com/PX4/PX4-Autopilot && cd PX4-Autopilot && make px4_sitl jmavsim` |
| Gazebo (optional, prettier) | `make px4_sitl gz_x500` from the PX4-Autopilot directory |
| Rust toolchain | `rustup toolchain install stable` (≥ 1.85) |

Falcon-side: `cargo build -p falcon-hitl-rfspoof` (no extra deps).

## Running it

Two terminals.

### Terminal 1 — PX4-SITL with jMAVSim

```bash
cd ~/git/PX4-Autopilot
make px4_sitl jmavsim
```

Wait for `INFO  [commander] Ready for takeoff!`. PX4 starts emitting
MAVLink (HEARTBEAT, GLOBAL_POSITION_INT, …) on `0.0.0.0:14550`.

### Terminal 2 — falcon-hitl-rfspoof

One command, all-Rust, no wrapper script:

```bash
cd ~/git/pulseengine/relay
cargo run -p falcon-hitl-rfspoof -- --preset=px4-sitl
```

`--preset=px4-sitl` fills in the PX4 stock takeoff coord
(**47.3977, 8.5456, 488 m**, Zürich / ETH), the default
listen address (`0.0.0.0:14550`), and a 30 s duration.

**v0.14.2** also wires the **round-trip** — when the geofence latches,
the harness encodes a MAVLink `COMMAND_LONG` (id 76) with
`MAV_CMD_NAV_RETURN_TO_LAUNCH` (cmd 20) and writes it to PX4's
offboard endpoint (`127.0.0.1:14580`). PX4 receives the command
and actually flies home. Closes the v0.14.0 deferred item.

Override individual fields after the preset, e.g.:

```bash
cargo run -p falcon-hitl-rfspoof -- --preset=px4-sitl \
  --duration=60 --listen=0.0.0.0:14560 --peer=127.0.0.1:14590
```

Fly the vehicle in PX4's `pxh>` console:

```text
pxh> commander takeoff
pxh> commander mode auto:loiter
```

A 100 m geofence centred on home: the loiter pattern stays inside
and the verdict ends in `PASS` with `latched: false`. To force a
trip, command a position outside the fence:

```text
pxh> commander mode auto:loiter
pxh> commander go --location 47.4100 8.5456 30
```

This drives the quad ~1.4 km north — well past the 100 m fence —
and the harness reports:

```
verdict = HitlVerdict {
    backend: "mavlink",
    latched: true,
    latched_at_s: Some(12.34),
    rtl_dispatched: true,
    spoof_first_seen_at_s: Some(11.79),
    failure: None,
}
PASS
```

`spoof_first_seen_at_s` here is **diagnostic only** (the heuristic
in `MavlinkBench` flags large position jumps; honest motion in PX4
doesn't trip it cleanly). The verified path doesn't depend on the
flag — what matters is `latched` + `rtl_dispatched`.

## What this proves

| Property | Evidence |
|---|---|
| Verified geofence trips on a real-vehicle position estimate | `latched: true` after the PX4 quad crosses the fence |
| Verified RTL dispatch chain reaches the actuator command code (`0xA17C`) | `rtl_dispatched: true` |
| The same code path the SITL + StubBench tests exercise also drives PX4-SITL | The only difference between `--backend=stub` (unit-tested) and `--backend=mavlink` (this) is which `FrameSource` supplies position frames — every line of the verified chain is identical |

## Where this stops

* No 3D-rendered geofence-violation video — that's PX4's GCS
  (QGroundControl) screenshot territory, not in this repo.
* No automated CI run — PX4-SITL is heavyweight (Gazebo needs a
  GPU; jMAVSim wants Java). CI keeps the StubBench negative + positive
  controls as the gate; PX4-SITL is the human-bench demo.
* No protection from PX4 itself ignoring the RTL command — falcon's
  RTL goes to its own command store; if you want PX4 to actually
  fly to home, you'd wire the relay-sc RTS output into a MAVLink
  COMMAND_LONG (`MAV_CMD_NAV_RETURN_TO_LAUNCH`). That's a v0.15-class
  follow-up (relay-mavlink would gain a downstream encoder for that
  command).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Harness verdict has `latched: false` after vehicle clearly moves | Wrong `--home=` (NED projection is offset; everything always inside) | Match PX4's takeoff coord; PX4 prints it at boot |
| `bind 0.0.0.0:14550: Address already in use` | QGC or another MAVLink consumer already on 14550 | Use a different port (`make px4_sitl jmavsim`'s default is 14550; in PX4 console: `mavlink stop-all && mavlink start -u 14560`) and pass `--listen=0.0.0.0:14560` |
| No frames at all (`spoof_first_seen_at_s: None`) | PX4 not emitting GLOBAL_POSITION_INT yet (boot incomplete) | Wait for `Ready for takeoff!` line in PX4 terminal |
