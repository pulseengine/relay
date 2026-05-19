# falcon-hello

**falcon v0.1 example — MAVLink heartbeat exchange over UDP loopback.**

The minimum viable falcon flight stack: boots, encodes a HEARTBEAT
message using `relay-mavlink`, sends it over UDP, and decodes the
inbound stream from a peer.

This demonstrates the v0.1 pipeline end-to-end: WIT types →
`relay-mavlink` codec → real socket I/O → `relay-ekf-stub` tick
→ round-trip parse. Real MAVLink wire format, real CRC, real
QGroundControl-compatible bytes.

## Run

In one terminal:

```sh
cargo run -p falcon-hello -- --mode gcs
```

In another:

```sh
cargo run -p falcon-hello -- --mode vehicle
```

Vehicle emits a HEARTBEAT every second; GCS receives and decodes
them. Output looks like:

```
vehicle: tx seq=0 type=2 status=3 mavlink_v=2 (21 bytes)
gcs: rx heartbeat from 127.0.0.1:14551 type=2 autopilot=8 status=3 custom_mode=0
```

Bounded run:

```sh
cargo run -p falcon-hello -- --mode vehicle --duration 10
cargo run -p falcon-hello -- --mode gcs --duration 10
```

## Connect to QGroundControl

The default ports are MAVLink-conventional, so QGroundControl on the
same host should auto-detect the vehicle:

```sh
cargo run -p falcon-hello -- --mode vehicle --remote 127.0.0.1:14550
```

QGC listens on 14550 by default; vehicle binds 14551 and targets
14550. With QGC running, the vehicle appears in the QGC vehicle
list within a few seconds (per SWREQ-FALCON-MAVLINK-P02).

The autopilot field reports `8` (MAV_AUTOPILOT_INVALID) because
falcon does not yet have a dedicated MAV_AUTOPILOT value registered
upstream. Status field is `3` (MAV_STATE_STANDBY). Vehicle class
is `2` (MAV_TYPE_QUADROTOR).

## What's tested

```sh
cargo test -p falcon-hello
```

includes:

- **`vehicle_and_gcs_exchange_heartbeats_over_udp`** — integration
  test that runs both modes against each other on real UDP sockets
  (ephemeral ports) and verifies the round trip.
- **`handle_inbound_rejects_unsupported_message`** — proves the
  dispatcher returns the right typed error for messages outside
  the v0.1 scope (no panic, no misparse).
- **`handle_inbound_propagates_bad_crc`** — corrupted CRC byte ⇒
  `CodecError::BadCrc`.
- **`handle_inbound_truncated`** — short buffer ⇒ `Truncated`.
- **`args_*`** — CLI parsing for the four documented invocations.

The underlying `relay-mavlink` crate (`cargo test -p relay-mavlink`)
adds 33 more tests covering the CRC algorithm against
CRC-16/MCRF4XX reference vectors, the HEARTBEAT payload encoding
against the MAVLink-canonical field order, and round-trip property
tests over arbitrary inputs.

## What this is NOT

- **Not a flight stack yet.** v0.1 doesn't actually fly anything —
  no controllers, no sensor drivers, no SITL hookup. That lands in
  v0.2 (ekf) through v0.5 (full waypoint flight). See
  [`relay/falcon/README.md`](../../falcon/README.md) for the
  release plan.
- **Not signed/AOT-compiled yet.** The example runs as a plain
  std binary. sigil signing + synth AOT compilation come with the
  hardware bring-up release (v0.6).
- **Not a WASM component yet.** The WIT files exist
  (`wit/worlds/relay-falcon.wit`) but `wit-bindgen` integration is
  gated until the relay-substrate's P3 streams land.

## Files

```
examples/falcon-hello/
├── Cargo.toml
├── README.md            (this file)
└── src/
    └── main.rs          (the CLI + vehicle/GCS loops + tests)
```

Underlying dependencies:

```
crates/relay-mavlink/    (no_std v2 codec — CRC, heartbeat, frame)
crates/relay-ekf-stub/   (no_std stub state-estimator)
wit/interfaces/relay-mavlink/protocol.wit
wit/interfaces/relay-control/dynamics.wit
wit/worlds/relay-falcon.wit
```

## Falcon release table

This example is the deliverable for **v0.1**.

Tracked at [`artifacts/features/FEAT-FALCON-rollout.yaml`](../../artifacts/features/FEAT-FALCON-rollout.yaml).

```
rivet list --type feature --tag falcon
```
