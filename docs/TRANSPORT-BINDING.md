# The `pulseengine:falcon-cascade` transport binding, v1

**Requirement:** SWREQ-FALCON-TRANSPORT-P01 · **Reference implementation:**
`crates/falcon-hitl/src/wire.rs` · **Seam:** `pulseengine:falcon-cascade@0.11.0`

## Why this document exists

We shipped a verified flight component and no way to talk to it. An integrator
drove it over UDP against Gazebo and **had to invent the wire themselves**.
This is the published binding so the next one does not have to.

It is **normative**: where this document and any capture disagree, this document
is the binding and the capture is prior art. What the capture contributed is the
*header shape*, deliberately, so an existing host needs the smallest possible
change.

## 1. Framing

Every message is an 8-byte header followed by a fixed-length payload.

```text
 off  size  field
   0     2  magic        0x1c 0xfa
   2     1  version      1
   3     1  message type 0=config 1=ack 2=sensor 3=motors
   4     4  sequence     u32, little-endian
   8     N  payload      the seam's own record, flat little-endian
```

| type | direction | payload | frame |
|---|---|---|---|
| `config` | host → component | 36 B (`vehicle-config`, 9×f32) | 44 B |
| `ack` | component → host | 4 B (the acknowledged sequence) | 12 B |
| `sensor` | host → component | 76 B (`sensor-frame`) | 84 B |
| `motors` | component → host | 16 B (`motor-pwm`, 4×f32) | 24 B |

**All integers and floats are little-endian.** Every field is byte-aligned;
there is no bit packing anywhere in this binding, deliberately — the one wire
decoder in this project that had bit-misaligned fields shipped garbage ESC
throttles past 40 tests and 9 proof harnesses.

### `version` is the binding's version, not the WIT package's

Bump it when any payload layout changes. Do **not** bump it when
`pulseengine:falcon-cascade` changes version without changing bytes: a host
cares about layout, not names. A host receiving an unknown version must
**reject and report both numbers** — never attempt a partial parse.

### Two places this differs from the integrator's capture

Called out precisely so an existing host knows what to change.

1. **The header is 8 bytes, not 6.** The requirement describes "a 6-byte
   header — magic `1c fa`, version, message type, little-endian `u32`
   sequence", but that field list sums to 8 (2+1+1+4). The capture's own frame
   sizes settle it: the motors frame was **24 B** and the seam's `motor-pwm` is
   **16 B**, so the header is exactly **8**. The label was wrong, not the
   fields.
2. **The sensor payload is 76 bytes; the capture implies 75.** A flat
   `sensor-frame` is imu 24 + `dt-s` 4 + `position-ned` 13 + `mag-body` 13 +
   `heading-rad` 5 + `motor-rpm` 17 = 76, while the captured 83-byte frame
   leaves 75. The likeliest cause is that the integrator sent `heading-rad` as
   a bare `f32`, dropping its presence byte, because their rig always has one.
   **This is not established** — the capture is not in this repository. This
   binding requires the presence byte. A host built from the capture must add
   one byte at offset 54 of the payload.

### Optional fields are fixed-width, not compact

Each optional field is a **presence byte followed by its value bytes**, and the
value bytes are always on the wire (zeroed when absent). The frame is therefore
the same size whether or not fields are set.

This costs bytes and buys three things: the MCU uses one static buffer; a
length disagreement is always an error rather than a legal short encoding; and
offsets are constant, so a capture is readable by eye.

**A presence byte of 0 means ABSENT, and a decoder must surface that as
absence — never as a zero measurement.** This project has shipped the opposite
twice: a battery whose absence read as a healthy pack (#413) and an
accelerometer whose absence read as gravity (#452). Both flew.

#### `sensor` payload layout

```text
 off  size  field
   0    24  imu            ax ay az gx gy gz (f32)
  24     4  dt-s           f32 — the host's ACTUAL period (see §3)
  28     1  position-ned present
  29    12  position-ned   n e d (f32)
  41     1  mag-body present
  42    12  mag-body       x y z (f32)
  54     1  heading-rad present
  55     4  heading-rad    f32
  59     1  motor-rpm present
  60    16  motor-rpm      m1..m4 (s32)
       ---
        76
```

## 2. The configuration handshake

`vehicle-config` is **load-bearing, not decoration**. Measured on the gz
falcon-quad: a wrong `hover-thrust` alone put the altitude estimate **58.8 m**
out, and with default tuning the vehicle **never left the ground**.

1. The host sends `config` with sequence `s`.
2. The component applies it and replies `ack` carrying `s`.
3. The host **must not** send `sensor` frames until that ack arrives.

`configure` may be called again later. Doing so **re-initialises the control
core, estimator state included**, because `loop-rate-hz` is fixed at
construction and a partially-applied configuration is worse than an explicit
reset. A host that never configures keeps the built-in defaults, which fly the
SITL mock plant and nothing else.

## 3. The tick contract

**`dt-s` is the host's actual period for this tick, not its nominal one.** This
clause exists because stages disagreed about the rate by up to **20×** and the
hold error went from 0.00 m to **27.17 m** with no error signal of any kind.
The same class of defect cost this project 2.5 months of mis-attribution when a
60 Hz filter designed for 1 kHz was fed at 250 Hz: the component now rebuilds
its rate-dependent filters when `dt-s` disagrees with its design rate by more
than 2%, so an honest `dt-s` is what makes that correction possible. **A host
that reports a nominal period while running a different one defeats it.**

### Late, dropped and duplicated

The sequence increments by one per message in each direction. A receiver
classifies each frame against the last it accepted — `SeqTracker` in the
reference implementation:

| condition | verdict | what the receiver must do |
|---|---|---|
| first frame seen | `First` | accept |
| `seq == last + 1` | `InOrder` | accept, step once |
| `seq == last` | `Duplicate` | accept the bytes, **do not step the controller twice** |
| `seq > last` (forward) | `Dropped { lost }` | accept, and treat `lost` ticks as missing |
| `seq < last` (backward) | `Late { by }` | **discard**; a newer frame already superseded it |

**`Late` and `Dropped` are distinct, with counts.** The requirement's
falsification condition says so explicitly: it is wrong "if a host that
satisfies the binding cannot tell a dropped tick from a late one". Reordering is
real on UDP, and a late frame is not a lost one.

**A `Late` or `Duplicate` frame does not advance `last`.** Otherwise one
reordered datagram makes every subsequent in-order frame look like a large drop.

**Wrapping is handled, not assumed away.** A `u32` sequence at 250 Hz wraps
after ~198 days. The comparison uses the half-space convention — a wrapped
difference below 2³¹ is forward, at or above is backward — so `0` arriving
after `u32::MAX` is one in-order tick, not a four-billion-frame drop.

## 4. What the host owes: the aiding-rate obligation

The component fuses what it is given. Absence is legal and **degradation is the
host's to accept**:

| field | optional? | what happens when absent |
|---|---|---|
| `imu` | **no** | required every tick |
| `dt-s` | **no** | required every tick (§3) |
| `position-ned` | yes | dead reckoning only; the hold drifts without bound |
| `mag-body` | yes | contributes no heading reference |
| `heading-rad` | yes | **yaw becomes unobservable** |
| `motor-rpm` | yes | the rotor-out FDI is **inert** — a dead rotor is never isolated |

Two of those are measured, not predicted:

- **Position fixes at ≤ 10 Hz made the hold diverge** (#434/#403); the
  integrator's rig ran 5 Hz and oscillated. **Supply ≥ 10 Hz.** The estimator
  fix at v1.140/v1.141 improves this; it does not remove the obligation.
- **Withholding `motor-rpm` leaves the FDI inert.** Measured on gz: with it,
  the component isolates a dead rotor in **2 ticks** and stays upright at
  0.18–0.38 rad. Without it, the airframe **inverts to 3.142 rad** and isolates
  nothing.

## 5. The reference host

`crates/falcon-hitl/src/wire.rs` — `no_std`, no-alloc, fixed buffers, so the
same codec runs on the MCU and on a host PC.

**`falcon-hitl`'s legacy 16/54-byte frames are SUPERSEDED by this binding** and
remain only so existing callers keep building. They cannot carry heading, rotor
RPM, or a battery that says ABSENT — exactly the fields the seam learned it
needed by being driven. New hosts use `wire`.

## What this binding does not yet give you

- **No mode and no failsafe.** The component wraps `FlightCore`, not
  `FlightSupervisor`, so there is no mode machine or failsafe latch to publish
  (#414). A consumer gets the estimate and **cannot see why** the vehicle is
  doing something — so it cannot report a failsafe as `STATUSTEXT` even though
  the MAVLink stack that would carry it is verified (MAVLINK-P06, v1.119).
  This is the binding's largest gap and it closes with #414.
- **`SimServer` is not yet re-pointed** at this binding; it still speaks the
  legacy frames.
- **No physical-transport contract.** Latency, jitter, MTU, reconnect and
  framing-error recovery are the host's. The sequence semantics above are what
  this binding gives you to detect the symptoms.
