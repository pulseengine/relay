# The `pulseengine:falcon-cascade` transport binding, v2

**Requirement:** SWREQ-FALCON-TRANSPORT-P01 · **Reference implementation:**
`crates/falcon-hitl/src/wire.rs` (codec) and `crates/falcon-hitl/src/lib.rs`
(`BindingHost`, `SimServer`) · **Seam:** `pulseengine:falcon-cascade@0.12.0`
(the `Calibration` frame carries `pilot.sensor-calibration` from relay#611)

> **What changed from v1, in one revision so a host's decoder changes once**
> (relay#609 finding 9, #572, #527):
>
> 1. **`sensor` gains `battery-v`** — a presence byte at payload offset 76 and
>    an `f32` at 77. The payload grows **76 → 81 B**; every v1 offset is
>    unchanged. Presence 0 is **ABSENT**; a present 0.0 V is a measured dead
>    pack and is *not* absent.
> 2. **Three new host → component frames** — `supervisor-config` (type 4),
>    `calibration` (type 5) and `command` (type 6) — so a host can drive
>    `pilot.configure-supervisor`, `pilot.configure-calibration` and
>    `pilot.command` over the binding. v1 could carry `configure` and `step`
>    only, so the supervised `@0.12.0` component could never be armed over its
>    only supported binding: it stayed `disarmed` and wrote zero motors.
> 3. **Non-finite floats are rejected.** A receiver refuses a frame with NaN or
>    ±infinity in any `f32` slot (`WireError::NonFinite`). v1 passed them
>    through. Say "no reading" with the presence byte, never with NaN.
> 4. **Presence bytes are strictly 0 or 1.** A `sensor` presence byte of any
>    other value is refused (`WireError::BadBool`), like the `command`
>    interlocks. An earlier revision of v2 read any non-zero value as present
>    (found by an integrator's encoder probe — jess#167).
>
> 5. **Every reply echoes the `seq` of the frame it answers** (§3) — a
>    `motors` reply its `sensor` frame's, an `ack` its control frame's. No
>    layout change. The reference host **refuses** a reply that does not
>    (§3, "What the reference host enforces").
>
> `version` is now **2**. A v1 host is refused with `BadVersion { got: 1,
> want: 2 }`, which is the intended way to discover the change.

> **v2 was amended before release, and `version` STAYS 2.** v2 has never been
> tagged or shipped in a release, so the pre-release amendments below are part
> of v2 itself rather than a v3: items 4 and 5 above, and the `calibration`
> payload growing **60 → 72 B** (`mag-offdiag` appended at 60/64/68, every
> earlier offset unchanged). A host built against an earlier draft of v2 sees
> `BadLength { want: 72 }` on a calibration frame, `BadBool` on a presence
> byte above 1, and replies whose `seq` now names the frame they answer. The
> integrator review on jess#167 confirmed the v2 layout from the encoder's
> bytes and agreed to these changes inside v2. **From the first release that
> ships v2, any further payload change bumps `version`** (§1).

## Why this document exists

We shipped a verified flight component and no way to talk to it. An integrator
drove it over UDP against Gazebo and **had to invent the wire themselves**.
This is the published binding so the next one does not have to.

It is **normative**: where this document and any capture disagree, this document
is the binding and the capture is prior art. What the capture contributed is the
*header shape*, deliberately, so an existing host needs the smallest possible
change.

> **The capture this binding's header shape came from is not in the repository,
> and two of its numbers do not reconcile with the seam — tracked in #503.** §1
> states which parts of the framing are normative choices rather than
> measurements.

**The offset tables below are tested, not typed.** Each sits between
`wire-layout` markers, and `the_binding_document_offset_tables_match_the_encoder`
in `wire.rs` parses this file and checks every row against bytes the encoder
produced from a frame where every field holds a different value. A wrong,
missing, duplicated or shifted row fails the build.

## 1. Framing

Every message is an 8-byte header followed by a fixed-length payload.

<!-- wire-layout: header 8 -->
```text
 off  size  type  field
   0     1  u8    magic0    0x1c
   1     1  u8    magic1    0xfa
   2     1  u8    version   2
   3     1  u8    type      0=config 1=ack 2=sensor 3=motors 4=supervisor-config 5=calibration 6=command
   4     4  u32   seq       little-endian
```
<!-- /wire-layout -->

The payload follows at frame offset 8. All offsets in the payload tables below
are **payload** offsets; add 8 for the frame offset.

| type | id | direction | payload | frame |
|---|---|---|---|---|
| `config` | 0 | host → component | 36 B (`vehicle-config`, 9×f32) | 44 B |
| `ack` | 1 | component → host | 4 B (the acknowledged sequence) | 12 B |
| `sensor` | 2 | host → component | 81 B (`sensor-frame`) | 89 B |
| `motors` | 3 | component → host | 16 B (`motor-pwm`, 4×f32) | 24 B |
| `supervisor-config` | 4 | host → component | 24 B (`supervisor-config`, 6×f32) | 32 B |
| `calibration` | 5 | host → component | 72 B (`sensor-calibration`, 18×f32) | 80 B |
| `command` | 6 | host → component | 3 B (`pilot-command` + 2 interlocks) | 11 B |

**All integers and floats are little-endian.** Every field is byte-aligned;
there is no bit packing anywhere in this binding, deliberately — the one wire
decoder in this project that had bit-misaligned fields shipped garbage ESC
throttles past 40 tests and 9 proof harnesses.

### What a receiver rejects

Every rejection is a refusal of the whole frame; nothing is salvaged.

| condition | error |
|---|---|
| fewer than 8 bytes | `TooShort` |
| magic is not `1c fa` | `BadMagic` |
| version is not 2 | `BadVersion { got, want }` |
| type is not 0–6 | `UnknownType` |
| payload length is not the type's exact length | `BadLength` |
| **any `f32` slot is NaN or ±infinity** | **`NonFinite { msg, offset }`** |
| `command.cmd` is not 0–7 | `BadCommand` |
| `command.level` / `command.throttle-low` is not 0 or 1 | `BadBool` |
| **a `sensor` presence byte (payload 28, 41, 54, 59, 76) is not 0 or 1** | **`BadBool { msg, offset, got }`** |

When a frame breaks more than one rule, the error reported is the FIRST in
this order: the non-finite rule over the `f32` slots in field order, then the
presence bytes in offset order. (The Kani sensor harness proves that order.)

**The non-finite rule applies to every `f32` SLOT, present or absent.** An
optional field's value bytes are checked even when its presence byte is 0 (the
encoder writes 0.0 there). It does **not** apply to presence bytes or to the
`s32` `motor-rpm` values: an rpm of −8388608 is the bytes `00 00 80 ff`, which
is the −inf bit pattern, and it is a valid rpm. `offset` in the error is the
payload offset of the slot, so a host can name the field from the tables below.

**The interlock bytes are strictly 0 or 1.** `level` and `throttle-low` are RC
arming interlocks; reading a stray `0xab` as `true` would be the fail-unsafe
direction.

**So are the presence bytes.** Each is the 0/1 discriminant of an
`option`: 0 is ABSENT, 1 is present, and 2, `0x80` or `0xff` is a corrupted
byte — refused as `BadBool` naming its offset, never read as "present". An
earlier revision of v2 read any non-zero byte as present; an integrator's
encoder probe measured a presence byte of 2 decoding as a reading
(jess#167). One error variant
covers both kinds of boolean byte because the rule is the same and
`msg`/`offset` name which byte it was.

### `version` is the binding's version, not the WIT package's

Bump it when any payload layout changes. Do **not** bump it when
`pulseengine:falcon-cascade` changes version without changing bytes: a host
cares about layout, not names. A host receiving an unknown version must
**reject and report both numbers** — never attempt a partial parse.

### Two places this differs from the integrator's capture

Called out precisely so an existing host knows what to change.

1. **This binding's header is 8 bytes; the requirement says 6.** The
   requirement describes "a 6-byte header — magic `1c fa`, version, message
   type, little-endian `u32` sequence". **That field list sums to 8** (2+1+1+4),
   and the field list is self-contained — it does not depend on any captured
   number. So the label and the fields disagree, and we take the fields.

   The captured motors frame (24 B) minus `motor-pwm` (16 B) also leaves 8,
   which is consistent. **But that is corroboration, not proof**: the 24 comes
   from the same unsourced sentence as the "6" (#503), and the sensor figure
   from that same sentence does *not* reconcile (below). We are trusting the
   number that fits and setting aside the one that does not, which is an
   inference.

   **So 8 is a normative choice**, made because it is what the requirement's own
   field enumeration adds up to and because it keeps an existing host's header
   unchanged. It is not a measurement.
2. **The v1 sensor payload was 76 bytes; the capture implies 75.** A flat
   v1 `sensor-frame` is imu 24 + `dt-s` 4 + `position-ned` 13 + `mag-body` 13 +
   `heading-rad` 5 + `motor-rpm` 17 = 76, while the captured 83-byte frame
   leaves 75. The likeliest cause is that the integrator sent `heading-rad` as
   a bare `f32`, dropping its presence byte, because their rig always has one.
   **This is not established** — the capture is not in this repository (#503).
   This binding requires the presence byte. A host built from the capture must
   add one byte at offset 54 of the payload — and, at v2, the five-byte
   `battery-v` element at 76.

### Optional fields are fixed-width, not compact

Each optional field is a **presence byte followed by its value bytes**, and the
value bytes are always on the wire (zeroed when absent). The frame is therefore
the same size whether or not fields are set.

This costs bytes and buys three things: the MCU uses one static buffer; a
length disagreement is always an error rather than a legal short encoding; and
offsets are constant, so a capture is readable by eye.

**A presence byte of 0 means ABSENT, 1 means present, and nothing else is
legal** (§1, "What a receiver rejects"). **A decoder must surface absence as
absence — never as a zero measurement.** This project has shipped the opposite
twice: a battery whose absence read as a healthy pack (#413) and an
accelerometer whose absence read as gravity (#452). Both flew.

### Payload layouts

Field names are the WIT's (`vec3` components are `x y z`; for `position-ned`
and `home` that is north, east, down). Units and signs are the WIT's own —
read `wit/falcon-cascade/cascade.wit` for each field's definition rather than
inferring it from its neighbours.

#### `sensor` (type 2) — `types.sensor-frame`

<!-- wire-layout: sensor 81 -->
```text
 off  size  type  field
   0     4  f32   imu.ax                m/s², body
   4     4  f32   imu.ay
   8     4  f32   imu.az
  12     4  f32   imu.gx                rad/s, body
  16     4  f32   imu.gy
  20     4  f32   imu.gz
  24     4  f32   dt-s                  s — the host's ACTUAL period (§3)
  28     1  u8    position-ned.present  0 = ABSENT, 1 = present, else BadBool
  29     4  f32   position-ned.x        NED m (north)
  33     4  f32   position-ned.y        (east)
  37     4  f32   position-ned.z        (down)
  41     1  u8    mag-body.present
  42     4  f32   mag-body.x            any consistent unit
  46     4  f32   mag-body.y
  50     4  f32   mag-body.z
  54     1  u8    heading-rad.present
  55     4  f32   heading-rad           rad, NED
  59     1  u8    motor-rpm.present
  60     4  s32   motor-rpm.m1          motor-pwm order
  64     4  s32   motor-rpm.m2
  68     4  s32   motor-rpm.m3
  72     4  s32   motor-rpm.m4
  76     1  u8    battery-v.present     v2 (#572): 0 = ABSENT, 1 = present
  77     4  f32   battery-v             V, whole pack
```
<!-- /wire-layout -->

#### `config` (type 0) — `types.vehicle-config`

<!-- wire-layout: config 36 -->
```text
 off  size  type  field
   0     4  f32   hover-thrust
   4     4  f32   loop-rate-hz
   8     4  f32   pos-var
  12     4  f32   process-floor-vel
  16     4  f32   process-floor-pos
  20     4  f32   altitude-kp
  24     4  f32   altitude-kd
  28     4  f32   altitude-ki
  32     4  f32   position-ki
```
<!-- /wire-layout -->

#### `ack` (type 1)

<!-- wire-layout: ack 4 -->
```text
 off  size  type  field
   0     4  u32   acked-seq    the sequence of the frame acknowledged
```
<!-- /wire-layout -->

#### `motors` (type 3) — `types.motor-pwm`

<!-- wire-layout: motors 16 -->
```text
 off  size  type  field
   0     4  f32   m1    [0, 1]
   4     4  f32   m2
   8     4  f32   m3
  12     4  f32   m4
```
<!-- /wire-layout -->

#### `supervisor-config` (type 4) — `pilot.supervisor-config`

<!-- wire-layout: supervisor-config 24 -->
```text
 off  size  type  field
   0     4  f32   home.x          NED m (north) — geofence centre AND RTL point
   4     4  f32   home.y          (east)
   8     4  f32   home.z          (down)
  12     4  f32   fence-radius    m about home
  16     4  f32   cruise-alt      m AGL, POSITIVE — NOT NED-signed
  20     4  f32   low-batt-v      V, whole pack (not per cell)
```
<!-- /wire-layout -->

**`cruise-alt` is positive-up**, unlike every other altitude in the seam: the
supervisor negates it itself. Sending −30.0 to mean "30 m up" commands 30 m
*below* the ground.

#### `calibration` (type 5) — `pilot.sensor-calibration` (relay#611)

<!-- wire-layout: calibration 72 -->
```text
 off  size  type  field
   0     4  f32   gyro-bias.x      rad/s, subtracted
   4     4  f32   gyro-bias.y
   8     4  f32   gyro-bias.z
  12     4  f32   accel-bias.x     m/s², subtracted
  16     4  f32   accel-bias.y
  20     4  f32   accel-bias.z
  24     4  f32   accel-scale.x    unitless, > 0, applied after de-biasing
  28     4  f32   accel-scale.y
  32     4  f32   accel-scale.z
  36     4  f32   mag-offset.x     same unit as sensor.mag-body, subtracted
  40     4  f32   mag-offset.y
  44     4  f32   mag-offset.z
  48     4  f32   mag-scale.x      unitless, > 0
  52     4  f32   mag-scale.y
  56     4  f32   mag-scale.z
  60     4  f32   mag-offdiag.xy   unitless, symmetric soft-iron off-diagonal
  64     4  f32   mag-offdiag.xz
  68     4  f32   mag-offdiag.yz
```
<!-- /wire-layout -->

The identity calibration (biases 0, scales 1, off-diagonals 0) means **not
calibrated**: the component's `CalibrationPresent` pre-arm row is
`calibration != identity`.

**`mag-offdiag` (payload 60/64/68) is appended**, so every earlier offset is
unchanged and the frame grew 60 → 72 B. With `mag-scale` as the diagonal it
completes a **symmetric 3×3 soft-iron matrix** — the shape common autopilot
calibrations produce (PX4 calls the off-diagonal `CAL_MAGn_ODIAG`) — which a
diagonal-only `mag-scale` cannot express. Every slot is checked by the
non-finite rule like any other `f32`.

> **Order:** the components are `[xy, xz, yz]`, the same order as the
> `mag-offdiag` field of `pilot.sensor-calibration` and of
> `relay_calib::CalParams` (relay#611), which is PX4's
> `CAL_MAGn_XODIAG / YODIAG / ZODIAG` in `Magnetometer::set_offdiagonal`'s
> order. The native test endpoint forwards all six fields, so the off-diagonal
> soft-iron correction is applied there too.

#### `command` (type 6) — `pilot.command(cmd, level, throttle-low)`

<!-- wire-layout: command 3 -->
```text
 off  size  type  field
   0     1  u8    cmd             pilot-command, WIT declaration order (below)
   1     1  u8    level           0/1 — RC interlock: vehicle level
   2     1  u8    throttle-low    0/1 — RC interlock: throttle stick low
```
<!-- /wire-layout -->

| `cmd` | `pilot-command` |
|---|---|
| 0 | `arm` |
| 1 | `request-takeoff` |
| 2 | `request-mission` |
| 3 | `request-loiter` |
| 4 | `request-rtl` |
| 5 | `request-land` |
| 6 | `request-disarm` |
| 7 | `terminate` |

## 2. The configuration handshake

`vehicle-config` is **load-bearing, not decoration**. Measured on the gz
falcon-quad: a wrong `hover-thrust` alone put the altitude estimate **58.8 m**
out, and with default tuning the vehicle **never left the ground**.

The four CONTROL frames — `config`, `supervisor-config`, `calibration` and
`command` — share one stop-and-wait handshake:

1. The host sends the control frame with sequence `s`.
2. The component replies `ack` carrying `s`.
3. The host **must not** send `sensor` frames, or another control frame, until
   that ack arrives. (`BindingHost` in the reference implementation enforces
   this: it returns `AwaitingAck` instead of the bytes.)

**An ack means RECEIVED, not APPLIED.** The component may discard a
configuration it is not in a state to take — relay#611 makes `configure`,
`configure-supervisor` and `configure-calibration` silently discarded unless
the supervisor is `disarmed`, and makes `arm` refused until the pre-arm table
passes. The binding has no frame that reports the outcome (see the gaps
below); a host learns it from the motors it gets back.

**A frame rejected by the decoder is not acked.** For `supervisor-config` and
`calibration` that rejection is also a statement: the latest thing the host
sent was invalid. Per relay#611's semantics for those two calls, a component
must then behave as if the host had sent an invalid configuration — the
supervisor configuration is dropped (arming refused on `FailsafeConfigured`)
and the calibration is replaced by the identity (arming refused on
`CalibrationPresent`) — rather than keep flying on the previous one. Through
#611's API that is: forward a `configure-calibration` with the identity (or
any invalid) calibration, or a `configure-supervisor` with an invalid
configuration. *(This rule reconciles #527 with #611 and is pending
maintainer confirmation; the native test endpoint implements it.)*

The order a host follows to fly the supervised component:

1. `config` → ack
2. `supervisor-config` → ack
3. `calibration` → ack
4. `sensor` frames, with `battery-v` present, until the estimator converges
5. `command` `arm` (with `level` and `throttle-low` true) → ack
6. `command` `request-takeoff` → ack, then keep sending `sensor` frames

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

**The component clamps `dt-s` to [0.0001, 0.1] s** (10 Hz – 10 kHz) so a garbage
frame cannot wind the filters. Silently — a host running outside that range is
not told that its period was overridden, so do not rely on the clamp as a
diagnostic. **A non-finite `dt-s` never reaches the component over this
binding:** the frame is rejected with `NonFinite { offset: 24 }`. (The
component's own fallback to 0.001 s for a non-finite value remains for callers
of the WIT interface that do not go through this binding.)

### Late, dropped and duplicated

**The host owns the sequence.** The host's `seq` increments by one for every
frame it sends — `sensor` and control frames alike, one counter. The
component does not number anything of its own.

**The echo rule: a reply's `seq` is the `seq` of the frame it answers.**

- A **`motors`** reply carries the `seq` of the **`sensor` frame it answers**.
  No layout change: it is the header `seq`.
- An **`ack`** carries the `seq` of the **control frame it acknowledges** — in
  its header, and (unchanged from earlier revisions) in its `acked-seq`
  payload, so both say the same thing.

So the component → host stream is the host's own sequence space, echoed. A
host indexes each reply to its tick by `seq`, not by arrival order — which is
the only correlation that survives loss: once one frame is lost, matching
replies to ticks by order is guesswork. A gap in the reply stream is a request
or a reply that was lost (`Dropped`); a host that sent `sensor` 41, 42, 43 and
gets `motors` 41 and 43 back knows tick 42 has no answer. (This rule was an
open point in an earlier draft of v2; it is pinned here. The integrator review
on jess#167 asked for exactly this.)

**What the reference host enforces** (`BindingHost::on_reply`; #614 — an
integrator's probe on jess#167 found the rule stated here but unchecked, and a
`motors` reply carrying a seq the host never sent was accepted as three lost
ticks). A reply that breaks the rule is **refused and changes nothing**:

| reply | accepted when | otherwise |
|---|---|---|
| `motors` seq `s` | `s` is the **outstanding** `sensor` frame — the newest one the host sent, not yet answered | refused, `StrayMotors` |
| `motors` seq `s` | `s` is the `motors` accepted last | accepted as `Duplicate`; do not act twice |
| `ack` | header `seq` == `acked-seq`, and it is the control frame awaited | refused, `AckSeqMismatch` or `UnexpectedAck` |
| `ack` | header `seq` == `acked-seq` == the ack accepted last | accepted as a duplicate ack |

`StrayMotors` says which way the reply fails: `Unsolicited` (no `sensor` frame
is outstanding), `NeverSent` (one is, but this seq never went on the wire), or
`Superseded` (the host sent this seq, but it is a `sensor` frame since
superseded by a newer one, or a control frame).

**Only the newest `sensor` frame is answerable.** A host that sends tick
`n + 1` before tick `n`'s reply arrived has given tick `n` up: a late reply to
`n` is refused, and `n` is counted in the `Dropped { lost }` of `n + 1`'s
reply. So `lost` counts frames the host **sent** whose replies it never
accepted — never more. The host's tracker starts at the host's own first
`seq`, so loss before the first accepted reply is reported too: `sensor` 1, 2,
3 out and only `motors` 3 back reads `Dropped { lost: 2 }`, not `First`. An
accepted `ack` advances the same baseline, so an acknowledged control frame
between two ticks is not counted lost. One consequence to know: if a control
frame is sent while a tick is outstanding and its ack arrives first, the
tick's `motors` reply is accepted but reads `Late` — the table below says
discard it.

**What a component does with a frame it will not act on.** A `Duplicate` or
`Late` `sensor` frame never steps the controller. The component may leave it
unanswered (the native test endpoint does) or answer it with the same `seq`
(which the host then classifies as `Duplicate`); it must never answer it with a
`seq` of its own. A `Duplicate` control frame is re-acked with its own `seq`
and **not re-applied**.

**The same rule, other polarity.** In `falcon-hitl`'s `LinkBackend` /
`SimServer` pair the flight computer leads with `motors` and the plant answers
with `sensor`; there the `sensor` reply echoes the `motors` frame's `seq`. A
reply echoes its request, whichever side asks. `LinkBackend` enforces it: the
exchange is synchronous, so a `sensor` reply whose `seq` is not the `motors`
frame just sent is refused — not cached, not classified, counted in
`misechoed_replies()` — and the next correctly echoed reply's
`Dropped { lost }` counts the exchanges in between. (`SimServer` is the side
that is asked, not the side that asks, so it has no reply to check; it
classifies the requests it receives with `SeqTracker` as the table below
says.)

Each receiver classifies the frames it receives against the last it accepted —
`SeqTracker` in the reference implementation:

| condition | verdict | what the receiver must do |
|---|---|---|
| first frame seen | `First` | accept |
| `seq == last + 1` | `InOrder` | accept, step once |
| `seq == last` | `Duplicate` | accept the bytes, **do not step the controller twice** |
| `seq > last` (forward) | `Dropped { lost }` | accept, and treat `lost` ticks as missing |
| `seq < last` (backward) | `Late { by }` | **discard**; a newer frame already superseded it |

A repeated `command` must not deliver its command twice.

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
| `battery-v` | yes | **the vehicle cannot arm** — the `BatteryOk` pre-arm row is false while no pack has been seen |

Two of those are measured, not predicted:

- **Position fixes at ≤ 10 Hz made the hold diverge** (#434/#403); the
  integrator's rig ran 5 Hz and oscillated. **Supply ≥ 10 Hz.** The estimator
  fix at v1.140/v1.141 improves this; it does not remove the obligation.
- **Withholding `motor-rpm` leaves the FDI inert.** Measured on gz: with it,
  the component isolates a dead rotor in **2 ticks** and stays upright at
  0.18–0.38 rad. Without it, the airframe **inverts to 3.142 rad** and isolates
  nothing. *(Evidence: the wasm rotor-out harness in #502 and its bench-evidence
  finding, which land separately — this document cites a measurement whose
  record is not yet on `main`.)*

## 5. The reference host

`crates/falcon-hitl/src/wire.rs` — `no_std`, no-alloc, fixed buffers, so the
same codec runs on the MCU and on a host PC. `crates/falcon-hitl/src/lib.rs`
carries the host side: `BindingHost` (sequencing, the handshake, reply
classification and the echo rule it enforces) and `SimServer` /
`sample_sensor_frame` (a plant's sensors, battery included, as a `sensor`
frame).

`crates/falcon-hitl/tests/binding_drives_the_supervisor.rs` is the executable
form of §2: a host that exchanges **only bytes** with a native
`FlightSupervisor` behind the binding configures it, calibrates it, warms the
estimator, arms and takes off — and the same run with the calibration frame,
the battery element or the supervisor configuration withheld is refused on the
named pre-arm row.

**`falcon-hitl`'s legacy 16/54-byte frames are SUPERSEDED by this binding** and
remain only so existing callers keep building. They cannot carry heading, rotor
RPM, or a battery that says ABSENT — exactly the fields the seam learned it
needed by being driven. New hosts use `wire`.

## What this binding does not yet give you

- **No state frame.** `observer.read-state` (mode, failsafe, the estimate) has
  no frame here, so a binding-only host cannot see `mode` or the `arm-blocked`
  failsafe bit: it learns that an `arm` was refused only because the motors
  stay at zero. The `ack` deliberately says "received", not "accepted".
- **No mission target.** `controller.step` takes a `waypoint` beside the
  `sensor-frame`, and the `sensor` frame does not carry one, so a binding host
  cannot set a mission leg. Takeoff, loiter, land and RTL need none (the
  supervisor derives those setpoints from `home` and `cruise-alt`).
- **The published wasm component is not yet behind a byte endpoint.** The
  end-to-end test above drives the native `FlightSupervisor`; the
  `calibration` frame's target, `pilot.configure-calibration`, exists only once
  relay#611 merges.
- **No physical-transport contract.** Latency, jitter, MTU, reconnect and
  framing-error recovery are the host's. The sequence semantics above are what
  this binding gives you to detect the symptoms.
