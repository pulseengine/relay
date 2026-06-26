#!/usr/bin/env python3
"""Generate authoritative MAVLink PARAM reference vectors for relay-mavlink tests.

Uses pymavlink (the reference implementation QGroundControl / MAVSDK / PX4 /
ArduPilot all conform to) to produce canonical on-wire PAYLOADS + the per-message
CRC_EXTRA for the four parameter-protocol messages relay-mavlink binds:

  PARAM_REQUEST_LIST (21), PARAM_REQUEST_READ (20), PARAM_SET (23), PARAM_VALUE (22)

These vectors are embedded as CONFORMANCE tests in crates/relay-mavlink/plain/
src/param.rs — the external oracle, the same discipline that caught the DroneCAN
LSB-first bit-order bug a self-constructed round-trip test could not. MAVLink's
"sorted by size descending" wire order and the CRC_EXTRA magic numbers are exactly
the constants that must come from the reference, not from memory.

Setup:  python3 -m venv /tmp/mavvec && /tmp/mavvec/bin/pip install pymavlink
Run:    python3 scripts/gen-mavlink-param-vectors.py   (if pymavlink is on PATH)

Each block prints: <message> id=<n> crc_extra=<n> len=<n> payload=<hex> + fields.
The decoder must reproduce the field values from the hex; the encoder (where
falcon emits PARAM_VALUE) must reproduce the hex from the field values.
"""
from pymavlink.dialects.v20 import common as m


def payload_hex(msg):
    """Canonical FIXED-LENGTH payload (re-inflate MAVLink2 trailing-zero truncation).

    pymavlink's MAVLink2 pack() truncates trailing zero bytes; the wire/decoder
    contract is the full declared payload, so re-pad to unpacker.size.
    """
    full = msg.pack(MAV)
    plen = full[1]                       # byte 1 = (possibly truncated) payload len
    payload = full[10:10 + plen]         # MAVLink2 header is 10 bytes
    payload += b"\x00" * (msg.unpacker.size - len(payload))
    return payload


# A dummy MAVLink object just to drive pack() (seq/sysid/compid don't affect payload).
MAV = m.MAVLink(file=None, srcSystem=255, srcComponent=190)

PARAM = b"MC_ROLL_P"            # 9 chars; param_id is char[16] NUL-padded on the wire
MAV_PARAM_TYPE_REAL32 = 9      # MAV_PARAM_TYPE enum value for f32

msgs = [
    ("PARAM_REQUEST_LIST",
     m.MAVLink_param_request_list_message(target_system=1, target_component=1)),
    ("PARAM_REQUEST_READ",
     m.MAVLink_param_request_read_message(
         target_system=1, target_component=1, param_id=PARAM, param_index=-1)),
    ("PARAM_SET",
     m.MAVLink_param_set_message(
         target_system=1, target_component=1, param_id=PARAM,
         param_value=8.0, param_type=MAV_PARAM_TYPE_REAL32)),
    ("PARAM_VALUE",
     m.MAVLink_param_value_message(
         param_id=PARAM, param_value=8.0, param_type=MAV_PARAM_TYPE_REAL32,
         param_count=2, param_index=0)),
]

for name, msg in msgs:
    hexs = payload_hex(msg).hex()
    print(f"{name} id={msg.get_msgId()} crc_extra={msg.crc_extra} "
          f"len={msg.unpacker.size} payload={hexs}")
    for f in msg.ordered_fieldnames:
        print(f"    {f} = {getattr(msg, f)!r}")
