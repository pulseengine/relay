#!/usr/bin/env python3
"""Generate authoritative DroneCAN v0 reference vectors for relay-dronecan tests.

Uses pydronecan (the reference implementation PX4/ArduPilot's libcanard conforms
to) to produce canonical on-wire payloads for the messages relay-dronecan
decodes. These vectors are embedded as `CONFORMANCE` tests in the message modules
(msg.rs / sensors.rs) — the external oracle that caught the LSB-first bit-order
bug a self-constructed round-trip test could not.

Setup:  python3 -m venv /tmp/dcvec && /tmp/dcvec/bin/pip install dronecan
Run:    /tmp/dcvec/bin/python scripts/gen-dronecan-vectors.py

Each line: <message> <hex payload> <field values>. The decoder must reproduce
the field values from the hex; the encoder (where falcon emits) must reproduce
the hex from the field values.
"""
from dronecan import uavcan


def wire(m):
    """Canonical on-wire payload (TAO serialization for the top-level message)."""
    b = m._pack(True)
    b += "0" * ((8 - len(b) % 8) % 8)
    return bytes(int(b[i:i + 8], 2) for i in range(0, len(b), 8))


def rt(cls, by):
    """Round-trip: decode the bytes back, to confirm they are canonical."""
    m = cls()
    m._unpack("".join(f"{x:08b}" for x in by), True)
    return m


def emit(tag, by, desc):
    print(f"{tag:14s} {by.hex():46s} {desc}")


# uavcan.protocol.NodeStatus (341) — sub-byte fields health/mode/sub_mode
m = uavcan.protocol.NodeStatus()
m.uptime_sec, m.health, m.mode, m.sub_mode, m.vendor_specific_status_code = 0x01020304, 2, 3, 1, 0xBEEF
emit("node_status", wire(m), "uptime=0x01020304 health=2 mode=3 sub_mode=1 vendor=0xBEEF")

# uavcan.equipment.esc.RawCommand (1030) — int14 array (FLIGHT-CRITICAL, falcon emits)
m = uavcan.equipment.esc.RawCommand()
m.cmd = [8191, 0, -8192, 4096]
emit("raw_command", wire(m), "cmd=[8191, 0, -8192, 4096]")

# uavcan.equipment.esc.Status (1034) — float16 + int18 rpm bit-field
m = uavcan.equipment.esc.Status()
m.error_count, m.voltage, m.current, m.temperature = 7, 10.0, 1.0, 290.0
m.rpm, m.power_rating_pct, m.esc_index = 5000, 50, 3
emit("esc_status", wire(m), "error_count=7 V=10.0 I=1.0 T=290.0 rpm=5000 power_rating=50 esc_index=3")

# uavcan.equipment.ahrs.MagneticFieldStrength (1002) — float16[3] from byte 0, NO ahrs_id
m = uavcan.equipment.ahrs.MagneticFieldStrength()
m.magnetic_field_ga = [1.0, -2.0, 0.5]
emit("mag", wire(m), "field_ga=[1.0, -2.0, 0.5]")

# uavcan.equipment.air_data.StaticPressure (1028) — float32 + float16
m = uavcan.equipment.air_data.StaticPressure()
m.static_pressure, m.static_pressure_variance = 101325.0, 1.0
emit("baro", wire(m), "pressure=101325.0 variance=1.0")

# uavcan.equipment.power.BatteryInfo (1092) — prefix temperature/voltage/current
m = uavcan.equipment.power.BatteryInfo()
m.temperature, m.voltage, m.current = 300.0, 22.2, 5.0
emit("battery", wire(m), "temperature=300.0 voltage=22.2 current=5.0 (prefix)")
