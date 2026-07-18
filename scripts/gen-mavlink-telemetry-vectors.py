#!/usr/bin/env python3
"""Authoritative MAVLink TELEMETRY reference vectors for relay-mavlink (v1.119).

Same discipline as gen-mavlink-param-vectors.py (the external-oracle rule that
caught the DroneCAN bit-order bug): pymavlink — the reference implementation —
produces the canonical payload bytes + CRC_EXTRA for the six messages the
MAVLINK-P06 telemetry stream emits:

  SYS_STATUS (1), GPS_RAW_INT (24), ATTITUDE (30), SERVO_OUTPUT_RAW (36),
  VFR_HUD (74), STATUSTEXT (253)

The relay-mavlink ENCODERS must reproduce these hex payloads from the field
values; the CRC_EXTRA constants must come from here, never from memory.

Run: python3 scripts/gen-mavlink-telemetry-vectors.py
"""
from pymavlink.dialects.v20 import common as m


def payload_hex(msg):
    mav = m.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.seq = 0
    packed = msg.pack(mav)
    payload = packed[10:-2]  # strip MAVLink2 header (10) + checksum (2)
    size = msg.unpacker.size
    payload = payload + b"\x00" * (size - len(payload))  # re-inflate truncation
    return payload.hex()


def show(name, msg):
    print(f"== {name} id={msg.id} crc_extra={msg.crc_extra} len={msg.unpacker.size}")
    print(f"   payload={payload_hex(msg)}")
    for f in msg.fieldnames:
        print(f"   {f} = {getattr(msg, f)}")
    print()


show("ATTITUDE", m.MAVLink_attitude_message(
    time_boot_ms=123456, roll=0.1, pitch=-0.05, yaw=1.5708,
    rollspeed=0.01, pitchspeed=-0.02, yawspeed=0.5))

show("SYS_STATUS", m.MAVLink_sys_status_message(
    onboard_control_sensors_present=0x3F, onboard_control_sensors_enabled=0x3F,
    onboard_control_sensors_health=0x3F, load=250, voltage_battery=15400,
    current_battery=1250, battery_remaining=87, drop_rate_comm=0,
    errors_comm=0, errors_count1=0, errors_count2=0, errors_count3=0,
    errors_count4=0))

show("GPS_RAW_INT", m.MAVLink_gps_raw_int_message(
    time_usec=1234567890, fix_type=3, lat=473977000, lon=85456000,
    alt=488000, eph=120, epv=180, vel=250, cog=9000, satellites_visible=14))

show("SERVO_OUTPUT_RAW", m.MAVLink_servo_output_raw_message(
    time_usec=1234567890, port=0,
    servo1_raw=1500, servo2_raw=1520, servo3_raw=1480, servo4_raw=1510,
    servo5_raw=0, servo6_raw=0, servo7_raw=0, servo8_raw=0))

show("VFR_HUD", m.MAVLink_vfr_hud_message(
    airspeed=0.0, groundspeed=2.5, heading=90, throttle=58,
    alt=2.0, climb=-0.5))

show("STATUSTEXT", m.MAVLink_statustext_message(
    severity=2, text=b"ROTOR 0 OUT: LANDING", id=0, chunk_seq=0))
