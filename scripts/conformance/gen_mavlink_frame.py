import random, sys, json
from pymavlink.dialects.v20 import common as m

SEED = 20240601
rng = random.Random(SEED)
skipped = 0
frames = []
negs = []


def crc16(body, extra):
    return m.x25crc(bytes(body) + bytes([extra])).crc & 0xFFFF


def u(b):
    return rng.getrandbits(b)


def s(b):
    return rng.randint(-(1 << (b - 1)), (1 << (b - 1)) - 1)


def fl():
    return rng.uniform(-100.0, 100.0)


def rb(n):
    return bytes(rng.getrandbits(8) for _ in range(n))


def lst(n):
    l = [u(8) for _ in range(n)]
    l[-1] = rng.randint(1, 255)
    return l


def rec(raw, wire, extra, payload, seq, sysid, comp, inc, cf, msgid):
    return dict(raw=bytes(raw), wire=wire, extra=extra, payload=bytes(payload), seq=seq, sysid=sysid, comp=comp, inc=inc, cf=cf, msgid=msgid)


def hand(msgid, payload, seq, sysid, comp, inc, cf, extra):
    global skipped
    try:
        payload = bytes(payload)
        hdr = bytes([0xFD, len(payload), inc, cf, seq, sysid, comp, msgid & 255, (msgid >> 8) & 255, (msgid >> 16) & 255])
        c = crc16(hdr[1:] + payload, extra)
        raw = hdr + payload + bytes([c & 255, c >> 8])
        frames.append(rec(raw, len(raw), extra, payload, seq, sysid, comp, inc, cf, msgid))
    except Exception:
        skipped += 1


def real(factory, seq, sysid, comp):
    global skipped
    try:
        msg = factory()
        mav = m.MAVLink(None, srcSystem=sysid, srcComponent=comp)
        mav.seq = seq
        p = bytes(msg.pack(mav))
        assert p[0] == 0xFD and len(p) == p[1] + 12
        # msg.id can be shadowed by a message field named id (statustext), so read the id from the wire header
        mid = p[7] | (p[8] << 8) | (p[9] << 16)
        frames.append(rec(p, len(p), msg.crc_extra, p[10:-2], seq, sysid, comp, p[2], p[3], mid))
    except Exception:
        skipped += 1


def arr(b):
    return '&[' + ','.join(str(x) for x in b) + ']'


facs = {
    'hb': lambda: m.MAVLink_heartbeat_message(type=u(8), autopilot=u(8), base_mode=u(8), custom_mode=u(32), system_status=u(8), mavlink_version=u(8)),
    'att': lambda: m.MAVLink_attitude_message(time_boot_ms=u(32), roll=fl(), pitch=fl(), yaw=fl(), rollspeed=fl(), pitchspeed=fl(), yawspeed=fl()),
    'sys': lambda: m.MAVLink_sys_status_message(onboard_control_sensors_present=u(32), onboard_control_sensors_enabled=u(32), onboard_control_sensors_health=u(32), load=u(16), voltage_battery=u(16), current_battery=s(16), battery_remaining=s(8), drop_rate_comm=u(16), errors_comm=u(16), errors_count1=u(16), errors_count2=u(16), errors_count3=u(16), errors_count4=u(16)),
    'gpi': lambda: m.MAVLink_global_position_int_message(time_boot_ms=u(32), lat=s(32), lon=s(32), alt=s(32), relative_alt=s(32), vx=s(16), vy=s(16), vz=s(16), hdg=u(16)),
    'cl': lambda: m.MAVLink_command_long_message(target_system=u(8), target_component=u(8), command=u(16), confirmation=u(8), param1=fl(), param2=fl(), param3=fl(), param4=fl(), param5=fl(), param6=fl(), param7=fl()),
    'st': lambda: m.MAVLink_statustext_message(severity=rng.randint(0, 7), text=bytes(rng.randint(65, 90) for _ in range(rng.randint(0, 50))), id=u(16), chunk_seq=u(8)),
    'ed': lambda: m.MAVLink_encapsulated_data_message(seqnr=u(16), data=lst(253)),
    'ftp': lambda: m.MAVLink_file_transfer_protocol_message(target_network=u(8), target_system=u(8), target_component=u(8), payload=lst(251)),
    'tun': lambda: m.MAVLink_tunnel_message(target_system=u(8), target_component=u(8), payload_type=u(16), payload_length=u(8), payload=lst(128)),
}

for b in [0, 1, 127, 128, 255]:
    real(facs['hb'], b, b, b)
real(lambda: m.MAVLink_heartbeat_message(type=0, autopilot=0, base_mode=0, custom_mode=0, system_status=0, mavlink_version=0), 0, 0, 0)
real(lambda: m.MAVLink_heartbeat_message(type=255, autopilot=255, base_mode=255, custom_mode=0xFFFFFFFF, system_status=255, mavlink_version=255), 255, 255, 255)
real(lambda: m.MAVLink_global_position_int_message(time_boot_ms=0xFFFFFFFF, lat=-2**31, lon=2**31 - 1, alt=-2**31, relative_alt=2**31 - 1, vx=-32768, vy=32767, vz=-1, hdg=65535), 7, 1, 1)
real(lambda: m.MAVLink_encapsulated_data_message(seqnr=65535, data=[255] * 253), 9, 2, 3)
for name, f in facs.items():
    for i in range(14):
        real(f, u(8), u(8), u(8))

IDS = [0, 1, 2, 255, 256, 257, 0x7FFF, 0xFFFF, 0x10000, 0x10001, 0x7FFFFF, 0x800000, 0xFFFFFE, 0xFFFFFF, 0x010203]
for mid in IDS:
    hand(mid, rb(rng.randint(0, 255)), u(8), u(8), u(8), 0, 0, u(8))
lens = list(range(0, 17)) + [31, 32, 63, 64, 127, 128, 129, 200, 252, 253, 254, 255] + [rng.randint(0, 255) for _ in range(30)]
for L in lens:
    hand(rng.choice(IDS + [rng.getrandbits(24)]), rb(L), u(8), u(8), u(8), 0, 0, u(8))
hand(0xFFFFFF, rb(255), 255, 255, 255, 0, 0, 255)
for inc in [0, 1, 2, 4, 0x80, 0xFE, 0xFF]:
    for cf in [0, 1, 0x80, 0xFF]:
        hand(rng.getrandbits(24), rb(rng.randint(0, 40)), u(8), u(8), u(8), inc, cf, u(8))
for _ in range(30):
    hand(rng.getrandbits(24), rb(rng.randint(0, 255)), u(8), u(8), u(8), u(8), u(8), u(8))
for b in [0, 1, 127, 128, 255]:
    hand(rng.getrandbits(24), rb(rng.randint(0, 30)), b, b, b, 0, 0, u(8))
    hand(rng.getrandbits(24), rb(rng.randint(0, 30)), b, 0, 255, 0, 0, u(8))
for e in [0, 1, 50, 127, 128, 254, 255]:
    hand(rng.getrandbits(24), rb(rng.randint(0, 30)), u(8), u(8), u(8), 0, 0, e)

base = list(frames)
for f in rng.sample(base, min(24, len(base))):
    g = dict(f)
    g['raw'] = f['raw'] + rb(rng.randint(1, 40))
    frames.append(g)


def neg(raw, extra, msgid, expect):
    negs.append(dict(raw=bytes(raw), extra=extra, msgid=msgid, expect=expect))


for f in rng.sample(base, min(40, len(base))):
    core = f['raw'][:f['wire']]
    w = len(core)
    cuts = set([0, 1, 9, 10, 11, w - 1, w - 2, rng.randint(0, w - 1)])
    for c in sorted(cuts):
        if 0 <= c < w:
            neg(core[:c], f['extra'], f['msgid'], 0)
    b = bytearray(core)
    b[-1] ^= rng.randint(1, 255)
    neg(b, f['extra'], f['msgid'], 1)
    b = bytearray(core)
    b[-2] ^= rng.randint(1, 255)
    neg(b, f['extra'], f['msgid'], 1)
    if len(f['payload']) > 0:
        b = bytearray(core)
        b[10 + rng.randrange(len(f['payload']))] ^= rng.randint(1, 255)
        neg(b, f['extra'], f['msgid'], 1)
    b = bytearray(core)
    b[0] = rng.choice([0x00, 0xFE, 0xFF, 0xFC, 0x55])
    neg(b, f['extra'], f['msgid'], 2)

for fk in ['hb', 'att', 'sys', 'gpi', 'cl']:
    for _ in range(2):
        try:
            msg = facs[fk]()
            mav = m.MAVLink(None, srcSystem=1, srcComponent=1)
            p = bytes(msg.pack(mav, force_mavlink1=True))
            assert p[0] == 0xFE
            neg(p, msg.crc_extra, 0, 2)
        except Exception:
            skipped += 1

out = []
out.append('// Generated by scripts/conformance/gen_mavlink_frame.py with pymavlink; do not edit.')
out.append("""pub struct FrameVec { pub raw: &'static [u8], pub wire_len: usize, pub crc_extra: u8, pub payload: &'static [u8], pub payload_len: u8, pub incompat: u8, pub compat: u8, pub seq: u8, pub sysid: u8, pub compid: u8, pub msgid: u32 }""")
out.append("""pub struct NegVec { pub raw: &'static [u8], pub crc_extra: u8, pub msgid: u32, pub expect: u8 }""")
out.append('pub static FRAME_VECTORS: &[FrameVec] = &[')
for f in frames:
    out.append('FrameVec{raw:%s,wire_len:%d,crc_extra:%d,payload:%s,payload_len:%d,incompat:%d,compat:%d,seq:%d,sysid:%d,compid:%d,msgid:%d},' % (arr(f['raw']), f['wire'], f['extra'], arr(f['payload']), len(f['payload']), f['inc'], f['cf'], f['seq'], f['sysid'], f['comp'], f['msgid']))
out.append('];')
out.append('pub static NEG_VECTORS: &[NegVec] = &[')
for g in negs:
    out.append('NegVec{raw:%s,crc_extra:%d,msgid:%d,expect:%d},' % (arr(g['raw']), g['extra'], g['msgid'], g['expect']))
out.append('];')
for line in out:
    print(line)


def cov(vals, mx):
    return {'n': len(vals), 'zero': sum(1 for v in vals if v == 0), 'max': sum(1 for v in vals if v == mx)}


report = {
    'seq': cov([f['seq'] for f in frames], 255),
    'sysid': cov([f['sysid'] for f in frames], 255),
    'compid': cov([f['comp'] for f in frames], 255),
    'incompat': cov([f['inc'] for f in frames], 255),
    'compat': cov([f['cf'] for f in frames], 255),
    'crc_extra': cov([f['extra'] for f in frames], 255),
    'payload_len': cov([len(f['payload']) for f in frames], 255),
    'msgid': cov([f['msgid'] for f in frames], 0xFFFFFF),
    'negative_vectors': len(negs),
    'skipped': skipped,
}
sys.stderr.write(json.dumps(report) + chr(10))
