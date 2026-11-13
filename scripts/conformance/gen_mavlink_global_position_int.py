import json
import random
import struct
import sys
from pymavlink.dialects.v20 import common as m

SEED = 20240733
TARGET = 300
NL = chr(10)
FIELDS = ['time_boot_ms', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'hdg']
RANGES = {
    'time_boot_ms': (0, 2 ** 32 - 1),
    'lat': (-2 ** 31, 2 ** 31 - 1),
    'lon': (-2 ** 31, 2 ** 31 - 1),
    'alt': (-2 ** 31, 2 ** 31 - 1),
    'relative_alt': (-2 ** 31, 2 ** 31 - 1),
    'vx': (-2 ** 15, 2 ** 15 - 1),
    'vy': (-2 ** 15, 2 ** 15 - 1),
    'vz': (-2 ** 15, 2 ** 15 - 1),
    'hdg': (0, 2 ** 16 - 1),
}
SIGNED = set(['lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz'])
FMT = '<IiiiihhhH'
FULL = struct.calcsize(FMT)
rng = random.Random(SEED)


def pw(lo, hi, extra=()):
    s = set([lo, hi, 0, 1, -1, lo + 1, hi - 1])
    ks = (7, 8, 14, 15) if hi < 2 ** 17 else (8, 16, 24, 31)
    for k in ks:
        for b in (2 ** k - 1, 2 ** k):
            s.add(b)
            s.add(-b)
    for e in extra:
        s.add(e)
    return sorted(v for v in s if lo <= v <= hi)


E7 = 10 ** 7
BOUND = {
    'time_boot_ms': pw(0, 2 ** 32 - 1, (1000, 86400000)),
    'lat': pw(-2 ** 31, 2 ** 31 - 1, (E7, -E7, 90 * E7, -90 * E7, 90 * E7 - 1, -90 * E7 + 1, 90 * E7 + 1, -90 * E7 - 1)),
    'lon': pw(-2 ** 31, 2 ** 31 - 1, (E7, -E7, 180 * E7, -180 * E7, 180 * E7 - 1, -180 * E7 + 1, 180 * E7 + 1, -180 * E7 - 1)),
    'alt': pw(-2 ** 31, 2 ** 31 - 1, (1000, -1000)),
    'relative_alt': pw(-2 ** 31, 2 ** 31 - 1, (1000, -1000)),
    'vx': pw(-2 ** 15, 2 ** 15 - 1, (100, -100, 1000, -1000)),
    'vy': pw(-2 ** 15, 2 ** 15 - 1, (100, -100, 1000, -1000)),
    'vz': pw(-2 ** 15, 2 ** 15 - 1, (100, -100, 1000, -1000)),
    'hdg': pw(0, 2 ** 16 - 1, (35999, 36000, 35998, 36001, 18000, 9000)),
}


def rnd(f):
    lo, hi = RANGES[f]
    r = rng.random()
    if f == 'lat' and r < 0.6:
        return rng.randint(-90 * E7, 90 * E7)
    if f == 'lon' and r < 0.6:
        return rng.randint(-180 * E7, 180 * E7)
    if f == 'hdg':
        if r < 0.1:
            return 65535
        if r < 0.7:
            return rng.randint(0, 35999)
        return rng.randint(0, 65535)
    if f in ('vx', 'vy', 'vz') and r < 0.5:
        return rng.randint(-3000, 3000)
    if f == 'time_boot_ms' and r < 0.5:
        return rng.randint(0, 86400000)
    if f in ('alt', 'relative_alt') and r < 0.5:
        return rng.randint(-500000, 9000000)
    return rng.randint(lo, hi)


def base():
    return dict((f, rnd(f)) for f in FIELDS)


specs = []
for f in FIELDS:
    for val in BOUND[f]:
        d = base()
        d[f] = val
        specs.append(d)
specs.append(dict((f, 0) for f in FIELDS))
specs.append(dict((f, RANGES[f][1]) for f in FIELDS))
specs.append(dict((f, RANGES[f][0]) for f in FIELDS))
alt_d = {}
for i, f in enumerate(FIELDS):
    alt_d[f] = RANGES[f][i % 2]
specs.append(alt_d)
# MAVLink2 trailing-zero truncation: progressively zero trailing fields
d = base()
for f in reversed(FIELDS):
    d[f] = 0
    specs.append(dict(d))
# exactly one non-zero field at each position
for f in FIELDS:
    for val in (1, RANGES[f][1]):
        d = dict((g, 0) for g in FIELDS)
        d[f] = val
        specs.append(d)
while len(specs) < TARGET:
    specs.append(base())


def build(d, seq, sysid, compid):
    msg = m.MAVLink_global_position_int_message(**d)
    mav = m.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    mav.seq = seq
    wire = bytes(msg.pack(mav))
    if wire[0] != 0xFD or wire[2] != 0 or wire[4] != seq or wire[5] != sysid or wire[6] != compid:
        raise ValueError('unexpected header')
    trimmed = wire[10:-2]
    if wire[1] != len(trimmed) or len(trimmed) < 1 or len(trimmed) > FULL:
        raise ValueError('unexpected payload length')
    full = trimmed + bytes(FULL - len(trimmed))
    vals = dict((f, getattr(msg, f)) for f in FIELDS)
    if struct.unpack(FMT, full) != tuple(vals[f] for f in FIELDS):
        raise ValueError('python-side unpack mismatch')
    msg_id = wire[7] | (wire[8] << 8) | (wire[9] << 16)
    if msg_id != msg.id:
        raise ValueError('msg id mismatch')
    return {'vals': vals, 'full': full, 'wire': wire, 'trunc_len': len(trimmed),
            'msg_id': msg_id, 'crc_extra': int(msg.crc_extra),
            'seq': seq, 'sysid': sysid, 'compid': compid}


vecs = []
skipped = 0
for d in specs:
    seq = rng.randint(0, 255)
    sysid = rng.randint(1, 255)
    compid = rng.randint(0, 255)
    try:
        vecs.append(build(d, seq, sysid, compid))
    except Exception:
        skipped += 1


def blit(b):
    return '&[' + ', '.join('0x%02x' % x for x in b) + ']'


# negative cases -----------------------------------------------------------
bad_len = []
bad_crc = []
trunc_frames = []
if vecs:
    first = vecs[0]['full']
    for L in (0, 1, 14, 27, 29, 30, 64):
        bad_len.append(first[:L] if L < FULL else first + bytes(L - FULL))
    for _ in range(10):
        v = vecs[rng.randrange(len(vecs))]
        L = rng.choice((0, 1, 7, 13, 20, 26, 27, 29, 31, 40))
        full = v['full']
        bad_len.append(full[:L] if L < FULL else full + bytes(L - FULL))
    for i in range(12):
        v = vecs[rng.randrange(len(vecs))]
        w = bytearray(v['wire'])
        ce = v['crc_extra']
        mode = i % 4
        bit = 1 << rng.randrange(8)
        if mode == 0:
            w[-1] ^= bit
        elif mode == 1:
            w[-2] ^= bit
        elif mode == 2:
            w[10 + rng.randrange(v['trunc_len'])] ^= bit
        else:
            ce = ce ^ bit
        bad_crc.append((bytes(w), ce))
    for i in range(10):
        v = vecs[rng.randrange(len(vecs))]
        w = v['wire']
        cut = (len(w) - 1, len(w) - 2, 10, 5, 0)[i % 5]
        trunc_frames.append((w[:cut], v['crc_extra']))

# emit Rust ------------------------------------------------------------------
out = []
out.append('// Generated by scripts/conformance/gen_mavlink_global_position_int.py from pymavlink ' + 'v20 common dialect. Do not edit.')
out.append('// SEED = %d' % SEED)
out.append('#[allow(dead_code)]')
out.append('pub struct GpiVec {')
out.append('    pub time_boot_ms: u32, pub lat: i32, pub lon: i32, pub alt: i32, pub relative_alt: i32,')
out.append('    pub vx: i16, pub vy: i16, pub vz: i16, pub hdg: u16,')
out.append('    pub seq: u8, pub sysid: u8, pub compid: u8, pub msg_id: u32, pub crc_extra: u8,')
out.append('    pub trunc_len: usize,')
out.append("    pub payload_full: &'static [u8],")
out.append("    pub wire: &'static [u8],")
out.append('}')
out.append('#[allow(dead_code)]')
out.append('pub struct GpiFrameNeg {')
out.append("    pub wire: &'static [u8],")
out.append('    pub crc_extra: u8,')
out.append('}')
out.append('')
out.append('pub static GPI_VECTORS: &[GpiVec] = &[')
for v in vecs:
    x = v['vals']
    out.append('    GpiVec { time_boot_ms: %d, lat: %d, lon: %d, alt: %d, relative_alt: %d, vx: %d, vy: %d, vz: %d, hdg: %d, seq: %d, sysid: %d, compid: %d, msg_id: %d, crc_extra: %d, trunc_len: %d, payload_full: %s, wire: %s },' % (
        x['time_boot_ms'], x['lat'], x['lon'], x['alt'], x['relative_alt'],
        x['vx'], x['vy'], x['vz'], x['hdg'], v['seq'], v['sysid'], v['compid'],
        v['msg_id'], v['crc_extra'], v['trunc_len'], blit(v['full']), blit(v['wire'])))
out.append('];')
out.append('')
out.append('pub static GPI_BAD_LEN: &[&[u8]] = &[')
for b in bad_len:
    out.append('    ' + blit(b) + ',')
out.append('];')
out.append('')
out.append('pub static GPI_BAD_CRC: &[GpiFrameNeg] = &[')
for w, ce in bad_crc:
    out.append('    GpiFrameNeg { wire: %s, crc_extra: %d },' % (blit(w), ce))
out.append('];')
out.append('')
out.append('pub static GPI_TRUNCATED_FRAMES: &[GpiFrameNeg] = &[')
for w, ce in trunc_frames:
    out.append('    GpiFrameNeg { wire: %s, crc_extra: %d },' % (blit(w), ce))
out.append('];')
sys.stdout.write(NL.join(out) + NL)

# coverage -------------------------------------------------------------------
cov = {}
for f in FIELDS:
    lo, hi = RANGES[f]
    vs = [v['vals'][f] for v in vecs]
    c = {'n': len(vs), 'zero': sum(1 for x in vs if x == 0), 'max': sum(1 for x in vs if x == hi)}
    if f in SIGNED:
        c['neg'] = sum(1 for x in vs if x < 0)
        c['min'] = sum(1 for x in vs if x == lo)
    cov[f] = c
cov['skipped'] = skipped
cov['vectors'] = len(vecs)
cov['truncated_payload_vectors'] = sum(1 for v in vecs if v['trunc_len'] < FULL)
cov['bad_len_cases'] = len(bad_len)
cov['bad_crc_cases'] = len(bad_crc)
cov['truncated_frame_cases'] = len(trunc_frames)
sys.stderr.write(json.dumps(cov) + NL)
