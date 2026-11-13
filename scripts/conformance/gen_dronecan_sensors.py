import json
import random
import struct
import sys
from dronecan import uavcan

SEED = 20240611
rng = random.Random(SEED)
NL = chr(10)
Q = chr(39)
N_TARGET = 260
cov = {}


def wire(m):
    b = m._pack(True)
    b += '0' * ((8 - len(b) % 8) % 8)
    return bytes(int(b[i:i + 8], 2) for i in range(0, len(b), 8))


def h2f(bits):
    return struct.unpack('<e', struct.pack('<H', bits))[0]


def f2b(v):
    return struct.unpack('<I', struct.pack('<f', v))[0]


def b2f(bits):
    return struct.unpack('<f', struct.pack('<I', bits))[0]


def rand_half():
    while True:
        b = rng.getrandbits(16)
        if ((b >> 10) & 0x1F) != 0x1F:
            return b


def rand_f32():
    while True:
        b = rng.getrandbits(32)
        if ((b >> 23) & 0xFF) != 0xFF:
            return b


def rand_f32_mixed():
    if rng.random() < 0.5:
        return rand_f32()
    return f2b(rng.uniform(-200000.0, 200000.0))


H_BOUND = [0x0000, 0x8000, 0x0001, 0x8001, 0x0002, 0x03FF, 0x83FF, 0x0400, 0x8400,
           0x3C00, 0xBC00, 0x3C01, 0x3BFF, 0x4000, 0xC000, 0x3800, 0xB800,
           0x7BFF, 0xFBFF, 0x3555]
for _e in (2, 5, 10, 14, 15, 16, 20, 25, 30):
    H_BOUND.append(_e << 10)
    H_BOUND.append(0x8000 | (_e << 10))

F_BOUND = [0x00000000, 0x80000000, 0x00000001, 0x80000001, 0x007FFFFF, 0x00800000,
           0x3F800000, 0xBF800000, 0x7F7FFFFF, 0xFF7FFFFF, 0x3F000000, 0xC0000000]
for _k in (-126, -100, -50, -10, 0, 10, 23, 24, 50, 100, 127):
    F_BOUND.append(f2b(2.0 ** _k))
    F_BOUND.append(f2b(-(2.0 ** _k)))
F_BOUND.append(f2b(101325.0))
F_BOUND.append(f2b(-101325.0))
F_BOUND.append(f2b(0.001))

U32_B = [0, 1, 2, 3, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFE, 0xFFFFFFFF]
U32_B += [(1 << k) - 1 for k in range(1, 33)]
U32_B += [1 << k for k in range(0, 32)]

RPM_B = [-131072, -131071, -1, 0, 1, 131070, 131071, 255, 256, -256, -255]
RPM_B += [1 << k for k in range(0, 17)]
RPM_B += [(1 << k) - 1 for k in range(1, 18)]
RPM_B += [-(1 << k) for k in range(1, 18)]

PR_B = [0, 1, 2, 3, 7, 15, 31, 63, 64, 65, 126, 127]
IX_B = [0, 1, 2, 3, 7, 15, 16, 17, 30, 31]


def rand_u32():
    if rng.random() < 0.5:
        return rng.getrandbits(32)
    return rng.getrandbits(rng.randint(1, 31))


# field spec: (name, kind, rust_type, bounds, random_fn)
ESC_FIELDS = [
    ('error_count', ('u', 0xFFFFFFFF), 'u32', U32_B, rand_u32),
    ('voltage', ('h',), 'u32', H_BOUND, rand_half),
    ('current', ('h',), 'u32', H_BOUND, rand_half),
    ('temperature', ('h',), 'u32', H_BOUND, rand_half),
    ('rpm', ('i', -131072, 131071), 'i32', RPM_B, lambda: rng.randint(-131072, 131071)),
    ('power_rating_pct', ('u', 127), 'u8', PR_B, lambda: rng.randint(0, 127)),
    ('esc_index', ('u', 31), 'u8', IX_B, lambda: rng.randint(0, 31)),
]
MAG_FIELDS = [
    ('ga_x', ('h',), 'u32', H_BOUND, rand_half),
    ('ga_y', ('h',), 'u32', H_BOUND, rand_half),
    ('ga_z', ('h',), 'u32', H_BOUND, rand_half),
]
BARO_FIELDS = [
    ('pressure', ('f',), 'u32', F_BOUND, rand_f32_mixed),
    ('variance', ('h',), 'u32', H_BOUND, rand_half),
]
BAT_FIELDS = [
    ('temperature', ('h',), 'u32', H_BOUND, rand_half),
    ('voltage', ('h',), 'u32', H_BOUND, rand_half),
    ('current', ('h',), 'u32', H_BOUND, rand_half),
]


def build_esc(v):
    m = uavcan.equipment.esc.Status()
    m.error_count = v[0]
    m.voltage = h2f(v[1])
    m.current = h2f(v[2])
    m.temperature = h2f(v[3])
    m.rpm = v[4]
    m.power_rating_pct = v[5]
    m.esc_index = v[6]
    by = wire(m)
    if len(by) != 14:
        return None
    if by[0:4] != struct.pack('<I', v[0]):
        return None
    for off, bits in ((4, v[1]), (6, v[2]), (8, v[3])):
        if by[off:off + 2] != struct.pack('<H', bits):
            return None
    return by


def build_mag(v):
    m = uavcan.equipment.ahrs.MagneticFieldStrength()
    m.magnetic_field_ga = [h2f(v[0]), h2f(v[1]), h2f(v[2])]
    by = wire(m)
    if len(by) < 6:
        return None
    if by[0:6] != b''.join(struct.pack('<H', x) for x in v):
        return None
    return by


def build_baro(v):
    m = uavcan.equipment.air_data.StaticPressure()
    m.static_pressure = b2f(v[0])
    m.static_pressure_variance = h2f(v[1])
    by = wire(m)
    if len(by) < 6:
        return None
    if by[0:4] != struct.pack('<I', v[0]) or by[4:6] != struct.pack('<H', v[1]):
        return None
    return by


def build_bat(v):
    m = uavcan.equipment.power.BatteryInfo()
    m.temperature = h2f(v[0])
    m.voltage = h2f(v[1])
    m.current = h2f(v[2])
    by = wire(m)
    if len(by) < 6:
        return None
    if by[0:6] != b''.join(struct.pack('<H', x) for x in v):
        return None
    return by


def note(key, v, kind):
    t = kind[0]
    signed = t != 'u'
    if key not in cov:
        cov[key] = {'n': 0, 'zero': 0, 'max': 0}
        if signed:
            cov[key]['neg'] = 0
            cov[key]['min'] = 0
    c = cov[key]
    if t == 'u':
        lo, hi = 0, kind[1]
    elif t == 'i':
        lo, hi = kind[1], kind[2]
    elif t == 'h':
        lo, hi = -65504.0, 65504.0
    else:
        lo, hi = -3.4028234663852886e+38, 3.4028234663852886e+38
    c['n'] += 1
    if v == 0:
        c['zero'] += 1
    if v == hi:
        c['max'] += 1
    if signed:
        if v < 0:
            c['neg'] += 1
        if v == lo:
            c['min'] += 1


def rname(f):
    return f[0] + '_bits' if f[1][0] in ('h', 'f') else f[0]


def lit(kind, v):
    if kind[0] == 'h':
        return '0x%08x' % f2b(h2f(v))
    if kind[0] == 'f':
        return '0x%08x' % v
    return str(v)


def val_of(kind, v):
    if kind[0] == 'h':
        return h2f(v)
    if kind[0] == 'f':
        return b2f(v)
    return v


def blit(by):
    return '&[' + ', '.join('0x%02x' % x for x in by) + ']'


def run(tag, sname, tname, fields, build, short_len):
    vecs = []
    seen = set()
    skipped = [0]

    def attempt(vals):
        key = tuple(vals)
        if key in seen:
            return
        seen.add(key)
        try:
            by = build(vals)
        except Exception:
            by = None
        if by is None:
            skipped[0] += 1
            return
        vecs.append((by, list(vals)))

    nmax = max(len(f[3]) for f in fields)
    for j in range(nmax):
        attempt([f[3][j % len(f[3])] for f in fields])
    for i, f in enumerate(fields):
        for b in f[3]:
            vals = [g[4]() for g in fields]
            vals[i] = b
            attempt(vals)
    tries = 0
    while len(vecs) < N_TARGET and tries < N_TARGET * 6:
        tries += 1
        attempt([g[4]() for g in fields])
    lines = []
    lines.append('#[derive(Clone, Copy, Debug)]')
    lines.append('pub struct %s {' % sname)
    lines.append('    pub bytes: &%sstatic [u8],' % Q)
    for f in fields:
        lines.append('    pub %s: %s,' % (rname(f), f[2]))
    lines.append('}')
    lines.append('pub static %s: &[%s] = &[' % (tname, sname))
    for by, vals in vecs:
        parts = ['bytes: ' + blit(by)]
        for f, v in zip(fields, vals):
            parts.append('%s: %s' % (rname(f), lit(f[1], v)))
            note(tag + '.' + f[0], val_of(f[1], v), f[1])
        lines.append('    %s { %s },' % (sname, ', '.join(parts)))
    lines.append('];')
    bases = [vecs[0][0], vecs[-1][0]] if vecs else []
    lines.append('pub static SHORT_%s: &[&[u8]] = &[' % tname)
    for b in bases:
        for k in range(short_len):
            lines.append('    %s,' % blit(b[:k]))
    lines.append('];')
    return lines, skipped[0]


SPECS = [
    ('esc_status', 'EscVec', 'ESC_STATUS', ESC_FIELDS, build_esc, 14),
    ('mag', 'MagVec', 'MAG', MAG_FIELDS, build_mag, 6),
    ('baro', 'BaroVec', 'BARO', BARO_FIELDS, build_baro, 6),
    ('battery', 'BatVec', 'BATTERY', BAT_FIELDS, build_bat, 6),
]

body = ['// Generated by scripts/conformance/gen_dronecan_sensors.py from the dronecan reference library; SEED=%d' % SEED,
        '// Float fields are stored as the binary32 bit pattern of the value given to the library.']
skipped = {}
for spec in SPECS:
    lines, sk = run(*spec)
    body.extend(lines)
    skipped[spec[0]] = sk
print(NL.join(body))
out = dict(cov)
out['skipped'] = sum(skipped.values())
out['skipped_by_msg'] = skipped
sys.stderr.write(json.dumps(out, sort_keys=True) + NL)
