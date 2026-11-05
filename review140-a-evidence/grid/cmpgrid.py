"""Compare two grid files cell by cell. Classification:
 mode rank first (Loiter 0 < Rtl/Mission/Takeoff 1 < Land 2 < Disarmed 3 <
 Terminated 4); same mode -> peak tilt; WORSE if fix peak exceeds parent by
 more than the noise band max(0.01 rad, 20 % of parent); non-finite > finite.
"""
import re, sys, os, math, collections
d = os.path.dirname(os.path.abspath(__file__))
RANK = {"Loiter": 0, "Rtl": 1, "Mission": 1, "Takeoff": 1, "Land": 2, "Disarmed": 3, "Terminated": 4}
def load(tag):
    out = []
    for l in open(f"{d}/grid_{tag}.txt"):
        m = re.match(r"CELL (\S+) (\S+) (\S+) (\S+) start=(\S+) loiter=(\S+) mode=(\S+) peak=(\S+)", l)
        out.append(((float(m[1]), m[2], m[3], m[4]), m[7], float(m[8]), m[5]))
    return out
def band(hz):
    if hz < 100: return "a 10-100"
    if hz < 120: return "b 100.5-119.5"
    if hz <= 120.02 + 1e-6: return "c 120.000-120.020"
    if hz < 140: return "d 120.5-139.5"
    if hz <= 200: return "e 140-200"
    return "f 250-10000"
A, B = load(sys.argv[1]), load(sys.argv[2])
assert len(A) == len(B)
NB = lambda p: max(0.01, 0.2 * p)
stats = collections.defaultdict(lambda: [0, 0, 0])
worse, better = [], []
for (ka, ma, pa, sa), (kb, mb, pb, sb) in zip(A, B):
    assert ka == kb, (ka, kb)
    ra, rb = RANK.get(ma, 9), RANK.get(mb, 9)
    fa, fb = math.isfinite(pa), math.isfinite(pb)
    if rb > ra or (rb == ra and ((fa and not fb) or (fa and fb and pb > pa + NB(pa)))):
        c = 2; worse.append((ka, (ma, pa), (mb, pb)))
    elif rb < ra or (rb == ra and ((fb and not fa) or (fa and fb and pb < pa - NB(pa)))):
        c = 1; better.append((ka, (ma, pa), (mb, pb)))
    else:
        c = 0
    stats[band(ka[0])][c] += 1
print(f"{sys.argv[1]} -> {sys.argv[2]}: noise band max(0.01 rad, 20%)")
tot = [0, 0, 0]
for b in sorted(stats):
    s = stats[b]; tot = [x + y for x, y in zip(tot, s)]
    print(f"  {b:20s} same {s[0]:5d} better {s[1]:4d} WORSE {s[2]:4d}")
print(f"  {'TOTAL':20s} same {tot[0]:5d} better {tot[1]:4d} WORSE {tot[2]:4d}")
lim = int(sys.argv[3]) if len(sys.argv) > 3 else 15
worse.sort(key=lambda w: (-RANK.get(w[2][0], 9) + RANK.get(w[1][0], 9), -(w[2][1] - w[1][1]) if math.isfinite(w[2][1]) else -1e9))
for w in worse[:lim]: print("  WORSE", w)
for w in better[:lim]: print("  better", w)
