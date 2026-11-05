"""The maintainer's gate (2026-11-05) for the filter-rate fix:
  >= 250 Hz: every cell no worse than main's SAME cell;
  <  250 Hz: every cell no worse than main's FIXED-dt cell at the same rate,
             rotor vib and pre-arm (jittered dt is judged against main's
             fixed-rate behaviour, not against main's jittered behaviour).
Same classification as cmpgrid.py (mode rank, then peak tilt beyond
max(0.01 rad, 20 %)).

  python3 vsfixed.py <main-tag> <branch-tag> [list-limit]
"""
import re, sys, os, math, collections
d = os.path.dirname(os.path.abspath(__file__))
R = {"Loiter": 0, "Rtl": 1, "Mission": 1, "Takeoff": 1, "Land": 2, "Disarmed": 3, "Terminated": 4}
def load(t):
    out = {}
    for l in open(f"{d}/grid_{t}.txt"):
        m = re.match(r"CELL (\S+) (\S+) (\S+) (\S+) start=(\S+) loiter=(\S+) mode=(\S+) peak=(\S+)", l)
        out[(m[1], m[2], m[3], m[4])] = (m[7], float(m[8]))
    return out
def worse(ref, x):
    (mr, pr), (mx, px) = ref, x
    if R[mx] != R[mr]:
        return R[mx] > R[mr]
    if not math.isfinite(pr):
        return False
    if not math.isfinite(px):
        return True
    return px > pr + max(0.01, 0.2 * pr)
M, B = load(sys.argv[1]), load(sys.argv[2])
lim = int(sys.argv[3]) if len(sys.argv) > 3 else 20
c, bad = collections.Counter(), []
for k, x in B.items():
    hz = float(k[0])
    if hz >= 250:
        ref, rule = M[k], ">=250 same cell"
    else:
        ref, rule = M[(k[0], k[1], k[2], "fixed")], "<250 vs main fixed-dt"
    w = worse(ref, x)
    c[(rule, k[1], w)] += 1
    if w:
        bad.append((k, ref, x))
print(f"gate: {sys.argv[2]} vs {sys.argv[1]} (unique cells {len(B)})")
for rule in [">=250 same cell", "<250 vs main fixed-dt"]:
    for v in ["0", "0.1", "0.25", "0.3"]:
        print(f"  {rule:22s} vib {v:4s} ok {c[(rule, v, False)]:5d} WORSE {c[(rule, v, True)]:4d}")
print(f"  TOTAL WORSE {len(bad)}")
bad.sort(key=lambda b: (b[0][1], float(b[0][0])))
for b in bad[:lim]:
    print("  WORSE", b)
