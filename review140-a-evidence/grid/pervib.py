import re, sys, os, math, collections
d = os.path.dirname(os.path.abspath(__file__))
R = {"Loiter": 0, "Rtl": 1, "Mission": 1, "Takeoff": 1, "Land": 2, "Disarmed": 3, "Terminated": 4}
def load(t):
    out = []
    for l in open(f"{d}/grid_{t}.txt"):
        m = re.match(r"CELL (\S+) (\S+) (\S+) (\S+) start=(\S+) loiter=(\S+) mode=(\S+) peak=(\S+)", l)
        out.append((m[2], m[7], float(m[8])))
    return out
def cls(a, b):
    ra, rb = R[a[1]], R[b[1]]
    pa, pb = a[2], b[2]
    nb = max(0.01, 0.2 * pa) if math.isfinite(pa) else 0
    if rb > ra: return "mode-worse"
    if rb < ra: return "mode-better"
    if math.isfinite(pa) and not math.isfinite(pb): return "tilt-worse"
    if math.isfinite(pb) and not math.isfinite(pa): return "tilt-better"
    if math.isfinite(pa) and pb > pa + nb: return "tilt-worse"
    if math.isfinite(pa) and pb < pa - nb: return "tilt-better"
    return "same"
P = load("parent")
for t in sys.argv[1:]:
    B = load(t)
    c = collections.Counter((a[0], cls(a, b)) for a, b in zip(P, B))
    for v in ["0", "0.1", "0.25", "0.3"]:
        print(f"parent->{t:8s} vib {v:4s} " + " ".join(f"{k} {c[(v,k)]:4d}" for k in ["same", "mode-better", "tilt-better", "mode-worse", "tilt-worse"]))
