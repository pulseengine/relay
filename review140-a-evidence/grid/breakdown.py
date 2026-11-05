import re, collections, sys, os
d = os.path.dirname(os.path.abspath(__file__))
def load(t):
    return [re.match(r"CELL (\S+) (\S+) (\S+) (\S+) start=(\S+) loiter=(\S+) mode=(\S+) peak=(\S+)", l).groups() for l in open(f"{d}/grid_{t}.txt")]
A, B = load(sys.argv[1]), load(sys.argv[2])
lo = float(sys.argv[3]) if len(sys.argv) > 3 else 0
hi = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
R = {"Loiter": 0, "Rtl": 1, "Mission": 1, "Takeoff": 1, "Land": 2, "Disarmed": 3, "Terminated": 4}
c = collections.Counter()
for a, b in zip(A, B):
    if not (lo <= float(a[0]) <= hi):
        continue
    ra, rb = R[a[6]], R[b[6]]
    c[(a[1], a[3], 'w' if rb > ra else 'b' if rb < ra else 's')] += 1
for vib in ['0', '0.1', '0.25', '0.3']:
    for p in ['fixed', 'alt', 'rand']:
        print(f"vib {vib:4s} {p:5s} mode-worse {c[(vib,p,'w')]:4d} mode-better {c[(vib,p,'b')]:4d} same {c[(vib,p,'s')]:4d}")
pc = collections.Counter((a[1], a[6]) for a in A if lo <= float(a[0]) <= hi)
print("parent outcomes", sorted(pc.items()))
