"""Failure counts (final mode Land / Disarmed / Terminated) per tree, rotor
vib level, dt pattern and rate band; the slow-rotation hold (Rtl at every
rate on the parent) counted separately."""
import re, sys, os, collections
d = os.path.dirname(os.path.abspath(__file__))
FAIL = {"Land", "Disarmed", "Terminated"}
def band(hz):
    if hz < 100: return "10-100"
    if hz < 120: return "100.5-119.5"
    if hz <= 120.0205: return "120.000-120.020"
    if hz < 133.4: return "120.5-133"
    if hz < 140: return "133.5-139.5"
    if hz <= 200: return "140-200"
    return "250-10000"
tags = sys.argv[1:]
data = {}
for t in tags:
    c = collections.Counter()
    for l in open(f"{d}/grid_{t}.txt"):
        f = l.split()
        hz, vib, pre, pat = float(f[1]), f[2], f[3], f[4]
        mode = re.search(r"mode=(\S+)", l)[1]
        c[(band(hz), vib, pat)] += mode in FAIL
        c[(band(hz), vib, pat, "n")] += 1
    data[t] = c
bands = ["10-100", "100.5-119.5", "120.000-120.020", "120.5-133", "133.5-139.5", "140-200", "250-10000"]
print("failures (Land/Disarmed/Terminated) / cells, per tree:", " | ".join(tags))
for b in bands:
    for vib in ["0", "0.1", "0.25", "0.3"]:
        row = []
        for pat in ["fixed", "alt", "rand"]:
            n = data[tags[0]][(b, vib, pat, "n")]
            row.append(f"{pat}: " + "/".join(str(data[t][(b, vib, pat)]) for t in tags) + f" of {n}")
        print(f"  {b:16s} vib {vib:4s}  " + "   ".join(row))
tot = {t: sum(v for k, v in data[t].items() if len(k) == 3) for t in tags}
print("TOTAL failures:", tot)
