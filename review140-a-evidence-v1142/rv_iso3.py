import sys, collections, re
S, stem = sys.argv[1], sys.argv[2]
tags = sys.argv[3].split(",")
fam = lambda pat: re.sub(r"s\d+$", "", pat)
res = {}
for tag in tags:
    g = collections.defaultdict(lambda: [0, 0, 0])
    for l in open(f"{S}/{stem}_{tag}.txt"):
        p = l.split()
        key = (p[1], int(float(p[2])), fam(p[5]), p[4])
        iso = [x for x in p if x.startswith("iso")][-1]
        mode = [x for x in p if x.startswith("mode=")][0][5:]
        g[key][0] += 1
        if iso != "iso=none": g[key][1] += 1
        if mode != "Loiter": g[key][2] += 1
    res[tag] = g
keys = sorted(res[tags[0]])
print("(scenario, hz, pattern, vib): isolated / non-Loiter final, of n   " + " | ".join(tags))
tot = {t: collections.Counter() for t in tags}
for k in keys:
    print(f"  {k[0]:6s} {k[1]:5d} {k[2]:7s} {k[3]:4s} " + " | ".join(f"{res[t][k][1]:2d}/{res[t][k][2]:2d} of {res[t][k][0]}" for t in tags))
    for t in tags:
        tot[t][(k[0], k[2], k[1] >= 1000)] += res[t][k][1]
print("isolations summed over vib, per (scenario, pattern, is_1kHz):")
for kk in sorted(tot[tags[0]]):
    print("  ", kk, " | ".join(f"{t} {tot[t][kk]}" for t in tags))
