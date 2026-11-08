import sys, re, collections, statistics
S, stem, tags = sys.argv[1], sys.argv[2], sys.argv[3].split(',')
only_diff = len(sys.argv) > 4
def load(t):
    d = {}
    for l in open(f"{S}/{stem}_{t}.txt"):
        p = l.split(); key = (p[1], p[2], p[4], re.sub(r"s\d+$", "", p[5]), p[5])
        f = dict(x.split('=', 1) for x in p if '=' in x and not x.startswith('iso@'))
        iso = p[-1]
        r = {}
        r['mode'] = f['mode']; r['inf'] = f['peak'] == 'inf'
        r['peak'] = float(f['peak']) if f['peak'] not in ('inf', 'NaN') else 99.0
        r['rmsh'] = float(f['rmsh']) if f['rmsh'] != 'NaN' else 99.0
        if p[1] == 'rotorout':
            r['bad'] = f['preiso'] == 'true' or f['fm'] != 'Some(1)'
            r['lat'] = float(f['lat'])
        else:
            r['bad'] = iso != 'iso=none'
            r['lat'] = 0
        r['nl'] = r['mode'] != 'Loiter' if p[1] == 'long' else (f['mode'] not in ('Loiter', 'Land', 'Disarmed'))
        d[key] = r
    return d
D = {t: load(t) for t in tags}
g = collections.defaultdict(lambda: collections.defaultdict(list))
for k in D[tags[0]]:
    gk = k[:4]
    for t in tags: g[gk][t].append(D[t][k])
print("scn hz vib pat : [bad(false-iso / rotor-out not isolated) | non-Loiter | tumble | medPeak | medRmsH | medLat]  per tag")
for gk in sorted(g, key=lambda x: (x[0], float(x[1]), x[2], x[3])):
    row = []
    for t in tags:
        L = g[gk][t]
        lats = [x['lat'] for x in L if x['lat'] >= 0]
        row.append(f"{sum(x['bad'] for x in L):2d}/{sum(x['nl'] for x in L):2d}/{sum(x['inf'] for x in L):2d} pk{statistics.median(x['peak'] for x in L):6.3f} h{statistics.median(x['rmsh'] for x in L):6.3f} lat{(statistics.median(lats) if lats else -1):6.3f}")
    s = " | ".join(row)
    if only_diff and len(set(r.split(' lat')[0].split(' h')[0] for r in row)) == 1: continue
    print(f"{gk[0]:8s} {gk[1]:>5s} {gk[2]:4s} {gk[3]:7s} n={len(g[gk][tags[0]])}  {s}")
