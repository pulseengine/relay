# usage: sum2.py <S> <stem> <main> <guard> [<mainP> <guardP>] ; ranks each cell, pairs by seed
import sys, re, collections
S, stem = sys.argv[1], sys.argv[2]; tags = sys.argv[3:]
def rank(p, f, iso):
    scn = p[1]; pk = f['peak']
    nan = pk in ('inf', 'NaN') or f.get('rmsh') == 'NaN'
    pkv = 99.0 if nan else float(pk)
    mode = f['mode']
    if nan: return 5, pkv
    if mode == 'Terminated' or pkv > 1.0: return 4, pkv
    if scn == 'rotorout':
        if f['preiso'] == 'true': return 2, pkv
        if f['fm'] != 'Some(1)': return 3, pkv
        return 0, pkv
    if scn == 'cycles':
        cyc = f['cyc']
        if 'Terminated' in cyc: return 4, pkv
        ok = cyc == 'Loiter,Disarmed;' * 3
        if ok and iso == 'iso=none': return 0, pkv
        if iso != 'iso=none': return 2, pkv
        return 3, pkv
    want = 'Loiter'
    if iso != 'iso=none': return 2, pkv
    if mode != want: return 3, pkv
    return 0, pkv
def load(t):
    d = {}
    for l in open(f"{S}/{stem}_{t}.txt"):
        p = l.split()
        f = {}
        for x in p:
            if '=' in x and not x.startswith('iso@'):
                k, v = x.split('=', 1); f[k] = v
        iso = p[-1]
        r, pk = rank(p, f, iso)
        lat = float(f['lat']) if 'lat' in f else -1
        d[(p[1], p[2], p[4], p[5])] = (r, pk, lat, l.strip())
    return d
D = [load(t) for t in tags]
NAMES = "ok,-,falseiso,stuck/notiso,tumble/term,nan".split(',')
g = collections.defaultdict(list)
for k in D[0]:
    pat = re.sub(r"s\d+$", "", k[3]); g[(k[0], k[1], k[2], pat)].append(k)
tot = collections.Counter()
print("group  n | main hist | guard hist | worse(ranked) better | robust-worse(both pairs)")
worse_list = []
for gk in sorted(g, key=lambda x: (x[0], float(x[1]), x[2], x[3])):
    ks = g[gk]
    def hist(d): c = collections.Counter(d[k][0] for k in ks); return ''.join(f"{c.get(i,0):2d}" + (',' if i < 5 else '') for i in range(6))
    w = [k for k in ks if D[1][k][0] > D[0][k][0]]
    b = [k for k in ks if D[1][k][0] < D[0][k][0]]
    rw = [k for k in w if len(D) >= 4 and D[3][k][0] > D[2][k][0]]
    tot['cells'] += len(ks); tot['worse'] += len(w); tot['better'] += len(b); tot['rworse'] += len(rw)
    mo = [k for k in rw if D[0][k][0] == 0]
    tot['rworse_mainok'] += len(mo)
    for k in rw: worse_list.append((k, D[0][k][0], D[1][k][0], D[2][k][0], D[3][k][0]))
    if w or b:
        print(f"{gk[0]:8s} {gk[1]:>5s} {gk[2]:4s} {gk[3]:10s} n={len(ks)} | M[{hist(D[0])}] G[{hist(D[1])}] | w{len(w)} b{len(b)} | rw{len(rw)} (mainOK {len(mo)})")
print(dict(tot))
print("ROBUST WORSE CELLS (key, main, guard, mainP, guardP):")
for x in worse_list: print(x)
# rotor-out latency deltas where both clean
dl = collections.defaultdict(list)
for k in D[0]:
    if k[0] == 'rotorout' and D[0][k][0] == 0 and D[1][k][0] == 0:
        pat = re.sub(r"s\d+$", "", k[3]); dl[(k[1], pat)].append(D[1][k][2] - D[0][k][2])
print("ROTOR-OUT latency delta guard-main (s), both clean: (hz,pat): n, min, max")
for kk in sorted(dl):
    v = dl[kk]
    if any(abs(x) > 1e-6 for x in v): print(kk, len(v), round(min(v), 3), round(max(v), 3))
