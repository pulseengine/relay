# Replicates resync_filters' guard logic on a dt sequence (no flight).
import random, sys
def rr(h, dt):
    if not (dt == dt and dt > 0 and dt != float('inf')): return None
    a = 1/dt
    return a if abs(a-h) > 0.02*h else None
def run(seq, hz):
    des=hz; fil=hz; hist=0; ring=[0.0]*16; ln=0; pos=0; g=False
    ing=0; entries=0; reb_main=0; reb_g=0; mism=0.0
    for dt in seq:
        r=rr(des,dt); c=r is not None
        if c: des=r; reb_main+=1
        hist=((hist<<1)|c)&0xffff
        if dt==dt and dt>0: ring[pos]=dt; pos=(pos+1)%16; ln=min(16,ln+1)
        n=bin(hist).count('1')
        if not g and ln==16 and n>=12: g=True; entries+=1
        elif g and n<=4:
            g=False
            if rr(fil,1/des) is not None: fil=des; reb_g+=1
            continue
        if g:
            v=sorted(ring); h2=1/((v[7]+v[8])/2)
            if rr(fil,1/h2) is not None: fil=h2; reb_g+=1
            ing+=1
            if dt==dt and dt>0: mism+=abs(fil*dt-1)
        elif c: fil=des; reb_g+=1
    N=len(seq); return dict(in_guard=round(ing/N,3), entries=entries, reb_main=reb_main, reb_guard=reb_g, mean_mismatch_in_guard=round(mism/max(ing,1),3))
if __name__=='__main__':
    hz=400; d=1/hz; R=random.Random(1); N=24000
    pats={
     'late5(20%x1.25)': [d*1.25 if R.random()<0.2 else d for _ in range(N)],
     'late10(10%x1.5)': [d*1.5 if R.random()<0.1 else d for _ in range(N)],
     'late20(5%x2)': [d*2 if R.random()<0.05 else d for _ in range(N)],
     'late30x1.5': [d*1.5 if R.random()<0.3 else d for _ in range(N)],
     'late40x1.5': [d*1.5 if R.random()<0.4 else d for _ in range(N)],
     'late30x1.25': [d*1.25 if R.random()<0.3 else d for _ in range(N)],
     'p3 [d,d,1.5d]': [ [d,d,1.5*d][i%3] for i in range(N)],
     'p3 [d,1.5d,1.5d]': [ [d,1.5*d,1.5*d][i%3] for i in range(N)],
     'p5 [d,L,d,L,d]': [ [d,1.5*d,d,1.5*d,d][i%5] for i in range(N)],
     'p4 [d,d,d,1.5d]': [ [d,d,d,1.5*d][i%4] for i in range(N)],
     'burst1s_then_late5': [ ([0.875*d,1.125*d][i%2] if i<400 else (d*1.25 if R.random()<0.2 else d)) for i in range(N)],
     'burst1s_then_p3': [ ([0.875*d,1.125*d][i%2] if i<400 else [d,d,1.5*d][i%3]) for i in range(N)],
     'burst1s_then_late30x1.5': [ ([0.875*d,1.125*d][i%2] if i<400 else (d*1.5 if R.random()<0.3 else d)) for i in range(N)],
    }
    for k,s in pats.items(): print(f"{k:28s}", run(s,hz))
