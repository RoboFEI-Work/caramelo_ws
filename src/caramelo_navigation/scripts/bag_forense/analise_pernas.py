import sys, csv, math, collections
import numpy as np
f=sys.argv[1]
D=collections.defaultdict(list)
for row in csv.DictReader(open(f)):
    D[row['kind']].append((float(row['t']),row['a'],row['b'],row['c'],row['d']))
nav=np.array([[r[0],float(r[1]),float(r[2]),float(r[3])] for r in D['nav']])
ref=np.array([[r[0],float(r[1]),float(r[2]),float(r[3])] for r in D['ref']])
wheel=np.array([[r[0]]+[float(x) for x in r[1:5]] for r in D['wheel']])
amcl=np.array([[r[0],float(r[1]),float(r[2]),float(r[3]),float(r[4])] for r in D['amcl']])
plan=np.array([[r[0],float(r[1]),float(r[2]),float(r[3]),float(r[4])] for r in D['plan']])
# pernas de follow_path: status 2=EXECUTING 4=SUCCEEDED 6=ABORTED 5=CANCELED
fps=D['fps']; legs=[]; cur=None
for t,gid,st,_,_ in fps:
    st=int(float(st))
    if st==2 and (cur is None or cur[0]!=gid): cur=[gid,t,None,None]; legs.append(cur)
    elif st in (4,5,6) and cur and cur[0]==gid: cur[2]=t; cur[3]=st
print("### pernas de FollowPath: tempo total, tempo em giro puro do shim (vx=vy=0,|wz|>=0.3), tempo DWB andando, giro acumulado (amcl), deslocamento")
tot_spin=tot_leg=0
for gid,t0,t1,st in legs:
    if t1 is None: continue
    n=nav[(nav[:,0]>=t0)&(nav[:,0]<=t1)]
    if len(n)<2: continue
    dt=np.diff(n[:,0]); dt=np.minimum(dt,0.3)
    spin=(n[:-1,1]==0)&(n[:-1,2]==0)&(np.abs(n[:-1,3])>=0.3)
    drive=(np.hypot(n[:-1,1],n[:-1,2])>0.001)
    p=amcl[(amcl[:,0]>=t0)&(amcl[:,0]<=t1)]
    giro=np.sum(np.abs(np.diff(np.unwrap(p[:,3]))))*57.3 if len(p)>1 else 0
    desl=math.hypot(p[-1,1]-p[0,1],p[-1,2]-p[0,2]) if len(p)>1 else 0
    tot_spin+=np.sum(dt[spin]); tot_leg+=t1-t0
    print(f"  {gid} {t0:6.0f}-{t1:6.0f}s dur {t1-t0:5.1f}s  giro-shim {np.sum(dt[spin]):5.1f}s ({np.sum(dt[spin])/(t1-t0)*100:3.0f}%)  DWB-andando {np.sum(dt[drive]):5.1f}s  giro acumulado {giro:5.0f} graus  desloc {desl:4.2f} m  status {st}")
print(f"  TOTAL: {tot_leg:.0f}s em FollowPath, {tot_spin:.0f}s ({tot_spin/tot_leg*100:.0f}%) girando no lugar pelo shim")
print("\n### maiores saltos em /mecanum_controller/reference (|dv| entre msgs consecutivas, m/s e rad/s)")
dv=np.abs(np.diff(ref[:,1:4],axis=0)); dtr=np.diff(ref[:,0])
mag=np.hypot(dv[:,0],dv[:,1])
idx=np.argsort(-mag)[:12]
for i in sorted(idx):
    print(f"  t={ref[i+1,0]:7.2f}s  de ({ref[i,1]:+.2f},{ref[i,2]:+.2f},{ref[i,3]:+.2f}) para ({ref[i+1,1]:+.2f},{ref[i+1,2]:+.2f},{ref[i+1,3]:+.2f}) em {dtr[i]*1000:.0f} ms  |dv|={mag[i]:.2f}")
print("\n### maiores saltos em /maxon/wheel_velocity (rad/s por 50 ms)")
dw=np.max(np.abs(np.diff(wheel[:,1:5],axis=0)),axis=1); idx=np.argsort(-dw)[:12]
for i in sorted(idx): print(f"  t={wheel[i+1,0]:7.2f}s  max |d omega|={dw[i]:5.2f}  rodas antes {np.round(wheel[i,1:5],1)} depois {np.round(wheel[i+1,1:5],1)}")
if len(sys.argv)>2:
    for w in sys.argv[2:]:
        a,b=map(float,w.split(':')); P=plan[(plan[:,0]>=a)&(plan[:,0]<=b)]
        print(f"\n### /plan na janela {a:.0f}-{b:.0f}: n={len(P)}; comprimento min/med/max {P[:,2].min():.2f}/{np.median(P[:,2]):.2f}/{P[:,2].max():.2f} m")
        for r in P[::max(1,len(P)//15)]: print(f"  t={r[0]:6.1f} pts={r[1]:.0f} L={r[2]:.2f} fim=({r[3]:.2f},{r[4]:.2f})")
