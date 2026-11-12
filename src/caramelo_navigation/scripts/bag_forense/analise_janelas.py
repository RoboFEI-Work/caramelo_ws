import sys, csv, math, collections
import numpy as np
f=sys.argv[1]; wins=[tuple(map(float,w.split(':'))) for w in sys.argv[2:]]
D=collections.defaultdict(list)
for row in csv.DictReader(open(f)):
    try: D[row['kind']].append((float(row['t']),row['a'],row['b'],row['c'],row['d']))
    except: pass
def arr(k,cols=(1,2,3,4)):
    a=np.array([[r[0]]+[float(r[c]) for c in cols] for r in D[k]]) if D[k] else np.zeros((0,5)); return a
nav=arr('nav'); ref=arr('ref'); odom=arr('odom'); wod=arr('wodom'); amcl=arr('amcl'); scan=arr('scan'); wheel=arr('wheel')
FLOOR=0.1215; WFLOOR=0.315
for (a,b) in wins:
    print(f"\n########## janela {a:.0f}-{b:.0f} s")
    n=nav[(nav[:,0]>=a)&(nav[:,0]<=b)]; o=odom[(odom[:,0]>=a)&(odom[:,0]<=b)]; w=wod[(wod[:,0]>=a)&(wod[:,0]<=b)]
    p=amcl[(amcl[:,0]>=a)&(amcl[:,0]<=b)]; s=scan[(scan[:,0]>=a)&(scan[:,0]<=b)]; wh=wheel[(wheel[:,0]>=a)&(wheel[:,0]<=b)]
    v=np.hypot(n[:,1],n[:,2]); wz=np.abs(n[:,3])
    mov=(v>0.001)|(wz>0.001)
    print(f"cmd_vel_nav: {len(n)} cmds; parado(0,0,0)={np.mean(~mov)*100:.0f}%  |v|>0: {np.sum(v>0.001)}  dos quais |v|<piso {FLOOR}: {np.sum((v>0.001)&(v<FLOOR))} ({np.mean(v[v>0.001]<FLOOR)*100 if np.any(v>0.001) else 0:.0f}%)")
    print(f"  |wz|>0: {np.sum(wz>0.001)}  dos quais <piso {WFLOOR}: {np.sum((wz>0.001)&(wz<WFLOOR))}")
    print(f"  so giro (v<0.001,wz>0): {np.sum((v<0.001)&(wz>0.001))}; lateral puro (|vy|>0.05,|vx|<0.02): {np.sum((np.abs(n[:,2])>0.05)&(np.abs(n[:,1])<0.02))}")
    hist=collections.Counter()
    for vx,vy,z in n[:,1:4]:
        hist[(round(vx,2),round(vy,2),round(z,2))]+=1
    print("  10 comandos mais frequentes (vx,vy,wz):", hist.most_common(10))
    print(f"  sinal de wz: trocas={np.sum(np.diff(np.sign(n[:,3][wz>0.05]))!=0)} em {np.sum(wz>0.05)} cmds com |wz|>0.05")
    if len(o): print(f"odom (EKF): |v| medio {np.mean(np.hypot(o[:,1],o[:,2])):.3f} max {np.max(np.hypot(o[:,1],o[:,2])):.3f}; |wz| medio {np.mean(np.abs(o[:,3])):.3f}")
    if len(w): print(f"odom/wheel: |v| medio {np.mean(np.hypot(w[:,1],w[:,2])):.3f} max {np.max(np.hypot(w[:,1],w[:,2])):.3f}; tempo c/ |v|>0.03: {np.mean(np.hypot(w[:,1],w[:,2])>0.03)*100:.0f}%")
    if len(wh): print(f"maxon/wheel_velocity: tempo c/ alguma roda >0.3 rad/s: {np.mean(np.max(np.abs(wh[:,1:5]),axis=1)>0.3)*100:.0f}%; max {np.max(np.abs(wh[:,1:5])):.2f}")
    if len(p):
        print(f"amcl: inicio ({p[0,1]:.2f},{p[0,2]:.2f},{math.degrees(p[0,3]):.0f}deg) fim ({p[-1,1]:.2f},{p[-1,2]:.2f},{math.degrees(p[-1,3]):.0f}deg); dist percorrida (amcl) {np.sum(np.hypot(np.diff(p[:,1]),np.diff(p[:,2]))):.2f} m; sigma_xy max {np.max(p[:,4]):.3f}")
        yaw_un=np.unwrap(p[:,3]); print(f"  yaw: giro acumulado {np.sum(np.abs(np.diff(yaw_un)))*57.3:.0f} graus; faixa {np.min(yaw_un)*57.3:.0f}..{np.max(yaw_un)*57.3:.0f}")
        step=max(1,len(p)//12)
        print("  trilha (t,x,y,yaw):"," ".join(f"[{r[0]:.0f}s {r[1]:.2f},{r[2]:.2f},{math.degrees(r[3]):.0f}]" for r in p[::step]))
    if len(s): print(f"scan: min geral mediana {np.median(s[:,1]):.2f} min {np.min(s[:,1]):.2f}; setor dir(-90..-45) mediana {np.median(s[:,2]):.2f}; frente mediana {np.median(s[:,3]):.2f} min {np.min(s[:,3]):.2f}; esq mediana {np.median(s[:,4]):.2f}")
