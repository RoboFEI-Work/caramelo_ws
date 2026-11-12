import sys, csv, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
R=0.05; LXY=float(sys.argv[3]) if len(sys.argv)>3 else 0.30
for f,bag in [(sys.argv[1],sys.argv[2])]:
    rows=list(csv.DictReader(open(f)))
    ref=np.array([[float(r['t']),float(r['a']),float(r['b']),float(r['c'])] for r in rows if r['kind']=='ref'])
    wh=np.array([[float(r['t'])]+[float(r[k]) for k in 'abcd'] for r in rows if r['kind']=='wheel'])
    od=np.array([[float(r['t']),float(r['a']),float(r['b']),float(r['c'])] for r in rows if r['kind']=='odom'])
    # esperado por roda (mecanum): (|vx|+|vy|+LXY*|wz|)/R como teto
    rt=ref[:,0]; bursts=[]
    for i in range(len(wh)):
        t=wh[i,0]; j=np.searchsorted(rt,t)-1; j=max(j,0)
        exp=(abs(ref[j,1])+abs(ref[j,2])+LXY*abs(ref[j,3]))/R
        mx=np.max(np.abs(wh[i,1:5]))
        if mx>8.0 and mx>2.5*max(exp,1.0): bursts.append((t,mx,exp,ref[j,1],ref[j,2],ref[j,3],t-ref[j,0]))
    # agrupa em eventos (>1 s de distancia)
    ev=[]; 
    for b in bursts:
        if ev and b[0]-ev[-1][-1][0]<1.0: ev[-1].append(b)
        else: ev.append([b])
    print(f"\n#### {f[:15]}: {len(ev)} eventos de roda > 8 rad/s e > 2.5x o esperado pela referencia")
    for e in ev:
        t0=e[0][0]; t1=e[-1][0]; mx=max(x[1] for x in e); r0=e[0]
        o=od[(od[:,0]>=t0)&(od[:,0]<=t1+0.3)]
        gaps=np.diff(ref[(ref[:,0]>=t0-1.0)&(ref[:,0]<=t1),0]); gmax=gaps.max()*1000 if len(gaps) else 0
        print(f"  t={t0:7.2f}-{t1:7.2f}s ({t1-t0:.2f}s) roda max {mx:5.1f} rad/s | ref no inicio ({r0[3]:+.2f},{r0[4]:+.2f},{r0[5]:+.2f}) esperado<= {r0[2]:.1f} | odom max |v| {np.max(np.hypot(o[:,1],o[:,2])) if len(o) else 0:.2f} |wz| {np.max(np.abs(o[:,3])) if len(o) else 0:.2f} | maior gap entre refs no 1s anterior {gmax:.0f} ms")
    # joint_states (encoder) em 2 janelas para confirmar que e' fisico
    wins=[(280.5,281.6),(168.0,168.9)] if '185756' in f else [(281.5,282.3),(748.0,748.4)]
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}; r.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states'])); M=get_message(types['/joint_states']); t0=None; k=0
    print("  joint_states (velocidade medida pelos encoders) nas janelas", wins)
    while r.has_next():
        top,data,t=r.read_next()
        if t0 is None: t0=t
        ts=(t-t0)/1e9
        if any(a<=ts<=b for a,b in wins):
            k+=1
            if k%4==0:
                m=deserialize_message(data,M); print(f"    t={ts:7.2f} {list(m.name)[:4]} vel={np.round(np.array(m.velocity[:4]),1)}")
        if ts>max(b for a,b in wins): break
