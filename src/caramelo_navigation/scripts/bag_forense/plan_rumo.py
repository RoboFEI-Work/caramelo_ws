import sys, math, bisect, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag=sys.argv[1]; wins=[tuple(map(float,w.split(':'))) for w in sys.argv[2:]]
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}; TOP=['/plan','/amcl_pose','/cmd_vel_nav']
r.set_filter(rosbag2_py.StorageFilter(topics=TOP)); M={t:get_message(types[t]) for t in TOP}
t0=None; plans=[]; amcl=[]; nav=[]
while r.has_next():
    top,data,t=r.read_next()
    if t0 is None: t0=t
    ts=(t-t0)/1e9; m=deserialize_message(data,M[top])
    if top=='/plan':
        P=m.poses
        if len(P)<2: plans.append((ts,0,None,None,yaw(P[0].pose.orientation) if P else None, yaw(P[0].pose.orientation) if P else None)); continue
        pts=np.array([(p.pose.position.x,p.pose.position.y) for p in P]); seg=np.hypot(*np.diff(pts,axis=0).T); cum=np.concatenate([[0],np.cumsum(seg)])
        i=int(np.searchsorted(cum,0.5)); i=min(i,len(pts)-1)
        h=math.atan2(pts[i,1]-pts[0,1],pts[i,0]-pts[0,0])
        plans.append((ts,cum[-1],h,pts[i],yaw(P[-1].pose.orientation),yaw(P[0].pose.orientation)))
    elif top=='/amcl_pose': amcl.append((ts,m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)))
    elif top=='/cmd_vel_nav': nav.append((ts,m.linear.x,m.linear.y,m.angular.z))
at=[x[0] for x in amcl]
def ang(d): return (d+math.pi)%(2*math.pi)-math.pi
for a,b in wins:
    print(f"\n### {bag.split('/')[-1][:15]} janela {a:.0f}-{b:.0f}: cada plano -> rumo do ponto a 0.5 m (shim), erro de rumo vs yaw do robo, yaw do goal (ultima pose)")
    prev=None; flips=0; big=0; rows=[]
    for ts,L,h,pt,gy,sy in plans:
        if not (a<=ts<=b): continue
        i=bisect.bisect_left(at,ts); i=min(max(i-1,0),len(amcl)-1); ry=amcl[i][3]
        if h is None: rows.append(f"  t={ts:6.1f} L={L:4.2f} (sem rumo) yaw_robo={math.degrees(ry):5.0f} yaw_goal={math.degrees(gy) if gy is not None else float('nan'):5.0f}"); continue
        err=ang(h-ry); 
        if prev is not None and abs(ang(h-prev))>math.radians(90): flips+=1
        if abs(err)>0.785: big+=1
        prev=h
        rows.append(f"  t={ts:6.1f} L={L:4.2f} rumo={math.degrees(h):5.0f} yaw_robo={math.degrees(ry):5.0f} erro={math.degrees(err):5.0f}{' SHIM' if abs(err)>0.785 else ''} yaw_goal={math.degrees(gy):5.0f} yaw_1a_pose={math.degrees(sy):5.0f}")
    n=len(rows); print(f"  planos={n}; mudancas de rumo >90 graus entre planos consecutivos={flips}; planos com erro>45 graus (shim engaja)={big}")
    step=max(1,n//40)
    for s in rows[::step]: print(s)
