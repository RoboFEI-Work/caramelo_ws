import sys, math, bisect, yaml, numpy as np, rosbag2_py
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, a, b, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
cm_times = [float(x) for x in sys.argv[5].split(',')] if len(sys.argv) > 5 else []
MAPD='/home/robofei-work/caramelo_ws/src/caramelo_mapping/maps/arena_extendida/'
my=yaml.safe_load(open(MAPD+'map.yaml')); img=np.array(Image.open(MAPD+my['image'])); res=my['resolution']; ox,oy=my['origin'][:2]
H,W=img.shape; ext=[ox,ox+W*res,oy,oy+H*res]
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def comp(T1,T2):  # T=(x,y,th): T1 then T2 (T1*T2)
    x1,y1,t1=T1; x2,y2,t2=T2; return (x1+math.cos(t1)*x2-math.sin(t1)*y2, y1+math.sin(t1)*x2+math.cos(t1)*y2, t1+t2)
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
TOP=['/tf','/tf_static','/plan','/scan','/amcl_pose','/local_costmap/costmap','/local_costmap/published_footprint']
r.set_filter(rosbag2_py.StorageFilter(topics=TOP)); M={t:get_message(types[t]) for t in TOP}
t0=None; m2o=[]; o2b=[]; plans=[]; scans=[]; amcl=[]; cms=[]; fps=[]; stat={}
while r.has_next():
    top,data,t=r.read_next()
    if t0 is None: t0=t
    ts=(t-t0)/1e9
    if top=='/tf_static':
        for tr in deserialize_message(data,M[top]).transforms:
            stat[(tr.header.frame_id,tr.child_frame_id)]=(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)); continue
    if ts<a-2: continue
    if ts>b+2: break
    m=deserialize_message(data,M[top])
    if top=='/tf':
        for tr in m.transforms:
            T=(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation))
            if tr.header.frame_id=='map' and tr.child_frame_id=='odom': m2o.append((ts,T))
            elif tr.header.frame_id=='odom' and tr.child_frame_id in ('base_footprint','base_link'): o2b.append((ts,T))
    elif top=='/plan': plans.append((ts,[(p.pose.position.x,p.pose.position.y) for p in m.poses]))
    elif top=='/scan': scans.append((ts,m.angle_min,m.angle_increment,np.array(m.ranges)))
    elif top=='/amcl_pose': amcl.append((ts,m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)))
    elif top=='/local_costmap/costmap': cms.append((ts,m.info.origin.position.x,m.info.origin.position.y,m.info.width,m.info.height,m.info.resolution,np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)))
    elif top=='/local_costmap/published_footprint': fps.append((ts,[(p.x,p.y) for p in m.polygon.points]))
# laser chain
laser=(0,0,0)
for (p,c),T in stat.items():
    if 'laser' in c or 'lidar' in c or 'scan' in c:
        # chain base_footprint->base_link->laser
        chain=T; par=p
        while par not in ('base_footprint',):
            found=False
            for (pp,cc),TT in stat.items():
                if cc==par: chain=comp(TT,chain); par=pp; found=True; break
            if not found: break
        laser=chain; print('laser frame',c,'em base_footprint:',np.round(laser,3))
def near(lst,ts):
    i=bisect.bisect_left([x[0] for x in lst],ts); i=min(max(i,0),len(lst)-1)
    if i>0 and abs(lst[i-1][0]-ts)<abs(lst[i][0]-ts): i-=1
    return lst[i]
def pose_at(ts):
    return comp(near(m2o,ts)[1], near(o2b,ts)[1])
fig,axs=plt.subplots(1,2 if cm_times else 1,figsize=(22,10) if cm_times else (14,10),squeeze=False); ax=axs[0][0]
ax.imshow(img,cmap='gray',extent=ext,origin='upper'); ax.set_xlim(a_:=None or ext[0],ext[1]); ax.set_ylim(ext[2],ext[3])
# plans
A=[p for p in plans if a<=p[0]<=b]; cmap=plt.get_cmap('cool')
for i,(ts,pts) in enumerate(A[::max(1,len(A)//25)]):
    if len(pts)>1: P=np.array(pts); ax.plot(P[:,0],P[:,1],'-',lw=0.8,alpha=0.6,color=cmap(i/max(1,len(A[::max(1,len(A)//25)])-1)))
# scans every ~4 s
S=[s for s in scans if a<=s[0]<=b]
for ts,am,inc,rg in S[::40]:
    T=comp(pose_at(ts),laser); ang=am+inc*np.arange(len(rg)); ok=np.isfinite(rg)&(rg>0.05)&(rg<6)
    xs=T[0]+rg[ok]*np.cos(T[2]+ang[ok]); ys=T[1]+rg[ok]*np.sin(T[2]+ang[ok]); ax.plot(xs,ys,'.',ms=1.5,color='red',alpha=0.35)
# trail
Am=np.array([x for x in amcl if a<=x[0]<=b])
if len(Am):
    sc=ax.scatter(Am[:,1],Am[:,2],c=Am[:,0],cmap='viridis',s=10,zorder=5); plt.colorbar(sc,ax=ax,label='t [s]',fraction=0.03)
    for row in Am[::max(1,len(Am)//40)]: ax.arrow(row[1],row[2],0.15*math.cos(row[3]),0.15*math.sin(row[3]),head_width=0.05,color='k',zorder=6)
    ax.plot(Am[0,1],Am[0,2],'g*',ms=16,zorder=7); ax.plot(Am[-1,1],Am[-1,2],'rX',ms=14,zorder=7)
# footprint at end
if fps:
    ts,poly=near(fps,b); P=np.array(poly+[poly[0]]); ax.plot(P[:,0],P[:,1],'m-',lw=2,zorder=8)
ax.set_title(f'{bag.split("/")[-1][:15]}  {a:.0f}-{b:.0f}s  planos(cool, {len(A)}), scan(vermelho), trilha AMCL, footprint final (magenta)'); ax.set_aspect('equal'); ax.grid(alpha=0.3)
ax.set_xlim(max(ext[0],Am[:,1].min()-2.5),min(ext[1],Am[:,1].max()+2.5)); ax.set_ylim(max(ext[2],Am[:,2].min()-2.0),min(ext[3],Am[:,2].max()+2.0))
for k,tc in enumerate(cm_times):
    ax2=axs[0][1] if k==0 else axs[0][1]
    ts,cx,cy,w,h,cres,grid=near(cms,tc); m2oT=near(m2o,ts)[1]
    # transform costmap corners to map: draw in odom frame but transform footprint/scans into odom
    im=ax2.imshow(grid,cmap='magma',extent=[cx,cx+w*cres,cy,cy+h*cres],origin='lower',vmin=0,vmax=254)
    ob=near(o2b,ts)[1]; fp=[(0.395,0.055),(0.395,-0.055),(0.285,-0.055),(0.285,-0.195),(-0.285,-0.195),(-0.285,0.195),(0.285,0.195),(0.285,0.055)]
    F=np.array([comp(ob,(x,y,0))[:2] for x,y in fp]+[comp(ob,(fp[0][0],fp[0][1],0))[:2]]); ax2.plot(F[:,0],F[:,1],'c-',lw=2)
    ax2.arrow(ob[0],ob[1],0.3*math.cos(ob[2]),0.3*math.sin(ob[2]),head_width=0.06,color='c')
    s=near(scans,ts); T=comp(ob,laser); ang=s[1]+s[2]*np.arange(len(s[3])); ok=np.isfinite(s[3])&(s[3]>0.05)&(s[3]<4)
    ax2.plot(T[0]+s[3][ok]*np.cos(T[2]+ang[ok]),T[1]+s[3][ok]*np.sin(T[2]+ang[ok]),'.',ms=2,color='lime')
    ax2.set_xlim(ob[0]-1.5,ob[0]+1.5); ax2.set_ylim(ob[1]-1.5,ob[1]+1.5); ax2.set_aspect('equal')
    mp=comp(m2oT,ob); ax2.set_title(f'costmap local (odom) t={ts:.1f}s; robo em map ({mp[0]:.2f},{mp[1]:.2f},{math.degrees(mp[2]):.0f}deg); scan=verde'); plt.colorbar(im,ax=ax2,fraction=0.03)
    break
plt.tight_layout(); plt.savefig(out,dpi=90); print('ok',out)
