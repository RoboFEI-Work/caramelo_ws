import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
TOP=['/cmd_vel_nav','/cmd_vel','/mecanum_controller/reference','/odom','/odom/wheel','/amcl_pose',
     '/follow_path/_action/status','/navigate_to_pose/_action/status','/collision_monitor_state',
     '/align_to_dock/_action/feedback','/plan','/local_plan','/maxon/wheel_velocity','/scan','/local_costmap/published_footprint']
for path in sys.argv[1:]:
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOP if t in types]))
    M={t:get_message(types[t]) for t in TOP if t in types}
    t0=None; out=open(path.rstrip('/').split('/')[-1][:15]+'_data.csv','w')
    out.write("t,kind,a,b,c,d\n")
    def w(t,k,*v): out.write(f"{(t-t0)/1e9:.3f},{k},"+",".join(f"{x}" if isinstance(x,str) else f"{x:.4f}" for x in v)+"\n")
    while r.has_next():
        top,data,t=r.read_next()
        if t0 is None: t0=t
        m=deserialize_message(data,M[top])
        if top=='/cmd_vel_nav': w(t,'nav',m.linear.x,m.linear.y,m.angular.z,0)
        elif top=='/cmd_vel': w(t,'cmd',m.linear.x,m.linear.y,m.angular.z,0)
        elif top=='/mecanum_controller/reference': w(t,'ref',m.twist.linear.x,m.twist.linear.y,m.twist.angular.z,0)
        elif top=='/odom': w(t,'odom',m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z,yaw(m.pose.pose.orientation))
        elif top=='/odom/wheel': w(t,'wodom',m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z,0)
        elif top=='/amcl_pose':
            c=m.pose.covariance; w(t,'amcl',m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),math.sqrt(c[0]+c[7]))
        elif top=='/follow_path/_action/status':
            if m.status_list: s=m.status_list[-1]; w(t,'fps',bytes(s.goal_info.goal_id.uuid).hex()[:8],s.status,0,0)
        elif top=='/navigate_to_pose/_action/status':
            if m.status_list: s=m.status_list[-1]; w(t,'nps',bytes(s.goal_info.goal_id.uuid).hex()[:8],s.status,0,0)
        elif top=='/collision_monitor_state': w(t,'cm',m.polygon_name or '-',m.action_type,0,0)
        elif top=='/align_to_dock/_action/feedback':
            fb=m.feedback; fields=[f for f in fb.get_fields_and_field_types()]
            vals=[]
            for f in fields[:4]:
                v=getattr(fb,f); vals.append(float(v) if isinstance(v,(int,float,bool)) else 0.0)
            while len(vals)<4: vals.append(0.0)
            w(t,'atd',*vals)
        elif top=='/plan':
            P=m.poses; L=sum(math.hypot(P[i+1].pose.position.x-P[i].pose.position.x,P[i+1].pose.position.y-P[i].pose.position.y) for i in range(len(P)-1))
            w(t,'plan',len(P),L,(P[-1].pose.position.x if P else 0),(P[-1].pose.position.y if P else 0))
        elif top=='/local_plan':
            P=m.poses; w(t,'lplan',len(P),(P[-1].pose.position.x-P[0].pose.position.x) if len(P)>1 else 0,(P[-1].pose.position.y-P[0].pose.position.y) if len(P)>1 else 0,0)
        elif top=='/maxon/wheel_velocity':
            d=list(m.data); w(t,'wheel',*(d+[0]*4)[:4])
        elif top=='/scan':
            rg=[x for x in m.ranges if x>0.05 and math.isfinite(x)]
            n=len(m.ranges); 
            def sec(a0,a1):
                i0=int((a0-m.angle_min)/m.angle_increment); i1=int((a1-m.angle_min)/m.angle_increment)
                v=[x for x in m.ranges[max(0,i0):min(n,i1)] if x>0.05 and math.isfinite(x)]; return min(v) if v else 9.9
            w(t,'scan',min(rg) if rg else 9.9,sec(-1.57,-0.785),sec(-0.785,0.785),sec(0.785,1.57))
    out.close(); print(path, 'ok')
