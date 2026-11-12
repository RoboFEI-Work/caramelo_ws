import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
LV={10:'DBG',20:'INFO',30:'WARN',40:'ERR',50:'FATAL'}
for path in sys.argv[1:]:
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/rosout']))
    M=get_message(types['/rosout']); t0=None
    out=open(path.rstrip('/').split('/')[-1][:15]+'_rosout.txt','w')
    while r.has_next():
        top,data,t=r.read_next()
        if t0 is None: t0=t
        m=deserialize_message(data,M)
        out.write(f"{(t-t0)/1e9:8.2f} {LV.get(m.level,m.level):5s} [{m.name}] {m.msg}\n")
    out.close()
