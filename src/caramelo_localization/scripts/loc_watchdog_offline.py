#!/usr/bin/env python3
"""Roda o vigia de localizacao sobre um bag (/amcl_pose + /odom) e lista quando
ele teria mandado reiniciar o AMCL. Uso: loc_watchdog_offline.py <bag> [...]"""
import bisect
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loc_watchdog_core import Vigia, Params  # noqa: E402


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    for bag in sys.argv[1:]:
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
               rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        r.set_filter(rosbag2_py.StorageFilter(topics=['/amcl_pose', '/odom']))
        M = {t: get_message(types[t]) for t in ['/amcl_pose', '/odom']}
        t0 = None
        od = []
        am = []
        while r.has_next():
            top, data, t = r.read_next()
            if t0 is None:
                t0 = t
            ts = (t - t0) / 1e9
            m = deserialize_message(data, M[top])
            pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))
            (od if top == '/odom' else am).append((ts, pose))
        ot = [x[0] for x in od]
        v = Vigia(Params())
        j = 0
        n_ok = n_fora = 0
        for ts, pose in am:
            while j < len(od) and od[j][0] <= ts:
                v.odom(od[j][0], od[j][1])
                j += 1
            i = max(0, min(bisect.bisect_left(ot, ts), len(od) - 1))
            res = v.amcl(ts, pose, od[i][1])
            if hasattr(v, 'ultimo'):
                dxy, dyaw, fxy, fyaw, _ = v.ultimo
                if dxy <= fxy and dyaw <= fyaw:
                    n_ok += 1
                else:
                    n_fora += 1
        print(f"#### {os.path.basename(bag)[:15]}: {len(am)} atualizacoes do AMCL; dentro da folga {n_ok}, "
              f"fora {n_fora}; RESETS: {len(v.e.resets)}")
        for t, dxy, dyaw, esp, amc in v.e.resets:
            print(f"   t={t:6.1f}s desvio {dxy:.2f} m / {math.degrees(dyaw):.0f} graus: AMCL dizia "
                  f"({amc[0]:.2f},{amc[1]:.2f},{math.degrees(amc[2]):.0f}) -> reinicia em "
                  f"({esp[0]:.2f},{esp[1]:.2f},{math.degrees(esp[2]):.0f})")


if __name__ == '__main__':
    main()
