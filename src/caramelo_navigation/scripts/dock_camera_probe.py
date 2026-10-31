#!/usr/bin/env python3
"""Mede a mesa pela camera do punho SEM mover o robo (2026-10-27).

Para validar e calibrar o docking por camera (camera_* no dock_align_node):
robo parado de frente para a mesa, braco em `home`. Imprime, por frame, o
yaw da face, o centro (c_axis/c_foot), a largura, a distancia e a inclinacao
corrigida — a mesma conta do dock_align_node (caramelo_dock_camera).

Calibracao: robo docado PERFEITO (trena nos cantos) ->
  camera_yaw_offset = -(yaw medido em rad), camera_y_offset = -(c_foot em m).

  ros2 run caramelo_navigation dock_camera_probe [--frames 20]   (0 = sem parar)
  ros2 run rqt_image_view rqt_image_view /dock_align/camera_debug   (o que ele ve)
Nenhuma dimensao de mesa e' configurada: largura, profundidade e altura saem
medidas (a mesa e' reconhecida pelo formato).
"""
import argparse
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import caramelo_dock_camera as dock_cam  # noqa: E402
import caramelo_dock_camera_debug as dock_cam_dbg  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    ap.add_argument("--info-topic", default="/camera/camera/depth/camera_info")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--debug-topic", default="/dock_align/camera_debug",
                    help="imagem de debug (mesmo topico do dock_align_node)")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node("dock_camera_probe")
    state = {"depth": None, "k": None, "seq": 0}

    def on_depth(msg):
        state["depth"] = msg
        state["seq"] += 1

    def on_info(msg):
        state["k"] = np.array(msg.k, dtype=float).reshape(3, 3)

    # RELIABLE: em best-effort o depth (600 KB) chegava a ~0,3 frame/s.
    node.create_subscription(Image, args.depth_topic, on_depth,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(CameraInfo, args.info_topic, on_info, qos_profile_sensor_data)
    dbg_pub = node.create_publisher(Image, args.debug_topic, 1)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    cfg = dock_cam.CamCfg()

    yaws, feet = [], []
    last = 0
    done = 0
    try:
        while rclpy.ok() and (args.frames <= 0 or done < args.frames):
            rclpy.spin_once(node, timeout_sec=0.2)
            msg = state["depth"]
            if msg is None or state["seq"] == last or state["k"] is None:
                continue
            last = state["seq"]
            try:
                tf = buf.lookup_transform(args.base_frame, msg.header.frame_id, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                print(f"sem TF ainda: {exc}")
                continue
            q, t = tf.transform.rotation, tf.transform.translation
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.step // 2)[:, :msg.width]
            m, why = dock_cam.measure_depth_frame(
                depth, state["k"], dock_cam.quat_to_rot(q.x, q.y, q.z, q.w),
                (t.x, t.y, t.z), cfg)
            done += 1
            img = dock_cam_dbg.draw(depth, m, f"probe frame {done}", why, cfg.stride)
            out = Image()
            out.header = msg.header
            out.height, out.width = img.shape[:2]
            out.encoding = "bgr8"
            out.step = out.width * 3
            out.data = img.tobytes()
            dbg_pub.publish(out)
            if m is None or not m.ok:
                print(f"[{done:2d}] sem mesa: {why or m.reason}  {'' if m is None else m.info}")
                continue
            yaws.append(m.face_yaw)
            if m.c_foot is not None:
                feet.append(m.c_foot)
            print(f"[{done:2d}] yaw={math.degrees(m.face_yaw):+6.2f} graus ({m.yaw_src:12s}) "
                  f"c_axis={'   ?  ' if m.c_axis is None else '%+6.1f' % (m.c_axis * 100)} "
                  f"c_foot={'   ?  ' if m.c_foot is None else '%+6.1f' % (m.c_foot * 100)} cm "
                  f"largura={'  ?  ' if m.width is None else '%.3f' % m.width} ({m.center_reason or m.sides}) "
                  f"front={m.front:.3f} h={m.table_h:.3f} incl={m.tilt_deg:.1f} graus")
    finally:
        if yaws:
            ys = sorted(yaws)
            print(f"\nmediana yaw = {math.degrees(ys[len(ys) // 2]):+.2f} graus "
                  f"({len(ys)} frames, dispersao {math.degrees(ys[-1] - ys[0]):.2f})")
        if feet:
            fs = sorted(feet)
            print(f"mediana c_foot = {fs[len(fs) // 2] * 100:+.1f} cm "
                  f"(dispersao {(fs[-1] - fs[0]) * 100:.1f})")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
