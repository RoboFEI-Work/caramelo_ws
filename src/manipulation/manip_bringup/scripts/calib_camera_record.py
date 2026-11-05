#!/usr/bin/env python3
"""Grava leituras de tags PARADAS para calibrar a montagem da camera do punho.

2026-11-04: a camera do punho esta no URDF com a montagem NOMINAL
(caramelo_description manipulator_arm.xacro: xyz 0 -0.055 0.0125, rpy
0 -90 -90 graus). Um erro de poucos graus nessa montagem faz a MESMA tag
aparecer em lugares diferentes conforme a pose do braco (log de 04/11: a tag
do slot muda 3-5 cm entre a leitura de longe e a de perto, e o z do tampo
varia 2,6 cm entre tags da mesma mesa). Este no NAO mexe no braco: so grava.

Uso: tags paradas na mesa (101..107 / 200 / cubos que ninguem vai tocar),
rodar este no e levar o braco a 15-30 poses DIFERENTES com as tags a vista
(rodar uma AMT1, ou mover pelo teleop/RViz) - varie altura, inclinacao,
giro da j1 e lado. Cada amostra so entra com o braco PARADO.

Pode ficar ligado durante a task inteira: toda vez que a BASE anda (nudge,
navegacao, docking - visto pelo TF odom <- manip_base_link) comeca um
"trecho" novo, e o solver trata a mesma tag em trechos diferentes como
pontos diferentes (ela mudou de lugar em relacao ao braco).

  ros2 run manip_bringup calib_camera_record.py --ros-args -p out:=/tmp/calib.jsonl

Depois: calib_camera_solve.py /tmp/calib.jsonl
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def tf_to_mat(t):
    q = t.transform.rotation
    p = t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = [p.x, p.y, p.z]
    return m


class CalibRecorder(Node):
    def __init__(self):
        super().__init__('calib_camera_record')
        self.base = self.declare_parameter('base_frame', 'manip_base_link').value
        self.flange = self.declare_parameter('flange_frame', 'link5').value
        self.optical = self.declare_parameter('optical_frame', 'camera_color_optical_frame').value
        ids = self.declare_parameter(
            'tag_ids', [101, 102, 103, 104, 105, 106, 107, 200]).value
        self.tags = ['tag_%d' % i for i in ids]
        self.out = os.path.expanduser(self.declare_parameter('out', '/tmp/calib_camera.jsonl').value)
        # Braco parado: flange nao andou mais que isto em `still_window_s`.
        self.still_m = self.declare_parameter('still_m', 0.001).value
        self.still_rad = math.radians(self.declare_parameter('still_deg', 0.3).value)
        self.still_window = self.declare_parameter('still_window_s', 0.4).value
        # Uma amostra por tag por pose: so grava de novo depois de o flange
        # andar `new_pose_m` ou girar `new_pose_deg`.
        self.new_pose_m = self.declare_parameter('new_pose_m', 0.02).value
        self.new_pose_rad = math.radians(self.declare_parameter('new_pose_deg', 5.0).value)
        # Trecho novo quando a base anda mais que isto (odom <- base do braco).
        self.odom = self.declare_parameter('odom_frame', 'odom').value
        self.seg_m = self.declare_parameter('segment_move_m', 0.005).value
        self.seg_rad = math.radians(self.declare_parameter('segment_move_deg', 0.5).value)
        self.segment = 0
        self.seg_ref = None

        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.last_stamp = {}
        self.last_pose = {}
        self.count = 0
        self.fh = open(self.out, 'a')
        self.nominal_written = False
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'[CALIB] gravando em {self.out}: {self.base} <- {self.flange}, {self.optical} -> '
            f'{", ".join(self.tags)}. Leve o braco a poses DIFERENTES com as tags a vista.')

    @staticmethod
    def pose_delta(a, b):
        dp = np.linalg.norm(a[:3, 3] - b[:3, 3])
        rr = a[:3, :3].T @ b[:3, :3]
        ang = math.acos(max(-1.0, min(1.0, (np.trace(rr) - 1.0) / 2.0)))
        return dp, ang

    def tick(self):
        if not self.nominal_written:
            try:
                nom = tf_to_mat(self.buf.lookup_transform(self.flange, self.optical, Time()))
                self.fh.write(json.dumps({'nominal_flange_T_optical': nom.tolist()}) + '\n')
                self.fh.flush()
                self.nominal_written = True
            except tf2_ros.TransformException:
                return
        for tag in self.tags:
            try:
                ot = self.buf.lookup_transform(self.optical, tag, Time())
            except tf2_ros.TransformException:
                continue
            stamp = Time.from_msg(ot.header.stamp)
            now = self.get_clock().now()
            if (now - stamp).nanoseconds > 0.5e9 or self.last_stamp.get(tag) == stamp.nanoseconds:
                continue
            try:
                bf = tf_to_mat(self.buf.lookup_transform(self.base, self.flange, stamp))
                bf_before = tf_to_mat(self.buf.lookup_transform(
                    self.base, self.flange, stamp - Duration(seconds=self.still_window)))
            except tf2_ros.TransformException:
                continue
            dp, ang = self.pose_delta(bf, bf_before)
            if dp > self.still_m or ang > self.still_rad:
                continue  # braco andando
            try:
                ob = tf_to_mat(self.buf.lookup_transform(self.odom, self.base, stamp))
            except tf2_ros.TransformException:
                ob = None
            if ob is not None:
                if self.seg_ref is None:
                    self.seg_ref = ob
                else:
                    dps, angs = self.pose_delta(ob, self.seg_ref)
                    if dps > self.seg_m or angs > self.seg_rad:
                        self.segment += 1
                        self.seg_ref = ob
                        self.last_pose.clear()
                        self.get_logger().info(
                            f'[CALIB] a base andou {dps * 1000:.0f} mm / {math.degrees(angs):.1f} graus '
                            f'- trecho {self.segment}.')
            prev = self.last_pose.get(tag)
            if prev is not None:
                dp2, ang2 = self.pose_delta(bf, prev)
                if dp2 < self.new_pose_m and ang2 < self.new_pose_rad:
                    continue  # mesma pose
            self.last_stamp[tag] = stamp.nanoseconds
            self.last_pose[tag] = bf
            self.fh.write(json.dumps({
                'tag': tag,
                'segment': self.segment,
                'stamp': stamp.nanoseconds,
                'base_T_flange': bf.tolist(),
                'optical_T_tag': tf_to_mat(ot).tolist(),
            }) + '\n')
            self.fh.flush()
            self.count += 1
            self.get_logger().info(
                f'[CALIB] amostra {self.count}: {tag} trecho {self.segment} (flange em '
                f'[{bf[0, 3]:+.3f} {bf[1, 3]:+.3f} {bf[2, 3]:+.3f}])')


def main():
    rclpy.init()
    node = CalibRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'[CALIB] {node.count} amostra(s) em {node.out}')
        node.fh.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
