#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    """Republish LaserScan messages on a fixed angular grid.

    Some LiDAR drivers publish a different number of ranges from scan to scan.
    slam_toolbox/Karto expects a stable laser model, so this adapter keeps the
    original angles and resamples each scan into a constant-size ranges array.
    """

    def __init__(self) -> None:
        super().__init__('scan_normalizer')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_fixed')
        self.declare_parameter('target_count', 0)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('range_min', 0.0)
        self.declare_parameter('fill_with_inf', True)
        self.declare_parameter('keep_intensities', True)

        self._target_count = int(self.get_parameter('target_count').value)
        self._range_max_override = float(self.get_parameter('range_max').value)
        self._range_min_override = float(self.get_parameter('range_min').value)
        self._fill_with_inf = bool(self.get_parameter('fill_with_inf').value)
        self._keep_intensities = bool(self.get_parameter('keep_intensities').value)

        self._angle_min: Optional[float] = None
        self._angle_max: Optional[float] = None
        self._angle_increment: Optional[float] = None
        self._reported_geometry = False
        self._reported_resize = False

        input_topic = str(self.get_parameter('input_scan_topic').value)
        output_topic = str(self.get_parameter('output_scan_topic').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(LaserScan, output_topic, qos)
        self._subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            qos,
        )

        self.get_logger().info(
            f'Normalizing LaserScan from {input_topic} to {output_topic}'
        )

    def _initialize_geometry(self, scan: LaserScan) -> bool:
        if self._angle_min is not None:
            return True

        if self._target_count <= 0:
            if scan.angle_increment > 0.0 and scan.angle_max > scan.angle_min:
                self._target_count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
            else:
                self._target_count = len(scan.ranges)

        if self._target_count < 2:
            self.get_logger().error('Cannot normalize scan: target_count must be >= 2')
            return False

        self._angle_min = float(scan.angle_min)
        self._angle_max = float(scan.angle_max)
        self._angle_increment = (self._angle_max - self._angle_min) / float(self._target_count - 1)
        self._reported_geometry = True

        self.get_logger().info(
            'Fixed scan geometry: '
            f'count={self._target_count}, '
            f'angle_min={self._angle_min:.6f}, '
            f'angle_max={self._angle_max:.6f}, '
            f'angle_increment={self._angle_increment:.9f}'
        )
        return True

    def _scan_callback(self, scan: LaserScan) -> None:
        if not self._initialize_geometry(scan):
            return

        assert self._angle_min is not None
        assert self._angle_max is not None
        assert self._angle_increment is not None

        output = LaserScan()
        output.header = scan.header
        output.angle_min = self._angle_min
        output.angle_max = self._angle_max
        output.angle_increment = self._angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = self._range_min_override if self._range_min_override > 0.0 else scan.range_min
        output.range_max = self._range_max_override if self._range_max_override > 0.0 else scan.range_max

        fill_value = math.inf if self._fill_with_inf else 0.0
        ranges = [fill_value] * self._target_count
        intensities = [0.0] * self._target_count if self._keep_intensities and scan.intensities else []

        input_count = len(scan.ranges)
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + float(index) * scan.angle_increment
            target_index = int(round((angle - self._angle_min) / self._angle_increment))
            if target_index < 0 or target_index >= self._target_count:
                continue

            if math.isfinite(value) and value > 0.0:
                ranges[target_index] = float(value)
            else:
                ranges[target_index] = value

            if intensities and index < len(scan.intensities):
                intensities[target_index] = float(scan.intensities[index])

        output.ranges = ranges
        output.intensities = intensities

        if input_count != self._target_count and not self._reported_resize:
            self.get_logger().warn(
                f'Resampling variable-size LaserScan: first changed count {input_count} -> {self._target_count}. '
                'Further resize warnings are suppressed.'
            )
            self._reported_resize = True

        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
