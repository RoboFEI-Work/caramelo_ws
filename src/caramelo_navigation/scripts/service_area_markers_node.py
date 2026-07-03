#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from caramelo_docking_common import resolve_map_folder
from caramelo_service_area_common import load_service_areas, normalized_service_area


def _quat_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ServiceAreaMarkers(Node):
    def __init__(self):
        super().__init__("service_area_markers_node")
        self.declare_parameter("map_name", "sala_520")
        self.declare_parameter("map_dir", "")
        self.declare_parameter("marker_topic", "/caramelo/service_areas/markers")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("robot_length", 0.71)
        self.declare_parameter("robot_width", 0.47)

        self.map_name = self.get_parameter("map_name").get_parameter_value().string_value
        map_dir = self.get_parameter("map_dir").get_parameter_value().string_value or None
        topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        period = self.get_parameter("publish_period").get_parameter_value().double_value
        self.robot_length = self.get_parameter("robot_length").get_parameter_value().double_value
        self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value

        self.map_folder = resolve_map_folder(self.map_name, map_dir)
        self.service_path = self.map_folder / "service_areas.yaml"
        self.publisher = self.create_publisher(MarkerArray, topic, 10)
        self._service_data = None
        self._mtime = None
        self._next_id = 1
        self.timer = self.create_timer(max(period, 0.2), self._on_timer)
        self.get_logger().info(f"Publicando Service Areas de {self.service_path} em {topic}")

    def _load_if_changed(self):
        mtime = Path(self.service_path).stat().st_mtime
        if self._service_data is None or self._mtime != mtime:
            self._service_data = load_service_areas(self.map_folder)
            self._mtime = mtime

    def _new_marker(self, ns: str, marker_type: int, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = self._next_id
        self._next_id += 1
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    @staticmethod
    def _color(area_type: str, alpha: float = 1.0) -> ColorRGBA:
        palette = {
            "start": (0.1, 0.7, 0.2),
            "finish": (0.9, 0.2, 0.2),
            "workstation": (0.1, 0.45, 0.95),
            "shelf": (0.95, 0.55, 0.1),
            "precision_placement": (0.55, 0.25, 0.9),
            "rotating_table": (0.0, 0.65, 0.65),
        }
        r, g, b = palette.get(area_type, (0.8, 0.8, 0.8))
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))

    def _pose_marker(self, area_id: str, area: dict, markers: MarkerArray):
        pose = area["final_pose"]
        frame_id = area["frame_id"]
        yaw = pose["yaw"]
        color = self._color(area["type"])
        qx, qy, qz, qw = _quat_from_yaw(yaw)

        final = self._new_marker("service_area_final_pose", Marker.SPHERE, frame_id)
        final.pose.position.x = pose["x"]
        final.pose.position.y = pose["y"]
        final.pose.position.z = 0.05
        final.scale.x = 0.16
        final.scale.y = 0.16
        final.scale.z = 0.08
        final.color = color
        markers.markers.append(final)

        arrow = self._new_marker("service_area_orientation", Marker.ARROW, frame_id)
        arrow.points = [
            Point(x=pose["x"], y=pose["y"], z=0.12),
            Point(x=pose["x"] + 0.45 * math.cos(yaw), y=pose["y"] + 0.45 * math.sin(yaw), z=0.12),
        ]
        arrow.scale.x = 0.04
        arrow.scale.y = 0.10
        arrow.scale.z = 0.16
        arrow.color = color
        markers.markers.append(arrow)

        robot = self._new_marker("service_area_robot_footprint", Marker.CUBE, frame_id)
        robot.pose.position.x = pose["x"]
        robot.pose.position.y = pose["y"]
        robot.pose.position.z = 0.02
        robot.pose.orientation.x = qx
        robot.pose.orientation.y = qy
        robot.pose.orientation.z = qz
        robot.pose.orientation.w = qw
        robot.scale.x = self.robot_length
        robot.scale.y = self.robot_width
        robot.scale.z = 0.03
        robot.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.22)
        markers.markers.append(robot)

        text = self._new_marker("service_area_names", Marker.TEXT_VIEW_FACING, frame_id)
        text.pose.position.x = pose["x"]
        text.pose.position.y = pose["y"]
        text.pose.position.z = 0.38
        text.scale.z = 0.18
        text.text = f"{area_id} - pose final do robo"
        text.color = color
        markers.markers.append(text)

    def _station_marker(self, area: dict, markers: MarkerArray):
        station = area["station"]
        if station["width"] <= 0.0 or station["depth"] <= 0.0:
            return
        pose = area["final_pose"]
        yaw = pose["yaw"]
        frame_id = area["frame_id"]
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        center_x = pose["x"] + 0.5 * station["depth"] * math.cos(yaw)
        center_y = pose["y"] + 0.5 * station["depth"] * math.sin(yaw)

        marker = self._new_marker("service_area_station", Marker.CUBE, frame_id)
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.03
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = station["depth"]
        marker.scale.y = station["width"]
        marker.scale.z = 0.06
        marker.color = self._color(area["type"], 0.35)
        markers.markers.append(marker)

    def _free_area_marker(self, area: dict, markers: MarkerArray):
        free_area = area["free_area"]
        pose = area["final_pose"]
        yaw = pose["yaw"]
        frame_id = area["frame_id"]
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        depth = max(float(free_area["depth"]), 0.01)
        width = max(float(free_area["width"]), self.robot_width)
        center_x = pose["x"] - 0.5 * depth * math.cos(yaw)
        center_y = pose["y"] - 0.5 * depth * math.sin(yaw)

        marker = self._new_marker("service_area_free_front", Marker.CUBE, frame_id)
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.01
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = depth
        marker.scale.y = width
        marker.scale.z = 0.02
        marker.color = ColorRGBA(r=0.0, g=0.85, b=0.25, a=0.14)
        markers.markers.append(marker)

    def _approach_marker(self, area_id: str, area: dict, markers: MarkerArray):
        pose = area["approach_pose"]
        frame_id = area["frame_id"]
        yaw = pose["yaw"]
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        marker = self._new_marker("service_area_approach_pose", Marker.CUBE, frame_id)
        marker.pose.position.x = pose["x"]
        marker.pose.position.y = pose["y"]
        marker.pose.position.z = 0.04
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.04
        marker.color = self._color(area["type"], 0.55)
        markers.markers.append(marker)

        text = self._new_marker("service_area_approach_names", Marker.TEXT_VIEW_FACING, frame_id)
        text.pose.position.x = pose["x"]
        text.pose.position.y = pose["y"]
        text.pose.position.z = 0.28
        text.scale.z = 0.12
        text.text = f"{area_id}_AP"
        text.color = self._color(area["type"], 0.75)
        markers.markers.append(text)

    def _on_timer(self):
        try:
            self._load_if_changed()
            markers = MarkerArray()
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            markers.markers.append(delete_all)
            self._next_id = 1
            for area_id, entry in self._service_data["service_areas"].items():
                area = normalized_service_area(area_id, entry, self._service_data.get("frame_id", "map"))
                self._station_marker(area, markers)
                self._free_area_marker(area, markers)
                self._approach_marker(area_id, area, markers)
                self._pose_marker(area_id, area, markers)
            self.publisher.publish(markers)
        except Exception as exc:
            self.get_logger().error(f"Falha publicando Service Areas: {exc}")


def main():
    rclpy.init()
    node = ServiceAreaMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
