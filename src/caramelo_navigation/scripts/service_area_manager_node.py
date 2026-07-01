#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from caramelo_docking_common import DEFAULT_DOCK_IDS, normalize_dock_id, resolve_map_folder, yaw_from_quaternion
from caramelo_msgs.action import SaveServiceAreaPose
from caramelo_msgs.srv import InitServiceAreas, ListServiceAreas, ValidateServiceAreas
from caramelo_service_area_common import (
    default_service_areas_doc,
    load_service_areas,
    normalized_service_area,
    service_area_path,
    sync_docking_from_service_areas,
    update_service_area_pose,
    validate_service_areas_data,
    write_yaml,
)


def _none_if_empty(value: str):
    value = (value or "").strip()
    return value if value else None


def _positive_or_none(value: float):
    if math.isfinite(float(value)) and float(value) > 0.0:
        return float(value)
    return None


def _number_or_none(value: float):
    if math.isfinite(float(value)):
        return float(value)
    return None


class ServiceAreaManager(Node):
    def __init__(self):
        super().__init__("service_area_manager_node")
        self.declare_parameter("default_map_name", "sala_520")
        self.declare_parameter("default_tf_timeout", 3.0)

        self.default_map_name = self.get_parameter("default_map_name").get_parameter_value().string_value
        self.default_tf_timeout = self.get_parameter("default_tf_timeout").get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_service(InitServiceAreas, "/caramelo/service_areas/init", self._init_service_areas)
        self.create_service(ListServiceAreas, "/caramelo/service_areas/list", self._list_service_areas)
        self.create_service(ValidateServiceAreas, "/caramelo/service_areas/validate", self._validate_service_areas)
        self.save_pose_action = ActionServer(
            self,
            SaveServiceAreaPose,
            "/caramelo/service_areas/save_pose",
            execute_callback=self._execute_save_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("Service Area Manager pronto: services init/list/validate e action save_pose.")

    def destroy_node(self):
        self.save_pose_action.destroy()
        super().destroy_node()

    def _request_map_name(self, map_name: str) -> str:
        return (map_name or self.default_map_name).strip() or "sala_520"

    def _resolve_map(self, map_name: str, map_dir: str):
        return resolve_map_folder(self._request_map_name(map_name), _none_if_empty(map_dir))

    def _goal_callback(self, goal_request):
        try:
            normalize_dock_id(goal_request.name)
        except Exception as exc:
            self.get_logger().warn(f"Rejeitando save_pose: {exc}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _init_service_areas(self, request, response):
        try:
            map_name = self._request_map_name(request.map_name)
            map_folder = self._resolve_map(map_name, request.map_dir)
            path = service_area_path(map_folder)
            created = False
            if not path.exists() or request.force_template:
                area_ids = [normalize_dock_id(area_id) for area_id in (request.area_ids or DEFAULT_DOCK_IDS)]
                write_yaml(path, default_service_areas_doc(map_name, area_ids), make_backup=path.exists())
                created = True
            docking_path = ""
            if request.sync_docking:
                docking_path, _, _ = sync_docking_from_service_areas(map_folder, make_backup=True)
                docking_path = str(docking_path)
            data = load_service_areas(map_folder)
            response.success = True
            response.message = "Service Areas prontas."
            response.map_folder = str(map_folder)
            response.service_areas_path = str(path)
            response.docking_path = docking_path
            response.created = created
            response.area_count = len(data["service_areas"])
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _list_service_areas(self, request, response):
        try:
            map_name = self._request_map_name(request.map_name)
            map_folder = self._resolve_map(map_name, request.map_dir)
            data = load_service_areas(map_folder)
            response.success = True
            response.message = "Service Areas listadas."
            response.map_folder = str(map_folder)
            response.service_areas_path = str(map_folder / "service_areas.yaml")
            for area_id, entry in data["service_areas"].items():
                area = normalized_service_area(area_id, entry, data.get("frame_id", "map"))
                response.area_ids.append(area_id)
                response.area_types.append(area["type"])
                response.x.append(float(area["final_pose"]["x"]))
                response.y.append(float(area["final_pose"]["y"]))
                response.yaw.append(float(area["final_pose"]["yaw"]))
                response.approach_x.append(float(area["approach_pose"]["x"]))
                response.approach_y.append(float(area["approach_pose"]["y"]))
                response.approach_yaw.append(float(area["approach_pose"]["yaw"]))
                response.dock_types.append(area["dock"]["type"])
                response.notes.append(str(area.get("notes", "")))
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _validate_service_areas(self, request, response):
        try:
            map_name = self._request_map_name(request.map_name)
            map_folder = self._resolve_map(map_name, request.map_dir)
            data = load_service_areas(map_folder)
            errors, warnings = validate_service_areas_data(data)
            response.errors = errors
            response.warnings = warnings
            response.map_folder = str(map_folder)
            response.service_areas_path = str(map_folder / "service_areas.yaml")
            response.docking_path = str(map_folder / "docking.yaml")
            response.dock_count = 0
            if errors:
                response.success = False
                response.message = "Service Areas invalidas."
                return response
            if request.sync_docking:
                docking_path, _, dock_count = sync_docking_from_service_areas(map_folder, make_backup=True)
                response.docking_path = str(docking_path)
                response.dock_count = dock_count
            response.success = True
            response.message = "Service Areas validas."
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _publish_feedback(self, goal_handle, state: str, base_frame: str, started_at: float):
        feedback = SaveServiceAreaPose.Feedback()
        feedback.state = state
        feedback.base_frame = base_frame
        feedback.elapsed_seconds = time.monotonic() - started_at
        goal_handle.publish_feedback(feedback)

    def _lookup_pose(self, goal_handle, frame: str, base_frame: str, timeout: float, started_at: float):
        deadline = time.monotonic() + timeout
        tried = []
        while time.monotonic() <= deadline and rclpy.ok():
            if goal_handle.is_cancel_requested:
                raise RuntimeError("SaveServiceAreaPose cancelada.")
            for source_frame in (base_frame, "base_footprint"):
                if source_frame not in tried:
                    tried.append(source_frame)
                try:
                    self._publish_feedback(goal_handle, "waiting_tf", source_frame, started_at)
                    transform = self.tf_buffer.lookup_transform(
                        frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.1),
                    )
                    trans = transform.transform.translation
                    rot = transform.transform.rotation
                    return source_frame, {
                        "x": trans.x,
                        "y": trans.y,
                        "yaw": yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w),
                    }
                except TransformException:
                    continue
            time.sleep(0.05)
        raise RuntimeError(f"Nao consegui obter TF de {frame} para {tried} em {timeout:.1f}s.")

    def _execute_save_pose(self, goal_handle):
        started_at = time.monotonic()
        request = goal_handle.request
        result = SaveServiceAreaPose.Result()
        try:
            map_name = self._request_map_name(request.map_name)
            map_folder = self._resolve_map(map_name, request.map_dir)
            area_id = normalize_dock_id(request.name)
            frame = request.frame or "map"
            base_frame = request.base_frame or "base_link"
            timeout = request.timeout if request.timeout > 0.0 else self.default_tf_timeout

            self._publish_feedback(goal_handle, "resolving_tf", base_frame, started_at)
            source_frame, pose = self._lookup_pose(goal_handle, frame, base_frame, timeout, started_at)
            self._publish_feedback(goal_handle, "writing_yaml", source_frame, started_at)
            service_path, service_backup = update_service_area_pose(
                map_folder=map_folder,
                map_name=map_name,
                area_id=area_id,
                pose=pose,
                area_type=_none_if_empty(request.area_type),
                frame_id=frame,
                approach_offset=_positive_or_none(request.approach_offset),
                docking_offset=_number_or_none(request.docking_offset),
                linear_tolerance=_positive_or_none(request.linear_tolerance),
                angular_tolerance=_positive_or_none(request.angular_tolerance),
                station_width=_positive_or_none(request.station_width),
                station_depth=_positive_or_none(request.station_depth),
                station_height=_positive_or_none(request.station_height),
                free_width=_positive_or_none(request.free_width),
                free_depth=_positive_or_none(request.free_depth),
                dock_type=_none_if_empty(request.dock_type),
                notes=_none_if_empty(request.notes),
                overwrite=request.yes,
            )
            docking_path = ""
            docking_backup = ""
            if request.sync_docking:
                self._publish_feedback(goal_handle, "syncing_docking", source_frame, started_at)
                docking_path_value, docking_backup_value, _ = sync_docking_from_service_areas(map_folder, make_backup=True)
                docking_path = str(docking_path_value)
                docking_backup = str(docking_backup_value or "")
            result.success = True
            result.message = "Service Area salva."
            result.service_areas_path = str(service_path)
            result.service_backup_path = str(service_backup or "")
            result.docking_path = docking_path
            result.docking_backup_path = docking_backup
            result.base_frame_used = source_frame
            result.x = float(pose["x"])
            result.y = float(pose["y"])
            result.yaw = float(pose["yaw"])
            goal_handle.succeed()
        except Exception as exc:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "SaveServiceAreaPose cancelada."
            else:
                goal_handle.abort()
                result.message = str(exc)
            result.success = False
        return result


def main():
    rclpy.init()
    node = ServiceAreaManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
