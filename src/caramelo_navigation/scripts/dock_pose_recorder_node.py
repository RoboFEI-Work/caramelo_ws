#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from caramelo_docking_common import default_dock_type, infer_kind, normalize_dock_id, resolve_map_folder, update_dock_pose, yaw_from_quaternion


class DockPoseRecorder(Node):
    def __init__(self):
        super().__init__("dock_pose_recorder_node")
        self.declare_parameter("map_name", "")
        self.declare_parameter("map_dir", "")
        self.declare_parameter("frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("update_service_area", True)
        self.declare_parameter("timeout", 1.0)
        self.map_name = self.get_parameter("map_name").value
        self.map_dir = self.get_parameter("map_dir").value or None
        self.frame = self.get_parameter("frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.update_service_area = bool(self.get_parameter("update_service_area").value)
        self.timeout = float(self.get_parameter("timeout").value)
        if not self.map_name:
            raise RuntimeError("Parametro map_name e obrigatorio.")
        self.map_folder = resolve_map_folder(self.map_name, self.map_dir)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(String, "/save_dock_pose", self._callback, 10)
        self.get_logger().info(f"Gravador de docks ativo para mapa '{self.map_name}' em {self.map_folder}")
        self.get_logger().info("Publique 'WS1' ou 'WS1:caramelo_front_dock' em /save_dock_pose.")

    def _lookup_pose(self):
        tried = []
        for source_frame in (self.base_frame, "base_footprint"):
            if source_frame in tried:
                continue
            tried.append(source_frame)
            try:
                transform = self.buffer.lookup_transform(self.frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=self.timeout))
                trans = transform.transform.translation
                rot = transform.transform.rotation
                yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
                return source_frame, [trans.x, trans.y, yaw]
            except TransformException as exc:
                self.get_logger().warn(f"Falha buscando TF {self.frame} -> {source_frame}: {exc}")
        raise RuntimeError(f"Nao consegui obter TF de {self.frame} para {tried}.")

    def _callback(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            self.get_logger().warn("Mensagem vazia em /save_dock_pose; ignorando.")
            return
        if ":" in raw:
            dock_id_raw, dock_type_raw = raw.split(":", 1)
            dock_type = dock_type_raw.strip() or None
        else:
            dock_id_raw = raw
            dock_type = None
        try:
            dock_id = normalize_dock_id(dock_id_raw)
            source_frame, pose = self._lookup_pose()
            fields = {"kind": infer_kind(dock_id)}
            update_dock_pose(self.map_folder, dock_id, dock_type or default_dock_type(dock_id), self.frame, pose, self.update_service_area, fields)
        except Exception as exc:
            self.get_logger().error(f"Nao foi possivel salvar dock '{raw}': {exc}")
            return
        self.get_logger().info(
            f"Dock salvo: {dock_id} type={dock_type or default_dock_type(dock_id)} "
            f"pose=[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.6f}] base={source_frame}"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DockPoseRecorder()
    except Exception as exc:
        print(f"Erro ao iniciar dock_pose_recorder_node: {exc}")
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
