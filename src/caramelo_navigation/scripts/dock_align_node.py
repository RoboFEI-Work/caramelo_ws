#!/usr/bin/env python3
"""Alinhamento fino HOLONOMICO a um dock (ADR-05, opcao c).

Complementa o opennav_docking: o controller do Docking Server e' unicycle
(so linear.x + angular.z, sem vy), entao ele nao usa as rodas mecanum para
corrigir lateralmente. Este no faz o trecho FINAL de alinhamento com holonomia
(vx, vy, wz), depois que o Docking Server ja levou o robo ate o staging/approach.

O que este no faz:
  1. Le a pose alvo do dock em docking.yaml (mesma base do opennav_docking).
  2. (Opcional) refina a pose alvo pela FACE da mesa no /scan -> corrige a
     distancia (X) e o yaw, que sao observaveis numa face reta.
  3. Servo-controle holonomico (P) ate a tolerancia, publicando TwistStamped.

LIMITES HONESTOS (ADR-05):
  - Y (lateral ao longo da face) NAO e' observavel numa face reta (problema da
    abertura). Este no deixa o Y por conta da localizacao (AMCL/EKF, ~2-5 cm).
    Corrigir Y de verdade exige detectar os CANTOS da mesa (casar retangulo
    ~80x50 cm) -> marcado como AJUSTAR_NO_ROBO em _refine_target_from_scan().
  - PISO DO ESC (2026-08-03, firmware corrigido): 0.1215 m/s / 0.315 rad/s de
    corpo, EXECUTADOS EXATOS (ganhos do PI + mapa ancorado; ver
    caramelo_hardware_ws/docs). O piso e por RODA (2.43 rad/s): em comandos
    mistos vx/vy/wz uma roda individual pode ficar sub-piso e o driver a
    arredonda — a garantia "executa exato" vale p/ movimentos de eixo puro.

Nada aqui foi validado no robo — a percepcao e os ganhos precisam de dados reais.
"""
import math
import os
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

# Helpers compartilhados de docking (carregam docking.yaml por pasta de mapa).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from caramelo_docking_common import load_yaml, resolve_map_folder, docking_file  # noqa: E402

from caramelo_msgs.action import AlignToDock  # noqa: E402


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(z, w, x=0.0, y=0.0):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _fit_line_tls(points):
    """Ajuste de reta por minimos quadrados totais (2x2) sem numpy.

    Retorna (cx, cy, nx, ny) = ponto medio + vetor NORMAL unitario da reta,
    ou None se os pontos forem poucos/degenerados. Usado para estimar a face
    plana da mesa (distancia e orientacao)."""
    n = len(points)
    if n < 8:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    # Autovetor do menor autovalor da covariancia = direcao NORMAL da reta.
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    # theta e' a direcao principal (ao longo da reta); a normal e' perpendicular.
    nx = -math.sin(theta)
    ny = math.cos(theta)
    return cx, cy, nx, ny


class DockAlignNode(Node):
    def __init__(self):
        super().__init__("dock_align_node")

        # --- Parametros ---
        self.declare_parameter("reference_topic", "/mecanum_controller/reference")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("control_frequency", 20.0)
        # Limites coerentes com o Nav2 (controller_server.yaml).
        self.declare_parameter("max_vel_x", 0.20)
        self.declare_parameter("max_vel_y", 0.16)
        # 0.9 -> 0.5 (2026-08-03): coerente com o max_vel_theta do DWB; giro
        # lento o bastante p/ o AMCL acompanhar e nao passar do alvo no fino.
        self.declare_parameter("max_vel_theta", 0.5)
        # Piso fisico do ESC pos-fix do firmware (2026-08-03): 650rpm = 0.1215
        # m/s / 0.315 rad/s de corpo, executado EXATO (mapa ancorado 1570us).
        self.declare_parameter("min_body_vel", 0.1215)
        self.declare_parameter("min_body_omega", 0.315)
        self.declare_parameter("kp_linear", 1.2)
        self.declare_parameter("kp_theta", 1.5)
        self.declare_parameter("default_xy_tolerance", 0.03)
        self.declare_parameter("default_yaw_tolerance", 0.05)
        self.declare_parameter("default_timeout", 30.0)
        self.declare_parameter("settle_cycles", 5)
        # Janela frontal (m) onde procuramos a face da mesa para refino.
        self.declare_parameter("refine_front_min_x", 0.10)
        self.declare_parameter("refine_front_max_x", 1.20)
        self.declare_parameter("refine_half_width_y", 0.45)
        # Guardas anti-falso-positivo do refino (2026-07-23):
        # residuo RMS maximo do ajuste de reta (face plana de verdade) e largura
        # minima da face detectada (rejeita perna de mesa/objeto isolado).
        self.declare_parameter("refine_max_residual", 0.02)
        # 0.30 -> 0.50 (2026-08-03, gauntlet WS1 r3): um fit com largura 0.39m
        # (segmento parcial da face vista de perto) passou nos guards com
        # residuo 5mm e girou o alvo 16.7 graus. Fits saudaveis medem 0.61-0.65
        # (face real 0.80m). Abaixo de 0.50 = vista degradada, sem confianca.
        self.declare_parameter("refine_min_face_width", 0.50)
        # Distancia frontal DESEJADA robo->face na pose final (calibrar no robo:
        # posicione o robo na pose perfeita e leia o face_dist do log). Valor
        # negativo = ainda nao calibrado -> o refino corrige SO o yaw.
        self.declare_parameter("refine_desired_face_dist", -1.0)
        # Re-refino periodico (s) durante o loop de alinhamento (2026-08-03):
        # o AMCL agora corrige map->odom DURANTE o docking (update_min 0.05) e a
        # pose refinada de 1x so envelhece; re-ancorar na face real a cada ~2s.
        # Os guards (residual/largura) descartam fit ruim e mantem o alvo vigente.
        # <= 0 desliga (comportamento antigo: refina 1x antes do loop).
        self.declare_parameter("refine_period", 2.0)
        # Trava de salto dos RE-refinos periodicos (2026-08-03): a mesa nao se
        # move — correcao maior que isso vs alvo vigente = fit ruim, descarta.
        # O 1o refino (pre-loop) fica livre: e ele que come o offset do AMCL
        # (ja medimos 11.4 graus legitimos).
        self.declare_parameter("refine_max_step_xy", 0.04)
        self.declare_parameter("refine_max_step_yaw", 0.14)

        self._ref_topic = self.get_parameter("reference_topic").value
        self._global_frame = self.get_parameter("global_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(TwistStamped, self._ref_topic, 10)
        self._last_scan = None
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._scan_cb, 10, callback_group=self._cb_group,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._action = ActionServer(
            self, AlignToDock, "align_to_dock",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"dock_align_node pronto. Publica em '{self._ref_topic}', "
            f"frames {self._global_frame}->{self._base_frame}."
        )

    def _scan_cb(self, msg):
        self._last_scan = msg

    # ------------------------------------------------------------------ TF
    def _robot_pose(self):
        """Pose (x, y, yaw) do robo no global_frame via TF, ou None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return t.x, t.y, _yaw_from_quat(q.z, q.w, q.x, q.y)

    # --------------------------------------------------------------- Docks
    def _load_dock_pose(self, dock_id, map_name, map_dir):
        map_folder = resolve_map_folder(map_name, map_dir or None)
        data = load_yaml(docking_file(map_folder))
        docks = data.get("docks", {})
        entry = docks.get(dock_id.strip().upper())
        if entry is None:
            raise KeyError(f"dock '{dock_id}' nao esta em {docking_file(map_folder)}")
        pose = entry.get("pose", [0.0, 0.0, 0.0])
        if entry.get("frame", "map") != self._global_frame:
            self.get_logger().warn(
                f"dock '{dock_id}' esta no frame '{entry.get('frame')}', "
                f"mas o no usa '{self._global_frame}'.")
        return float(pose[0]), float(pose[1]), float(pose[2])

    # --------------------------------------------------------- Refino LiDAR
    def _scan_points_in_base(self, scan):
        """Pontos validos do scan TRANSFORMADOS para o base_frame (2D).

        2026-07-23: a versao antiga usava os angulos crus do scan assumindo
        0 grau = frente do robo — mas o LiDAR e' montado de cabeca para baixo
        (RPY 180/0/180) e o crop do scan_normalizer poe a frente fisica em
        ~180 graus, entao px saia sempre <= 0 e o refino NUNCA achava pontos.
        Agora usamos a TF real (base <- frame do scan), que absorve a inversao.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(f"refino: sem TF {self._base_frame}<-{scan.header.frame_id}: {exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # Linhas X e Y da matriz de rotacao (aplicadas a pontos no plano do laser).
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.z * q.w)
        r10 = 2.0 * (q.x * q.y + q.z * q.w)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)

        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            pts.append((r00 * lx + r01 * ly + t.x, r10 * lx + r11 * ly + t.y))
        return pts

    def _refine_target_from_scan(self, target):
        """Refina a pose alvo pela FACE da mesa no /scan.

        Corrige YAW (esquadra com a normal da face) e, se
        refine_desired_face_dist estiver calibrado (>=0), tambem X (distancia
        perpendicular a face). Y (lateral) continua por conta da localizacao
        (problema da abertura, ADR-05) — no pick, a camera fecha a lateral.

        Retorna (target_refinado, confiante: bool). Sem confianca, o chamador
        usa a pose do banco (comportamento identico ao antigo).
        """
        scan = self._last_scan
        if scan is None:
            self.get_logger().warn("refino: nenhum /scan recebido ainda.")
            return target, False
        # Guard de idade (2026-08-03): com o re-refino periodico, um /scan
        # CONGELADO (LiDAR/USB) faria o alvo seguir o robo — runaway rumo a
        # mesa. Scan velho -> sem confianca, mantem o alvo vigente.
        scan_age = (
            self.get_clock().now() - Time.from_msg(scan.header.stamp)
        ).nanoseconds * 1e-9
        if scan_age > 0.5:
            self.get_logger().warn(
                f"refino: /scan com {scan_age:.1f}s de idade; descartado.")
            return target, False
        fx_min = self.get_parameter("refine_front_min_x").value
        fx_max = self.get_parameter("refine_front_max_x").value
        hy = self.get_parameter("refine_half_width_y").value
        max_residual = self.get_parameter("refine_max_residual").value
        min_width = self.get_parameter("refine_min_face_width").value
        desired_dist = self.get_parameter("refine_desired_face_dist").value

        all_pts = self._scan_points_in_base(scan)
        if all_pts is None:
            return target, False
        pts = [p for p in all_pts if fx_min <= p[0] <= fx_max and abs(p[1]) <= hy]

        fit = _fit_line_tls(pts)
        if fit is None:
            self.get_logger().warn(
                f"refino: pontos insuficientes na janela frontal ({len(pts)} < 8).")
            return target, False
        cx, cy, nx, ny = fit
        # Normal apontando para FRENTE do robo (da base em direcao a face).
        if nx < 0:
            nx, ny = -nx, -ny
        face_dist = cx * nx + cy * ny
        face_yaw = math.atan2(ny, nx)  # orientacao da normal no frame do robo

        # Guardas anti-falso-positivo: face plana (residuo), larga o bastante
        # (nao e' perna de mesa/objeto), pouco inclinada e a distancia sensata.
        residual_rms = math.sqrt(
            sum(((p[0] - cx) * nx + (p[1] - cy) * ny) ** 2 for p in pts) / len(pts))
        tang = (-ny, nx)
        along = [ (p[0] - cx) * tang[0] + (p[1] - cy) * tang[1] for p in pts ]
        face_width = max(along) - min(along)

        # Log SEMPRE (e' o instrumento de calibracao do refine_desired_face_dist).
        self.get_logger().info(
            f"refino face: dist={face_dist:.3f} m, yaw_rel={math.degrees(face_yaw):.1f} deg, "
            f"largura={face_width:.2f} m, residuo={residual_rms*1000:.0f} mm, pts={len(pts)}")

        if residual_rms > max_residual:
            self.get_logger().warn("refino: residuo alto (face nao-plana?); descartando.")
            return target, False
        if face_width < min_width:
            self.get_logger().warn("refino: face estreita demais (perna/objeto?); descartando.")
            return target, False
        if abs(_normalize_angle(face_yaw)) > math.radians(30) or not (0.05 < face_dist < fx_max):
            self.get_logger().warn("refino: face torta/distante demais; descartando.")
            return target, False

        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn("refino: sem TF do robo; descartando.")
            return target, False
        rx, ry, ryaw = pose

        tx, ty, tyaw = target
        # Yaw alvo: esquadrar com a normal da face (direcao dela no mapa).
        refined_yaw = _normalize_angle(ryaw + face_yaw)

        refined_tx, refined_ty = tx, ty
        if desired_dist >= 0.0:
            # Plano da face no mapa: p . n_map = c_face. Move o alvo do banco ao
            # longo da normal para que a distancia final robo->face = calibrada.
            n_map = (math.cos(ryaw + face_yaw), math.sin(ryaw + face_yaw))
            c_face = rx * n_map[0] + ry * n_map[1] + face_dist
            desired_proj = c_face - desired_dist
            shift = desired_proj - (tx * n_map[0] + ty * n_map[1])
            refined_tx = tx + shift * n_map[0]
            refined_ty = ty + shift * n_map[1]
            self.get_logger().info(
                f"refino aplicado: X ajustado {shift*100:+.1f} cm ao longo da normal "
                f"(alvo dist={desired_dist:.3f} m) + yaw esquadrado "
                f"({math.degrees(face_yaw):+.1f} deg). Y mantido do banco.")
        else:
            self.get_logger().info(
                f"refino aplicado: SO yaw esquadrado ({math.degrees(face_yaw):+.1f} deg) — "
                "refine_desired_face_dist nao calibrado (X do banco).")
        return (refined_tx, refined_ty, refined_yaw), True

    # ------------------------------------------------------------- Controle
    def _publish_cmd(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._cmd_pub.publish(msg)

    def _apply_floor(self, vx, vy, wz, moving_xy, moving_yaw):
        """Respeita o piso do ESC: se a base precisa se mover mas o comando ficou
        abaixo do minimo executavel, sobe para o minimo (senao nao anda). Isso
        causa movimento em degrau perto do goal — limite fisico ate o firmware."""
        min_v = self.get_parameter("min_body_vel").value
        min_w = self.get_parameter("min_body_omega").value
        speed = math.hypot(vx, vy)
        if moving_xy and 0.0 < speed < min_v:
            scale = min_v / speed
            vx *= scale
            vy *= scale
        if moving_yaw and 0.0 < abs(wz) < min_w:
            wz = math.copysign(min_w, wz)
        return vx, vy, wz

    def _execute(self, goal_handle):
        g = goal_handle.request
        result = AlignToDock.Result()

        xy_tol = g.xy_tolerance or self.get_parameter("default_xy_tolerance").value
        yaw_tol = g.yaw_tolerance or self.get_parameter("default_yaw_tolerance").value
        timeout = g.timeout or self.get_parameter("default_timeout").value
        max_vx = self.get_parameter("max_vel_x").value
        max_vy = self.get_parameter("max_vel_y").value
        max_wz = self.get_parameter("max_vel_theta").value
        kp = self.get_parameter("kp_linear").value
        kp_t = self.get_parameter("kp_theta").value
        settle_target = int(self.get_parameter("settle_cycles").value)
        period = 1.0 / float(self.get_parameter("control_frequency").value)

        try:
            target = self._load_dock_pose(g.dock_id, g.map_name, g.map_dir)
        except Exception as exc:  # noqa: BLE001
            return self._abort(goal_handle, result, f"falha ao carregar dock: {exc}")

        if g.use_lidar_refine:
            self._feedback(goal_handle, "refinando", 0.0, 0.0, 0.0)
            target, confident = self._refine_target_from_scan(target)
            if not confident:
                self.get_logger().warn("refino por LiDAR sem confianca; usando pose do banco.")

        tx, ty, tyaw = target
        start = self.get_clock().now()
        rate = self.create_rate(1.0 / period)
        settle = 0
        refine_period = float(self.get_parameter("refine_period").value)
        max_step_xy = float(self.get_parameter("refine_max_step_xy").value)
        max_step_yaw = float(self.get_parameter("refine_max_step_yaw").value)
        last_refine = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0.0, 0.0, 0.0)
                goal_handle.canceled()
                result.success = False
                result.message = "cancelado"
                return result

            if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout:
                self._publish_cmd(0.0, 0.0, 0.0)
                return self._abort(goal_handle, result, "timeout no alinhamento")

            # Re-refino periodico: re-ancora o alvo na face real da mesa
            # enquanto o AMCL move map->odom sob o robo. Sem confianca, os
            # guards do refino mantem o alvo vigente.
            if (
                g.use_lidar_refine and refine_period > 0.0
                and (self.get_clock().now() - last_refine).nanoseconds * 1e-9
                >= refine_period
            ):
                last_refine = self.get_clock().now()
                new_target, confident = self._refine_target_from_scan(
                    (tx, ty, tyaw))
                if confident:
                    step_xy = math.hypot(new_target[0] - tx, new_target[1] - ty)
                    step_yaw = abs(_normalize_angle(new_target[2] - tyaw))
                    if step_xy <= max_step_xy and step_yaw <= max_step_yaw:
                        tx, ty, tyaw = new_target
                    else:
                        self.get_logger().warn(
                            "re-refino descartado: salto implausivel "
                            f"(dxy={step_xy * 100:.1f}cm, "
                            f"dyaw={math.degrees(step_yaw):.1f}deg)")

            pose = self._robot_pose()
            if pose is None:
                rate.sleep()
                continue
            rx, ry, ryaw = pose

            # Erro no global, projetado no frame do robo.
            dx = tx - rx
            dy = ty - ry
            ex = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            ey = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            eyaw = _normalize_angle(tyaw - ryaw)

            dist = math.hypot(ex, ey)
            self._feedback(goal_handle, "alinhando", ex, ey, eyaw)

            if dist <= xy_tol and abs(eyaw) <= yaw_tol:
                settle += 1
                self._publish_cmd(0.0, 0.0, 0.0)
                if settle >= settle_target:
                    result.success = True
                    result.message = "alinhado"
                    result.final_x_error = ex
                    result.final_y_error = ey
                    result.final_yaw_error = eyaw
                    goal_handle.succeed()
                    return result
                rate.sleep()
                continue
            settle = 0

            vx = _clamp(kp * ex, max_vx)
            vy = _clamp(kp * ey, max_vy)
            wz = _clamp(kp_t * eyaw, max_wz)
            vx, vy, wz = self._apply_floor(
                vx, vy, wz, dist > xy_tol, abs(eyaw) > yaw_tol)
            self._publish_cmd(vx, vy, wz)
            rate.sleep()

        self._publish_cmd(0.0, 0.0, 0.0)
        return self._abort(goal_handle, result, "no encerrado")

    def _feedback(self, goal_handle, phase, ex, ey, eyaw):
        fb = AlignToDock.Feedback()
        fb.phase = phase
        fb.x_error = ex
        fb.y_error = ey
        fb.yaw_error = eyaw
        goal_handle.publish_feedback(fb)

    def _abort(self, goal_handle, result, message):
        self.get_logger().warn(message)
        result.success = False
        result.message = message
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
