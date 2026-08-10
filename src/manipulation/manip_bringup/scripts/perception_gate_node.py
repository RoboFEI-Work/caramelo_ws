#!/usr/bin/env python3
"""Gate da percepcao: a RealSense so streama durante pick/place.

Pedido do operador (auditoria 2026-08-07, adendo): camera + AprilTag comem
CPU/USB o tempo todo, inclusive durante a navegacao, onde ninguem olha imagem.
Em 03/08 o apriltag_node chegou a 125% de CPU e derrubou o controller do Nav2
de 20 Hz para 2-8 Hz; em USB 2 a banda disputada ainda derrubava o driver.

Este no desliga os streams color/depth do driver POR PARAMETRO enquanto nenhum
pick/place esta ativo, e religa quando um comeca. Sem imagem publicada, o
apriltag_node nao roda callback nenhum — o gate cobre os dois de uma vez.

Por que parametro e nao matar/subir o processo do driver: religar exige
re-enumeracao USB (2-4 s, com historico de processo orfao segurando o
/dev/video neste projeto). Desligar o stream mede 2,6 s ate o 1o frame voltar,
mantem a conexao viva e nao tem modo de falha "camera nao volta".

Medido no notebook (2026-08-10, D455 em USB 3): OFF -> ON entrega o 1o frame em
2,58 s e as tags voltam a ser detectadas em seguida. Como o pick chega no
estagio detecting_tag ~3,1 s depois de comecar, os nos de pick/place esperam a
camera explicitamente (perception_warmup_timeout) em vez de contar com essa
folga de meio segundo.

DESLIGADO POR PADRAO (decisao do operador, 2026-08-10). No 1o teste no robo o
driver TRAVOU no liga-desliga: pedimos enable_color+enable_depth no MESMO
SetParameters e o realsense2_camera parou em "Stopping Sensor: RGB Camera" e
nunca mais respondeu (nem a param get). Corrigido aqui (um stream por
requisicao, em cadeia, que e o padrao medido funcionando) + watchdog que
desiste em vez de cutucar driver travado — mas a razao original do gate era a
disputa de banda no USB 2, que morreu quando a camera foi para o USB 3. Sem
ganho que justifique o risco, o default virou `use_perception_gate:=false`.

Para experimentar de novo: `use_perception_gate:=true`. Antes de confiar,
faca um stress de uns 10 ciclos liga/desliga e confirme que o driver sobrevive.

O gate e OPCIONAL e desacoplado: se este no nao estiver rodando, a camera fica
ligada o tempo todo e todo o resto se comporta como antes.
"""

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class PerceptionGate(Node):
    def __init__(self):
        super().__init__("perception_gate")

        self.camera_node = self.declare_parameter(
            "camera_node", "/camera/camera").value
        self.streams = self.declare_parameter(
            "streams", ["enable_color", "enable_depth"]).value
        # Atraso para desligar: evita apagar a camera entre o pick e o place
        # de um mesmo objeto (o BT manda os dois em sequencia) e paga o
        # religamento uma vez so.
        self.off_delay_s = float(self.declare_parameter("off_delay_s", 20.0).value)
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        # Enquanto ocioso, reaplica o desligamento de tempos em tempos: se o
        # driver respawnar, ele volta streamando (default do launch) e o gate
        # precisa perceber. Nunca reaplica com a camera LIGADA (um
        # set_parameters redundante pode reiniciar o stream no meio de um pick).
        self.idle_reapply_s = float(
            self.declare_parameter("idle_reapply_s", 15.0).value)

        # Espectador humano (pedido do operador 2026-08-10): com o gate ligado,
        # abrir o RViz durante a navegacao mostraria tela preta. Em vez de
        # obrigar a desligar o gate, o gate PERCEBE quem esta olhando: qualquer
        # assinante alem da linha de base (o apriltag_node, que assina o raw
        # o tempo todo) ou qualquer assinante do compressed = alguem quer ver,
        # entao a camera fica ligada enquanto a janela estiver aberta.
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw").value
        self.viewer_baseline = int(self.declare_parameter(
            "viewer_subscriber_baseline", 1).value)
        self.viewer_seen = False

        self.pick_active = False
        self.place_active = False
        self.desired_on = True
        self.applied_on = None
        self.off_deadline = None
        self.last_apply_time = None
        self.pending = False
        self.pending_since = None
        self.apply_timeout_s = float(
            self.declare_parameter("apply_timeout_s", 15.0).value)
        self.warned_unavailable = False

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, "/manip/pick_active", self._on_pick, latched)
        self.create_subscription(
            Bool, "/manip/place_active", self._on_place, latched)

        self.client = self.create_client(
            SetParameters, f"{self.camera_node}/set_parameters")

        self.create_timer(0.5, self._reconcile)

        if not self.enabled:
            self.get_logger().warn(
                "perception_gate carregado mas DESABILITADO (enabled:=false) — "
                "a camera fica ligada o tempo todo.")
        else:
            self.get_logger().info(
                f"perception_gate ativo: {self.camera_node} streama so durante "
                f"pick/place (desliga {self.off_delay_s:.0f}s depois de ocioso).")

    def _on_pick(self, msg):
        self.pick_active = bool(msg.data)

    def _on_place(self, msg):
        self.place_active = bool(msg.data)

    def _reconcile(self):
        if not self.enabled:
            return

        if self.pending:
            # Watchdog: o driver ja travou dentro do proprio callback de
            # parametro ("Stopping Sensor: RGB Camera" e nunca mais respondeu).
            # Se isso acontecer, para de cutucar e avisa alto — insistir num
            # driver travado nao ressuscita ninguem e enche o log.
            elapsed = (
                self.get_clock().now().nanoseconds * 1e-9 - (self.pending_since or 0.0))
            if self.pending_since is not None and elapsed > self.apply_timeout_s:
                self.enabled = False
                self.pending = False
                self.get_logger().fatal(
                    f"driver da camera nao respondeu ao gate em {elapsed:.0f}s — "
                    "GATE DESATIVADO para esta rodada. A camera pode ter travado; "
                    "se as imagens sumirem, reinicie o stack de manipulacao.")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        viewer = self._viewer_watching()
        if viewer != self.viewer_seen:
            self.viewer_seen = viewer
            self.get_logger().info(
                "espectador abriu a imagem (RViz?) — mantendo a camera ligada"
                if viewer else "espectador fechou a imagem — gate volta a mandar")
        want_on = self.pick_active or self.place_active or viewer

        if want_on:
            self.off_deadline = None
            self.desired_on = True
        elif self.desired_on:
            # Comecou a ociosidade: agenda o desligamento.
            if self.off_deadline is None:
                self.off_deadline = now + self.off_delay_s
            elif now >= self.off_deadline:
                self.off_deadline = None
                self.desired_on = False

        stale = (
            not self.desired_on
            and self.last_apply_time is not None
            and self.idle_reapply_s > 0.0
            and (now - self.last_apply_time) >= self.idle_reapply_s
        )
        if self.desired_on == self.applied_on and not stale:
            return

        if not self.client.service_is_ready():
            if not self.warned_unavailable:
                self.get_logger().warn(
                    f"{self.camera_node}/set_parameters indisponivel — gate "
                    "aguardando o driver da camera subir.")
                self.warned_unavailable = True
            return
        self.warned_unavailable = False

        self._apply(self.desired_on, now)

    def _viewer_watching(self):
        try:
            raw = self.count_subscribers(self.image_topic)
            compressed = self.count_subscribers(self.image_topic + "/compressed")
        except Exception:  # noqa: BLE001 - grafo pode oscilar
            return False
        return raw > self.viewer_baseline or compressed > 0

    def _apply(self, turn_on, now):
        # UM stream por requisicao, em cadeia (2026-08-10): mandar
        # enable_color+enable_depth no MESMO SetParameters fez o driver
        # travar em "Stopping Sensor: RGB Camera" e nunca mais responder —
        # ele para/reinicia o sensor dentro do callback do parametro e nao
        # aguenta os dois de uma vez. Uma chamada por stream e o padrao que
        # foi medido funcionando nos dois sentidos.
        self.pending = True
        self.pending_since = now
        self._apply_next(list(self.streams), turn_on, now)

    def _apply_next(self, remaining, turn_on, now):
        if not remaining:
            self.pending = False
            self.pending_since = None
            changed = self.applied_on != turn_on
            self.applied_on = turn_on
            self.last_apply_time = now
            if changed:
                self.get_logger().info(
                    "camera LIGADA (pick/place ou espectador)" if turn_on
                    else "camera DESLIGADA (ociosa) — CPU/USB liberados p/ a navegacao")
            return

        stream = remaining[0]
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=stream,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=turn_on),
            )
        ]
        target = turn_on
        future = self.client.call_async(request)

        def done(fut):
            try:
                response = fut.result()
            except Exception as exc:  # noqa: BLE001 - servico pode sumir
                self.pending = False
                self.pending_since = None
                self.get_logger().warn(f"gate falhou ao aplicar {stream}: {exc}")
                return
            ok = all(r.successful for r in response.results)
            if not ok:
                reasons = "; ".join(
                    r.reason for r in response.results if not r.successful)
                self.pending = False
                self.pending_since = None
                self.get_logger().warn(f"gate recusado pelo driver: {reasons}")
                return
            self._apply_next(remaining[1:], target, now)

        future.add_done_callback(done)

    def restore_camera(self):
        """Deixa a camera ligada ao sair — ninguem herda um robo cego."""
        if not self.enabled or self.applied_on is not False:
            return
        if not self.client.wait_for_service(timeout_sec=2.0):
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=stream,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=True),
            )
            for stream in self.streams
        ]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info("camera religada no shutdown do gate.")


def main():
    rclpy.init()
    node = PerceptionGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.restore_camera()
        except Exception:  # noqa: BLE001 - shutdown e best effort
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
