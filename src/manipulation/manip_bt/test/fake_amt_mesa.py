#!/usr/bin/env python3
"""Mesa de precisao FALSA para testar a AMT sem robo (2026-11-16).

Guarda quem esta' em qual slot e serve, com as mesmas interfaces do robo:
  /detections   a vista sintetica de pegar_obj (mesma camera do test_amt_sort_core:
                so' entra na imagem o que caberia de verdade, com paralaxe)
  /pick_tag     tira o bloco do slot para um pote de bordo (3 potes)
  /place_tag    stack_on = tag do slot: poe o bloco no slot (falha se ocupado)
  /nudge_base   anda a base de lado (+ = esquerda) e a vista anda junto

Regras de alcance, como no robo: de pegar_obj (final_attempt=false) so' acha o
que esta' na imagem; a mais de 24 cm de lado responde unreachable com sugestao
de passo; em final_attempt=true acha ate' 45 cm (varredura) mas nao sugere passo.
Place (2026-11-17): slot a mais de 8 cm do eixo do robo pede a base alinhada
(unreachable + passo = -x), como o pp_alinhar_base_m do place de verdade.

ATENCAO: nao rodar com o robo no ar (mesmos nomes de action). So' em ROS_DOMAIN_ID=99.

Parametros:
  arranjo      "5,2,6,1,4,3,0" = bloco em cada slot 101..107 (0 = vazio)
  fail_places  ["tag_1:2"] = os 2 primeiros places da tag falham (bloco segue a bordo)
  sem_nudge    true = nao sobe o /nudge_base
  bordo        "1,6" = blocos que ja' comecam nos potes de bordo
  potes_yaml   arquivo dos potes de bordo, gravado a cada operacao como o pick/place de
               verdade fazem (o amt_sort_node le dele o que ficou a bordo de uma rodada para outra)
  nudge_recusa_acima  0.16 = passo maior que isto e' recusado inteiro (obstaculo_lateral),
                      como a perna da mesa no robo em 17/11 17h57 (0 = nao recusa)
  latency_sec  0.2
"""
import math
import os
import sys
import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point as TagPoint
from caramelo_msgs.action import NudgeBase
from my_robot_msgs.action import PickTag, PlaceTag
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "manip_bringup", "test"))
from test_amt_sort_core import SLOTS, mesa  # noqa: E402

PASSO = 0.107


def quantizar(falta):
    mag = min(0.25, max(0.10, round(abs(falta) / 0.05) * 0.05))
    return math.copysign(mag, falta)


class MesaFalsa(Node):
    def __init__(self):
        super().__init__("fake_amt_mesa")
        arranjo = self.declare_parameter("arranjo", "5,2,6,1,4,3,0").value
        self.atraso = float(self.declare_parameter("latency_sec", 0.2).value)
        self.declare_parameter("fail_places", Parameter.Type.STRING_ARRAY)
        falhas = self.get_parameter_or("fail_places", Parameter("fail_places", value=[])).value or []
        self.falhas = {f.split(":")[0]: int(f.split(":")[1]) for f in falhas if ":" in f}
        self.estado = {s: (int(c) or None) for s, c in zip(SLOTS, arranjo.split(","))}
        self.bordo = [int(c) for c in self.declare_parameter("bordo", "").value.split(",") if c]
        self.desvio = 0.0
        self.recusa = float(self.declare_parameter("nudge_recusa_acima", 0.0).value)
        self.potes_yaml = self.declare_parameter("potes_yaml", "").value
        self.trava = threading.Lock()
        g = ReentrantCallbackGroup()
        self.det = self.create_publisher(AprilTagDetectionArray, "/detections", 20)
        self.create_timer(1.0 / 6.0, self._camera, callback_group=g)
        ActionServer(self, PickTag, "/pick_tag", lambda gh: self._op(gh, "pick"), callback_group=g)
        ActionServer(self, PlaceTag, "/place_tag", lambda gh: self._op(gh, "place"), callback_group=g)
        if not self.declare_parameter("sem_nudge", False).value:
            ActionServer(self, NudgeBase, "/nudge_base", self._nudge, callback_group=g)
        self._registrar("inicio")

    def _registrar(self, texto):
        if self.potes_yaml:
            potes = (self.bordo + [None] * 3)[:3]
            with open(self.potes_yaml, "w", encoding="utf-8") as f:
                f.write("containers:\n" + "".join(
                    f"  container{i + 1}: {{occupied: {'true' if c else 'false'}, tag_frame: '{f'tag_{c}' if c else ''}'}}\n"
                    for i, c in enumerate(potes)))
        fila = ",".join(str(self.estado[s] or 0) for s in SLOTS)
        self.get_logger().info(f"{texto} | ESTADO {fila} bordo={sorted(self.bordo)} desvio={self.desvio:+.2f}")

    def _vista(self):
        with self.trava:
            return mesa(self.estado, self.desvio)

    def _camera(self):
        m = AprilTagDetectionArray()
        m.header.stamp = self.get_clock().now().to_msg()
        for tid, cantos in self._vista().items():
            d = AprilTagDetection()
            d.id = int(tid)
            d.corners = [TagPoint(x=float(c[0]), y=float(c[1])) for c in cantos]
            m.detections.append(d)
        self.det.publish(m)

    def _op(self, gh, op):
        g = gh.request
        res = (PickTag if op == "pick" else PlaceTag).Result()
        time.sleep(self.atraso)
        cubo = int(g.tag_frame.replace("tag_", ""))
        with self.trava:
            if op == "pick":
                slot = next((s for s, c in self.estado.items() if c == cubo), None)
                ref = cubo
            else:
                slot = int(g.stack_on.replace("tag_", "")) if g.stack_on else None
                ref = slot
            x = None if slot is None else (SLOTS.index(slot) - 3) * PASSO + self.desvio
        visto = ref in self._vista() or (g.final_attempt and x is not None and abs(x) <= 0.45)
        motivo = ""
        if slot is None:
            motivo = "tag_nao_vista"
        elif op == "place" and cubo not in self.bordo:
            motivo = "tag_nao_a_bordo"
        elif not visto:
            motivo = "tag_nao_vista"
        elif op == "place" and not g.final_attempt and abs(x) > 0.08:
            motivo = "fora_de_alcance"
            res.unreachable = True
            res.suggested_base_shift_m = float(-math.copysign(min(0.25, max(0.10, abs(x))), x))
        elif abs(x) > 0.24:
            motivo = "fora_de_alcance"
            if not g.final_attempt:
                res.unreachable = True
                res.suggested_base_shift_m = float(quantizar(-(x - math.copysign(0.18, x))))
        elif op == "pick" and len(self.bordo) >= 3:
            motivo = "sem_pote_livre"
        elif op == "place" and self.estado[slot] is not None:
            motivo = "slot_ocupado"
        elif op == "place" and self.falhas.get(g.tag_frame, 0) > 0:
            self.falhas[g.tag_frame] -= 1
            motivo = "falha_injetada"
        if motivo:
            # Como os nos de verdade (contrato 2.5): pulo = success=true + skipped=true.
            pulo = motivo.startswith("tag_") or res.unreachable
            res.success, res.skipped, res.fail_reason = pulo, pulo, motivo
            self.get_logger().info(f"{op} {g.tag_frame} slot {slot} FALHOU: {motivo}"
                                   f"{' (final)' if g.final_attempt else ''}")
        else:
            with self.trava:
                if op == "pick":
                    self.estado[slot] = None
                    self.bordo.append(cubo)
                else:
                    self.estado[slot] = cubo
                    self.bordo.remove(cubo)
            res.success = True
            self._registrar(f"{op} {g.tag_frame} slot {slot} ok")
        gh.succeed()
        return res

    def _nudge(self, gh):
        res = NudgeBase.Result()
        dy = float(gh.request.dy)
        time.sleep(self.atraso)
        if not 0.099 <= abs(dy) <= 0.251:
            res.success, res.reason = False, "abaixo_do_passo_minimo" if abs(dy) < 0.1 else "acima_do_curso_maximo"
        elif self.recusa > 0.0 and abs(dy) > self.recusa:
            res.success, res.reason = False, "obstaculo_lateral"
            self.get_logger().info(f"nudge {dy:+.2f} RECUSADO (obstaculo_lateral)")
        else:
            with self.trava:
                self.desvio += dy
            res.success, res.travelled = True, dy
            self._registrar(f"nudge {dy:+.2f}")
        gh.succeed()
        return res


def main():
    rclpy.init()
    no = MesaFalsa()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(no)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
