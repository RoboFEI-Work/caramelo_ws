#!/usr/bin/env python3
"""pp_furo_node: mede o furo da mesa de precisao e o cubo em relacao a ele.

2026-11-16 (branch mudanca_pp, PP refeita do zero a pedido do operador). O
place para o braco, chama o servico e este no' responde quanto o CUBO tem de
andar para ficar em cima do centro do furo. A conta esta' em pp_furo.py.

Este no' NAO move nada: so' olha. Quem move e' o mtc_place_action_node.

  /manip/pp_furo/slot    String latched, ex. "tag_104" (publicado pelo place)
  /manip/pp_furo/medir   std_srvs/Trigger. success=true e a mensagem
        furo=visto|pela_tag; lat_mm=..; rad_mm=..; giro_graus=..; alt_mm=..; n=..
      lat/rad = quanto o cubo tem de andar (+lat = j1 crescendo, +rad = para longe
      da base); giro = quanto a garra tem de girar; alt = fundo do cubo acima do
      tampo. furo=visto: o furo estava inteiro na imagem; pela_tag: tapado pelo
      cubo, posicao pela tag do slot + vetor tag->furo medido antes.
      success=false: sem_tag, sem_imagem ou furo_nunca_visto.
      Em qualquer resposta pode vir "; ocupado=<id>": ha' um bloco com essa tag
      no slot (o place so' leva isso em conta na olhada de garra vazia).

A imagem crua e' assinada UMA vez, como bytes (2026-11-17: criar e destruir a
assinatura a cada place derrubou o no' no meio da AMT, 15h47); so' sao guardados
quadros durante um place na PP e so' os usados na medida sao decodificados.

Imagem e tags vem do MESMO quadro (casadas pelo carimbo do apriltag_ros); a
medida e' feita dentro da imagem, entao um relogio de camera adiantado (visto em
16/11) nao a estraga. So' entram quadros que CHEGARAM depois da chamada: o
place chama com o braco ja' parado.
"""

import json
import math
import os
import struct
import threading
import time
from collections import OrderedDict, deque

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

import amt_sort_core
import pp_furo as F


class PpFuro(Node):
    def __init__(self):
        super().__init__("pp_furo")
        dp = self.declare_parameter
        self.cfg = F.FuroCfg(
            lado_tag_mm=float(dp("lado_tag_mm", 31.8).value),
            fundo_cubo_cam_mm=tuple(dp("fundo_cubo_cam_mm", [-13.25, -32.20, 131.78]).value),
            angulo_ref_graus=float(dp("angulo_ref_graus", 180.0).value))
        self.amostras = int(dp("amostras", 3).value)
        self.prazo_s = float(dp("prazo_s", 3.0).value)
        self.salvar_dir = os.path.expanduser(dp("salvar_dir", "~/.ros/pp_furo").value)
        self.img_topic = dp("image_topic", "/camera/camera/color/image_raw").value

        self._trava = threading.Lock()
        self._imgs = OrderedDict()            # carimbo (ns) -> Image
        self._pares = deque(maxlen=12)        # (chegada monotonica, Image, {id: cantos})
        self._k = None
        self._q5 = None
        self.slot = None
        self.slot_ids = [int(v) for v in dp("slot_tag_ids", [101, 102, 103, 104, 105, 106, 107]).value]
        self._ativo = False                   # ha' um place em andamento
        self._pedido = False                  # ...e ele e' na PP (publicou o slot)
        self.memoria = {}                     # slot -> F.Memoria (o furo nao muda de lugar na mesa)

        self._g = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/manip/pp_furo/slot", self._on_slot, latched, callback_group=self._g)
        self.create_subscription(Bool, "/manip/place_active", self._on_ativo, latched, callback_group=self._g)
        self.create_subscription(AprilTagDetectionArray, dp("detections_topic", "/detections").value,
                                 self._on_det, 20, callback_group=self._g)
        self.create_subscription(CameraInfo, self.img_topic.rsplit("/", 1)[0] + "/camera_info",
                                 self._on_info, qos_profile_sensor_data, callback_group=self._g)
        self.create_subscription(JointState, "/joint_states", self._on_js, 20, callback_group=self._g)
        self.create_subscription(Image, self.img_topic, self._on_img, qos_profile_sensor_data,
                                 raw=True, callback_group=self._g)
        self.create_service(Trigger, "/manip/pp_furo/medir", self._on_medir, callback_group=self._g)
        self.get_logger().info("pp_furo pronto: /manip/pp_furo/medir (so' olha, nao move o braco).")

    # ---- entradas ------------------------------------------------------------
    def _on_slot(self, msg):
        try:
            self.slot = int(str(msg.data).replace("tag_", ""))
        except ValueError:
            self.get_logger().warn(f"slot ilegivel: '{msg.data}'")
            return
        self._pedido = True

    def _on_ativo(self, msg):
        self._ativo = bool(msg.data)
        if not self._ativo:
            self._pedido = False
            with self._trava:
                self._imgs.clear()
                self._pares.clear()

    def _on_img(self, cru):
        # So' durante um place NA PP; o resto do tempo o quadro e' descartado sem decodificar.
        if not (self._ativo and self._pedido):
            return
        # Carimbo direto do CDR: 4 bytes de cabecalho, depois header.stamp (sec, nanosec).
        chave = struct.unpack_from("<iI" if cru[1] == 1 else ">iI", cru, 4)
        with self._trava:
            self._imgs[chave] = cru
            while len(self._imgs) > 30:      # ~1 s a 30 Hz: cobre o atraso do apriltag
                self._imgs.popitem(last=False)

    def _on_info(self, msg):
        self._k = [float(v) for v in msg.k]

    def _on_js(self, msg):
        if "manip_joint5" in msg.name:
            self._q5 = float(msg.position[msg.name.index("manip_joint5")])

    def _on_det(self, msg):
        chave = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self._trava:
            img = self._imgs.get(chave)
            if img is not None:
                self._pares.append((time.monotonic(), img,
                                    {d.id: [[c.x, c.y] for c in d.corners] for d in msg.detections}))

    @staticmethod
    def _bgr(cru):
        msg = deserialize_message(cru, Image)
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1].copy() if msg.encoding == "rgb8" else a.copy()

    # ---- servico -------------------------------------------------------------
    def _on_medir(self, _req, resp):
        resp.success = False
        if self.slot is None or self._k is None or self._q5 is None:
            resp.message = "sem_slot" if self.slot is None else "sem_camera_info" if self._k is None else "sem_juntas"
            return resp
        t0 = time.monotonic()
        novos = []
        while time.monotonic() - t0 < self.prazo_s:
            with self._trava:
                # 0,25 s de folga: o quadro tem de ter sido exposto com o braco ja' parado
                novos = [p for p in self._pares if p[0] >= t0 + 0.25]
            if len(novos) >= self.amostras:
                break
            time.sleep(0.05)
        if not novos:
            resp.message = "sem_imagem"
            return resp
        mem = self.memoria.setdefault(self.slot, F.Memoria())
        oks, vistos, bgr_ult, com_tag, dono = [], 0, None, 0, None
        for _t, img, tags in novos[-self.amostras:]:
            dono = next((c for c, s in amt_sort_core.arranjo_do_quadro(tags, self.slot_ids)[0].items()
                         if s == self.slot), dono)
            if self.slot not in tags:
                continue
            com_tag += 1
            bgr = self._bgr(img)
            if bgr is None:
                continue
            r = F.medir(bgr, self._k, tags[self.slot], self._q5, mem, self.cfg)
            bgr_ult = bgr
            vistos += r["furo"] == "visto"
            if r["ok"]:
                oks.append(r)
        ocupado = f"; ocupado={dono}" if dono is not None else ""
        if not oks:
            resp.message = ("sem_tag" if com_tag == 0 else "furo_nunca_visto") + ocupado
            self._salvar(bgr_ult, resp.message, {})
            return resp
        med = {k: float(np.median([o[k] for o in oks])) for k in ("lat_mm", "rad_mm", "alt_mm", "giro_graus")}
        furo = "visto" if vistos * 2 > len(oks) else "pela_tag"
        resp.success = True
        resp.message = (f"furo={furo}; lat_mm={med['lat_mm']:+.2f}; rad_mm={med['rad_mm']:+.2f}; "
                        f"giro_graus={med['giro_graus']:+.2f}; alt_mm={med['alt_mm']:.1f}; n={len(oks)}; "
                        f"vetor_mm={mem.vetor_mm[0]:+.2f},{mem.vetor_mm[1]:+.2f}" + ocupado)
        self.get_logger().info(f"[pp_furo] slot {self.slot}: {resp.message}")
        self._salvar(bgr_ult, resp.message, med)
        return resp

    def _salvar(self, bgr, texto, med):
        """Guarda o quadro e a medida (para conferir depois o que a camera viu)."""
        if not self.salvar_dir or bgr is None:
            return
        try:
            pasta = os.path.join(self.salvar_dir, time.strftime("%Y%m%d"))
            os.makedirs(pasta, exist_ok=True)
            nome = time.strftime("%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}_slot{self.slot}"
            cv2.imwrite(os.path.join(pasta, nome + ".jpg"), bgr)
            with open(os.path.join(pasta, "medidas.jsonl"), "a", encoding="utf-8") as fh:
                fh.write(json.dumps({"arq": nome + ".jpg", "slot": self.slot, "texto": texto, "q5": self._q5,
                                     "k": self._k, **med}) + "\n")
        except OSError as ex:
            self.get_logger().warn(f"nao gravei o quadro da medida: {ex}")


def main():
    rclpy.init()
    no = PpFuro()
    ex = MultiThreadedExecutor(num_threads=4)
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
