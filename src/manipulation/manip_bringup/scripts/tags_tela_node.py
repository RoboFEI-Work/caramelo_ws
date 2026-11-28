#!/usr/bin/env python3
"""tags_tela: mostra em TELA CHEIA as tags AprilTag identificadas pela camera.

2026-11-28 (pedido do operador): "adicione para aparecer na tela, em tela cheia,
as tags identificadas".

O que faz:
  * assina /detections (apriltag_msgs/AprilTagDetectionArray, o mesmo topico que
    o pick/place usa) e junta os IDs vistos nos ultimos `janela_s` segundos;
  * desenha uma tela BRANCA com "TAGS IDENTIFICADAS:" e os IDs em preto, o maior
    que couber, no tamanho real da tela (a do robo e' 800x480);
  * a janela abre na 1a deteccao e fica aberta. Sem tag na camera, continua
    mostrando as ultimas identificadas (a camera so' liga no pick/place);
  * ESC ou 'q' fecha a janela (reabre na proxima deteccao).

So' LE: nao publica em topico de comando, nao chama servico nem action.
Sem DISPLAY o no so' avisa e fica parado (nao derruba o launch).

Quem lanca: manip_bringup/launch/manip_stack.launch.py (use_tags_tela, default true).
Na mao: ros2 run manip_bringup tags_tela_node.py
"""
import os
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

JANELA = 'Caramelo - tags identificadas'
TITULO = 'TAGS IDENTIFICADAS:'


def texto_ids(ids) -> str:
    """[3, 1, 12] -> '1  3  12' (ordem crescente, sem repetir)."""
    return '  '.join(str(i) for i in sorted(set(ids)))


class TagsTela(Node):

    def __init__(self):
        super().__init__('tags_tela')
        self.declare_parameter('detections_topic', '/detections')
        # Tag vista ha' ate' isto (s) ainda conta como "identificada agora".
        self.declare_parameter('janela_s', 1.0)
        self._vistas = {}      # id -> hora (monotonic) da ultima deteccao
        self._ultimas = []     # ultimo conjunto mostrado (fica na tela sem deteccao)
        topic = self.get_parameter('detections_topic').value
        self.create_subscription(AprilTagDetectionArray, topic, self._on_det, qos_profile_sensor_data)
        self.get_logger().info(f'tags_tela pronto: {topic} -> tela cheia.')

    def _on_det(self, msg: AprilTagDetectionArray) -> None:
        agora = time.monotonic()
        for d in msg.detections:
            self._vistas[int(d.id)] = agora

    def ids_atuais(self):
        """IDs vistos na janela; sem nenhum, as ultimas identificadas."""
        limite = time.monotonic() - float(self.get_parameter('janela_s').value)
        atuais = [i for i, t in self._vistas.items() if t >= limite]
        if atuais:
            self._ultimas = sorted(atuais)
        return self._ultimas


def desenhar(cv2, np, W, H, ids_txt):
    fonte = cv2.FONT_HERSHEY_DUPLEX
    img = np.full((H, W, 3), 255, np.uint8)

    def linha(texto, y_centro, alt_max):
        (tw, th), _ = cv2.getTextSize(texto, fonte, 1.0, 2)
        escala = min(0.92 * W / max(tw, 1), alt_max / max(th, 1))
        esp = max(2, int(escala * 2.5))
        (tw, th), _ = cv2.getTextSize(texto, fonte, escala, esp)
        cv2.putText(img, texto, ((W - tw) // 2, y_centro + th // 2), fonte, escala,
                    (0, 0, 0), esp, cv2.LINE_AA)

    linha(TITULO, int(H * 0.25), int(H * 0.18))
    linha(ids_txt or '-', int(H * 0.65), int(H * 0.38))
    return img


def main() -> None:
    rclpy.init()
    node = TagsTela()
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        node.get_logger().warn('tags_tela: sem DISPLAY - a tela das tags nao vai abrir.')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
        return

    import cv2
    import numpy as np

    aberta = False
    W, H = 800, 480            # tela do robo; corrigido pelo tamanho real da janela
    mostrado = None
    img = None
    checar_em = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ids = node.ids_atuais()
            if not ids and not aberta:
                time.sleep(0.05)
                continue
            if not aberta:
                # Tela cheia pedida DEPOIS do 1o imshow (o Qt ignora antes) e
                # reaplicada se o gerenciador de janelas desfizer.
                cv2.namedWindow(JANELA, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                aberta, mostrado, checar_em = True, None, 0.0
            txt = texto_ids(ids)
            if txt != mostrado or img is None:
                mostrado = txt
                img = desenhar(cv2, np, W, H, txt)
            cv2.imshow(JANELA, img)
            tecla = cv2.waitKey(50) & 0xFF
            if time.monotonic() >= checar_em:
                checar_em = time.monotonic() + 0.5
                try:
                    if cv2.getWindowProperty(JANELA, cv2.WND_PROP_FULLSCREEN) != cv2.WINDOW_FULLSCREEN:
                        cv2.setWindowProperty(JANELA, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                        try:
                            cv2.setWindowProperty(JANELA, cv2.WND_PROP_TOPMOST, 1)
                        except cv2.error:
                            pass
                    _x, _y, w, h = cv2.getWindowImageRect(JANELA)
                    if w >= 200 and h >= 120 and (w, h) != (W, H):
                        W, H, img = w, h, None   # redesenha no tamanho real
                except cv2.error:
                    pass
            if tecla in (27, ord('q')):
                cv2.destroyWindow(JANELA)
                aberta, img = False, None
                node._vistas.clear()
                node._ultimas = []
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
