#!/usr/bin/env python3
"""task_viewer: janela em TELA CHEIA, por cima de tudo, durante o pick/place (2026-11-25).

Pedido do operador: "abrir sempre em primeiro plano e em tela cheia [...] mostrando
qual tag ele esta pegando ou colocando. Assim que terminar o pick/place ele tem que
fechar automaticamente para nao gastar ram".

Como funciona (dois processos):
  * SUPERVISOR (modo padrao, leve): assina /manip/task_label (String, latched),
    publicado pelos nos de pick e place ("PEGANDO TAG 25 | Mesa15",
    "COLOCANDO TAG 25 no container azul | WS_3"; vazio = terminou). Texto nao
    vazio -> abre a JANELA num processo filho; vazio -> espera close_delay_s e
    mata o filho (a RAM do OpenCV/Qt e da imagem volta ao sistema).
  * JANELA (--janela): tela BRANCA so' com o texto preto, grosso e enorme
    (pedido do operador 25/11): "PICK:" ou "PLACE:" e embaixo "TAG x". Sem
    imagem da camera (menos RAM). cv2 + Qt5: WND_PROP_FULLSCREEN e
    WND_PROP_TOPMOST. ESC ou 'q' fecha so' a janela desta vez (reabre no
    proximo pick/place).

2026-11-27 (AMT, pedido do operador): "assim que ele identificar a ordem atual
da mesa, colocar na telinha ORDEM ATUAL: TAGS, e conforme ele der um place novo
atualizar essa ordem". O amt_sort_node publica /manip/amt_ordem (latched,
"ORDEM ATUAL: 3 1 - 2 4 5 6"; vazio = fim). O SUPERVISOR decide o que a tela
mostra: o rotulo do pick/place enquanto houver um; senao a ordem da AMT. A
janela so' fecha com os dois vazios (entre um pick e um place da AMT ela fica
aberta mostrando a ordem). A janela le' so' o que o supervisor publica em
/manip/task_viewer/texto (um publicador so': sem a corrida de latched entre
pick e place ao entrar).

Custo fora do pick/place: so' o supervisor.
So' LE: nao publica em topico de comando, nao chama servico nem action.

Quem lanca: manip_bringup/launch/manip_stack.launch.py (use_task_viewer, default
true; precisa de DISPLAY). Na mao:
  ros2 run manip_bringup task_viewer_node.py
  ros2 topic pub -1 /manip/task_label std_msgs/String "{data: 'PEGANDO TAG 25'}"   # teste
"""
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LABEL_TOPIC = '/manip/task_label'
ORDEM_TOPIC = '/manip/amt_ordem'
TEXTO_TOPIC = '/manip/task_viewer/texto'   # o que a janela mostra (so' o supervisor publica)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
WINDOW = 'Caramelo - pick/place'


class Supervisor(Node):

    def __init__(self):
        super().__init__('task_viewer')
        self.declare_parameter('close_delay_s', 0.5)
        self._child = None
        self._close_at = None
        self._task = ''     # rotulo do pick/place em andamento
        self._ordem = ''    # ordem atual da AMT
        self._texto = None  # ultimo texto publicado para a janela
        self._texto_pub = self.create_publisher(String, TEXTO_TOPIC, LATCHED)
        self._sem_display = not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY')
        if self._sem_display:
            self.get_logger().warn('task_viewer: sem DISPLAY — a janela do pick/place nao vai abrir.')
        self.create_subscription(String, LABEL_TOPIC, self._on_label, LATCHED)
        self.create_subscription(String, ORDEM_TOPIC, self._on_ordem, LATCHED)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'task_viewer pronto: {LABEL_TOPIC} -> janela em tela cheia.')

    def _on_label(self, msg: String) -> None:
        self._task = msg.data.strip()
        self._atualizar()

    def _on_ordem(self, msg: String) -> None:
        self._ordem = msg.data.strip()
        self._atualizar()

    def _atualizar(self) -> None:
        """Durante a AMT so' a ordem (2026-11-27, operador: "para a AMT deixe ele so'
        com IDENTIFIQUEI A ORDEM: TAGS, nao precisa mostrar o pick/place"); fora
        dela, o pick/place em andamento; sem os dois, fecha."""
        texto = self._ordem or self._task
        if texto != self._texto:
            self._texto = texto
            self._texto_pub.publish(String(data=texto))
        if texto:
            self._close_at = None
            self._open(texto)
        elif self._child is not None and self._close_at is None:
            self._close_at = time.monotonic() + float(self.get_parameter('close_delay_s').value)

    def _open(self, label: str) -> None:
        if self._sem_display or (self._child is not None and self._child.poll() is None):
            return
        cmd = [sys.executable, os.path.abspath(__file__), '--janela',
               '--ros-args', '-r', '__node:=task_viewer_janela']
        # 2026-11-25: o rotulo vai junto. A janela nasce DEPOIS da mensagem, e o
        # latched so' existe enquanto o publicador vive: com "ros2 topic pub -1"
        # (teste na mao) ele ja' saiu e a janela ficava so' com "...".
        env = dict(os.environ, CARAMELO_TASK_LABEL=label)
        self._child = subprocess.Popen(cmd, env=env)
        self.get_logger().info(f'janela do pick/place aberta (pid {self._child.pid}).')

    def _close(self) -> None:
        child, self._child = self._child, None
        if child is None:
            return
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        self.get_logger().info('janela do pick/place fechada.')

    def _tick(self) -> None:
        if self._close_at is not None and time.monotonic() >= self._close_at and not self._texto:
            self._close_at = None
            self._close()

    def destroy_node(self):
        self._close()
        return super().destroy_node()


def titulo_e_tag(label: str):
    """'PEGANDO TAG 25  |  Mesa15' -> ('IDENTIFIQUEI:', 'TAG 25'); 'COLOCANDO ...' -> 'PLACE:';
    'ORDEM ATUAL: 3 1 - 2' -> ('ORDEM ATUAL:', '3 1 - 2')."""
    texto = label.strip().upper()
    for prefixo in ('IDENTIFIQUEI A ORDEM', 'ORDEM ATUAL'):
        if texto.startswith(prefixo):
            resto = texto[len(prefixo):].lstrip(':').strip()
            return 'IDENTIFIQUEI A ORDEM:', resto or '-'
    # 'PICK ...' / 'PLACE ...' tambem valem (teste na mao: "pick tag 20").
    # 2026-11-26 (operador): no pick a tela diz IDENTIFIQUEI, nao PICK.
    titulo = 'IDENTIFIQUEI:' if texto.startswith(('PEGANDO', 'PICK')) else (
        'PLACE:' if texto.startswith(('COLOCANDO', 'PLACE')) else '')
    m = re.search(r'TAG[ _]*([A-Z0-9_]+)', texto)
    tag = f'TAG {m.group(1)}' if m else texto
    return titulo, tag


def janela() -> None:
    """Processo filho: tela branca com 'IDENTIFIQUEI:' / 'PLACE:' e 'TAG x' enormes."""
    import cv2
    import numpy as np

    rclpy.init(args=sys.argv)
    node = Node('task_viewer_janela')
    estado = {'label': os.environ.get('CARAMELO_TASK_LABEL', '').strip()}

    def on_label(msg):
        # So' o supervisor publica aqui (pick/place ou ordem da AMT). Vazio e'
        # ignorado: quem fecha a janela e' o supervisor.
        if msg.data.strip():
            estado['label'] = msg.data.strip()

    node.create_subscription(String, TEXTO_TOPIC, on_label, LATCHED)

    # 2026-11-27 (operador: "ela nao esta abrindo em tela cheia"): com o highgui
    # em Qt5 o pedido de tela cheia feito ANTES de a janela aparecer pode ser
    # ignorado. Agora: janela sem barra de ferramentas do Qt (GUI_NORMAL), tela
    # cheia pedida depois do 1o imshow e reaplicada se o gerenciador desfizer.
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)

    def tela_cheia():
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        try:
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_TOPMOST, 1)
        except cv2.error:
            pass  # backend sem "sempre por cima": segue so' em tela cheia

    W, H = 1920, 1080
    fonte = cv2.FONT_HERSHEY_DUPLEX
    desenhado = None
    quadro = None

    def linha(img, texto, y_centro, alt_max):
        """Texto preto, grosso, centralizado, o maior que couber (90% da largura)."""
        (tw, th), _ = cv2.getTextSize(texto, fonte, 1.0, 2)
        escala = min(0.9 * W / tw, alt_max / th)
        esp = max(6, int(escala * 2.8))
        (tw, th), _ = cv2.getTextSize(texto, fonte, escala, esp)
        cv2.putText(img, texto, ((W - tw) // 2, y_centro + th // 2), fonte, escala,
                    (0, 0, 0), esp, cv2.LINE_AA)

    proxima_checagem = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if estado['label'] != desenhado:
                desenhado = estado['label']
                titulo, tag = titulo_e_tag(desenhado) if desenhado else ('', '...')
                quadro = np.full((H, W, 3), 255, np.uint8)
                if titulo:
                    linha(quadro, titulo, int(H * 0.30), int(H * 0.28))
                    linha(quadro, tag, int(H * 0.70), int(H * 0.34))
                else:
                    linha(quadro, tag, H // 2, int(H * 0.34))
            cv2.imshow(WINDOW, quadro)
            tecla = cv2.waitKey(50) & 0xFF
            if time.monotonic() >= proxima_checagem:
                proxima_checagem = time.monotonic() + 0.5
                try:
                    if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN) != cv2.WINDOW_FULLSCREEN:
                        tela_cheia()
                except cv2.error:
                    pass
            if tecla in (27, ord('q')):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> None:
    if '--janela' in sys.argv:
        sys.argv.remove('--janela')
        janela()
        return
    rclpy.init(args=sys.argv)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
