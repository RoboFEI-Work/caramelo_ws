"""Nó ROS único do painel web.

Um nó só, girando numa thread própria, guardando o último estado de cada coisa
que o painel mostra. O servidor HTTP lê esse estado; ele nunca fala ROS direto.
É a mesma regra da GUI Qt (widget não fala ROS), pelo mesmo motivo: o transporte
muda, o contrato não.

Por que o estado fica guardado em vez de ser repassado a cada mensagem: o painel
é para uso EM REDE, e numa rede de competição a banda que ele gastar sai da
navegação. O servidor amostra o estado na taxa que a tela precisa (10 Hz para a
pose, 5 Hz para o LiDAR), independente da taxa em que o robô publica.

DUAS REGRAS QUE NÃO SE NEGOCIAM NESTE ARQUIVO (as duas já custaram bancada):

  1. Meta de navegação só pela action navigate_to_pose. NUNCA publicar em
     /goal_pose: o bt_navigator do Nav2 Jazzy também assina esse tópico, e um
     clique no mapa viraria DOIS goals, um preemptando o outro, com o painel
     mostrando o status do que já morreu.
  2. Teleop só por /cmd_vel_key. NUNCA por /mecanum_controller/reference:
     escrever lá pula o twist_mux (que dá prioridade ao comando manual sobre a
     navegação) e pula o collision_monitor — ou seja, entrega a base sem freio
     de emergência nenhum.
"""
import io
import math
import threading
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import (BatteryState, CompressedImage, Image, Imu,
                             LaserScan)
from tf2_ros import Buffer, TransformListener

from caramelo_web import arenas

try:
    from caramelo_msgs.action import RunMission
    from caramelo_msgs.msg import MissionStatus
except ImportError:  # pragma: no cover - o painel funciona sem a missão
    RunMission = None
    MissionStatus = None

ESTADOS_MISSAO = {
    0: "parado", 1: "planejando", 2: "pre-flight",
    3: "executando", 4: "concluida", 5: "falhou", 6: "abortada",
}

# --- O que o MENU PRINCIPAL mostra ------------------------------------------
# Copiado de caramelo_gui/src/core/home_screen.cpp e main_window.cpp: mesmo
# menu, mesmos titulos, mesmos contextos. Fica no servidor, e nao na tela, para
# a web nao poder divergir da GUI por um titulo redigitado errado. Vai uma vez
# so por conexao (primeiro quadro do WebSocket), porque nada disto muda.
MENU_PRINCIPAL = {
    "robo": {"nome": "Caramelo", "equipe": "RoboFEI@Work"},
    "cartoes": [
        {
            "id": "operacao", "titulo": "Operacao",
            "subtitulo": "Navegar, dockar e teleoperar",
            "mostra_mapa": True,
            "secoes": [
                {"id": "mapa", "titulo": "Mapa"},
                {"id": "navegar", "titulo": "Navegar"},
                {"id": "waypoints", "titulo": "Waypoints"},
                {"id": "docking", "titulo": "Docking"},
                {"id": "localizacao", "titulo": "Localizacao"},
                {"id": "teleop", "titulo": "Teleop"},
            ],
        },
        {
            "id": "competicao", "titulo": "Competicao",
            "subtitulo": "Rodar uma prova de ponta a ponta",
            # Sem mapa de proposito: aqui o que importa e o passo da missao.
            "mostra_mapa": False,
            "secoes": [{"id": "missao", "titulo": "Missao"}],
        },
        {
            "id": "mapas", "titulo": "Mapas",
            "subtitulo": "Escolher a arena em que o robo esta",
            "mostra_mapa": False,
            "secoes": [{"id": "arenas", "titulo": "Arenas"}],
        },
        {
            "id": "mapeamento", "titulo": "Mapeamento",
            "subtitulo": "Criar mapa, waypoints e docks",
            "mostra_mapa": True,
            # Uma ferramenta so, com o fluxo inteiro: criar o mapa, marcar
            # pontos, paredes virtuais, conferir.
            "secoes": [{"id": "mapeamento", "titulo": "Mapeamento"}],
        },
        {
            "id": "avancado", "titulo": "Modo Avancado",
            "subtitulo": "Sensores, sistema e diagnostico",
            "mostra_mapa": True,
            "secoes": [
                {"id": "estado", "titulo": "Estado do robo"},
                {"id": "sensores", "titulo": "Sensores ao vivo"},
                {"id": "areas", "titulo": "Service areas"},
                # As duas que a GUI nao tem: quem esta longe do robo nao alcanca
                # o teclado dele.
                {"id": "terminal", "titulo": "Terminal", "so_web": True},
                {"id": "tela", "titulo": "Tela do robo", "so_web": True},
            ],
        },
    ],
}

# Estados de cartao, com as MESMAS cores e rotulos de home_screen.cpp. Estado
# sozinho nao ajuda ninguem ("BLOQUEADO" so informa que algo deu errado): o
# motivo, que vem junto, e a parte util.
ESTADOS_CARTAO = {
    "pronto": {"rotulo": "PRONTO", "cor": "#27ae60"},
    "degradado": {"rotulo": "ATENCAO", "cor": "#f2994a"},
    "bloqueado": {"rotulo": "BLOQUEADO", "cor": "#eb5757"},
    "desconhecido": {"rotulo": "SEM DADOS", "cor": "#7f93b0"},
}

# Niveis do DiagnosticStatus (0 OK, 1 WARN, 2 ERROR, 3 STALE) em linguagem de
# tela.
ESTADOS_COMPONENTE = {
    0: {"estado": "ok", "rotulo": "OK", "cor": "#27ae60"},
    1: {"estado": "atencao", "rotulo": "ATENCAO", "cor": "#f2994a"},
    2: {"estado": "falha", "rotulo": "FALHA", "cor": "#eb5757"},
    3: {"estado": "sem_dados", "rotulo": "SEM DADOS", "cor": "#7f93b0"},
}

# Diagnostico traduzido, componente por componente.
#
# Quem le esta tela nao sabe ROS e nao vai abrir terminal por vontade propria:
# nenhuma frase daqui cita topico, action, no ou pacote. "O LiDAR nao esta
# respondendo" e texto de tela; "/scan STALE" nao e. Por isso as frases do
# health_monitor (que citam /dev/lidar_usb, ROS_DOMAIN_ID, bringup) NAO sao
# repassadas: o nivel vem de la, a explicacao sai daqui.
#
# Componente novo no health_monitor precisa de uma entrada aqui. Sem ela o
# painel ainda mostra o estado, mas nao sabe explicar o que fazer.
COMPONENTES = {
    "caramelo/rede_raspberry": {
        "nome": "Conexao com o robo",
        "ok": "O computador de bordo esta respondendo.",
        "atencao": "A conexao com o computador de bordo esta falhando.",
        "falha": "Sem conexao com o computador de bordo.",
        "sem_dados": "Ainda nao recebi nada do computador de bordo.",
        "acao": "Confira o cabo de rede e espere o robo terminar de ligar.",
    },
    "caramelo/controllers": {
        "nome": "Motores",
        "ok": "Os motores estao prontos.",
        "atencao": "Falta algum motor responder.",
        "falha": "Os motores nao estao prontos: o robo nao anda.",
        "sem_dados": "Nao consigo falar com os motores.",
        "acao": "Desligue e ligue o robo pela chave de forca, e espere ele subir.",
    },
    "caramelo/lidar": {
        "nome": "LiDAR",
        "ok": "Enxergando normalmente.",
        "atencao": "Enxergando devagar: o robo pode reagir tarde a um obstaculo.",
        "falha": "O LiDAR nao esta respondendo.",
        "sem_dados": "O LiDAR nao esta respondendo.",
        "acao": "Veja se o LiDAR esta girando e se o cabo dele esta firme.",
    },
    "caramelo/imu": {
        "nome": "Sensor de inclinacao",
        "ok": "Respondendo.",
        "atencao": "Respondendo devagar.",
        "falha": "O sensor de inclinacao nao esta respondendo.",
        "sem_dados": "O sensor de inclinacao nao esta respondendo.",
        "acao": "Confira o cabo do sensor de inclinacao.",
    },
    "caramelo/odometria": {
        "nome": "Contagem de giro das rodas",
        "ok": "O robo esta contando o quanto anda.",
        "atencao": "A contagem esta chegando devagar.",
        "falha": "O robo nao esta contando o quanto anda.",
        "sem_dados": "O robo nao esta contando o quanto anda.",
        "acao": "Religue o robo; se continuar, o problema esta nos motores.",
    },
    "caramelo/tf": {
        "nome": "Posicao no mapa",
        "ok": "O robo sabe onde esta.",
        "atencao": "A posicao no mapa esta instavel.",
        "falha": "O robo nao sabe onde esta.",
        "sem_dados": "O robo nao sabe onde esta.",
        "acao": "Escolha a arena e marque no mapa onde o robo esta.",
    },
    "caramelo/bateria": {
        "nome": "Bateria",
        "ok": "Carga suficiente.",
        "atencao": "Bateria baixa.",
        "falha": "Bateria no fim.",
        "sem_dados": "Sem leitura da bateria.",
        "acao": "Troque a bateria antes de comecar uma prova.",
    },
    "caramelo/nav2": {
        "nome": "Navegacao",
        "ok": "Pronta para receber destinos.",
        "atencao": "Ligada, mas ainda nao aceita destinos.",
        "falha": "A navegacao nao esta funcionando.",
        "sem_dados": "A navegacao ainda nao foi iniciada.",
        "acao": "Inicie a navegacao antes de mandar o robo a algum lugar.",
    },
}

# Numeros do diagnostico que o operador consegue interpretar. O resto dos
# valores do health_monitor (nome de topico, de servico, de controller) fica
# fora: e informacao de quem abre terminal, e essa tela nao e para isso.
NUMEROS_DO_DIAGNOSTICO = {
    "frequencia_hz": "Mensagens por segundo",
    "minimo_hz": "Minimo esperado",
    "ultima_msg_s": "Silencio (s)",
    "percentual": "Carga (%)",
    "tensao_v": "Tensao (V)",
}

# Depois disto o diagnostico e velho demais para pintar cartao. Sem o teto, o
# painel continuaria mostrando PRONTO em verde com o robo ja desligado -- foi o
# que aconteceu quando o cabo de rede caiu no meio de um teste: a tela ficou
# congelada mostrando tudo certo.
IDADE_DIAGNOSTICO = 5.0

# --- Sensores ao vivo do Modo Avancado --------------------------------------
# A IMU muda de nome entre o robo e a simulacao. A GUI pergunta ao grafo qual
# existe porque la uma assinatura substituiria a outra; aqui as duas convivem
# sem custo, e assinar as duas evita ficar consultando o grafo esperando o
# topico aparecer (ele aparece depois, quando o bringup da Raspberry sobe).
TOPICOS_IMU = ("/imu/data_raw", "/imu")
TOPICO_BATERIA = "/battery_state"

# Camera: NUNCA pelo WebSocket. Vira MJPEG sob demanda, como o espelho de tela.
# Uma camera de 30 quadros por segundo dentro do JSON de estado tiraria da
# navegacao toda a banda da rede de competicao.
FPS_CAMERA = 5.0
LARGURA_CAMERA = 640
QUALIDADE_CAMERA = 60
# Sem ninguem olhando, a assinatura cai. E a mesma regra do hideEvent da GUI:
# camera a 30 Hz nao pode continuar consumindo CPU do computador que roda a
# navegacao enquanto a aba do navegador esta fechada.
JANELA_CAMERA_OCIOSA = 5.0


class _Frequencimetro:
    """Media movel da taxa de um topico.

    Sem a media o numero pisca a cada quadro e o operador nao consegue ler
    (mesma conta do SensorBridge da GUI).
    """

    def __init__(self):
        self.hz = 0.0
        self._ultimo = 0.0

    def registrar(self, agora: float) -> float:
        if self._ultimo > 0.0 and agora > self._ultimo:
            instantanea = 1.0 / (agora - self._ultimo)
            self.hz = instantanea if self.hz <= 0.0 else (self.hz * 0.8 + instantanea * 0.2)
        self._ultimo = agora
        return self.hz


def _numero(valor):
    """float seguro para JSON: NaN e infinito viram None.

    BatteryState chega com NaN em corrente e tensao quando o driver nao mede
    aquilo, e NaN dentro do json.dumps sai como `NaN`, que o JSON.parse do
    navegador recusa -- a tela inteira parava de atualizar por causa de um
    campo que nem estava sendo mostrado.
    """
    try:
        valor = float(valor)
    except (TypeError, ValueError):
        return None
    return valor if math.isfinite(valor) else None


def _numeros_do_componente(bruto) -> list:
    """So os numeros que o operador consegue interpretar."""
    valores = (bruto or {}).get("valores") or {}
    saida = []
    for chave, rotulo in NUMEROS_DO_DIAGNOSTICO.items():
        valor = str(valores.get(chave, "")).strip()
        if valor and valor != "n/a":
            saida.append({"rotulo": rotulo, "valor": valor})
    return saida


def _rotulo_de_camera(topico: str) -> str:
    """Nome de camera que o operador entende, a partir do nome tecnico."""
    t = (topico or "").lower()
    if "depth" in t:
        return "Camera de profundidade"
    if "infra" in t or "/ir" in t:
        return "Camera infravermelha"
    if "color" in t or "rgb" in t:
        return "Camera colorida"
    return "Camera do robo"

# --- Teleop pela rede -------------------------------------------------------
# O ESC desta base interpreta AUSÊNCIA de PWM como ré em velocidade máxima, e
# não existe botão de emergência de hardware. Logo: enquanto o operador dirige,
# alguém tem que estar publicando SEMPRE — parar de publicar não é "parar".
#
# Daí as duas janelas, contadas do último comando recebido do navegador:
#   JANELA_COMANDO  300 ms sem comando novo => publica Twist ZERO (o freio).
#   JANELA_TELEOP   depois disso, publica zero por mais um pouco e PARA.
#
# A segunda janela existe por causa do twist_mux: cmd_vel_key tem prioridade 90
# e a navegação tem 80, então um fluxo eterno de zeros vindos daqui deixaria o
# Nav2 mudo para sempre. O timeout do twist_mux para o teclado é 0,5 s; parar de
# publicar 0,6 s depois do último comando garante que a base recebeu vários
# zeros ANTES de o mux devolver o volante para a navegação.
JANELA_COMANDO = 0.30
JANELA_TELEOP = 0.90
PULSO_TELEOP = 0.05

# Tetos duros, independentes do que a tela mandar. São EXATAMENTE os do
# controller_server (max_vel_x / max_vel_y / max_vel_theta): quem dirige pela
# rede tem atraso de vídeo e não pode andar mais rápido do que o robô anda
# sozinho.
#
# Corrigido na integração: estavam 0.22 / 0.18 / 0.55, 10% ACIMA do controller.
# O comentário já dizia que o teleop remoto não podia ser mais rápido que a
# navegação, mas a "folga mínima" fazia o oposto — o operador com meio segundo
# de atraso de vídeo dirigia mais rápido do que o Nav2 jamais dirige.
#
# ACOPLAMENTO: se caramelo_navigation/config/controller_server.yaml mudar estes
# três valores, mude aqui junto.
LIMITE_VX = 0.20
LIMITE_VY = 0.16
LIMITE_WZ = 0.50


def _limitar(valor, teto) -> float:
    try:
        valor = float(valor)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(valor):
        return 0.0
    return max(-teto, min(teto, valor))


class NoDoPainel(Node):
    def __init__(self):
        super().__init__("caramelo_web")

        self._lock = threading.Lock()
        self.pose = None            # {x, y, yaw, t}
        self.scan = None            # {angle_min, angle_increment, ranges, range_max, t}
        self.diagnostico = {}       # nome -> {level, message}
        self.missao = None          # último MissionStatus, já traduzido
        self.plano = None           # último dry-run da missão, para a tela mostrar
        self.imu = None             # {roll, pitch, yaw, giro_z, acel_*, hz, t}
        self.bateria = None         # {percentual, tensao, corrente, t}
        self._diag_t = 0.0          # quando o ultimo /diagnostics chegou

        self._hz_lidar = _Frequencimetro()
        self._hz_imu = _Frequencimetro()
        self._hz_camera = _Frequencimetro()

        # Bloco de 1 Hz (cartoes, componentes, sensores) calculado no maximo
        # duas vezes por segundo: ele le YAML de arena e consulta o grafo, e o
        # WebSocket pede quadro dez vezes por segundo.
        self._pesado = {}
        self._pesado_t = 0.0

        # Camera: nada assinado ate alguem pedir.
        self._camera_sub = None
        self._camera_topico = ""
        self._camera_jpeg = None
        self._camera_info = {}
        self._camera_visto = 0.0     # ultima vez que um navegador pegou quadro
        self._camera_convertido = 0.0
        # Trava propria: assinar e largar a camera acontece na thread do HTTP,
        # enquanto a thread do ROS entrega quadros. Usar a trava do estado aqui
        # prenderia o callback do LiDAR atras de um create_subscription.
        self._camera_trava = threading.Lock()

        # Fila curta de recados para o operador. Só o que MUDOU desde o último
        # envio viaja pelo WebSocket -- repetir os 40 últimos avisos 10 vezes por
        # segundo seria mais tráfego do que a pose e o LiDAR juntos.
        self._eventos = deque(maxlen=40)
        self._serie_evento = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan, "/scan", self._ao_scan, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._ao_diagnostico, 10)

        # IMU e bateria ficam assinadas o tempo todo: sao mensagens pequenas, e
        # o custo de descobrir no grafo quando elas aparecem seria maior que o
        # de recebe-las. A camera, que e grande, e o contrario -- so sob demanda.
        for topico in TOPICOS_IMU:
            self.create_subscription(
                Imu, topico, self._ao_imu, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, TOPICO_BATERIA, self._ao_bateria, 10)

        if MissionStatus is not None:
            # MESMA QoS do bt_yaml_executor: transient_local com histórico fundo.
            # O publicador nasce junto com a missão, então o painel é sempre um
            # late joiner -- com volatile ou depth 1 ele perderia o começo de
            # toda missão e só veria o último passo.
            qos = QoSProfile(
                depth=64,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                MissionStatus, "/caramelo/mission/status", self._ao_status_missao, qos)

        # Só a action, nunca /goal_pose: o bt_navigator também assina esse
        # tópico, e publicar nele dispararia DOIS goals para um clique.
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._meta_handle = None

        self._limpar_global = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self._limpar_local = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")

        # /initialpose é o mesmo tópico da ferramenta "2D Pose Estimate" do RViz
        # e o mesmo que a GUI Qt usa. O AMCL assina com QoS padrão; profundidade
        # 1 basta porque só a última estimativa interessa.
        self._pose_inicial = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1)

        self._dock = ActionClient(self, DockRobot, "/dock_robot")
        self._undock = ActionClient(self, UndockRobot, "/undock_robot")

        self._missao_cliente = (
            ActionClient(self, RunMission, "/caramelo/run_mission")
            if RunMission is not None else None)
        self._missao_handle = None
        self._missao_ocupada = False

        # Teleop: Twist SEM stamp, porque é assim que o twist_mux deste robô
        # está configurado (use_stamped: false) e é o que o
        # teleop_twist_keyboard publica depois do remap para cmd_vel_key.
        self._cmd_vel = self.create_publisher(Twist, "/cmd_vel_key", 1)
        self._teleop_cmd = (0.0, 0.0, 0.0)
        self._teleop_fresco_ate = 0.0
        self._teleop_ate = 0.0

        self.create_timer(0.1, self._amostrar_pose)
        self.create_timer(1.0, self._vigiar_camera)

        # O pulso do teleop NAO e um timer do ROS. MEDIDO (2026-08-18): com
        # create_timer(0.05) o executor entregava o callback a 10,3 Hz, com
        # intervalo travado em 100 ms -- ele coalesce com o timer de pose e a
        # taxa pedida nao acontece. Para um heartbeat de tela seria irrelevante;
        # para o freio de um robo cujo ESC le ausencia de PWM como re' em
        # velocidade maxima, nao e. Aqui o pulso e uma thread propria, imune a
        # qualquer engasgo do executor (uma rajada de /scan, um lookup de TF
        # lento): o freio nunca fica esperando a fila de outra coisa.
        self._teleop_parar_evento = threading.Event()
        self._teleop_thread = threading.Thread(
            target=self._laco_teleop, daemon=True)
        self._teleop_thread.start()

    # ------------------------------------------------------------- callbacks
    def _ao_scan(self, msg: LaserScan):
        agora = time.time()
        # O resumo (taxa, quantas leituras prestam, abertura) vai junto com os
        # pontos porque e a mesma pergunta do Modo Avancado da GUI: "o sensor
        # publica" e "o dado presta" sao coisas diferentes -- um LiDAR pode
        # estar na frequencia certa e enxergar so ruido.
        validos = 0
        pontos = []
        for i, r in enumerate(msg.ranges):
            # Fora de [range_min, range_max] nao e leitura, e ruido: a GUI
            # tambem nao desenha esses pontos.
            valido = math.isfinite(r) and msg.range_min <= r <= msg.range_max
            if valido:
                validos += 1
            # Decimado por 2 já aqui: o navegador não distingue 800 de 400
            # pontos na tela, e são bytes que não sobem na rede.
            if i % 2 == 0:
                pontos.append(round(float(r), 3) if valido else None)
        with self._lock:
            hz = self._hz_lidar.registrar(agora)
            self.scan = {
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
                "ranges": pontos,
                "hz": round(hz, 1),
                "validos": validos,
                "total": len(msg.ranges),
                "t": agora,
            }

    def _ao_diagnostico(self, msg: DiagnosticArray):
        agora = time.time()
        with self._lock:
            for st in msg.status:
                self.diagnostico[st.name] = {
                    "level": int.from_bytes(st.level, "big")
                    if isinstance(st.level, bytes) else int(st.level),
                    "message": st.message,
                    # Os valores por componente (frequencia, carga da bateria)
                    # sao o que o Modo Avancado mostra abaixo do farol. Guardar
                    # so nivel e mensagem obrigava a tela a adivinhar numeros.
                    "valores": {kv.key: kv.value for kv in st.values},
                }
            self._diag_t = agora

    def _ao_imu(self, msg: Imu):
        agora = time.time()
        # Quaternion -> roll/pitch/yaw sem puxar tf2 so por isto (mesma conta do
        # SensorBridge da GUI).
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        graus = 180.0 / math.pi
        with self._lock:
            hz = self._hz_imu.registrar(agora)
            self.imu = {
                "roll": round(math.atan2(sinr, cosr) * graus, 1),
                "pitch": round(math.asin(sinp) * graus, 1),
                "yaw": round(math.atan2(siny, cosy) * graus, 1),
                "giro_z": _numero(round(msg.angular_velocity.z, 3)),
                "acel_x": _numero(round(msg.linear_acceleration.x, 2)),
                "acel_y": _numero(round(msg.linear_acceleration.y, 2)),
                "acel_z": _numero(round(msg.linear_acceleration.z, 2)),
                "hz": round(hz, 1),
                "t": agora,
            }

    def _ao_bateria(self, msg: BatteryState):
        # O driver publica 0..1 ou 0..100 dependendo da versao; normaliza aqui,
        # senao a tela mostra "0%" com a bateria cheia.
        pct = _numero(msg.percentage)
        if pct is not None and pct <= 1.0:
            pct = pct * 100.0
        with self._lock:
            self.bateria = {
                "percentual": None if pct is None else round(pct),
                "tensao": _numero(msg.voltage),
                "corrente": _numero(msg.current),
                "carregando": int(msg.power_supply_status) == 1,
                "t": time.time(),
            }

    # -------------------------------------------------------------- camera
    #
    # A camera NAO viaja pelo WebSocket: ela sai em MJPEG, sob demanda, pela
    # mesma ideia do espelho de tela. E por isso que ela e a unica assinatura
    # que nasce desligada -- 30 quadros por segundo dentro do JSON de estado
    # tirariam da navegacao a banda inteira da rede de competicao.
    def topicos_de_camera(self) -> list:
        """Cameras publicando AGORA, com nome que o operador entende.

        A lista vem do grafo e nao de uma constante: no robo a camera so
        aparece quando a parte do braco e ligada, e na simulacao ela nao aparece
        nunca (o modelo tem a camera so como geometria).
        """
        achadas = []
        for nome, tipos in self.get_topic_names_and_types():
            for tipo in tipos:
                if tipo in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                    achadas.append({
                        "id": nome,
                        "rotulo": _rotulo_de_camera(nome),
                        "comprimida": tipo.endswith("CompressedImage"),
                    })
                    break
        return sorted(achadas, key=lambda c: c["id"])

    def assinar_camera(self, topico: str) -> tuple:
        """Liga UMA camera. O nome vem do navegador, entao so vale o que existe.

        Sem esta conferencia, um pedido forjado faria o painel assinar qualquer
        topico do grafo -- e a assinatura fica de pe consumindo rede.
        """
        topico = (topico or "").strip()
        disponiveis = {c["id"]: c for c in self.topicos_de_camera()}
        if topico not in disponiveis:
            return False, "Essa camera nao esta publicando agora."

        with self._lock:
            self._camera_visto = time.time()
        with self._camera_trava:
            if topico == self._camera_topico and self._camera_sub is not None:
                return True, "Camera ligada."
            self._desligar_camera_interno()
            comprimida = disponiveis[topico]["comprimida"]
            self._camera_sub = self.create_subscription(
                CompressedImage if comprimida else Image, topico,
                self._ao_camera_comprimida if comprimida else self._ao_camera,
                qos_profile_sensor_data)
            self._camera_topico = topico
        with self._lock:
            self._hz_camera = _Frequencimetro()
            self._camera_jpeg = None
            self._camera_info = {"rotulo": disponiveis[topico]["rotulo"]}
        return True, "Camera ligada."

    def desligar_camera(self) -> tuple:
        with self._camera_trava:
            self._desligar_camera_interno()
        return True, "Camera desligada."

    def _desligar_camera_interno(self):
        # Chamar sempre com _camera_trava na mao.
        if self._camera_sub is not None:
            self.destroy_subscription(self._camera_sub)
        self._camera_sub = None
        self._camera_topico = ""
        with self._lock:
            self._camera_jpeg = None
            self._camera_info = {}

    def _vigiar_camera(self):
        """Ninguem olhando ha alguns segundos: larga a camera.

        Mesma regra do hideEvent da GUI. A diferenca e que aqui nao existe
        evento de "fechou a tela": a aba do navegador pode sumir sem avisar, e
        sem este relogio a camera continuaria consumindo CPU e rede para
        ninguem.
        """
        with self._lock:
            ocioso = time.time() - self._camera_visto
        if ocioso > JANELA_CAMERA_OCIOSA:
            with self._camera_trava:
                if self._camera_sub is not None:
                    self._desligar_camera_interno()

    def _ao_camera(self, msg: Image):
        agora = time.time()
        with self._lock:
            self._hz_camera.registrar(agora)
        # Converter todo quadro de uma camera de 30 Hz em JPEG dentro do Python
        # comeria a CPU do computador que tambem roda a navegacao. Para a
        # pergunta desta tela ("a camera esta enxergando?"), cinco quadros por
        # segundo respondem igual.
        if agora - self._camera_convertido < 1.0 / FPS_CAMERA:
            return
        self._camera_convertido = agora
        jpeg, largura, altura, erro = _jpeg_de_imagem(msg)
        self._guardar_quadro(jpeg, msg.encoding, largura, altura, erro, agora)

    def _ao_camera_comprimida(self, msg: CompressedImage):
        agora = time.time()
        with self._lock:
            self._hz_camera.registrar(agora)
        if agora - self._camera_convertido < 1.0 / FPS_CAMERA:
            return
        self._camera_convertido = agora

        dados = bytes(msg.data)
        formato = str(msg.format or "")
        largura = altura = 0
        erro = ""
        try:
            from PIL import Image as ImagemPil
            with ImagemPil.open(io.BytesIO(dados)) as img:
                largura, altura = img.size
                ja_jpeg = "jpeg" in formato.lower() or "jpg" in formato.lower()
                # JPEG do tamanho certo passa direto, sem recodificar: e o
                # caminho mais barato que existe para a camera chegar na tela.
                if not ja_jpeg or largura > LARGURA_CAMERA:
                    dados, largura, altura, erro = _para_jpeg(img)
        except Exception:
            dados, erro = None, "Chegou imagem que o painel nao conseguiu abrir."
        self._guardar_quadro(dados, formato, largura, altura, erro, agora)

    def _guardar_quadro(self, jpeg, formato, largura, altura, erro, agora):
        with self._lock:
            self._camera_jpeg = jpeg
            self._camera_info = {
                "rotulo": self._camera_info.get("rotulo", ""),
                "formato": str(formato),
                "largura": int(largura),
                "altura": int(altura),
                "hz": round(self._hz_camera.hz, 1),
                "mensagem": erro,
                "t": agora,
            }

    def quadro_camera(self) -> tuple:
        """Ultimo quadro em JPEG, para a rota MJPEG do servidor.

        Pedir quadro E dizer "tem gente olhando": e isto que segura a assinatura
        viva contra o relogio de _vigiar_camera.
        """
        with self._lock:
            self._camera_visto = time.time()
            return self._camera_jpeg, dict(self._camera_info)

    def _ao_status_missao(self, msg):
        with self._lock:
            self.missao = {
                "estado": ESTADOS_MISSAO.get(int(msg.state), "?"),
                "mission_id": msg.mission_id,
                "task_id": msg.task_id,
                "passo": int(msg.action_index),
                "total": int(msg.action_total),
                "tipo": msg.action_kind,
                "alvo": msg.action_target,
                "subestagio": msg.stage,
                "mensagem": msg.message,
                "decorrido": float(msg.elapsed),
            }

    def _amostrar_pose(self):
        """Lê a TF map->base_footprint. Sem localização, a pose fica None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.pose = {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "yaw": yaw,
                "t": time.time(),
            }

    # ----------------------------------------------------------- recados
    def _avisar(self, texto: str, ok: bool = True):
        with self._lock:
            self._serie_evento += 1
            self._eventos.append({
                "n": self._serie_evento, "texto": texto,
                "ok": bool(ok), "t": time.time()})

    # ---------------------------------------------------------------- leitura
    def quadro(self, ciclo: int = 0, desde_evento: int = 0) -> dict:
        """UM quadro do /ws/estado, com as frequencias ja decididas aqui.

        E este o metodo que o WebSocket deve chamar, a 10 Hz, passando um
        contador que so cresce. A politica de banda mora aqui e nao no servidor
        porque quem sabe o custo de cada bloco e quem o monta:

          pose        10 Hz   e o que se mexe na tela
          LiDAR        5 Hz   ja decimado pela metade na chegada
          o resto      1 Hz   cartoes, diagnostico, sensores, capacidades

        O primeiro quadro de cada conexao vem COMPLETO, com o menu junto: uma
        tela que abre e fica um segundo sem saber o nome dos proprios botoes
        parece travada.
        """
        primeiro = ciclo <= 0
        dados = self.instantaneo(
            com_scan=primeiro or (ciclo % 2 == 0),
            desde_evento=desde_evento,
            com_diagnostico=primeiro or (ciclo % 10 == 0))
        if primeiro:
            dados["menu"] = MENU_PRINCIPAL
        return dados

    def menu(self) -> dict:
        """O menu principal: titulos, subtitulos, contextos e secoes."""
        return MENU_PRINCIPAL

    def instantaneo(self, com_scan: bool = True, desde_evento: int = 0,
                    com_diagnostico: bool = True) -> dict:
        """Foto do estado. Para o WebSocket, prefira quadro().

        com_diagnostico=False deixa de fora o bloco de 1 Hz (cartoes,
        componentes, sensores, capacidades); a tela continua com o ultimo que
        recebeu, que e o comportamento certo -- nada ali muda dez vezes por
        segundo.
        """
        agora = time.time()
        with self._lock:
            pose = dict(self.pose) if self.pose else None
            scan = dict(self.scan) if (com_scan and self.scan) else None
            missao = dict(self.missao) if self.missao else None
            serie = self._serie_evento
            novos = [dict(e) for e in self._eventos if e["n"] > desde_evento]
            teleop_ligado = agora <= self._teleop_ate
        # "Dado velho" é diferente de "sem dado": sem esta marca, uma tela
        # congelada pareceria um robô parado.
        if pose and agora - pose["t"] > 2.0:
            pose["velho"] = True
        if scan and agora - scan["t"] > 2.0:
            scan["velho"] = True
        foto = {
            "pose": pose, "scan": scan, "missao": missao,
            "eventos": novos, "serie_evento": serie,
            "teleop_ligado": teleop_ligado,
        }
        if com_diagnostico:
            foto.update(self._bloco_lento(agora))
        return foto

    def _bloco_lento(self, agora: float) -> dict:
        """O que a tela pede uma vez por segundo, calculado no maximo duas.

        Montar este bloco le YAML de arena e consulta o grafo do ROS. Com o
        WebSocket pedindo quadro dez vezes por segundo e mais de um navegador
        aberto, refazer tudo a cada pedido gastaria CPU do computador que roda a
        navegacao para produzir exatamente a mesma resposta.
        """
        if self._pesado and (agora - self._pesado_t) < 0.5:
            return self._pesado
        with self._lock:
            diag = dict(self.diagnostico)
        bloco = {
            "diagnostico": diag,          # cru, como sempre esteve
            "componentes": self.componentes_de_saude(diag, agora),
            "cartoes": self.cartoes(diag, agora),
            "sensores": self.sensores(agora),
            "capacidades": self.capacidades(),
        }
        self._pesado = bloco
        self._pesado_t = agora
        return bloco

    # ------------------------------------------------------- menu principal
    def cartoes(self, diag: dict = None, agora: float = None) -> dict:
        """O menu principal ja DECIDIDO: estado e motivo de cada cartao.

        Traducao 1:1 de HomeScreen::reavaliar (caramelo_gui/src/core). A conta
        fica aqui, e nao no navegador, porque "o que da para fazer agora, e o
        que esta me impedindo" tem que ter a MESMA resposta na GUI e na web --
        duas contas separadas viram duas respostas diferentes no dia em que uma
        das duas for corrigida.

        Nenhum texto daqui cita topico, action, no ou pacote.
        """
        agora = agora or time.time()
        with self._lock:
            if diag is None:
                diag = dict(self.diagnostico)
            idade = agora - self._diag_t if self._diag_t else None

        # Sem diagnostico, ou com diagnostico velho, e "nao sei" -- nunca
        # "esta tudo bem". Um painel congelado mostrando PRONTO em verde com o
        # robo desligado e pior que um painel que assume que nao sabe.
        sem_dados = (not diag) or idade is None or idade > IDADE_DIAGNOSTICO

        def nivel(nome):
            if sem_dados:
                return 3
            item = diag.get(nome)
            return 3 if item is None else int(item.get("level", 3))

        arena = arenas.estado_da_arena_ativa()
        capacidades = self.capacidades()
        cartoes = {}

        def aplicar(chave, estado, motivo=""):
            cartoes[chave] = dict(ESTADOS_CARTAO[estado], estado=estado, motivo=motivo)

        # --- Operacao ---
        if sem_dados:
            aplicar("operacao", "desconhecido", "Aguardando o robo responder.")
        elif nivel("caramelo/rede_raspberry") >= 2:
            aplicar("operacao", "bloqueado", "Sem conexao com o robo.")
        elif nivel("caramelo/lidar") >= 2 or nivel("caramelo/odometria") >= 2:
            aplicar("operacao", "bloqueado", "Sensores de navegacao sem dados.")
        elif nivel("caramelo/tf") >= 2:
            aplicar("operacao", "degradado",
                    "O robo ainda nao sabe onde esta. Defina a posicao inicial em Operacao.")
        elif nivel("caramelo/nav2") >= 1:
            aplicar("operacao", "degradado", "Navegacao ainda subindo.")
        else:
            aplicar("operacao", "pronto")

        # --- Competicao ---
        if not capacidades["missao"]:
            aplicar("competicao", "bloqueado",
                    "Sistema de missoes fora do ar. Reinicie o robo.")
        elif not arena["existe"]:
            aplicar("competicao", "bloqueado", "Escolha a arena antes.")
        elif nivel("caramelo/tf") >= 2:
            aplicar("competicao", "degradado", "Defina a posicao inicial do robo.")
        else:
            aplicar("competicao", "pronto")

        # --- Mapas ---
        # A GUI so pergunta se ha nome de arena. Aqui a pergunta e se a arena
        # AINDA EXISTE: achado em campo, o robo guardava "warehouse", que ja
        # tinha sido apagada de maps/ -- e a tela dizia que estava tudo pronto.
        aplicar("mapas", "pronto" if arena["existe"] else "degradado",
                "" if arena["existe"] else arena["motivo"])

        # --- Mapeamento ---
        if sem_dados:
            aplicar("mapeamento", "desconhecido", "Aguardando o robo responder.")
        elif nivel("caramelo/lidar") >= 2:
            aplicar("mapeamento", "bloqueado", "LiDAR sem dados: nao da para mapear.")
        else:
            aplicar("mapeamento", "pronto")

        # Diagnostico existe justamente para quando algo esta errado.
        aplicar("avancado", "pronto")

        if sem_dados:
            estado_robo = "Aguardando o robo responder"
        elif nivel("caramelo/rede_raspberry") >= 2:
            estado_robo = "Sem conexao com o robo"
        elif nivel("caramelo/tf") >= 2:
            estado_robo = "Ligado, mas sem saber onde esta"
        else:
            estado_robo = "Pronto para operar"

        return {
            "robo": {
                "nome": MENU_PRINCIPAL["robo"]["nome"],
                "equipe": MENU_PRINCIPAL["robo"]["equipe"],
                "estado": estado_robo,
                "arena": ("Arena: " + arena["nome"]) if arena["nome"] else
                         "Arena: nenhuma escolhida",
                "arena_nome": arena["nome"],
                "arena_existe": arena["existe"],
            },
            "cartoes": cartoes,
            "acoes": self._acoes_rapidas(arena, capacidades, nivel),
        }

    def _acoes_rapidas(self, arena: dict, capacidades: dict, nivel) -> dict:
        """A barra inferior do menu, com o MOTIVO de cada botao desabilitado.

        Botao cinza sem explicacao faz o operador achar que a interface esta
        quebrada; com o motivo, ele sabe o que fazer.
        """
        base_id, motivo = "", ""
        if not arena["existe"]:
            motivo = arena["motivo"]
        else:
            resumo = arenas.resumo(arena["nome"])
            base = resumo.get("base", {})
            base_id = base.get("id", "")
            if not base.get("existe"):
                motivo = "Sem base definida — crie um dock START em Mapeamento."
            elif base.get("sem_pose"):
                motivo = ("A base ainda nao tem posicao gravada. Leve o robo ate ela "
                          "e grave a pose em Mapeamento.")
            elif not capacidades["dock"]:
                motivo = "O encaixe automatico nao esta ligado neste robo."
            elif nivel("caramelo/tf") >= 2:
                motivo = "O robo nao sabe onde esta: marque no mapa a posicao dele antes."

        return {
            "parar": {
                "rotulo": "Parar o robo",
                "habilitado": True,
                # O rotulo NAO promete parada de emergencia: este robo nao tem
                # botao de emergencia de hardware, e prometer um seria pior que
                # nao ter nenhum.
                "ajuda": ("Cancela o destino atual e aborta a missao. Nao e um botao "
                          "de emergencia: este robo nao tem um."),
            },
            "base": {
                "rotulo": "Levar para a base",
                "habilitado": not motivo,
                "motivo": motivo,
                "dock_id": base_id,
            },
            "diagnostico": {
                "rotulo": "Diagnostico",
                "habilitado": True,
                "contexto": "avancado",
            },
        }

    # ----------------------------------------------------------- diagnostico
    def componentes_de_saude(self, diag: dict = None, agora: float = None) -> list:
        """Estado de cada parte do robo, em portugues, para o Modo Avancado.

        O nivel vem do /diagnostics; a EXPLICACAO sai de COMPONENTES, aqui do
        lado. As frases originais citam /dev, nome de servico e comando de
        terminal -- uteis para quem desenvolve, inuteis para quem opera, e
        proibidas nesta tela.
        """
        agora = agora or time.time()
        with self._lock:
            if diag is None:
                diag = dict(self.diagnostico)
            idade = agora - self._diag_t if self._diag_t else None
        velho = idade is None or idade > IDADE_DIAGNOSTICO

        saida = []
        for ident, texto in COMPONENTES.items():
            bruto = diag.get(ident)
            presente = bruto is not None and not velho
            nivel = 3 if not presente else int(bruto.get("level", 3))
            estado = ESTADOS_COMPONENTE.get(nivel, ESTADOS_COMPONENTE[3])
            saida.append({
                "id": ident,
                "nome": texto["nome"],
                "estado": estado["estado"],
                "rotulo": estado["rotulo"],
                "cor": estado["cor"],
                "mensagem": texto[estado["estado"]],
                # Acao so quando ha o que fazer: repetir "confira o cabo" com o
                # sensor funcionando treina o operador a ignorar o texto.
                "acao": "" if nivel == 0 else texto["acao"],
                "numeros": _numeros_do_componente(bruto),
                "presente": bool(presente),
            })

        # Componente que o health_monitor publicou e o painel ainda nao sabe
        # traduzir: aparece com o estado, sem inventar explicacao (e sem repetir
        # a frase tecnica dele, que citaria topico).
        for ident in sorted(set(diag) - set(COMPONENTES)):
            nivel = 3 if velho else int(diag[ident].get("level", 3))
            estado = ESTADOS_COMPONENTE.get(nivel, ESTADOS_COMPONENTE[3])
            saida.append({
                "id": ident,
                "nome": ident.split("/")[-1].replace("_", " ").capitalize(),
                "estado": estado["estado"],
                "rotulo": estado["rotulo"],
                "cor": estado["cor"],
                "mensagem": "Esta parte do robo ainda nao tem descricao no painel.",
                "acao": "",
                "numeros": _numeros_do_componente(diag[ident]),
                "presente": not velho,
            })
        return saida

    # -------------------------------------------------------------- sensores
    def sensores(self, agora: float = None) -> dict:
        """Sensores ao vivo do Modo Avancado.

        Os cartoes de estado dizem se a mensagem chega; aqui da para ver se o
        dado PRESTA: um sensor pode publicar na frequencia certa e enxergar so
        ruido. Os pontos do LiDAR nao se repetem aqui -- eles ja vao em `scan`,
        a 5 Hz, e mandar a mesma lista duas vezes seria pagar a rede em dobro.
        """
        agora = agora or time.time()
        with self._lock:
            scan = dict(self.scan) if self.scan else None
            imu = dict(self.imu) if self.imu else None
            bateria = dict(self.bateria) if self.bateria else None
            diag = dict(self.diagnostico)
            camera_topico = self._camera_topico
            camera_info = dict(self._camera_info)

        lidar = {"tem_dado": False}
        if scan:
            idade = agora - scan["t"]
            abertura = float(scan["angle_max"]) - float(scan["angle_min"])
            lidar = {
                "tem_dado": idade <= 2.0,
                # Taxa velha e mentira: zera em vez de congelar o ultimo numero.
                "hz": 0.0 if idade > 2.0 else scan.get("hz", 0.0),
                "validos": scan.get("validos", 0),
                "total": scan.get("total", 0),
                "abertura_graus": round(abertura * 180.0 / math.pi),
                "alcance_max": round(float(scan["range_max"]), 1),
                "idade": round(idade, 1),
                # O Caramelo recorta o LiDAR. Desenhar o circulo inteiro daria a
                # impressao de cobertura total, que e justamente o engano que
                # termina em colisao frontal.
                "tem_setor_cego": abertura < (2.0 * math.pi - 0.05),
            }

        if imu:
            idade = agora - imu["t"]
            imu["idade"] = round(idade, 1)
            imu["tem_dado"] = idade <= 2.0
            if idade > 2.0:
                imu["hz"] = 0.0
        else:
            imu = {"tem_dado": False}

        # Sem publicacao de bateria, o numero ainda pode vir do diagnostico --
        # e melhor mostrar a carga de um segundo atras do que nao mostrar carga.
        if bateria is None:
            valores = (diag.get("caramelo/bateria") or {}).get("valores") or {}
            if valores.get("percentual"):
                bateria = {
                    "percentual": _numero(valores.get("percentual")),
                    "tensao": _numero(valores.get("tensao_v")),
                    "corrente": None,
                    "carregando": False,
                    "t": agora,
                }
        if bateria is None:
            bateria = {"tem_dado": False}
        else:
            bateria["tem_dado"] = True

        fontes = self.topicos_de_camera()
        camera = {
            "fontes": fontes,
            "ativa": camera_topico,
            "ligada": bool(camera_topico),
        }
        camera.update(camera_info)
        if not fontes:
            camera["mensagem"] = (
                "Nenhuma camera publicando agora. No robo ela aparece quando a parte "
                "do braco e ligada; na simulacao o robo nao tem camera.")
        elif not camera_topico:
            camera["mensagem"] = camera.get("mensagem") or "Escolha uma camera para ver."

        return {"lidar": lidar, "imu": imu, "bateria": bateria, "camera": camera}


    def nos_no_grafo(self) -> list:
        return sorted(n for n, _ns in self.get_node_names_and_namespaces())

    def navegacao_pronta(self) -> bool:
        return self._nav.server_is_ready()

    def capacidades(self) -> dict:
        """O que está no ar AGORA. A tela usa isto para desabilitar botão.

        Botão que existe e não funciona é pior do que botão ausente: o operador
        aperta, nada acontece, e ele passa a duvidar do painel inteiro.
        """
        return {
            "navegacao": self._nav.server_is_ready(),
            "dock": self._dock.server_is_ready(),
            "undock": self._undock.server_is_ready(),
            "missao": bool(self._missao_cliente and
                           self._missao_cliente.server_is_ready()),
            "costmaps": (self._limpar_global.service_is_ready() or
                         self._limpar_local.service_is_ready()),
            "missao_ocupada": self._missao_ocupada,
        }

    # -------------------------------------------------------------- navegação
    def enviar_meta(self, x: float, y: float, yaw: float) -> tuple:
        if not self._nav.server_is_ready():
            return False, "A navegação não está no ar, então o robô não sabe ir a lugar nenhum."
        meta = NavigateToPose.Goal()
        meta.pose.header.frame_id = "map"
        meta.pose.header.stamp = self.get_clock().now().to_msg()
        meta.pose.pose.position.x = float(x)
        meta.pose.pose.position.y = float(y)
        meta.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        meta.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        futuro = self._nav.send_goal_async(meta)
        futuro.add_done_callback(self._ao_aceitar_meta)
        return True, "Destino enviado. O robô está indo."

    def _ao_aceitar_meta(self, futuro):
        try:
            handle = futuro.result()
        except Exception:
            handle = None
        if handle is None or not handle.accepted:
            with self._lock:
                self._meta_handle = None
            self._avisar(
                "A navegação recusou o destino (ele está dentro de um obstáculo?).",
                ok=False)
            return
        with self._lock:
            self._meta_handle = handle
        # Guardar o handle é o que torna "Parar" possível: sem ele, cancelar
        # exigiria varrer os goals do servidor e adivinhar qual é o nosso.
        handle.get_result_async().add_done_callback(self._ao_terminar_meta)

    def _ao_terminar_meta(self, futuro):
        try:
            resultado = futuro.result()
            codigo = int(resultado.status)
        except Exception:
            codigo = -1
        with self._lock:
            self._meta_handle = None
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED (action_msgs/GoalStatus).
        if codigo == 4:
            self._avisar("O robô chegou ao destino.")
        elif codigo == 5:
            self._avisar("O robô parou onde estava, a pedido.")
        else:
            self._avisar("O robô não conseguiu chegar ao destino.", ok=False)

    def cancelar_navegacao(self) -> tuple:
        with self._lock:
            handle = self._meta_handle
        if handle is None:
            return False, "O robô não está indo a lugar nenhum agora."
        handle.cancel_goal_async()
        return True, "Pedido de parada enviado."

    def parar_robo(self) -> tuple:
        """O botao vermelho do menu principal, com o mesmo efeito da GUI.

        Cancela a navegacao em voo E aborta a missao, na mesma ordem que
        MainWindow faz. NAO e um E-stop: este robo nao tem botao de emergencia
        de hardware, e a mensagem devolvida nao pode prometer um.

        O teleop entra junto, e pelo caminho de sempre (teleop_parar), que
        publica ZERO e continua publicando pela janela combinada. Parar de
        publicar nao e parar: o ESC desta base le ausencia de PWM como re em
        velocidade maxima.
        """
        self.teleop_parar()
        parou_nav, _ = self.cancelar_navegacao()
        parou_missao, _ = self.abortar_missao()
        if parou_nav or parou_missao:
            return True, "Mandei o robo parar: cancelei o destino e a missao."
        return True, ("O robo ja estava parado. Nao havia destino nem missao em "
                      "andamento.")

    def levar_para_base(self) -> tuple:
        """Manda o robo encostar no dock START da arena ativa.

        Recusa com o MESMO motivo que a barra inferior mostra. Conferir de novo
        aqui nao e desconfianca da tela: a rota e HTTP, e um pedido que chega
        sem passar pela tela nao tem botao cinza nenhum para respeitar.
        """
        estado = self.cartoes()["acoes"]["base"]
        if not estado["habilitado"]:
            return False, estado["motivo"]
        return self.dockar(estado["dock_id"])

    def limpar_costmaps(self) -> tuple:
        feitos = 0
        for cliente in (self._limpar_global, self._limpar_local):
            if cliente.service_is_ready():
                cliente.call_async(ClearEntireCostmap.Request())
                feitos += 1
        if not feitos:
            return False, "A navegação não está no ar, não há memória de obstáculos para apagar."
        return True, ("Memória de obstáculos apagada. O robô volta a considerar "
                      "só o que os sensores mostram agora.")

    # ------------------------------------------------------------ localização
    def definir_pose_inicial(self, x: float, y: float, yaw: float) -> tuple:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # Mesmos números da GUI Qt: incerteza de meio metro (0,25 = 0,5²) e
        # ~15° de direção. Zerar a covariância faria o AMCL acreditar cegamente
        # num palpite feito com o dedo em cima de um mapa.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._pose_inicial.publish(msg)
        return True, ("Avisei ao robô onde ele está. Gire-o um pouco no lugar "
                      "para a localização assentar.")

    # ---------------------------------------------------------------- docking
    def dockar(self, dock_id: str) -> tuple:
        dock_id = (dock_id or "").strip()
        if not dock_id:
            return False, "Escolha em qual dock o robô deve encostar."
        if not self._dock.server_is_ready():
            return False, "O encaixe automático não está no ar (a navegação subiu com docking?)."
        meta = DockRobot.Goal()
        meta.use_dock_id = True
        meta.dock_id = dock_id
        meta.navigate_to_staging_pose = True
        meta.max_staging_time = 120.0
        futuro = self._dock.send_goal_async(meta)
        futuro.add_done_callback(
            lambda f: self._ao_resultado_simples(f, f"Encaixe em {dock_id}"))
        return True, f"O robô está indo encostar em {dock_id}."

    def desdockar(self, tipo: str) -> tuple:
        if not self._undock.server_is_ready():
            return False, "O desencaixe automático não está no ar."
        meta = UndockRobot.Goal()
        # Tipo vazio é válido: o servidor usa o dock em que ele acha que está.
        meta.dock_type = (tipo or "").strip()
        meta.max_undocking_time = 30.0
        futuro = self._undock.send_goal_async(meta)
        futuro.add_done_callback(
            lambda f: self._ao_resultado_simples(f, "Saída do dock"))
        return True, "O robô está saindo do dock."

    def _ao_resultado_simples(self, futuro, rotulo: str):
        try:
            handle = futuro.result()
        except Exception:
            handle = None
        if handle is None or not handle.accepted:
            self._avisar(f"{rotulo}: o robô recusou o pedido.", ok=False)
            return

        def concluido(f):
            try:
                resultado = f.result()
                ok = int(resultado.status) == 4 and bool(
                    getattr(resultado.result, "success", True))
            except Exception:
                ok = False
            self._avisar(
                f"{rotulo}: {'concluído' if ok else 'não deu certo'}.", ok=ok)

        handle.get_result_async().add_done_callback(concluido)

    # ----------------------------------------------------------------- missão
    def rodar_missao(self, opcoes: dict, dry_run: bool) -> tuple:
        if self._missao_cliente is None:
            return False, "Este robô não tem o pacote de missões instalado."
        if not self._missao_cliente.server_is_ready():
            return False, "O executor de missões não está no ar."
        if self._missao_ocupada:
            return False, "Já existe uma missão em andamento."

        meta = RunMission.Goal()
        meta.task_yaml = str(opcoes.get("task_yaml") or "")
        meta.map_name = str(opcoes.get("map_name") or "")
        meta.map_dir = str(opcoes.get("map_dir") or "")
        meta.dry_run = bool(dry_run)
        meta.simulate_nav = bool(opcoes.get("simulate_nav", False))
        meta.use_lidar_refine = bool(opcoes.get("use_lidar_refine", True))
        meta.finish_dock_id = str(opcoes.get("finish_dock_id") or "")
        meta.skip_startup_home = bool(opcoes.get("skip_startup_home", False))
        # actions_yaml preenchido reusa o plano JÁ MOSTRADO ao operador. Sem
        # isso, "Começar" replanejaria e poderia executar algo diferente do que
        # está na tela dele.
        meta.actions_yaml = "" if dry_run else str(opcoes.get("actions_yaml") or "")
        meta.preflight_timeout = float(opcoes.get("preflight_timeout") or 120.0)

        self._missao_ocupada = True
        if dry_run:
            with self._lock:
                self.plano = {"estado": "planejando", "linhas": [], "mensagem": ""}
        futuro = self._missao_cliente.send_goal_async(meta)
        futuro.add_done_callback(
            lambda f: self._ao_aceitar_missao(f, dry_run))
        return True, ("Montando o plano..." if dry_run else "Missão iniciada.")

    def _ao_aceitar_missao(self, futuro, dry_run: bool):
        try:
            handle = futuro.result()
        except Exception:
            handle = None
        if handle is None or not handle.accepted:
            self._missao_ocupada = False
            with self._lock:
                self._missao_handle = None
                if dry_run:
                    self.plano = {
                        "estado": "falhou", "linhas": [],
                        "mensagem": "O executor recusou o pedido de plano."}
            self._avisar("O executor de missões recusou o pedido.", ok=False)
            return
        with self._lock:
            self._missao_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._ao_terminar_missao(f, dry_run))

    def _ao_terminar_missao(self, futuro, dry_run: bool):
        try:
            resposta = futuro.result()
            resultado = resposta.result
            ok = bool(resultado.success)
            mensagem = str(resultado.message or "")
            linhas = list(resultado.plan_rows or [])
            actions_yaml = str(resultado.actions_yaml or "")
        except Exception as erro:
            ok, mensagem, linhas, actions_yaml = False, str(erro), [], ""
        self._missao_ocupada = False
        with self._lock:
            self._missao_handle = None
            if dry_run:
                self.plano = {
                    "estado": "pronto" if ok else "falhou",
                    "linhas": linhas,
                    "actions_yaml": actions_yaml,
                    "mensagem": mensagem,
                }
        if dry_run:
            self._avisar(
                f"Plano pronto: {len(linhas)} passos." if ok
                else f"Não consegui montar o plano. {mensagem}", ok=ok)
        else:
            self._avisar(
                "Missão concluída." if ok else f"Missão encerrada. {mensagem}", ok=ok)

    def abortar_missao(self) -> tuple:
        with self._lock:
            handle = self._missao_handle
        if handle is None:
            return False, "Não há missão em andamento."
        # O servidor manda SIGINT no executor, que faz o halt limpo da árvore.
        # Nunca mata o processo: matar deixaria o braço parado no ar.
        handle.cancel_goal_async()
        return True, "Pedido de parada da missão enviado."

    def plano_atual(self) -> dict:
        with self._lock:
            return dict(self.plano) if self.plano else {}

    # ----------------------------------------------------------------- teleop
    def teleop(self, vx, vy, wz) -> tuple:
        vx = _limitar(vx, LIMITE_VX)
        vy = _limitar(vy, LIMITE_VY)
        wz = _limitar(wz, LIMITE_WZ)
        agora = time.time()
        with self._lock:
            self._teleop_cmd = (vx, vy, wz)
            self._teleop_fresco_ate = agora + JANELA_COMANDO
            self._teleop_ate = agora + JANELA_TELEOP
        return True, "ok"

    def teleop_parar(self) -> tuple:
        """Freia AGORA e mantém a publicação de zeros pela janela combinada.

        Não é o mesmo que só parar de mandar comando: parar de publicar é o que
        o ESC lê como ré em velocidade máxima. Aqui o zero é explícito.
        """
        agora = time.time()
        with self._lock:
            self._teleop_cmd = (0.0, 0.0, 0.0)
            self._teleop_fresco_ate = 0.0
            self._teleop_ate = agora + JANELA_TELEOP
        return True, "Robô parado."

    def _laco_teleop(self):
        while not self._teleop_parar_evento.is_set():
            try:
                self._pulso_teleop()
            except Exception:
                # Publicar velocidade nunca pode derrubar a thread do freio: se
                # ela morresse, o robo pararia de receber zeros exatamente na
                # situacao em que mais precisa deles.
                pass
            self._teleop_parar_evento.wait(PULSO_TELEOP)

    def parar_teleop(self):
        self._teleop_parar_evento.set()
        self._teleop_thread.join(timeout=1.0)

    def _pulso_teleop(self):
        agora = time.time()
        with self._lock:
            if agora > self._teleop_ate:
                # Fora de qualquer sessão de teleop: silêncio total, para o
                # twist_mux devolver a base para a navegação.
                return
            fresco = agora <= self._teleop_fresco_ate
            vx, vy, wz = self._teleop_cmd
        msg = Twist()
        if fresco:
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
        # Fora da janela de frescor, o Twist sai zerado: é o watchdog. Se a rede
        # engasgar ou a aba fechar no meio de um movimento, o robô para em
        # 300 ms em vez de seguir com o último comando.
        self._cmd_vel.publish(msg)


def _para_jpeg(img) -> tuple:
    """Reduz e comprime. Devolve (bytes, largura, altura, erro)."""
    if img.width > LARGURA_CAMERA:
        altura = max(1, round(img.height * LARGURA_CAMERA / img.width))
        img = img.resize((LARGURA_CAMERA, altura))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=QUALIDADE_CAMERA)
    return buf.getvalue(), img.width, img.height, ""


# Formatos que o painel desenha. Sao os mesmos do SensorBridge da GUI; a segunda
# string e como o PIL le os bytes crus (BGR sem esta conversao sai com a pessoa
# de rosto azul).
_FORMATOS_DE_IMAGEM = {
    "rgb8": ("RGB", "RGB"),
    "bgr8": ("RGB", "BGR"),
    "rgba8": ("RGBA", "RGBA"),
    "bgra8": ("RGBA", "BGRA"),
    "mono8": ("L", "L"),
}


def _jpeg_de_imagem(msg) -> tuple:
    par = _FORMATOS_DE_IMAGEM.get(msg.encoding)
    if par is None:
        return (None, 0, 0,
                f"Esta camera envia a imagem num formato que o painel nao "
                f"desenha ({msg.encoding}).")
    try:
        from PIL import Image as ImagemPil
        modo, cru = par
        # step (bytes por linha) entra no decodificador: sem ele, uma imagem com
        # linha alinhada sai enviesada, como TV fora de sintonia.
        img = ImagemPil.frombuffer(
            modo, (int(msg.width), int(msg.height)), bytes(msg.data),
            "raw", cru, int(msg.step), 1)
        return _para_jpeg(img)
    except Exception:
        return None, 0, 0, "Chegou imagem que o painel nao conseguiu abrir."


def subir_no_em_thread() -> tuple:
    """Inicia o rclpy e gira o nó numa thread. Devolve (no, parar)."""
    if not rclpy.ok():
        rclpy.init()
    no = NoDoPainel()
    executor = SingleThreadedExecutor()
    executor.add_node(no)

    parar_evento = threading.Event()

    def girar():
        while rclpy.ok() and not parar_evento.is_set():
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=girar, daemon=True)
    thread.start()

    def parar():
        no.parar_teleop()
        parar_evento.set()
        thread.join(timeout=2.0)
        executor.remove_node(no)
        no.destroy_node()

    return no, parar
