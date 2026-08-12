# Preparar um notebook novo para operar o Caramelo

Passo a passo para colocar um segundo PC operando o robô. Pressupõe **Ubuntu 24.04 com
ROS 2 Jazzy desktop já instalado** e cobre o que falta além disso.

A informação aqui estava espalhada entre `caramelo_hardware_ws/docs/relogio_chrony.md`,
`docs/guia_missoes.md`, `docs/AUDITORIA.md` e a memória do projeto — este documento
consolida e resolve os conflitos entre eles.

---

## Passo 1 — Ver o que já existe

Cole o bloco inteiro no notebook novo. Ele **não altera nada**, só lista o que falta.

```bash
echo "=== ROS ==="; echo "DOMAIN=$ROS_DOMAIN_ID RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE RMW=${RMW_IMPLEMENTATION:-<default ok>}"

echo "=== pacotes ROS criticos ==="
for p in navigation2 slam-toolbox robot-localization moveit-ros-move-group \
         moveit-task-constructor-core pick-ik warehouse-ros-sqlite behaviortree-cpp-v3 \
         ros2-control ros2-controllers realsense2-camera apriltag-ros dynamixel-sdk \
         twist-mux joy-teleop tf-transformations xacro; do
  dpkg -s ros-jazzy-$p >/dev/null 2>&1 || echo "  FALTA ros-jazzy-$p"
done

echo "=== pacotes de sistema ==="
for p in chrony sshpass ffmpeg espeak-ng alsa-utils v4l-utils libyaml-cpp-dev \
         python3-colcon-common-extensions python3-rosdep avahi-daemon; do
  dpkg -s $p >/dev/null 2>&1 || echo "  FALTA $p"
done

echo "=== piper (voz) ==="; [ -x ~/piper/piper ] && echo "  ok" || echo "  FALTA ~/piper/piper"

echo "=== workspaces ==="
for w in ros2_ws caramelo_hardware_ws caramelo_ws; do
  [ -d ~/$w/src ] && echo "  $w: branch $(git -C ~/$w rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')" \
                  || echo "  FALTA ~/$w"
done

echo "=== grupos ==="; id -nG | tr ' ' '\n' | grep -E "^(dialout|plugdev)$" || echo "  FALTA dialout/plugdev"
echo "=== chrony ==="; systemctl is-active chrony 2>/dev/null; ls /etc/chrony/conf.d/ 2>/dev/null
echo "=== ssh p/ Pi ==="; grep -q ControlMaster ~/.ssh/config 2>/dev/null && echo "  ok" || echo "  FALTA ControlMaster no ~/.ssh/config"
```

Os passos seguintes só precisam ser executados para o que aparecer como `FALTA`.

---

## Passo 2 — Pacotes ROS que entraram depois do setup inicial

Estes vieram com a integração da manipulação e costumam faltar em máquinas preparadas
antes disso:

```bash
sudo apt install -y \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-msgs \
  ros-jazzy-moveit-task-constructor-capabilities \
  ros-jazzy-pick-ik \
  ros-jazzy-warehouse-ros-sqlite \
  ros-jazzy-behaviortree-cpp-v3 \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  ros-jazzy-apriltag-ros
```

> `realsense2_camera` e `apriltag_ros` **não estão declarados em nenhum `package.xml`** do
> projeto — o `rosdep` não os instala. Sem eles a câmera e a detecção de tags não sobem, e
> o erro aparece só na hora do launch.

---

## Passo 3 — Pacotes de sistema

```bash
sudo apt install -y chrony sshpass ffmpeg espeak-ng alsa-utils v4l-utils \
                    libyaml-cpp-dev avahi-daemon
```

Para que serve cada um que não é óbvio: o **`ffplay`** (do `ffmpeg`) é o player que aplica
o efeito na voz do robô; **`espeak-ng`** é o TTS de reserva quando o Piper falha; e
**`avahi-daemon`** resolve `raspberrypi.local`, de onde o RViz baixa as malhas 3D do robô
por HTTP.

---

## Passo 4 — Piper (voz), instalação manual

```bash
cd ~
wget https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_amd64.tar.gz
tar -xzf piper_amd64.tar.gz      # cria ~/piper/piper
```

> **Não instale `piper` pelo apt** — esse pacote é um utilitário de mouse gamer, sem
> relação com TTS.

O modelo de voz `pt_BR-jeff` **já vem versionado** no clone do `caramelo_ws`
(`src/manipulation/manip_audio/models/pt_BR-jeff/`), não precisa baixar nada além disso.
O `speech_node` procura o executável em `~/piper/piper` quando o parâmetro é `auto`, que é
o padrão dos launches.

---

## Passo 5 — Grupos do usuário

```bash
sudo usermod -aG dialout,plugdev $USER
```

É preciso **fazer logout e login** para valer.

---

## Passo 6 — Clonar os workspaces

```bash
cd ~ && git clone -b dev                 https://github.com/RoboFEIatWork/caramelo_hardware_ws.git
cd ~ && git clone -b robot_manipulation  https://github.com/RoboFEIatWork/caramelo_ws.git
```

**O `~/ros2_ws` NÃO é necessário no PC** (validado em 10/08 no segundo notebook). Ele
contém `apriltag_ros` — que você instalou por apt no passo 2 — mais `rplidar_ros`,
`sllidar_ros2` e `wit_ros2_imu`, que são drivers de LiDAR e IMU e **rodam na Raspberry**.
Se o `.bashrc` da máquina tiver uma linha carregando esse workspace, ela vai dar
`No such file or directory` a cada terminal novo; comente-a:

```bash
grep -n "ros2_ws" ~/.bashrc
sed -i '/ros2_ws\/install\/setup\.bash/s/^/# /' ~/.bashrc
```

Só clone o `ros2_ws` se a detecção de AprilTag se comportar diferente da deste notebook
(aqui o nó vem do fonte, lá virá do pacote apt):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b master https://github.com/christianrauch/apriltag_ros.git
git clone -b ros2   https://github.com/Slamtec/rplidar_ros.git
git clone -b main   https://github.com/Slamtec/sllidar_ros2.git
git clone -b main   https://github.com/vmayres/wit_ros2_imu.git
```

**Não clone o `manip_ws`.** Ele foi absorvido pelo `caramelo_ws` em julho e está parado
desde então; instalá-lo cria pacotes duplicados com os de `caramelo_ws/src/manipulation/`.

**Por que o `caramelo_hardware_ws` também vai no PC**, mesmo sendo "da Raspberry": é ele
que fornece o pacote `caramelo_description`, com o URDF do robô. Sem ele o `move_group`
não sobe.

---

## Passo 7 — Compilar, nesta ordem

Sempre de um **terminal novo**, com o `.bashrc` padrão — nunca de um terminal onde você
tenha feito `source` manual de outros workspaces.

```bash
cd ~/ros2_ws              && colcon build --symlink-install
cd ~/caramelo_hardware_ws && colcon build --symlink-install

cd ~/caramelo_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys mtc_tutorial
colcon build --symlink-install
```

O `--skip-keys mtc_tutorial` é necessário: o `raspberry_bringup` declara dependência de um
pacote que não existe em lugar nenhum, e sem o skip o `rosdep` aborta.

---

## Passo 8 — `~/.bashrc`

```bash
source /opt/ros/jazzy/setup.bash

# ATENCAO: local_setup.bash, NUNCA setup.bash. O setup.bash grava a cadeia de
# underlays da hora do build e envenena a ordem para sempre.
# Ordem obrigatoria: ros2_ws -> caramelo_hardware_ws -> caramelo_ws (o ultimo vence).
source ~/ros2_ws/install/local_setup.bash
[ -f ~/caramelo_hardware_ws/install/local_setup.bash ] && source ~/caramelo_hardware_ws/install/local_setup.bash

export ROS_DOMAIN_ID=67
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

[ -f "$HOME/caramelo_ws/install/local_setup.bash" ] && source "$HOME/caramelo_ws/install/local_setup.bash"
```

**Por que a ordem importa tanto:** existe um pacote chamado `caramelo_localization` nos
**dois** workspaces, com conteúdos diferentes — no `caramelo_ws` ele traz o AMCL e o
keepout, no `caramelo_hardware_ws` traz o EKF. Quem for carregado por último vence. Com a
ordem invertida, o AMCL e o keepout somem **sem erro nenhum**.

**Não defina `RMW_IMPLEMENTATION`.** O projeto roda no Fast DDS padrão, e qualquer
divergência de RMW ou de `ROS_DOMAIN_ID` entre PC e Raspberry quebra a descoberta em
silêncio.

Depois de editar, **feche o terminal e abra outro** — `exec bash` não limpa as variáveis
antigas.

---

## Passo 9 — Rede compartilhada (o PC vira 10.42.0.1)

O `10.42.0.1` não é IP fixo: é o gateway que o NetworkManager cria no modo *shared*.

Pela interface gráfica: **Configurações → Rede → (a conexão) → IPv4 → Método:
"Compartilhada com outros computadores"**.

Neste notebook existem as duas formas, e ambas funcionam:

| Forma | Interface | Observação |
|---|---|---|
| Cabo Ethernet | `enp3s0` | é a usada no dia a dia |
| Hotspot WiFi "Ayres" | `wlo1` | modo AP, WPA-PSK |

Enquanto o PC for `10.42.0.1`, a Raspberry se conecta e sincroniza sem nenhuma alteração.

---

## Passo 10 — Chrony (relógio)

A Raspberry não tem RTC: acorda com a hora errada e deriva mais de 1 s por hora. Com o
relógio fora, o AMCL descarta os scans em silêncio e o robô planeja mas não anda.

### No notebook (servidor de tempo)

```bash
sudo apt install -y chrony
printf 'allow 10.42.0.0/24\nlocal stratum 10\n' | \
  sudo tee /etc/chrony/conf.d/caramelo-ntp-server.conf
sudo systemctl enable --now chrony
```

O `local stratum 10` é o que faz o PC continuar servindo a hora **mesmo sem internet** —
que é exatamente o cenário da competição.

Conferir: `chronyc tracking` e `sudo ss -ulpn | grep :123`.

### Na Raspberry

**Nada a fazer**, desde que o notebook novo também seja `10.42.0.1`. A Pi já está
configurada apontando para esse endereço.

Só se o IP for diferente:

```bash
# NA PI, apenas se o PC novo NAO for 10.42.0.1
sudo tee /etc/chrony/conf.d/caramelo-notebook.conf > /dev/null <<'EOF'
server <IP-DO-PC-NOVO> iburst prefer minpoll 4 maxpoll 6
EOF
sudo systemctl restart chrony
```

Para referência (já aplicado na Pi): os `pool` estão comentados no `chrony.conf` e o
`makestep 1.0 -1` fica no **arquivo principal**, não num drop-in — o `confdir` é lido
antes, e o `makestep 1 3` padrão sobrescreveria.

---

## Passo 11 — SSH para a Raspberry

```bash
ssh-keygen -t ed25519          # so se ainda nao houver chave
sshpass -p raspberrypi ssh-copy-id -o StrictHostKeyChecking=accept-new raspberrypi@raspberrypi.local
ssh raspberrypi@raspberrypi.local 'echo ok'    # nao pode pedir senha
```

E crie `~/.ssh/config` **com multiplexação** — sem ela o sshd da Pi satura quando várias
conexões abrem em rajada, e novas conexões ficam minutos sem responder:

```
Host raspberrypi.local
  HostName raspberrypi.local
  User raspberrypi
  ControlMaster auto
  ControlPath ~/.ssh/cm-%r@%h:%p
  ControlPersist 10m
```

---

## Passo 12 — Copiar o `limpar_stack.sh`

O script `caramelo_ws/scripts/limpar_stack.sh` está **fora do git** — não vem no clone.
Copie-o à mão do notebook atual e dê `chmod +x`. Ele mata processos ROS órfãos e evita a
classe de falha mais cara do projeto.

---

## Passo 13 — Regra para os dois notebooks juntos

Ambos ficam em `ROS_DOMAIN_ID=67`, então **só um pode estar ligado por vez**.

Dois PCs no mesmo domínio criam dois `move_group`, dois nós de pick, dois de tudo — e o
sintoma não é erro: é o robô se comportando de forma inexplicável, com comandos aceitos,
hardware obedecendo e o software reportando falha.

Antes de operar, no PC que vai comandar:

```bash
./scripts/limpar_stack.sh
ros2 node list | sort | uniq -c | awk '$1>1'    # tem que sair VAZIO
```

Se aparecer qualquer nó duplicado, o outro notebook ainda está no ar — desligue o ROS dele
ou tire-o da rede do robô antes de continuar.

---

## Passo 14 — Validar

1. **Ambiente**
   `echo "DOMAIN=$ROS_DOMAIN_ID RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE RMW=$RMW_IMPLEMENTATION"`
   → `67`, `SUBNET`, RMW vazio.
2. **Relógio** — na Pi: `chronyc sources -v` mostra `10.42.0.1` com `^*`, e
   `chronyc tracking` traz `System time` abaixo de 10 ms.
3. **Descoberta** — com o bringup da Pi no ar: `ros2 topic hz /joint_states` em torno de
   100 Hz, e `ros2 node list | sort | uniq -c | awk '$1>1'` vazio.
4. **Modelo do robô**
   `ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=arena3_520 use_rviz:=true`
   → o RViz abre, o mapa aparece, e o log do `move_group` diz
   `Loading robot model 'caramelo'`. Se disser `'manip'`, subiu o launch errado do MoveIt.
5. **Voz** — `ros2 topic pub --once /manip/speech std_msgs/msg/String "{data: teste}"`.
6. **Câmera** — a D455 precisa de porta **USB 3**: `lsusb -t` tem que mostrar `5000M`. Em
   USB 2 ela só funciona a 640x480x5 e derruba frames.
7. **Missão** —
   `ros2 run caramelo_navigation run_mission --map-name arena3_520 --task arena_test.yaml`.

---

## Armadilhas conhecidas

**RealSense duplicada.** No notebook atual convivem o `librealsense2` do repositório da
Intel (2.56.5, lista apontando para `jammy`, hoje desabilitada) e o
`ros-jazzy-realsense2-camera` (4.57.7). No notebook novo, fique **só com o pacote ROS** —
as duas versões juntas são fonte de conflito.

**Dois launches quebram sem o `manip_ws`.** `manip_bringup.launch.xml` e
`manip_pc_test.launch.xml` apontam para `$(env HOME)/manip_ws/src/apriltag_ros/cfg/tags_36h11.yaml`.
O arquivo correto já existe em `manip_bringup/config/tags_36h11.yaml` — que é o que o
`manip_stack.launch.py` usa. Corrija os dois ou simplesmente não os use.

**RViz não abre de terminal do VS Code.** O snap do VS Code injeta uma `libpthread`
incompatível e o RViz morre com `symbol lookup error`. Use sempre um terminal normal do
sistema.

**Ordem de ligação.** Suba sempre a **Raspberry primeiro e o PC depois**. O `move_group`
não reconecta sozinho aos controladores quando o bringup da Pi reinicia — ele fica
aceitando comandos que nunca chegam ao braço.

**Desligar a Pi.** Sempre `sudo poweroff` antes de cortar a energia; corte a frio já
corrompeu o cartão SD duas vezes.
