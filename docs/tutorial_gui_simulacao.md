# Tutorial — Operar o Caramelo simulado pela GUI (mapa novo → dock → navegar)

Fluxo completo usando a **GUI** com o robô **simulado** (`caramelo_simulation_ws`).
Use terminais normais (fora do VSCode, que contamina o ambiente com libs do snap).

---

## 1. Ligar o robô simulado (Terminal 1)

```bash
source ~/caramelo_simulation_ws/install/setup.bash
ros2 launch caramelo_simulation sim_bringup.launch.py world:=warehouse use_rviz:=false
```

Espere o Gazebo abrir com o robô no armazém. Isso é o "robô de verdade":
controllers, LiDAR, IMU, EKF — o mesmo contrato da Raspberry.

## 2. Ligar a GUI (Terminal 2)

```bash
source ~/caramelo_ws/install/setup.bash
source ~/caramelo_simulation_ws/install/setup.bash   # POR ULTIMO (malhas do sim)
ros2 launch caramelo_gui caramelo_gui.launch.py
```

> A ordem do source importa: o do sim por último faz as malhas 3D do robô
> carregarem no mapa da GUI.

O que você deve ver:
- **Mapa central sempre visível** com a grade e o **robô** (sem mapa ainda, o
  frame cai automaticamente para `odom` — o robô aparece mesmo assim).
- **Início**: cartões de saúde. Na simulação o cartão **IMU fica vermelho**
  (a sim publica `/imu`, o robô real `/imu/data_raw`) — é esperado, ignore.
- **Bateria: —** (nada publica bateria ainda).
- Botão **Logs** (barra inferior) abre os logs dos nós, escondido por padrão.

## 3. Criar um mapa NOVO pela GUI

1. Sidebar → **Mapeamento**: nome do mapa (ex.: `deposito`), marque
   **"Relogio da simulacao (Gazebo)"** e clique **Iniciar SLAM**.
   O mapa começa a aparecer no painel central (camada *Mapa*).
2. Sidebar → **Teleop** → **Ligar teleop**. Abre uma janelinha de teclado
   (xterm) — dirija com ela (ou joystick). Ande devagar cobrindo o ambiente e
   feche os contornos; veja o mapa crescer em tempo real.
3. Quando o mapa estiver bom: **Mapeamento → Salvar mapa** (cria
   `maps/deposito/map.pgm|yaml`) e depois **Parar SLAM**.
4. *(Opcional)* Sidebar → **Editor de Mapa**: Mapa=`deposito` → **Carregar** →
   pincel **Borracha** nos ruídos → **Salvar (com backup)**.
5. *(Opcional)* Desligue o Teleop (para não disputar comando com o Nav2).

## 4. Ligar a navegação com o novo mapa

Sidebar → **Navegacao**:
- Mapa: `deposito`
- Marque **"Com docking server"** e **"Relogio da simulacao (Gazebo)"**
- **Ligar navegacao (Nav2)**

> O SLAM precisa estar PARADO (SLAM e AMCL disputam o TF do mapa — o launch
> aborta sozinho se tentar os dois; o erro aparece nos Logs).

Aguarde os cartões **TF** e **Nav2** do Início ficarem verdes.

## 5. Localizar o robô (o "arrastar o fantasma")

No painel **Mapa & Camadas** (seção *Localizacao*):
1. **Posicionar robo** — aparece o retângulo laranja (fantasma) e a ferramenta
   *Interagir* já fica ativa.
2. **Arraste o fantasma pelo corpo** até onde o robô está de verdade no mapa e
   **gire pelo anel**. O **scan verde acompanha em tempo real** — case os
   pontos verdes com as paredes do mapa.
3. **Confirmar pose** → o AMCL relocaliza (nuvem colapsa no robô).

## 6. Criar a docking station

1. Leve o robô até a estação: **Teleop ligado**, pare o robô na **pose final**
   (parado de frente para a mesa, na posição exata de trabalho). Desligue o
   teleop.
2. Sidebar → **Service Areas**: Mapa=`deposito`, Nome=`WS1`,
   Tipo=`workstation` → **Salvar pose atual** → **Validar (+sync docking)**.
   - Isso grava a área E sincroniza o `docking.yaml` (o dock `WS1` nasce com a
     pose e o offset de aproximação derivados).
   - A estação aparece desenhada no mapa (camada *Service Areas*).

## 7. Navegar

**Meta manual:** painel **Mapa & Camadas** → **Definir Goal** → clique no mapa
e arraste para dar a direção. Status/cancelar/limpar costmaps no módulo
**Navegacao**.

**Waypoints:** sidebar → **Waypoints** → `WP1` → **Novo (pose do robo)** →
arraste a esfera azul pelo mapa (ferramenta *Interagir*) → **Salvar** →
**Ir ate o selecionado** ou **Seguir todos (em ordem)**.

**Docking:** sidebar → **Docking**: Dock=`WS1`, Mapa=`deposito` → **Dockar**
(o robô navega até a pose de aproximação e faz o approach). Para o
**Alinhamento fino (holonômico)**, rode antes num terminal:
```bash
source ~/caramelo_ws/install/setup.bash
ros2 run caramelo_navigation dock_align_node --ros-args -p use_sim_time:=true
```
Depois **Undock** para sair da estação.

---

## Problemas comuns

| Sintoma | Causa/solução |
|---|---|
| Robô não aparece no mapa | Ordem do source errada (sim por último) ou sim parada |
| Mapa não cresce no SLAM | Esqueceu **Relogio da simulacao** marcado |
| "SLAM e AMCL juntos" nos logs | Pare um antes de ligar o outro (trava RK-02) |
| Fantasma não arrasta | Clique em **Posicionar robo** de novo (ativa *Interagir*) e arraste pelo corpo laranja |
| Goal não faz nada | Navegação desligada, ou robô não localizado (passo 5) |
| Cartão IMU vermelho na sim | Esperado (`/imu` vs `/imu/data_raw`) |
