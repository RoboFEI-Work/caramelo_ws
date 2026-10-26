# Interface gráfica (GUI Qt) e painel web do Caramelo

**Estado (2026-10-26):** a branch `robot_ui` (GUI reestruturada em menu + painel web) foi
mesclada na branch de trabalho e adaptada às interfaces atuais do robô (missão via
`run_mission`, tasks 2026 em `missions/`, voz, câmera anotada, docking, service areas).
Compila e sobe em modo isolado (`ROS_DOMAIN_ID=99`, sem robô). **Ainda não foi validada
com o robô ligado** — a lista do que conferir está na §7.

Sumário: 1 Visão geral · 2 Como subir · 3 Telas da GUI · 4 Abas do painel web ·
5 Missão pela interface (como funciona) · 6 Interfaces ROS usadas · 7 Pendente de validar ·
8 O que veio da `robot_ui` e o que ficou de fora.

---

## 1. Visão geral

Dois **clientes** do mesmo grafo ROS 2, com o mesmo contrato:

| | `caramelo_gui` | `caramelo_web` |
|---|---|---|
| O que é | App Qt5/C++ com RViz embutido, roda no PC de operação | FastAPI + página estática; abre em qualquer navegador da rede |
| Para quem | operador ao lado do robô (touchscreen/monitor do PC) | engenharia à distância: terminal, espelho de tela, sensores |
| Código | `src/caramelo_gui/` | `src/caramelo_web/` |

Nenhum dos dois **substitui** o terminal nem sobe o stack do robô: navegação + manipulação
continuam subindo por `ros2 launch caramelo_bringup robot_manipulation.launch.py
map_name:=<mapa>` (README §1). As interfaces **consomem** esse stack e sobem só o que é
delas: o `health_monitor` (`/diagnostics`) e o `mission_server` (`/caramelo/run_mission`).

> **Não ligue o Nav2 pela GUI** com o `robot_manipulation.launch.py` no ar. O botão
> "Ligar navegação" (Operação → Navegar) detecta o `bt_navigator` e se desliga sozinho,
> mas a regra continua: com o launch de missão no ar, a GUI é monitor e ferramenta de
> comissionamento.

## 2. Como subir

```bash
# 0. como sempre: Pi com hardware_bringup; PC com ./scripts/limpar_stack.sh
# 1. stack do robô (navegação + manipulação)
ros2 launch caramelo_bringup robot_manipulation.launch.py map_name:=<mapa>

# 2. GUI (sobe junto o health_monitor e o mission_server)
ros2 launch caramelo_gui caramelo_gui.launch.py

# 3. painel web (opcional; imprime o link com a chave de acesso)
ros2 launch caramelo_web web_panel.launch.py
```

| Launch | Arg | Default | Efeito |
|---|---|---|---|
| `caramelo_gui.launch.py` | `use_diagnostics` | `true` | sobe `caramelo_diagnostics/diagnostics.launch.py` (`/diagnostics`) |
| | `use_mission_server` | `true` | sobe `caramelo_navigation/mission_server.launch.py` |
| | `use_sim_time` | `false` | obrigatório `true` em simulação (relógio único) |
| `web_panel.launch.py` | `porta`, `host` | `8080`, `0.0.0.0` | onde o painel escuta |
| | `token` | sorteado | fixa a chave de acesso (favorito/QR) |
| | `use_mission_server` | `true` | idem GUI |
| | `use_camera` | `false` | `web_video_server` na 8081 (não é necessário: a câmera anotada já sai pelo próprio painel) |
| | `use_rosbridge` | `false` | rosbridge na 9090 — deixe desligado (expõe `/mecanum_controller/reference` sem trava) |

**Pré-requisitos do painel web** (fora do rosdep, instalar no usuário, sem sudo):
`python3 -m pip install --user --break-system-packages fastapi uvicorn` (servidor) e
`... mss` (espelho de tela, opcional). Sem o `fastapi` o `caramelo_web_server` nem sobe —
foi o caso deste PC em 2026-10-26 (§7, item 9).

**Um `mission_server` por robô.** GUI e painel juntos não duplicam: o nó confere o grafo ao
subir e sai se já houver outro `/caramelo/run_mission` (`single_instance:=true`). Para subir
só o servidor: `ros2 launch caramelo_navigation mission_server.launch.py`.

**Testar sem o robô** (o robô fica em `ROS_DOMAIN_ID=67`; nunca use esse domínio em teste):

```bash
export ROS_DOMAIN_ID=99
ros2 launch caramelo_gui caramelo_gui.launch.py           # abre com tudo "SEM DADOS"
ros2 launch caramelo_web web_panel.launch.py              # ou: ros2 run caramelo_web caramelo_web_server --sem-ros
```

Build/testes (worktree limpa, sem `build/`/`install/` prévios):

```bash
colcon build --symlink-install --packages-up-to caramelo_gui caramelo_web caramelo_navigation
colcon test --packages-select caramelo_gui caramelo_web caramelo_navigation && colcon test-result --verbose
python3 -m pytest src/caramelo_web/test src/caramelo_navigation/test -q
```

## 3. Telas da GUI

A GUI abre num **menu inicial** (5 cartões com estado e motivo, coluna do robô com arena
e estado, barra inferior com *Parar o robô* — cancela a navegação e aborta a missão, **não
é E-stop** —, *Levar para a base* e *Diagnóstico*). Cada cartão abre um **contexto** com
seções em abas; o mapa (RViz embutido) só aparece nos contextos que precisam dele.

| Contexto | Seção | O que faz hoje | Por baixo |
|---|---|---|---|
| **Operação** (com mapa) | Mapa | ferramentas *Definir destino* (clique+arraste), *Mover câmera*, *Interagir*; liga/desliga camadas do RViz | `navigate_to_pose` (tópico privado `/caramelo_gui/goal_pose`, nunca `/goal_pose`) |
| | Navegar | liga/desliga o Nav2 **quando ele ainda não está no ar** (detecta `bt_navigator`), cancela meta, limpa costmaps, mostra o status | `caramelo_navigation/bringup.launch.py` por `QProcess`; `clear_entirely_*_costmap` |
| | Waypoints | criar na pose do robô, mover por interactive marker, salvar por mapa, ir até um, seguir todos | `maps/<mapa>/waypoints.yaml`, `follow_waypoints` |
| | Docking | *Dockar*, *Alinhamento fino (holonômico)*, *Sair da estação*, *Regravar a pose desta estação*; só oferece estações com pose gravada | `/dock_robot`, `align_to_dock` (`use_lidar_refine` default ligado), `/undock_robot` (tipo da arena), `/save_dock_pose` → `dock_pose_recorder_node` |
| | Localização | robô-fantasma arrastável com o scan reprojetado; *Confirmar* publica a pose e a grava para o próximo boot | `/initialpose`; `~/.config/caramelo/robot_state.yaml` (`initial_pose`) |
| | Teleop | liga/desliga joystick + teclado (xterm) + twist_mux | `caramelo_utils/teleop.launch.py` |
| **Competição** (sem mapa) | Missão | 1 task + mapa → 2 *Ver plano* (só o `task_planner`, nada se move) → 3 *Verificar servidores* (lista do pré-flight do `run_mission`, incluindo `/nudge_base`) → 4 *EXECUTAR* / *ABORTAR*; estado, tempo, última linha do executor e **voz do robô** | action `/caramelo/run_mission` + `/caramelo/mission/status`; `/manip/speech_status` (§5) |
| **Mapas** (sem mapa ao vivo) | Arenas | lista das arenas (`maps/*/map.yaml`), preview com docks, service areas, paredes virtuais e waypoints por cima, avisos do que falta; *Usar esta arena* (carrega no `map_server` e grava a arena ativa); criar ponto clicando, arrastar ponto, remover; limpar ruído do PGM; pintar paredes virtuais; renomear/duplicar/excluir (lixeira) | `/map_server/load_map`; `save_service_area_pose --pose`, `save_dock_pose --pose`; `init_keepout_mask`; backups em `<arena>/.backup/` |
| **Mapeamento** (com mapa) | Mapeamento | fluxo em 4 passos: **1 Criar o mapa** (SLAM + teleop, salvar com nome, prepara START/FINISH e a máscara de keepout) → **2 Marcar pontos** (estação/dock/waypoint: pose atual do robô **ou** clique no mapa) → **3 Paredes virtuais** → **4 Conferir** | `slam.launch.py use_rviz:=false`, `map_saver_server`, `init_service_areas --areas START FINISH --sync-docking`, `init_keepout_mask`, `save_service_area_pose`, `save_dock_pose` |
| **Modo Avançado** (com mapa) | Estado do robô | um cartão por componente do `health_monitor` (rede/Pi, controllers, LiDAR, IMU, odometria, TF, Nav2, bateria); *Modo Engenharia* mostra causa/ação/frequência | `/diagnostics` |
| | Sensores | LiDAR polar (com o setor cego real), IMU em graus, câmera — **`/manip/vision_debug/image` (câmera do punho anotada) vem selecionada quando existe**; nada é assinado fora desta tela | `/scan`, `/imu/data_raw` ou `/imu`, `sensor_msgs/Image` |
| | Service areas | salvar a pose atual como área (nome validado pelo rulebook), listar, *Validar (+sync docking)* | `service_area_manager_node`: `/caramelo/service_areas/{list,validate,save_pose}` |

Faixa superior: `< Menu`, título do contexto, **Arena: <nome>** e **estado** da máquina de
estados da GUI sempre visíveis; botão *Logs* abre o `/rosout`.

**Arena ativa** (`~/.config/caramelo/robot_state.yaml`): memória **da interface** (GUI e
painel leem e gravam). O stack do robô **não** lê esse arquivo — o mapa continua sendo
`map_name:=` do launch. Escolher a arena na tela Mapas serve para as outras telas
concordarem sobre qual arena está em uso (docking, missão, mapeamento).

## 4. Abas do painel web

Mesmo menu da GUI (`ros_node.py: MENU_PRINCIPAL`), servido ao navegador no primeiro quadro
do WebSocket. Link com a chave impresso pelo servidor ao subir; sem a chave o painel
**só olha** (mapa, pose, sensores, saúde, missão); com a chave comanda, abre terminal e
espelha a tela.

| Aba | O que faz hoje |
|---|---|
| **Operação** | mapa da arena com robô, LiDAR, estações e pontos; clique = meta (arraste = direção), modo "o robô está aqui" (`/initialpose`); parar, limpar costmaps, dockar/sair de uma estação; teleop com watchdog (`/cmd_vel_key`, nunca `/mecanum_controller/reference`) |
| **Competição** | lista as tasks de `missions/` e `missions/tasks geradas/` (formato 2026, oficiais primeiro), opções (braço em home, refino LiDAR, FINISH, ensaio sem Nav2), pendências com atalho, *Montar o plano* (dry-run), *Começar*, *Abortar*; estado, tempo, mensagem do executor e **voz do robô** |
| **Mapas** | escolher a arena ativa, ver docks/áreas/paredes/waypoints, gravar/apagar pontos (importa os módulos `caramelo_service_area_common`/`caramelo_docking_common`, mesma gravação dos scripts) |
| **Mapeamento** | ligar/desligar o SLAM, salvar o mapa numa arena nova, limpar ruído, pintar paredes virtuais |
| **Modo Avançado** | estado por componente, sensores ao vivo (LiDAR, IMU, bateria), **câmera sob demanda** (`/manip/vision_debug/image` primeiro na lista), uso de CPU/memória, **terminal** (bash real do PC do robô) e **espelho da tela** (MJPEG) |

Rotas: `src/caramelo_web/caramelo_web/server.py` (cabeçalho). Conferir sem robô:
`ros2 run caramelo_web caramelo_web_server --port 8080 --token teste --sem-ros`.

## 5. Missão pela interface (como funciona)

```
GUI / painel ──action /caramelo/run_mission──▶ mission_server ──import──▶ run_mission.py
                                                     │  task_planner → actions.yaml
                                                     │  _preflight (ros2 action list)
                                                     │  bt_yaml_executor (subprocess)
                                     ◀── /caramelo/mission/status (MissionStatus) ──┘
```

- `caramelo_navigation/scripts/mission_server.py` **importa** o `run_mission.py` (instalado
  também como módulo) e chama as funções dele: `_resolve_task_yaml`, `resolve_map_folder`,
  `_validate_map_folder`, `_resolve_apriltag_cfg`, `_generate_actions`, `_print_plan`,
  `_preflight`, `_executor_cmd`. Nada da missão é reimplementado: o que roda pela tela é o
  que roda no terminal.
- **Ver plano / Montar o plano** (`dry_run`): só o `task_planner`; devolve as linhas do
  plano (`result.plan_rows`) e o caminho do `actions.yaml`, que **Executar** reusa (o que o
  operador leu é o que roda). O `--dry-run` do `bt_yaml_executor` **não** é usado: ele monta
  a árvore e espera o `move_group`, o que travaria com o stack desligado.
- **Executar**: `PLANNING` → `PREFLIGHT` (mesma lista e timeout do `run_mission`; a espera não
  é interrompível — um cancelamento vale quando ela termina) → `RUNNING` (`bt_yaml_executor`
  como subprocesso; **cada linha do log dele vira `message`** do status, ecoada no terminal
  do servidor) → `DONE` (exit 0) / `ABORTED` (exit 130 ou cancel) / `FAILED` (outro código).
- **Abortar** manda `SIGINT` ao executor (halt limpo da árvore, cancels em voo); nunca
  `SIGKILL`.
- Opções da tela → executor: *Simular navegação* = `simulate_navigation:=true`;
  *Refinar docking com LiDAR* desmarcado = `use_lidar_refine:=false` (o `run_mission` só
  sabe ligar); *Ir ao FINISH* desmarcado = `finish_dock_id:=''` (aspas literais — a string
  vazia crua derruba o executor); *Braço em home* desmarcado = `startup_home:=false`.
- **Sem progresso por passo.** O `bt_yaml_executor` atual não publica `MissionStatus`
  (a versão da `robot_ui` publicava `action_index/total` por nó da árvore — não foi
  mesclada). A GUI mostra a barra em "ocupado" + última linha do executor + voz; o painel
  mostra estado, tempo e mensagem. Os campos existem na mensagem para quando o executor
  passar a publicar.
- `task_id` do status vem do YAML da task; `mission_id` é o nome do `actions.yaml`
  (`~/.ros/caramelo_missions/<stamp>_actions.yaml`).

## 6. Interfaces ROS usadas (resumo)

| Interface | Tipo | GUI | Web | Servidor |
|---|---|---|---|---|
| `/caramelo/run_mission` | action `caramelo_msgs/RunMission` | cliente | cliente | `mission_server` |
| `/caramelo/mission/status` | `caramelo_msgs/MissionStatus` (transient_local, 64) | assina | assina | `mission_server` |
| `/manip/speech_status` | `std_msgs/String` (latched) | assina | assina | `manip_audio/speech_node` |
| `/manip/vision_debug/image` | `sensor_msgs/Image` | Sensores | câmera sob demanda | `manip_bringup/vision_debug_node` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | assina | assina | `caramelo_diagnostics/health_monitor` |
| `navigate_to_pose`, `follow_waypoints` | nav2 actions | cliente | cliente (meta) | Nav2 |
| `/dock_robot`, `/undock_robot` | nav2 actions | cliente | cliente | `opennav_docking` |
| `align_to_dock` | `caramelo_msgs/AlignToDock` | cliente | — | `dock_align_node` |
| `/save_dock_pose` | `std_msgs/String` | publica | — | `dock_pose_recorder_node` |
| `/caramelo/service_areas/{list,validate}` + `save_pose` | srv/action `caramelo_msgs` | cliente | — | `service_area_manager_node` |
| `/map_server/load_map`, `/map_saver_server/save_map` | nav2 srv | cliente | `map_saver_cli` | Nav2 / slam.launch |
| `/initialpose` | `PoseWithCovarianceStamped` | publica | publica | AMCL |
| `/cmd_vel_key` | `Twist` | — | publica (watchdog 300/900 ms) | twist_mux |
| scripts `save_service_area_pose`, `save_dock_pose` (`--pose`), `init_service_areas`, `init_keepout_mask` | `ros2 run` por `QProcess` | sim | módulos importados | `caramelo_navigation` |

## 7. Pendente de validar com o robô

Tudo abaixo compila e sobe sem o robô; **nenhum item foi exercitado com o stack real**:

1. **Missão ponta a ponta pela GUI e pelo painel** (BMT em `missions/tasks geradas/`,
   `--dry-run` primeiro): plano, pré-flight, execução, abortar no meio (SIGINT → halt da
   árvore → `ABORTED`), fim normal (`DONE`) e falha (`FAILED`). Conferir que o terminal do
   `mission_server` mostra o log do executor e que `/caramelo/mission/status` acompanha.
2. **Dois mission_servers** (GUI + painel subidos juntos): o segundo deve sair com o aviso
   `single_instance`. Conferir também que o pré-flight com AMT1 ainda exige `/amt1_sort`
   (a tela só lista os servidores comuns).
3. **`--pose` dos scripts** (criar/arrastar ponto na tela Mapas e no passo 2 do
   Mapeamento, sem o robô no lugar): o `service_areas.yaml`/`docking.yaml` gravados batem
   com o que `validate_service_areas --sync-docking` espera (ver README §9 item 2: a base
   ainda tem dois escritores desses arquivos).
4. **Docking pela GUI**: `align_to_dock` com `timeout` fixo de 30 s na GUI (o `GoToWS` usa
   `align_timeout` de 90 s) — pode ser curto para o fluxo v5 com correção final de yaw.
5. **Câmera anotada** na GUI (Sensores) e no painel: `vision_debug` só processa com
   espectador; conferir que aparece e some sem travar o `perception_gate`.
6. **Voz**: `Voz: falando: ...` na tela Competição e na aba do painel acompanhando o
   `speech_node` (latched).
7. **Mapeamento pela GUI** com o `slam.launch.py` atual (RK-02 só avisa) e o salvamento
   (`map_saver_server`), seguido de rebuild de `caramelo_mapping` para o `map_name:=` do
   launch enxergar a arena nova (a GUI lê a árvore fonte; o launch de localização lê o
   share instalado).
8. **Cartões de saúde** com os nomes do `health_monitor` atual (`caramelo/*`): a GUI e o
   painel mapeiam exatamente esses 8 nomes.
9. **Painel web (HTTP)**: neste PC não há `fastapi`/`uvicorn` instalados, então só o nó ROS
   do painel (`ros_node.py`: capacidades, voz, câmeras, tasks, arenas) e os testes foram
   exercitados; as rotas, o WebSocket, o terminal (`~/.caramelo_env.sh` com
   `ROS_DOMAIN_ID=67`), o espelho de tela (`mss`) e o teleop com watchdog ficam para
   validar depois de instalar as dependências (§2).
10. **Simulação**: `use_sim_time:=true` nos dois launches (o `caramelo_simulation_ws` não
    está neste PC).

## 8. O que veio da `robot_ui` e o que ficou de fora

Veio (pacotes de interface inteiros + aditivos): `caramelo_gui`, `caramelo_web`,
`caramelo_msgs/{MissionStatus.msg,RunMission.action}`, `caramelo_navigation/{mission_server.py,
mission_server.launch.py}` (reescrito sobre o `run_mission.py` atual), flag `--pose` em
`save_service_area_pose`/`save_dock_pose`, arg `use_sim_time` no `diagnostics.launch.py`.

Ficou de fora (a base é a fonte da verdade; candidatos a portar depois, caso a caso):

| Mudança da `robot_ui` | Por que ficou de fora | Vale revisitar? |
|---|---|---|
| `bt_yaml_executor.cpp`: publica `MissionStatus` por nó da árvore (`action_index/total`, `stage`), param `task_id`; `task_planner.cpp` grava `task_id` no `actions.yaml` | manipulação fica como na base (261 linhas num arquivo muito alterado desde o merge-base) | **sim** — é o que dá progresso passo a passo na tela; a mensagem já tem os campos |
| `caramelo_service_area_common.py`: `sync_docking_from_service_areas` por **merge** (preserva docks/plugins já no arquivo) e guarda "pose zerada não sobrescreve pose gravada" | navegação fica como na base; a base já inclui START/FINISH no sync (2026-08-17), mas reconstrói o arquivo do zero | **sim** — relacionado ao README §9 item 2 |
| `caramelo.launch.py` (bringup unificado lendo `robot_state.yaml`) e include do `mission_server` no `robot_manipulation.launch.py` | bringup fica como na base; o mapa continua `map_name:=`; o `mission_server` sobe com a GUI/painel | opcional (`use_mission_server` no bringup, default `false`) |
| `global_localization.launch.py`: resolve a pasta do mapa na árvore fonte antes do share | localização fica como na base | sim, se o fluxo "mapa novo pela GUI sem rebuild" for desejado |
| `init_keepout_mask.py`: resolver compartilhado + `CARAMELO_MAPS_DIR` | a GUI/painel já passam `--map-dir` explícito | não precisa |
| `run_mission.py` refatorado em `caramelo_mission_common.py` | o `run_mission` da base é a fonte única; o servidor o importa | não |
