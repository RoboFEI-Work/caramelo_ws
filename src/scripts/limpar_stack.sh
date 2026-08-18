#!/usr/bin/env bash
# Limpa TODO processo ROS do PC antes de subir o stack.
#
# Por que isso existe (2026-08-10): Ctrl-C num "ros2 launch" mata o launch mas
# NAO leva os filhos junto. Os sobreviventes viram nos duplicados no grafo, e o
# efeito e devastador porque NADA acusa erro:
#   - dois /amcl        -> nome ambiguo -> lifecycle_manager nao ativa o
#                          map_server -> /map nunca publica -> RViz sem mapa
#   - dois /move_group  -> handshake de action de 3 s e resultados perdidos
#                          ("unknown result response") -> execucao "falha"
#                          mesmo com o braco obedecendo
#   - dois bringups     -> dois controladores disputando o mesmo hardware
#
# Num dia de testes isso custou horas de diagnostico errado. Rode este script
# sempre que derrubar um launch, ANTES de subir o proximo.
#
# Uso:  ./scripts/limpar_stack.sh
# Obs:  mexe SO no PC. O bringup da Raspberry e derrubado la, na mao.

set -u

PADROES=(
  "ros2 launch"
  "nav2_"
  "moveit_ros_move_group"
  "manip_task_execution"
  "manip_commander"
  "manip_audio"
  "caramelo_navigation"
  "caramelo_utils"
  "realsense2_camera"
  "apriltag_ros"
  "opennav"
  "rviz2"
  "robot_state_publisher"
  "joint_state_publisher"
)

echo "== derrubando processos do stack no PC =="
ALVOS=""
for p in "${PADROES[@]}"; do
  # -f casa a linha de comando inteira; excluimos o proprio script e o pgrep
  IDS=$(pgrep -f "$p" 2>/dev/null | grep -v "^$$\$" || true)
  ALVOS="$ALVOS $IDS"
done

ALVOS=$(echo "$ALVOS" | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u)
if [ -z "$ALVOS" ]; then
  echo "  nada rodando — ja estava limpo"
else
  for pid in $ALVOS; do
    [ "$pid" = "$$" ] && continue
    CMD=$(ps -o cmd= -p "$pid" 2>/dev/null | cut -c1-60)
    [ -z "$CMD" ] && continue
    echo "  kill $pid  $CMD"
    kill -9 "$pid" 2>/dev/null || true
  done
fi

sleep 3

echo "== conferindo =="
RESTO=$(pgrep -af "nav2_|moveit_ros_move_group|manip_task_execution|manip_commander|realsense2_camera|apriltag_ros|rviz2|opennav|ros2 launch" 2>/dev/null \
        | grep -v pgrep | grep -v limpar_stack || true)
if [ -z "$RESTO" ]; then
  echo "  PC LIMPO"
else
  echo "  AINDA SOBROU (mate na mao):"
  echo "$RESTO" | sed 's/^/    /'
fi

# O daemon do ros2 guarda cache do grafo e mostra nos que ja morreram.
ros2 daemon stop >/dev/null 2>&1 || true
echo "  daemon do ros2 reiniciado (limpa cache do grafo)"

echo
echo "== checagem rapida antes de subir =="
echo "  Pi:  ros2 node list | sort | uniq -c | awk '\$1>1'    <- tem que sair vazio"
echo "  Pi:  pgrep -af ros2_control_node                      <- tem que ter SO UM"
