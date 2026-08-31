#!/usr/bin/env bash
# Cenarios de bancada da AMT1 (2026-08-28), sem robo, sem camera e sem MoveIt.
#
# Roda em ROS_DOMAIN_ID=99 (nunca encosta no robo real). Tres familias:
#   * planner / regression / dry_run: task_planner e bt_yaml_executor --dry-run
#     (nenhum server no ar);
#   * casos do NO REAL (amt1_sort_action_node com observe_move_enabled=false)
#     sobre a mesa falsa (fake_amt1_table.py: TF das tags + /pick_tag +
#     /place_tag) e o fake_nav_actions.py (nudge_base + /manip/base_shift_total),
#     goal via `ros2 action send_goal /amt1_sort`; `mission` roda a missao
#     inteira (bt_yaml_executor amt1_actions.yaml) com o no real;
#   * bench_*: so o executor (folha Amt1SortBT) contra o fake_amt1_sort.py.
# Confere por grep as linhas esperadas. Codigo de saida 0 so se TUDO bater.
#
# Uso: run_amt1_scenario.sh [all|<caso>]
#   planner        yaml AMT1 -> goto PP_1 + amt1_sort com expected_tags 1..6;
#                  variantes AMT-1 / "Advanced Manipulation Test 1" tambem;
#                  PP_1 com cubos fora de active_service_areas -> erro
#   regression     BMT/ATT1, "AMT2" e "AMT10" -> fluxo antigo, sem amt1_sort
#                  (AMT2/AMT10 com 0 transferencias -> AVISO "parece AMT1")
#   dry_run        executor --dry-run -> <Amt1Sort name="amt1_1" .../>;
#                  max_onboard 0 aceito, "tres"/4 e expected_tags ruins -> erro
#   main           5,2,3,1,4,6 -> 3 picks / 3 places, 0 nudges, final 1..6
#   two_cycles     2,1,4,3,5,6 -> 4 picks / 4 places
#   sorted         1..6 -> 0 ops, "ja esta ordenada"
#   nudge          cubos em x=+-0.35 com reach_x_max 0.28 -> 2 nudges + re-registro
#   b1             max_onboard 1 -> buffer_insuficiente, 0 picks
#   search         busca lateral (2026-08-28): tags em x=-0.35..+0.35 com campo
#                  de visao view_x_max 0.25 -> ve 4, nudge ate +0.20 (ve a 5),
#                  ate -0.20 (ve a 6), volta ao centro e re-registra; antes
#                  do pick_5 e do pick_4 leva a base para onde viu cada tag
#                  ("indo para onde vi a tag", orcamento da busca) -> final
#                  1..6, missing [], 6 nudges (busca 6, ops 0)
#   search_budget  search com search_max_nudges 4 e reach_x_max 0.28: a busca
#                  gasta os 4 dela (ida, ida em 2 passos, volta), o "ir para
#                  onde vi a tag" fica sem orcamento ("tento daqui") e as ops
#                  usam o max_nudges (6) PROPRIO: 2 nudges de alcance ->
#                  success, final 1..6, nudges 6 (busca 4, ops 2)
#   search_pick_view  search com pick_respects_view na mesa falsa (o /pick_tag
#                  responde tag_nao_encontrada com a tag fora do campo de
#                  visao): so passa porque o no vai para onde viu a tag ANTES
#                  do pick -> success; variante _nobudget (search_max_nudges
#                  4): sem orcamento o pick nao ve a tag_5, o no repete 1x
#                  (re-observa com o braco) e falha limpo: tag_nao_encontrada,
#                  0 picks, mesa intacta
#   short_course   nudge_short_course 1 no fake_nav: o 1o nudge aborta com
#                  curso_incompleto (andou 80%, total publicado com o parcial)
#                  -> o no le o result em ABORTED, conta o passo PARCIAL,
#                  re-registra e repete a op -> success (2 nudges, como o caso
#                  nudge); variante _search: o 1o nudge da busca e parcial ->
#                  "sigo para o alvo", success com 6 nudges
#   unseen         tag_3 nunca vista -> busca lateral (2 posicoes, 4 nudges)
#                  nao acha -> allow_partial=false (default): fail_reason
#                  tag_nao_encontrada, missing [3], 0 picks, mesa intacta;
#                  variante unseen_partial (allow_partial:=true): busca e
#                  depois ordena o que viu -> partial, missing [3], 3 picks
#   fail_place     1o place responde skipped+unreachable sem sugestao 2x (ramo
#                  PP do place real) -> re-registro + repete 1x, fora_de_alcance,
#                  recoverOnboard devolve os 2 cubos: partial=true, 0 a bordo
#   abort_place    1o place ABORTA -> place_falhou SEM recuperacao (cubo pode
#                  estar na garra): 2 cubos ficam nos containers, partial=false
#   timeout_child  pick estoura op_timeout_s -> o no cancela o filho, responde
#                  pick_falhou (SUCCEEDED) e NAO morre (2o goal observe_only OK)
#   cancel         cancel no meio -> goal filho cancelado, status CANCELED
#   mission        executor + fakes + no real -> ORDENACAO CONCLUIDA, SUCCESS
#   bench_ok       executor + fake_amt1_sort -> SUCCESS
#   bench_fail     fake aborta -> FAILURE (exit 1)
#   bench_result   fake devolve success=false -> INCOMPLETA + SUCCESS
#   bench_reject   fake rejeita -> FAILURE (exit 1)
#   bench_watchdog fake mudo -> watchdog de feedback cancela -> FAILURE
#   bench_deadline prazo total (amt1_timeout) estourado so AVISA (1x) e a
#                  missao segue ate SUCCESS; com o fake mudo o cancel e SO do
#                  watchdog (mensagem com o sufixo do prazo)
# Logs: $AMT1_LOG_DIR (default /tmp/amt1_scenario); estado dos containers e
# lock em $AMT1_BENCH_DIR (default /tmp/amt1_bench).
export ROS_DOMAIN_ID=99
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
# Os setup.bash do ROS usam variaveis nao definidas: sourcear ANTES do set -u.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

MAP="$WS_ROOT/src/caramelo_mapping/maps/arenalegal_2026"
TAGS_YAML="$WS_ROOT/src/manipulation/manip_bringup/config/tags_36h11.yaml"
WS_MAP="$MAP/ws_table_mapping.yaml"
MISSION="$WS_ROOT/missions/amt1_pp1.yaml"
BT_DIR="$WS_ROOT/src/manipulation/manip_bt/behavior_tree_manip"
LOG_DIR="${AMT1_LOG_DIR:-/tmp/amt1_scenario}"
BENCH_DIR="${AMT1_BENCH_DIR:-/tmp/amt1_bench}"
CONTAINER_YAML="$BENCH_DIR/container_states.yaml"
LOCK_FILE="$BENCH_DIR.lock"
ACTIONS="$HERE/amt1_actions.yaml"
MODE="${1:-all}"
mkdir -p "$LOG_DIR" "$BENCH_DIR"
# Binarios DIRETOS (nao `ros2 run`): o wrapper python do ros2 run engole o
# SIGINT/KILL e deixava o no real orfao no ar — o caso seguinte via dois
# /amt1_sort e goals duplicados.
NODE_BIN="$(ros2 pkg prefix manip_task_execution)/lib/manip_task_execution/amt1_sort_action_node"
EXEC_BIN="$(ros2 pkg prefix manip_bt)/lib/manip_bt/bt_yaml_executor"
PLANNER_BIN="$(ros2 pkg prefix manip_bt)/lib/manip_bt/task_planner"
for bin in "$NODE_BIN" "$EXEC_BIN" "$PLANNER_BIN"; do
  [ -x "$bin" ] || { echo "binario ausente: $bin (colcon build?)"; exit 2; }
done
# Sobras de uma rodada anterior corrompem os casos (servers duplicados).
LEFTOVERS="$(pgrep -af '^(python3 )?[^ ]*(amt1_sort_action_node|fake_amt1_table\.py|fake_amt1_sort\.py|fake_nav_actions\.py|fake_pick_place\.py)( |$)' || true)"
if [ -n "$LEFTOVERS" ]; then
  echo "AVISO: ja ha processos da bancada no ar (mate-os antes, se forem seus):"
  echo "$LEFTOVERS"
fi

GOAL='{ws: PP_1, table_pose: Mesa10, expected_tags: [1, 2, 3, 4, 5, 6], direction: left_to_right, max_onboard: 3, observe_only: false}'
LAYOUT_MAIN="['5:-0.25','2:-0.15','3:-0.05','1:0.05','4:0.15','6:0.25']"
LAYOUT_TWO="['2:-0.25','1:-0.15','4:-0.05','3:0.05','5:0.15','6:0.25']"
LAYOUT_SORTED="['1:-0.25','2:-0.15','3:-0.05','4:0.05','5:0.15','6:0.25']"
# Pontas fora do alcance (0.35 > reach_x_max 0.28): pick_6 pede nudge para a
# esquerda, pick_5 pede para a direita (a mesa fica parada no odom).
LAYOUT_NUDGE="['6:-0.35','2:-0.12','3:-0.04','1:0.04','4:0.12','5:0.35']"
# Busca lateral (2026-08-28): mesma ordem do main, mas as pontas em x=+-0.35
# ficam fora do campo de visao view_x_max 0.25 da mesa falsa. shift + =
# esquerda => x_manip = x0 + shift cresce: tag_5 (x0 -0.35) aparece com a
# base em +0.20 (x_manip -0.15) e tag_6 (x0 +0.35) com a base em -0.20.
LAYOUT_SEARCH="['5:-0.35','2:-0.20','3:-0.05','1:0.05','4:0.20','6:0.35']"

FAKE_PIDS=()
# SIGINT (nao SIGTERM): os fakes fecham o rclpy direito, o participante DDS
# anuncia a saida e a mesa falsa imprime a "ordem final".
cleanup() {
  for pid in "${FAKE_PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null
  done
  for pid in "${FAKE_PIDS[@]:-}"; do
    [ -n "$pid" ] || continue
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
  done
  FAKE_PIDS=()
}
trap cleanup EXIT

FAILURES=0
fail() { echo "  [FALHOU] $*"; FAILURES=$((FAILURES + 1)); }
ok() { echo "  [ok] $*"; }

# expect_count <log> <n> <texto literal>
expect_count() {
  local log="$1" want="$2" text="$3"
  local got
  got=$(grep -c -F -- "$text" "$log" 2>/dev/null || true)
  if [ "${got:-0}" -eq "$want" ]; then
    ok "$want x '$text'"
  else
    fail "esperava $want x '$text', achei ${got:-0} ($(basename "$log"))"
  fi
}

# expect_min <log> <n> <texto literal>  (pelo menos n)
expect_min() {
  local log="$1" want="$2" text="$3"
  local got
  got=$(grep -c -F -- "$text" "$log" 2>/dev/null || true)
  if [ "${got:-0}" -ge "$want" ]; then
    ok ">= $want x '$text' (achei ${got:-0})"
  else
    fail "esperava >= $want x '$text', achei ${got:-0} ($(basename "$log"))"
  fi
}

expect_exit() {
  local got="$1" want="$2" what="$3"
  if [ "$got" -eq "$want" ]; then
    ok "codigo de saida $got ($what)"
  else
    fail "codigo de saida $got, esperava $want ($what)"
  fi
}

wait_for_action() {
  local name="$1" tries="${2:-30}"
  while [ "$tries" -gt 0 ]; do
    if ros2 action list 2>/dev/null | grep -q -F -- "$name"; then
      return 0
    fi
    tries=$((tries - 1))
    sleep 1
  done
  return 1
}

# wait_for_line <log> <texto literal> <tentativas de 0.5 s>
wait_for_line() {
  local log="$1" text="$2" tries="${3:-60}"
  while [ "$tries" -gt 0 ]; do
    if grep -q -F -- "$text" "$log" 2>/dev/null; then
      return 0
    fi
    tries=$((tries - 1))
    sleep 0.5
  done
  return 1
}

settle() { cleanup; sleep 2; }  # descoberta DDS assenta antes do proximo caso

# ---------------------------------------------------------------- processos
start_fake_nav() {  # <caso> <params>
  local name="$1" params="$2"
  # shellcheck disable=SC2086
  python3 "$HERE/fake_nav_actions.py" --ros-args -p latency_sec:=0.3 $params \
    > "$LOG_DIR/$name.fake_nav.log" 2>&1 &
  FAKE_PIDS+=("$!")
}

start_fake_table() {  # <caso> <layout> <params extra>
  local name="$1" layout="$2" params="$3"
  TABLE_LOG="$LOG_DIR/$name.fake_table.log"
  rm -f "$CONTAINER_YAML"
  # shellcheck disable=SC2086
  python3 "$HERE/fake_amt1_table.py" --ros-args -p latency_sec:=0.3 \
    -p "layout:=$layout" -p container_state_file:="$CONTAINER_YAML" $params \
    > "$TABLE_LOG" 2>&1 &
  FAKE_PIDS+=("$!")
}

start_fake_sort() {  # <caso> <params>
  local name="$1" params="$2"
  SORT_LOG="$LOG_DIR/$name.fake_sort.log"
  # shellcheck disable=SC2086
  python3 "$HERE/fake_amt1_sort.py" --ros-args $params > "$SORT_LOG" 2>&1 &
  FAKE_PIDS+=("$!")
}

start_node() {  # <caso> <params extra do no real>
  local name="$1" params="$2"
  NODE_LOG="$LOG_DIR/$name.node.log"
  # shellcheck disable=SC2086
  "$NODE_BIN" --ros-args \
    -p observe_move_enabled:=false -p speech_enabled:=false -p search_mode:=base \
    -p container_state_file:="$CONTAINER_YAML" -p manipulator_lock_file:="$LOCK_FILE" \
    -p observe_dwell_s:=1.0 $params > "$NODE_LOG" 2>&1 &
  NODE_PID="$!"
  FAKE_PIDS+=("$NODE_PID")
}

# O no real precisa continuar vivo depois de um goal que deu errado (um
# canceled()/abort() fora de hora derruba o processo por std::terminate).
node_alive() {  # <rotulo>
  if kill -0 "${NODE_PID:-0}" 2>/dev/null; then
    ok "amt1_sort_action_node continua vivo ($1)"
  else
    fail "amt1_sort_action_node MORREU ($1): veja $NODE_LOG"
  fi
}

# Sobe fake_nav + mesa falsa + no real e espera os 3 no grafo.
# start_bench <caso> <layout> <params mesa> <params no> <params nav>
start_bench() {
  local name="$1" layout="$2" table_params="$3" node_params="$4" nav_params="${5:-}"
  echo "=== caso: $name ==="
  start_fake_nav "$name" "$nav_params"
  start_fake_table "$name" "$layout" "$table_params"
  start_node "$name" "$node_params"
  if ! wait_for_action "/nudge_base" || ! wait_for_action "/pick_tag" || \
     ! wait_for_action "/amt1_sort"; then
    fail "servers nao apareceram no grafo (ROS_DOMAIN_ID=99): veja $LOG_DIR/$name.*.log"
    return 1
  fi
  sleep 2   # TF das tags ja circulando antes do goal
  return 0
}

# send_goal <caso> <goal yaml>  -> GOAL_LOG, GOAL_RC
send_goal() {
  local name="$1" goal="$2"
  GOAL_LOG="$LOG_DIR/$name.goal.log"
  timeout 300 ros2 action send_goal /amt1_sort my_robot_msgs/action/Amt1Sort "$goal" \
    --feedback > "$GOAL_LOG" 2>&1
  GOAL_RC=$?
}

# run_executor <caso> <yaml> <params extra>  -> CASE_LOG, CASE_RC
run_executor() {
  local name="$1" yaml="$2" params="$3"
  CASE_LOG="$LOG_DIR/$name.executor.log"
  # shellcheck disable=SC2086
  timeout 300 "$EXEC_BIN" "$yaml" --ros-args \
    -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
    -p finish_dock_id:="''" -p server_wait_timeout:=20.0 $params \
    > "$CASE_LOG" 2>&1
  CASE_RC=$?
}

expect_containers() {  # <n ocupados> <rotulo>
  local want="$1" occupied
  occupied=$(grep -c -F "occupied: true" "$CONTAINER_YAML" 2>/dev/null || true)
  if [ "${occupied:-0}" -eq "$want" ]; then
    ok "$want container(s) de bordo ocupado(s) no fim ($2)"
  else
    fail "${occupied:-0} container(s) ocupado(s) no fim, esperava $want ($2): $CONTAINER_YAML"
  fi
}

containers_all_empty() { expect_containers 0 "$1"; }  # <rotulo>

# ---------------------------------------------------------------- planner
case_planner() {
  echo "=== caso: planner ==="
  local out="$LOG_DIR/planner.actions.yaml" log="$LOG_DIR/planner.log"
  "$PLANNER_BIN" "$MISSION" "$out" "$TAGS_YAML" "$WS_MAP" > "$log" 2>&1
  expect_exit $? 0 "task_planner amt1_pp1.yaml"
  expect_count "$log" 1 "[task_planner] AMT1: ordenar 6 tags na PP_1 (Mesa10, left_to_right, max 3 a bordo)"
  expect_count "$out" 1 "kind: goto"
  expect_count "$out" 1 "kind: amt1_sort"
  expect_count "$out" 2 "ws: PP_1"
  expect_count "$out" 1 "mesa: Mesa10"
  expect_count "$out" 1 "table_pose: Mesa10"
  expect_count "$out" 1 "direction: left_to_right"
  expect_count "$out" 1 "max_onboard: 3"
  if python3 - "$out" <<'EOF'
import sys, yaml
root = yaml.safe_load(open(sys.argv[1]))
acts = root["actions"]
assert [a["kind"] for a in acts] == ["goto", "amt1_sort"], acts
sort = acts[1]
assert sort["expected_tags"] == [1, 2, 3, 4, 5, 6], sort
assert isinstance(sort["max_onboard"], int) and sort["max_onboard"] == 3, sort
EOF
  then ok "yaml: kinds [goto, amt1_sort], expected_tags [1..6], max_onboard int 3"
  else fail "yaml do planner fora do esperado: $out"; fi

  # Identificacao (achado 10): task_id AMT-1 / AMT_1 / "AMT 1" e o name
  # "Advanced Manipulation Test 1" tambem ativam a rotina.
  local variant yaml
  for variant in "AMT-1|Ordenar os cubos" "AMT_1|Ordenar os cubos" "AMT 1|Ordenar os cubos" \
                 "X7|Advanced Manipulation Test 1" "X7|ADVANCED  MANIPULATION   TASK  1 (final)"; do
    yaml="$LOG_DIR/planner.variant.yaml"
    out="$LOG_DIR/planner.variant.actions.yaml"
    log="$LOG_DIR/planner.variant.log"
    sed -e "s/^task_id: AMT1.*/task_id: ${variant%%|*}/" -e "s/^name: .*/name: ${variant#*|}/" \
      "$MISSION" > "$yaml"
    "$PLANNER_BIN" "$yaml" "$out" "$TAGS_YAML" "$WS_MAP" > "$log" 2>&1
    expect_exit $? 0 "task_planner variante '${variant/|/ / }'"
    expect_count "$log" 1 "[task_planner] AMT1: ordenar 6 tags na PP_1 (Mesa10, left_to_right, max 3 a bordo)"
    expect_count "$out" 1 "kind: amt1_sort"
  done

  # Achado 9: os cubos estao na PP_1 mas so a PP_2 esta ativa -> erro claro,
  # sem redirecionar a rotina para outra mesa.
  yaml="$LOG_DIR/planner.inactive.yaml"
  out="$LOG_DIR/planner.inactive.actions.yaml"
  log="$LOG_DIR/planner.inactive.log"
  sed -e 's/^active_service_areas: .*/active_service_areas: [PP_2]/' "$MISSION" > "$yaml"
  "$PLANNER_BIN" "$yaml" "$out" "$TAGS_YAML" "$WS_MAP" > "$log" 2>&1
  expect_exit $? 1 "task_planner PP_1 com cubos fora de active_service_areas"
  expect_count "$log" 1 "AMT1: start_state tem objetos em PP_1, mas ela nao esta ativa"
  expect_count "$log" 0 "kind: amt1_sort"
}

# ---------------------------------------------------------------- regression
case_regression() {
  echo "=== caso: regression ==="
  local yaml out log
  for task in BMT ATT1; do
    yaml="$BT_DIR/$task.yaml"
    out="$LOG_DIR/regression.$task.actions.yaml"
    log="$LOG_DIR/regression.$task.log"
    "$PLANNER_BIN" "$yaml" "$out" "$TAGS_YAML" "$BT_DIR/ws_table_mapping.yaml" > "$log" 2>&1
    expect_exit $? 0 "task_planner $task.yaml"
    expect_count "$out" 0 "amt1_sort"
    expect_count "$log" 0 "AMT1"
    expect_min "$out" 1 "kind: pick"
    expect_min "$out" 1 "kind: place"
    expect_min "$log" 1 "objeto(s) a transportar"
  done
  # Mesmo yaml da AMT1 com task_id AMT2/AMT10 e nome generico: fluxo antigo
  # (objetos que comecam e terminam na mesma estacao => nada a transportar);
  # o planner so AVISA que "parece AMT1" (achado 10), sem mudar o fluxo.
  local variant
  for variant in "AMT2|Advanced Manipulation Task 2" "AMT10|Advanced Manipulation Task 10"; do
    yaml="$LOG_DIR/regression.${variant%%|*}.yaml"
    sed -e "s/^task_id: AMT1.*/task_id: ${variant%%|*}/" -e "s/^name: .*/name: ${variant#*|}/" \
      "$MISSION" > "$yaml"
    out="$LOG_DIR/regression.${variant%%|*}.actions.yaml"
    log="$LOG_DIR/regression.${variant%%|*}.log"
    "$PLANNER_BIN" "$yaml" "$out" "$TAGS_YAML" "$WS_MAP" > "$log" 2>&1
    expect_exit $? 0 "task_planner ${variant%%|*} (nao-AMT1)"
    expect_count "$out" 0 "amt1_sort"
    expect_count "$log" 0 "[task_planner] AMT1:"
    expect_count "$log" 1 "[task_planner] AVISO: parece AMT1 - task_id/name nao bateram (task_id=${variant%%|*}, name=${variant#*|})"
    expect_count "$log" 1 "0 objeto(s) a transportar"
  done
}

# ---------------------------------------------------------------- dry_run
case_dry_run() {
  echo "=== caso: dry_run ==="
  local log="$LOG_DIR/dry_run.log"
  timeout 60 "$EXEC_BIN" "$ACTIONS" --dry-run --ros-args \
    -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
    -p finish_dock_id:="''" > "$log" 2>&1
  expect_exit $? 0 "bt_yaml_executor --dry-run amt1_actions.yaml"
  expect_count "$log" 1 "[0] goto  PP_1 -> PP1  (dock+align)  [falhou => pula a estacao]"
  expect_count "$log" 1 "[1] amt1  ordenar 6 tags na PP_1 (mesa=Mesa10, left_to_right, max_onboard=3)"
  expect_count "$log" 1 '<Amt1Sort name="amt1_1" ws="{action_1_ws}" table_pose="{action_1_table_pose}" expected_tags="{action_1_expected_tags}" direction="{action_1_direction}" max_onboard="{action_1_max_onboard}" timeout="{amt1_timeout}" stage_timeout="{amt1_stage_timeout}"/>'
  expect_count "$log" 0 "<ReachQueue"
  expect_count "$log" 0 "<NudgeBase"
  # O yaml que o task_planner gera precisa passar pelo mesmo caminho.
  local planned="$LOG_DIR/dry_run.planner.actions.yaml" log2="$LOG_DIR/dry_run.planner.log"
  "$PLANNER_BIN" "$MISSION" "$planned" "$TAGS_YAML" "$WS_MAP" > "$log2" 2>&1 \
    && timeout 60 "$EXEC_BIN" "$planned" --dry-run --ros-args \
      -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
      -p finish_dock_id:="''" >> "$log2" 2>&1
  expect_exit $? 0 "dry-run do yaml gerado pelo task_planner"
  expect_count "$log2" 1 '<Amt1Sort name="amt1_1"'

  # Achado 11: max_onboard 0..3 (0 = todos os containers vazios) e
  # expected_tags validados com mensagem clara em vez de excecao do yaml-cpp.
  # Cada variante: <regex sed de origem>|<texto novo>|<exit>|<linha esperada>.
  local variant yaml log3
  for variant in "max_onboard: 3|max_onboard: 0|0|[1] amt1  ordenar 6 tags na PP_1 (mesa=Mesa10, left_to_right, max_onboard=0)" \
                 "max_onboard: 3|max_onboard: tres|1|actions[1] amt1_sort max_onboard=tres nao e inteiro." \
                 "max_onboard: 3|max_onboard: 4|1|actions[1] amt1_sort max_onboard=4 fora de 0..3 (0 = todos os containers vazios)." \
                 "expected_tags: \[1, 2, 3, 4, 5, 6\]|expected_tags: [1, dois, 3]|1|actions[1] amt1_sort expected_tags: 'dois' nao e inteiro" \
                 "expected_tags: \[1, 2, 3, 4, 5, 6\]|expected_tags: '1,x,3'|1|actions[1] amt1_sort expected_tags: token invalido 'x'"; do
    IFS='|' read -r from to rc text <<< "$variant"
    yaml="$LOG_DIR/dry_run.variant.yaml"
    log3="$LOG_DIR/dry_run.variant.log"
    sed -e "s/$from/$to/" "$ACTIONS" > "$yaml"
    timeout 60 "$EXEC_BIN" "$yaml" --dry-run --ros-args \
      -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
      -p finish_dock_id:="''" > "$log3" 2>&1
    expect_exit $? "$rc" "dry-run com '$to'"
    expect_count "$log3" 1 "$text"
  done
}

# ---------------------------------------------------------------- no real
case_main() {
  start_bench main "$LAYOUT_MAIN" "" "" || { settle; return; }
  send_goal main "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "picks: 3"
  expect_count "$GOAL_LOG" 1 "places: 3"
  expect_count "$GOAL_LOG" 1 "nudges: 0"
  expect_count "$NODE_LOG" 1 "[AMT1] buffer = min(max_onboard 3, vazios 3) = 3 (todos vazios)"
  expect_count "$NODE_LOG" 1 "[AMT1] vistas 6 tag(s): ordem [5, 2, 3, 1, 4, 6] alvo [1, 2, 3, 4, 5, 6] faltam [] ignoradas []"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 6 op(s), 3 pick(s), 3 place(s), buffer 3: pick_5 pick_1 place_1_slot_0 pick_4 place_4_slot_3 place_5_slot_4"
  expect_count "$NODE_LOG" 1 "op_3/6_place_1_slot_0: PlaceTag{tag_frame=tag_1 table_pose=Mesa10 ws=PP_1 stack_on=tag_amt1_slot_0 final_attempt=false}"
  expect_count "$NODE_LOG" 1 "[AMT1] fim: success=true partial=false fail_reason='' observed=[5, 2, 3, 1, 4, 6] final=[1, 2, 3, 4, 5, 6] missing=[] picks=3 places=3 nudges=0 total=+0.00 m braco=conhecido"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$LOG_DIR/main.fake_nav.log" 0 "goal aceito em 'nudge'"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty main
}

case_two_cycles() {
  start_bench two_cycles "$LAYOUT_TWO" "" "" || { settle; return; }
  send_goal two_cycles "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 8 op(s), 4 pick(s), 4 place(s), buffer 3: pick_2 pick_1 place_1_slot_0 place_2_slot_1 pick_4 pick_3 place_3_slot_2 place_4_slot_3"
  expect_count "$NODE_LOG" 1 "final=[1, 2, 3, 4, 5, 6] missing=[] picks=4 places=4 nudges=0"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  containers_all_empty two_cycles
}

case_sorted() {
  start_bench sorted "$LAYOUT_SORTED" "" "" || { settle; return; }
  send_goal sorted "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "picks: 0"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 0 op(s), 0 pick(s), 0 place(s), buffer 3:"
  expect_count "$NODE_LOG" 1 "mesa ja ordenada"
  expect_count "$NODE_LOG" 0 "PickTag{"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 0 "goal aceito em 'pick'"
}

case_nudge() {
  # fake_nav: slip 0.85 (anda 85% do pedido). pick_6 em x=-0.35 -> sugestao
  # +0.15 (anda +0.1275); pick_5 em x=+0.35+0.1275 -> sugestao -0.25 (anda
  # -0.2125). A mesa fica parada no odom => re-registro com delta ~0.
  start_bench nudge "$LAYOUT_NUDGE" "-p reach_x_max:=0.28" "" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal nudge "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "nudges: 2"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 8 op(s), 4 pick(s), 4 place(s), buffer 3: pick_6 pick_1 place_1_slot_0 pick_4 place_4_slot_3 pick_5 place_5_slot_4 place_6_slot_5"
  expect_count "$NODE_LOG" 1 "op_1/8_pick_6: alvo fora de alcance (sugestao +0.15 m) — nudge +0.15 m (1/6, total +0.00 m)"
  expect_count "$NODE_LOG" 1 "[AMT1] nudge +0.15 m ok: andou +0.12"
  expect_count "$NODE_LOG" 1 "op_6/8_pick_5: alvo fora de alcance (sugestao -0.25 m) — nudge -0.25 m (2/6, total +0.13 m)"
  expect_count "$NODE_LOG" 1 "[AMT1] nudge -0.25 m ok: andou -0.21"
  # 2o re-registro: tag_1 e tag_4 ja foram pousadas pelo robo (erro de pouso
  # nao pode virar ancora) => so 3 ancoras originais (2, 3, 5) de 5 na mesa.
  expect_count "$NODE_LOG" 2 "[AMT1] re-registro aplicado: delta ("
  expect_count "$NODE_LOG" 1 "m em odom com 6 ancora(s), espalhamento 0.000 m. Ancoras originais: 6"
  expect_count "$NODE_LOG" 1 "m em odom com 3 ancora(s), espalhamento 0.000 m. Ancoras originais: 3"
  expect_count "$NODE_LOG" 2 "m (pousado pelo robo)"
  expect_count "$NODE_LOG" 0 "usando tambem"
  expect_count "$NODE_LOG" 0 "re-registro NAO aplicado"
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_6"
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 1 "fail_reason='' observed=[6, 2, 3, 1, 4, 5] final=[1, 2, 3, 4, 5, 6] missing=[] picks=4 places=4 nudges=2 total=-0.09 m braco=conhecido"
  expect_count "$LOG_DIR/nudge.fake_nav.log" 1 "'nudge' concluido (fake): pedido +0.150 m, andou +0.128 m"
  expect_count "$LOG_DIR/nudge.fake_nav.log" 1 "'nudge' concluido (fake): pedido -0.250 m, andou -0.212 m"
  expect_count "$TABLE_LOG" 1 "'pick' FORA DE ALCANCE tag_6: x_manip -0.350 m"
  expect_count "$TABLE_LOG" 1 "'pick' FORA DE ALCANCE tag_5: x_manip +0.47"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "shift -0.085 m"
  containers_all_empty nudge
}

case_b1() {
  start_bench b1 "$LAYOUT_MAIN" "" "" || { settle; return; }
  send_goal b1 "${GOAL/max_onboard: 3/max_onboard: 1}"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: buffer_insuficiente"
  expect_count "$GOAL_LOG" 1 "picks: 0"
  expect_count "$NODE_LOG" 1 "[AMT1] buffer = min(max_onboard 1, vazios 3) = 1 (todos vazios)"
  expect_count "$NODE_LOG" 1 "fail_reason='buffer_insuficiente'"
  expect_count "$NODE_LOG" 0 "[AMT1] plano:"
  expect_count "$NODE_LOG" 0 "PickTag{"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [5, 2, 3, 1, 4, 6]"
  expect_count "$TABLE_LOG" 0 "goal aceito em"
}

# Trilha da busca lateral com o fake_nav (slip 0.85, min 0.08, max 0.25),
# search_offsets_m [+0.20, -0.20], search_arrive_tol_m 0.08 e volta ao
# centro (shift0) — igual nos casos search/unseen/unseen_partial/
# search_budget/search_pick_view:
#   alvo +0.20: nudge +0.20 -> anda +0.170 (resta 0.030 < 0.08: chegou);
#   alvo -0.20: nudge -0.25 -> -0.042; nudge -0.16 -> -0.176 (chegou);
#   volta a 0:  nudge +0.18 -> -0.026.
# Sao 4 nudges SO da busca, que tem contador PROPRIO (search_max_nudges,
# 4o argumento, default 8; o max_nudges 6 das ops fica intacto); a base
# termina em -0.026 m. Os "indo para onde vi a tag" das ops debitam desse
# mesmo contador (caso search: 5/8 e 6/8).
expect_search_trail() {  # <node log> <faltam na 1a posicao> <faltam na 2a posicao> [search_max_nudges]
  local log="$1" miss1="$2" miss2="$3" max="${4:-8}"
  expect_count "$log" 1 "[AMT1] search_nudge: base em +0.000 m, alvo +0.20 m (resta +0.200 m) — nudge +0.20 m (1/$max, total +0.00 m)."
  expect_count "$log" 1 "[AMT1] nudge +0.20 m ok: andou +0.17"
  expect_count "$log" 1 "[AMT1] search_nudge: base em +0.170 m, alvo +0.20 m (resta +0.030 m): chegou (1 passo(s))."
  expect_count "$log" 1 "[AMT1] search_nudge: base em +0.170 m, alvo -0.20 m (resta -0.370 m) — nudge -0.25 m (2/$max, total +0.17 m)."
  expect_count "$log" 1 "[AMT1] nudge -0.25 m ok: andou -0.21"
  expect_count "$log" 1 "[AMT1] search_nudge: base em -0.042 m, alvo -0.20 m (resta -0.158 m) — nudge -0.16 m (3/$max, total -0.04 m)."
  expect_count "$log" 1 "[AMT1] search_nudge: base em -0.176 m, alvo -0.20 m (resta -0.024 m): chegou (2 passo(s))."
  expect_count "$log" 1 "[AMT1] search_nudge: base em -0.176 m, alvo +0.00 m (resta +0.176 m) — nudge +0.18 m (4/$max, total -0.18 m)."
  expect_count "$log" 1 "[AMT1] search_nudge: base em -0.026 m, alvo +0.00 m (resta +0.026 m): chegou (1 passo(s))."
  expect_count "$log" 1 "[AMT1] busca 1/2 em +0.170 m: $miss1"
  expect_count "$log" 1 "[AMT1] busca 2/2 em -0.176 m: $miss2"
  expect_count "$log" 1 "stage=searching_1"
  expect_count "$log" 1 "stage=searching_2"
  expect_count "$log" 1 "stage=search_nudge_+0.20"
  expect_count "$log" 1 "stage=search_nudge_-0.25"
  expect_count "$log" 2 "stage=search_observing_static"
  expect_count "$log" 1 "stage=search_return_center"
}

case_search() {
  # Campo de visao 0.25 na mesa falsa: do centro o no ve so [2, 3, 1, 4]
  # (tag_5 em -0.35 e tag_6 em +0.35 nao entram no TF). Busca lateral:
  # +0.20 -> tag_5 aparece (x_manip -0.18), tag_4 some (+0.37) mas a amostra
  # dela FICA (acumula); -0.20 -> tag_6 aparece (+0.17), "Encontrei todas as
  # tags"; volta ao centro (-0.026) e re-registra os slots pelas 4 tags
  # intactas visiveis de la. Os slots vem do frame fixo T0 (ref(x=-0.350)
  # para tag_5, vista com a base deslocada). Antes do pick_5 o no leva a
  # base para onde viu a tag_5 (+0.170 -> chega em +0.141; 5/8 do contador
  # da busca) e re-registra; antes do pick_4 volta para onde a viu (-0.026
  # -> -0.001; 6/8). reach_x_max 0.40 para as ops nao precisarem de nudge
  # de alcance: nudges 6 (busca 6, ops 0), base termina em -0.001 m.
  # speech_enabled=true so para as falas da busca sairem no log (dominio 99).
  start_bench search "$LAYOUT_SEARCH" "-p view_x_max:=0.25 -p reach_x_max:=0.40" \
    "-p speech_enabled:=true" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal search "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "partial: false"
  expect_count "$GOAL_LOG" 1 "picks: 3"
  expect_count "$GOAL_LOG" 1 "places: 3"
  expect_count "$GOAL_LOG" 1 "nudges: 6"
  expect_count "$GOAL_LOG" 1 "missing_tags: []"
  expect_count "$GOAL_LOG" 1 "current_stage: searching_1"
  expect_count "$GOAL_LOG" 1 "current_stage: search_return_center"
  expect_min "$GOAL_LOG" 1 "current_stage: pick_goto_seen_5"
  expect_count "$NODE_LOG" 1 "busca_max_nudges=8 busca_em_observe_only=false"
  expect_count "$TABLE_LOG" 1 "view_x_max 0.25 m"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral: faltam [5, 6] apos a observacao inicial — 2 posicao(oes) [+0.20, -0.20] m (base em +0.000 m, poses [tag_direita, tag_esquerda])."
  expect_count "$NODE_LOG" 1 ". Vou me mover para procurar"
  expect_count "$NODE_LOG" 1 "[AMT1] busca 1/2: alvo +0.20 m (faltam [5, 6])."
  expect_count "$NODE_LOG" 1 "[AMT1] busca 2/2: alvo -0.20 m (faltam [6])."
  expect_search_trail "$NODE_LOG" "5 tag(s) acumulada(s), faltam [6]." "6 tag(s) acumulada(s), faltam []."
  expect_count "$NODE_LOG" 1 "[SPEECH] Encontrei todas as tags"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam []."
  expect_count "$NODE_LOG" 1 "[AMT1] vistas 6 tag(s): ordem [5, 2, 3, 1, 4, 6] alvo [1, 2, 3, 4, 5, 6] faltam [] ignoradas []"
  expect_count "$NODE_LOG" 0 "tags esperadas nao vistas"
  # Referencia fixa T0: a tag_5 foi vista com a base em +0.17 m (x_manip
  # -0.18) e a tag_6 em -0.176 m, mas os slots saem no x do inicio do goal.
  expect_count "$NODE_LOG" 1 "[AMT1] slot 0 = tag_amt1_slot_0 <- tag_5  ref(x=-0.350 y=0.350 z=0.142)"
  expect_count "$NODE_LOG" 1 "[AMT1] slot 4 = tag_amt1_slot_4 <- tag_4  ref(x=0.200 y=0.350 z=0.142)"
  expect_count "$NODE_LOG" 1 "[AMT1] slot 5 = tag_amt1_slot_5 <- tag_6  ref(x=0.350 y=0.350 z=0.142)"
  # Re-registro logo apos a volta ao centro (antes do plano): 4 ancoras
  # (2, 3, 1, 4), delta ~0 porque a mesa falsa nao escorrega.
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral moveu a base (voltou ao centro, base em -0.026 m): re-registrando os slots pelas tags intactas."
  expect_count "$NODE_LOG" 1 "[SPEECH] Ordem atual: 5, 2, 3, 1, 4, 6. Vou ordenar de 1 a 6"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 6 op(s), 3 pick(s), 3 place(s), buffer 3: pick_5 pick_1 place_1_slot_0 pick_4 place_4_slot_3 place_5_slot_4"
  # "Indo para onde vi a tag" (2026-08-28): tag_5 foi vista em +0.170 m e a
  # base esta em -0.026 m (|delta| 0.196 > 0.12) => nudge +0.20 (5/8) antes
  # do pick_5 + re-registro (4 ancoras: 5, 2, 3, 1); tag_4 foi vista pela
  # ultima vez em -0.026 m (re-registro do centro) => nudge -0.17 (6/8)
  # antes do pick_4 + re-registro (3 ancoras: 2, 3, 4).
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: indo para onde vi a tag_5: base em -0.026 m, tag vista em +0.170 m (|delta| 0.196 m > 0.12 m; nudges da busca 4/8)."
  expect_count "$NODE_LOG" 1 "[AMT1] pick_goto_seen_5: base em -0.026 m, alvo +0.17 m (resta +0.196 m) — nudge +0.20 m (5/8, total -0.03 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] pick_goto_seen_5: base em +0.141 m, alvo +0.17 m (resta +0.029 m): chegou (1 passo(s))."
  expect_count "$NODE_LOG" 1 "stage=pick_goto_seen_5 ("
  expect_count "$NODE_LOG" 1 "stage=pick_goto_seen_5_+0.20"
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: indo para onde vi a tag_4: base em +0.141 m, tag vista em -0.026 m (|delta| 0.167 m > 0.12 m; nudges da busca 5/8)."
  expect_count "$NODE_LOG" 1 "[AMT1] pick_goto_seen_4: base em +0.141 m, alvo -0.03 m (resta -0.167 m) — nudge -0.17 m (6/8, total +0.14 m)."
  expect_count "$NODE_LOG" 1 "stage=pick_goto_seen_4 ("
  expect_count "$NODE_LOG" 2 "indo para onde vi a tag_"
  expect_count "$NODE_LOG" 3 "[AMT1] re-registro aplicado: delta ("
  expect_count "$NODE_LOG" 2 "m em odom com 4 ancora(s), espalhamento 0.000 m. Ancoras originais: 4"
  expect_count "$NODE_LOG" 1 "m em odom com 3 ancora(s), espalhamento 0.000 m. Ancoras originais: 3"
  expect_count "$NODE_LOG" 0 "re-registro NAO aplicado"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$NODE_LOG" 0 "pick nao viu"
  expect_count "$NODE_LOG" 0 "nao consegui ir"
  expect_count "$NODE_LOG" 1 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 1 "PickTag{tag_frame=tag_4"
  expect_count "$NODE_LOG" 1 "[AMT1] fim: success=true partial=false fail_reason='' observed=[5, 2, 3, 1, 4, 6] final=[1, 2, 3, 4, 5, 6] missing=[] picks=3 places=3 nudges=6 total=-0.00 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 6 (busca 6, ops 0)"
  expect_count "$LOG_DIR/search.fake_nav.log" 6 "goal aceito em 'nudge'"
  expect_count "$LOG_DIR/search.fake_nav.log" 1 "'nudge' concluido (fake): pedido +0.200 m, andou +0.170 m"
  # goto-seen pede exatamente o que resta (+0.196; o no loga arredondado +0.20).
  expect_count "$LOG_DIR/search.fake_nav.log" 1 "'nudge' concluido (fake): pedido +0.196 m, andou +0.167 m"
  expect_count "$LOG_DIR/search.fake_nav.log" 1 "'nudge' concluido (fake): pedido -0.250 m, andou -0.212 m"
  expect_count "$TABLE_LOG" 0 "FORA DE ALCANCE"
  expect_count "$TABLE_LOG" 1 "'pick' tag_5 ok (x_manip -0.209 m)"
  expect_count "$TABLE_LOG" 1 "'pick' tag_4 ok (x_manip +0.199 m)"
  node_alive search
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  expect_count "$TABLE_LOG" 1 "shift -0.001 m"
  containers_all_empty search
}

case_search_budget() {
  # Orcamento PROPRIO da busca (2026-08-28): search_max_nudges 4 e
  # reach_x_max 0.28 (default da mesa). A busca gasta os 4 dela (ida +0.20,
  # ida -0.20 em 2 passos, volta ao centro em -0.026). O "ir para onde vi a
  # tag" antes do pick_5 e do pick_4 ja nao tem orcamento da busca ("tento
  # daqui") e o pick responde fora de alcance: as ops usam o max_nudges (6)
  # PROPRIO — nudge +0.15 (tag_5: x -0.376 -> -0.249) e nudge -0.10 (tag_4:
  # x +0.301 -> +0.216) —, com re-registro a cada passo. Sob o orcamento
  # unico antigo (max_nudges 6 compartilhado) isto teria estourado.
  start_bench search_budget "$LAYOUT_SEARCH" "-p view_x_max:=0.25" \
    "-p search_max_nudges:=4" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal search_budget "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "partial: false"
  expect_count "$GOAL_LOG" 1 "picks: 3"
  expect_count "$GOAL_LOG" 1 "places: 3"
  expect_count "$GOAL_LOG" 1 "nudges: 6"
  expect_count "$GOAL_LOG" 1 "missing_tags: []"
  expect_count "$NODE_LOG" 1 "busca_max_nudges=4"
  expect_search_trail "$NODE_LOG" "5 tag(s) acumulada(s), faltam [6]." "6 tag(s) acumulada(s), faltam []." 4
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam []."
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral moveu a base (voltou ao centro, base em -0.026 m): re-registrando os slots pelas tags intactas."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: indo para onde vi a tag_5: base em -0.026 m, tag vista em +0.170 m (|delta| 0.196 m > 0.12 m; nudges da busca 4/4)."
  expect_count "$NODE_LOG" 1 "[AMT1] pick_goto_seen_5: orcamento de nudges da busca esgotado (4/4; ops 0/6) com a base em -0.026 m (alvo +0.17 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: nao consegui ir para onde vi a tag_5 (search_max_nudges esgotado) — tento daqui (base em -0.026 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: alvo fora de alcance (sugestao +0.15 m) — nudge +0.15 m (1/6, total -0.03 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] nudge +0.15 m ok: andou +0.12"
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: indo para onde vi a tag_4: base em +0.101 m, tag vista em -0.026 m"
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: nao consegui ir para onde vi a tag_4 (search_max_nudges esgotado) — tento daqui (base em +0.101 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: alvo fora de alcance (sugestao -0.10 m) — nudge -0.10 m (2/6, total +0.10 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] nudge -0.10 m ok: andou -0.08"
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_4"
  # Re-registros: apos a volta da busca (4 ancoras: 2, 3, 1, 4), apos o
  # nudge +0.15 (4: 5, 2, 3, 1) e apos o nudge -0.10 (3: 2, 3, 4 — a tag_1
  # ja foi pousada pelo robo e a tag_5 esta a bordo).
  expect_count "$NODE_LOG" 3 "[AMT1] re-registro aplicado: delta ("
  expect_count "$NODE_LOG" 2 "m em odom com 4 ancora(s), espalhamento 0.000 m. Ancoras originais: 4"
  expect_count "$NODE_LOG" 1 "m em odom com 3 ancora(s), espalhamento 0.000 m. Ancoras originais: 3"
  expect_count "$NODE_LOG" 0 "re-registro NAO aplicado"
  expect_count "$NODE_LOG" 0 "nudge_falhou"
  expect_count "$NODE_LOG" 0 "pick nao viu"
  expect_count "$NODE_LOG" 1 "[AMT1] fim: success=true partial=false fail_reason='' observed=[5, 2, 3, 1, 4, 6] final=[1, 2, 3, 4, 5, 6] missing=[] picks=3 places=3 nudges=6 total=+0.02 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 6 (busca 4, ops 2)"
  expect_count "$LOG_DIR/search_budget.fake_nav.log" 6 "goal aceito em 'nudge'"
  expect_count "$TABLE_LOG" 1 "'pick' FORA DE ALCANCE tag_5: x_manip -0.376 m (> 0.28) -> sugestao +0.15 m (final_attempt=false)."
  expect_count "$TABLE_LOG" 1 "'pick' FORA DE ALCANCE tag_4: x_manip +0.301 m (> 0.28) -> sugestao -0.10 m (final_attempt=false)."
  expect_count "$TABLE_LOG" 1 "'pick' tag_5 ok (x_manip -0.249 m)"
  expect_count "$TABLE_LOG" 1 "'pick' tag_4 ok (x_manip +0.216 m)"
  node_alive search_budget
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  expect_count "$TABLE_LOG" 1 "shift +0.016 m"
  containers_all_empty search_budget
}

case_search_pick_view() {
  # pick_respects_view na mesa falsa: o /pick_tag responde tag_nao_encontrada
  # (sem sugestao) se a tag esta fora do campo de visao (view 0.25) — como a
  # varredura do pick real, que nao acha uma tag que so foi vista com a base
  # deslocada. Do centro (-0.026) a tag_5 fica em x -0.376: o pick SO passa
  # porque o no vai ANTES para onde a viu (+0.170 -> chega em +0.141, x
  # -0.209); idem tag_4 (vista em -0.026, base em +0.141 -> -0.001).
  # reach_x_max default 0.28: nenhum nudge de alcance (busca 6, ops 0).
  start_bench search_pick_view "$LAYOUT_SEARCH" "-p view_x_max:=0.25 -p pick_respects_view:=true" \
    "" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal search_pick_view "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "partial: false"
  expect_count "$GOAL_LOG" 1 "picks: 3"
  expect_count "$GOAL_LOG" 1 "places: 3"
  expect_count "$GOAL_LOG" 1 "nudges: 6"
  expect_count "$TABLE_LOG" 1 "pick_respects_view True"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam []."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: indo para onde vi a tag_5: base em -0.026 m, tag vista em +0.170 m"
  expect_count "$NODE_LOG" 1 "[AMT1] pick_goto_seen_5: base em +0.141 m, alvo +0.17 m (resta +0.029 m): chegou (1 passo(s))."
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: indo para onde vi a tag_4: base em +0.141 m, tag vista em -0.026 m"
  expect_count "$NODE_LOG" 1 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 1 "PickTag{tag_frame=tag_4"
  expect_count "$NODE_LOG" 0 "pick nao viu"
  expect_count "$NODE_LOG" 0 "nao consegui ir"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$NODE_LOG" 1 "final=[1, 2, 3, 4, 5, 6] missing=[] picks=3 places=3 nudges=6 total=-0.00 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 6 (busca 6, ops 0)"
  expect_count "$TABLE_LOG" 1 "'pick' tag_5 ok (x_manip -0.209 m)"
  expect_count "$TABLE_LOG" 1 "'pick' tag_4 ok (x_manip +0.199 m)"
  expect_count "$TABLE_LOG" 0 "fora do campo de visao"
  expect_count "$TABLE_LOG" 0 "FORA DE ALCANCE"
  node_alive search_pick_view
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty search_pick_view

  # Variante SEM orcamento da busca (search_max_nudges 4, gasto na ida e na
  # volta): o no nao consegue ir para onde viu a tag_5 ("tento daqui"), o
  # pick nao a ve do centro (fora do campo de visao), o no tenta ir de novo
  # (sem orcamento), re-observa 1x so com o braco (retry_not_seen 1) e falha
  # LIMPO em 2 picks: tag_nao_encontrada, 0 picks, mesa intacta, sem laco.
  start_bench search_pick_view_nobudget "$LAYOUT_SEARCH" "-p view_x_max:=0.25 -p pick_respects_view:=true" \
    "-p search_max_nudges:=4" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal search_pick_view_nobudget "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal (search_max_nudges:=4)"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: tag_nao_encontrada"
  expect_count "$GOAL_LOG" 1 "picks: 0"
  expect_count "$GOAL_LOG" 1 "places: 0"
  expect_count "$GOAL_LOG" 1 "nudges: 4"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam []."
  expect_count "$NODE_LOG" 3 "[AMT1] op_1/6_pick_5: indo para onde vi a tag_5: base em -0.026 m, tag vista em +0.170 m (|delta| 0.196 m > 0.12 m; nudges da busca 4/4)."
  expect_count "$NODE_LOG" 3 "[AMT1] op_1/6_pick_5: nao consegui ir para onde vi a tag_5 (search_max_nudges esgotado) — tento daqui (base em -0.026 m)."
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: pick nao viu tag_5 (tag_nao_encontrada) — re-observando das poses laterais (1/1)."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: tag_5 tambem nao apareceu na re-observacao."
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5 falhou: tag_nao_encontrada — op_1/6_pick_5: tag_5 nao encontrada na mesa"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$NODE_LOG" 0 "recover_place"
  expect_count "$NODE_LOG" 1 "nudges 4 (busca 4, ops 0)"
  expect_count "$TABLE_LOG" 2 "'pick' PULADO tag_5: fora do campo de visao (x_manip -0.376 m, |x| > view_x_max 0.25; pick_respects_view)."
  expect_count "$LOG_DIR/search_pick_view_nobudget.fake_nav.log" 4 "goal aceito em 'nudge'"
  node_alive search_pick_view_nobudget
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [5, 2, 3, 1, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty search_pick_view_nobudget
}

case_short_course() {
  # nudge_short_course 1 no fake_nav: o 1o nudge (pick_6, +0.15) aborta com
  # reason curso_incompleto e travelled +0.120 (80%), total publicado com o
  # parcial — como o dock_align_node real. O no le o result mesmo em
  # ABORTED, conta o passo PARCIAL (1/6 ops), re-registra e repete a op; o
  # 2o nudge (pick_5, -0.25) e normal. Mesmo layout/resultado do caso nudge.
  start_bench short_course "$LAYOUT_NUDGE" "-p reach_x_max:=0.28" "" \
    "-p nudge_slip:=0.85 -p nudge_short_course:=1" || { settle; return; }
  send_goal short_course "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "nudges: 2"
  expect_count "$LOG_DIR/short_course.fake_nav.log" 1 "nudge_short_course=1"
  expect_count "$LOG_DIR/short_course.fake_nav.log" 1 "'nudge' CURSO INCOMPLETO (fake, nudge_short_course): pedido +0.150 m, andou +0.120 m -> abort com reason curso_incompleto."
  expect_count "$LOG_DIR/short_course.fake_nav.log" 1 "'nudge' concluido (fake): pedido -0.250 m, andou -0.212 m"
  expect_count "$LOG_DIR/short_course.fake_nav.log" 2 "goal aceito em 'nudge'"
  expect_count "$NODE_LOG" 1 "op_1/8_pick_6: alvo fora de alcance (sugestao +0.15 m) — nudge +0.15 m (1/6, total +0.00 m)"
  expect_count "$NODE_LOG" 1 "[AMT1] nudge +0.15 m PARCIAL (curso_incompleto): andou +0.120 m (total +0.120 m, 1/6 ops"
  expect_count "$NODE_LOG" 1 "[AMT1] nudge +0.15 m parcial (curso_incompleto) mas a base andou +0.120 m: sigo com o re-registro e repito a op."
  expect_count "$NODE_LOG" 0 "FALHOU"
  expect_count "$NODE_LOG" 0 "nudge_falhou"
  expect_count "$NODE_LOG" 1 "op_6/8_pick_5: alvo fora de alcance (sugestao -0.25 m) — nudge -0.25 m (2/6, total +0.12 m)"
  expect_count "$NODE_LOG" 1 "[AMT1] nudge -0.25 m ok: andou -0.21"
  expect_count "$NODE_LOG" 2 "[AMT1] re-registro aplicado: delta ("
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_6"
  expect_count "$NODE_LOG" 2 "PickTag{tag_frame=tag_5"
  expect_count "$NODE_LOG" 1 "fail_reason='' observed=[6, 2, 3, 1, 4, 5] final=[1, 2, 3, 4, 5, 6] missing=[] picks=4 places=4 nudges=2 total=-0.09 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 2 (busca 0, ops 2)"
  expect_count "$TABLE_LOG" 1 "'pick' FORA DE ALCANCE tag_6: x_manip -0.350 m"
  expect_count "$TABLE_LOG" 1 "'pick' tag_6 ok (x_manip -0.230 m)"
  node_alive short_course
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  expect_count "$TABLE_LOG" 1 "| shift -0.092 m"
  containers_all_empty short_course

  # Variante na busca: o 1o nudge da busca (+0.20) e parcial (+0.160): o
  # nudgeTo conta o passo (1/8), "sigo para o alvo" e, como resta 0.040
  # (< 0.08), da por chegado; a tag_5 aparece de +0.160 m. O resto segue
  # como o caso search (6 nudges: 4 da busca + 2 indo para onde viu as tags).
  start_bench short_course_search "$LAYOUT_SEARCH" "-p view_x_max:=0.25 -p reach_x_max:=0.40" \
    "" "-p nudge_slip:=0.85 -p nudge_short_course:=1" || { settle; return; }
  send_goal short_course_search "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal (busca com passo parcial)"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$GOAL_LOG" 1 "nudges: 6"
  expect_count "$GOAL_LOG" 1 "missing_tags: []"
  expect_count "$LOG_DIR/short_course_search.fake_nav.log" 1 "'nudge' CURSO INCOMPLETO (fake, nudge_short_course): pedido +0.200 m, andou +0.160 m"
  expect_count "$LOG_DIR/short_course_search.fake_nav.log" 6 "goal aceito em 'nudge'"
  expect_count "$NODE_LOG" 1 "[AMT1] search_nudge: base em +0.000 m, alvo +0.20 m (resta +0.200 m) — nudge +0.20 m (1/8, total +0.00 m)."
  expect_count "$NODE_LOG" 1 "[AMT1] nudge +0.20 m PARCIAL (curso_incompleto): andou +0.160 m (total +0.160 m, 1/8 busca"
  expect_count "$NODE_LOG" 1 "[AMT1] search_nudge: passo PARCIAL (curso_incompleto): pedi +0.20 m, andou +0.160 m — sigo para o alvo +0.20 m."
  expect_count "$NODE_LOG" 1 "[AMT1] search_nudge: base em +0.160 m, alvo +0.20 m (resta +0.040 m): chegou (1 passo(s))."
  expect_count "$NODE_LOG" 1 "[AMT1] busca 1/2 em +0.160 m: 5 tag(s) acumulada(s), faltam [6]."
  expect_count "$NODE_LOG" 1 "6 tag(s) acumulada(s), faltam []."
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s)"
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5: indo para onde vi a tag_5:"
  expect_count "$NODE_LOG" 1 "[AMT1] op_4/6_pick_4: indo para onde vi a tag_4:"
  expect_count "$NODE_LOG" 0 "FALHOU"
  expect_count "$NODE_LOG" 0 "nudge_falhou"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$NODE_LOG" 1 "final=[1, 2, 3, 4, 5, 6] missing=[] picks=3 places=3 nudges=6"
  expect_count "$NODE_LOG" 1 "nudges 6 (busca 6, ops 0)"
  node_alive short_course_search
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty short_course_search
}

case_unseen() {
  # tag_3 NUNCA entra no TF (unseen_tags): a busca lateral roda nas 2
  # posicoes (4 nudges, as 5 tags vistas continuam acumuladas) e nao acha.
  # allow_partial=false (default desde 2026-08-28: na AMT1 as 6 tags sao
  # obrigatorias) => tag_nao_encontrada, missing [3], nenhum cubo movido.
  # 2026-08-29: allow_partial passou a true por default (operador: ordena o
  # que viu); este caso cobre o modo ESTRITO, por isso forca false.
  start_bench unseen "$LAYOUT_MAIN" "-p unseen_tags:=['tag_3']" "-p allow_partial:=false" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal unseen "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: tag_nao_encontrada"
  expect_count "$GOAL_LOG" 1 "picks: 0"
  expect_count "$GOAL_LOG" 1 "places: 0"
  expect_count "$GOAL_LOG" 1 "nudges: 4"
  expect_count "$GOAL_LOG" 1 "current_stage: searching_1"
  expect_count "$GOAL_LOG" 1 "current_stage: search_return_center"
  expect_count "$NODE_LOG" 1 "allow_partial=false busca=true offsets=[+0.20, -0.20]"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral: faltam [3] apos a observacao inicial — 2 posicao(oes) [+0.20, -0.20] m (base em +0.000 m, poses [tag_direita, tag_esquerda])."
  expect_search_trail "$NODE_LOG" "5 tag(s) acumulada(s), faltam [3]." "5 tag(s) acumulada(s), faltam [3]."
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam [3]."
  expect_count "$NODE_LOG" 1 "[AMT1] vistas 5 tag(s): ordem [5, 2, 1, 4, 6] alvo [1, 2, 4, 5, 6] faltam [3] ignoradas []"
  expect_count "$NODE_LOG" 1 "[AMT1] tags esperadas nao vistas: [3]."
  expect_count "$NODE_LOG" 1 "[AMT1] faltam as tags [3] mesmo apos a busca lateral (2 posicao(oes)) e allow_partial=false: nenhum cubo movido"
  expect_count "$NODE_LOG" 0 "[AMT1] plano:"
  expect_count "$NODE_LOG" 0 "PickTag{"
  expect_count "$NODE_LOG" 0 "recover_onboard"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=true fail_reason='tag_nao_encontrada' observed=[5, 2, 1, 4, 6] final=[] missing=[3] picks=0 places=0 nudges=4 total=-0.03 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 4 (busca 4, ops 0)"
  expect_count "$NODE_LOG" 0 "re-registro"
  expect_count "$LOG_DIR/unseen.fake_nav.log" 4 "goal aceito em 'nudge'"
  node_alive unseen
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [5, 2, 3, 1, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  expect_count "$TABLE_LOG" 0 "goal aceito em 'pick'"
  containers_all_empty unseen

  # Variante allow_partial:=true: a busca roda do mesmo jeito (4 nudges) e
  # depois o no ordena so o que viu, como antes de 2026-08-28. A base fica
  # em -0.026 m (tag_5 em x_manip -0.276): reach_x_max 0.30 para nao
  # depender do milimetro. Os slots continuam no x do inicio (T0). Como a
  # busca moveu a base, o no re-registra os slots do centro (5 ancoras,
  # todas visiveis) — e isso atualiza "onde vi" de todas as tags para
  # -0.026 m, logo nenhum pick precisa ir a outro lugar (0 goto).
  start_bench unseen_partial "$LAYOUT_MAIN" "-p unseen_tags:=['tag_3'] -p reach_x_max:=0.30" \
    "-p allow_partial:=true" "-p nudge_slip:=0.85" || { settle; return; }
  send_goal unseen_partial "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal (allow_partial:=true)"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: ''"
  expect_count "$GOAL_LOG" 1 "picks: 3"
  expect_count "$GOAL_LOG" 1 "places: 3"
  expect_count "$GOAL_LOG" 1 "nudges: 4"
  expect_count "$NODE_LOG" 1 "allow_partial=true busca=true offsets=[+0.20, -0.20]"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral terminada: 2 posicao(oes), 4 nudge(s), base em -0.026 m, faltam [3]."
  expect_count "$NODE_LOG" 1 "[AMT1] vistas 5 tag(s): ordem [5, 2, 1, 4, 6] alvo [1, 2, 4, 5, 6] faltam [3] ignoradas []"
  expect_count "$NODE_LOG" 1 "[AMT1] tags esperadas nao vistas: [3]."
  expect_count "$NODE_LOG" 0 "nenhum cubo movido"
  expect_count "$NODE_LOG" 1 "[AMT1] slot 0 = tag_amt1_slot_0 <- tag_5  ref(x=-0.250 y=0.350 z=0.142)"
  expect_count "$NODE_LOG" 1 "[AMT1] busca lateral moveu a base (voltou ao centro, base em -0.026 m): re-registrando os slots pelas tags intactas."
  expect_count "$NODE_LOG" 1 "m em odom com 5 ancora(s), espalhamento 0.000 m. Ancoras originais: 5"
  expect_count "$NODE_LOG" 1 "[AMT1] re-registro aplicado: delta ("
  expect_count "$NODE_LOG" 0 "indo para onde vi"
  expect_count "$NODE_LOG" 1 "[AMT1] plano: 6 op(s), 3 pick(s), 3 place(s), buffer 3: pick_5 pick_1 place_1_slot_0 pick_4 place_4_slot_2 place_5_slot_3"
  expect_count "$NODE_LOG" 0 "alvo fora de alcance"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=true fail_reason='' observed=[5, 2, 1, 4, 6] final=[1, 2, 4, 5, 6] missing=[3] picks=3 places=3 nudges=4 total=-0.03 m braco=conhecido"
  expect_count "$NODE_LOG" 1 "nudges 4 (busca 4, ops 0)"
  expect_count "$NODE_LOG" 1 "ordenadas as tags vistas; faltaram [3]"
  node_alive unseen_partial
  settle
  # tag_3 fica parada em x=-0.05 entre o slot 1 (-0.15) e o slot 2 (+0.05).
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty unseen_partial
}

case_fail_place() {
  # 1o place (op_3 place_1_slot_0) responde skipped+unreachable com sugestao
  # 0 (base_nao_vista, o ramo PP do place real) 2x: o no re-registra os
  # slots (4 ancoras originais: 2,3,4,6) e repete 1x, depois fora_de_alcance.
  # Garra vazia => recoverOnboard devolve os 2 cubos a bordo aos slots livres
  # (mais perto de x=0 primeiro): tag_5 -> slot 3 (+0.05), tag_1 -> slot 0
  # (-0.25). Mesa consistente [1, 2, 3, 5, 4, 6], 0 a bordo, partial=true.
  start_bench fail_place "$LAYOUT_MAIN" \
    "-p fail_stage:=place -p fail_times:=2 -p fail_mode:=unreachable" "" || { settle; return; }
  send_goal fail_place "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: fora_de_alcance"
  expect_count "$GOAL_LOG" 1 "picks: 2"
  expect_count "$GOAL_LOG" 1 "places: 2"
  expect_count "$NODE_LOG" 2 "op_3/6_place_1_slot_0: PlaceTag{tag_frame=tag_1 table_pose=Mesa10 ws=PP_1 stack_on=tag_amt1_slot_0 final_attempt=false}"
  expect_count "$NODE_LOG" 1 "[AMT1] op_3/6_place_1_slot_0: place sem sugestao de deslocamento (base_nao_vista) — re-registrando os slots e repetindo 1x."
  expect_count "$NODE_LOG" 1 "m em odom com 4 ancora(s), espalhamento 0.000 m. Ancoras originais: 4"
  expect_count "$NODE_LOG" 1 "[AMT1] op_3/6_place_1_slot_0 falhou: fora_de_alcance"
  expect_count "$NODE_LOG" 1 "stage=recover_onboard"
  expect_count "$NODE_LOG" 1 "recover_place_5_slot_3: PlaceTag{tag_frame=tag_5 table_pose=Mesa10 ws=PP_1 stack_on=tag_amt1_slot_3 final_attempt=false}"
  expect_count "$NODE_LOG" 1 "recover_place_1_slot_0: PlaceTag{tag_frame=tag_1 table_pose=Mesa10 ws=PP_1 stack_on=tag_amt1_slot_0 final_attempt=false}"
  expect_count "$NODE_LOG" 0 "continua a bordo"
  expect_count "$NODE_LOG" 0 "abortou/estourou"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=true fail_reason='fora_de_alcance' observed=[5, 2, 3, 1, 4, 6] final=[1, 2, 3, 5, 4, 6] missing=[] picks=2 places=2 nudges=0 total=+0.00 m braco=conhecido"
  expect_count "$TABLE_LOG" 2 "'place' PULADO tag_1: base tag_amt1_slot_0 nao vista (fail_mode=unreachable); cubo fica no container."
  node_alive fail_place
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 5, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty fail_place
}

case_abort_place() {
  # 1o place (op_3 place_1_slot_0) ABORTA: o cubo pode estar na garra, entao
  # o no NAO devolve nada (um place em pegar_obj deixaria o cubo cair): os 2
  # cubos a bordo (5 e 1) ficam nos containers com o yaml preservado,
  # partial=false, e o no continua vivo para o operador intervir.
  start_bench abort_place "$LAYOUT_MAIN" "-p fail_stage:=place -p fail_times:=1" "" || { settle; return; }
  send_goal abort_place "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: false"
  expect_count "$GOAL_LOG" 1 "fail_reason: place_falhou"
  expect_count "$GOAL_LOG" 1 "picks: 2"
  expect_count "$GOAL_LOG" 1 "places: 0"
  expect_count "$GOAL_LOG" 1 "2 cubo(s) a bordo [5, 1] ficam nos containers (memoria yaml preservada); cubo possivelmente na garra apos falha_simulada"
  expect_count "$NODE_LOG" 1 "[AMT1] op_3/6_place_1_slot_0 falhou: place_falhou — op_3/6_place_1_slot_0: place abortou (falha_simulada): falha simulada (fake)"
  expect_count "$NODE_LOG" 1 "[AMT1] place_falhou abortou/estourou o prazo (falha_simulada): o cubo pode estar na garra — NAO solto nada. 2 cubo(s) a bordo [5, 1] ficam nos containers (memoria yaml preservada); so devolvo o braco a pegar_obj."
  expect_count "$NODE_LOG" 0 "recover_onboard"
  expect_count "$NODE_LOG" 0 "recover_place"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=false fail_reason='place_falhou' observed=[5, 2, 3, 1, 4, 6] final=[0, 2, 3, 0, 4, 6] missing=[] picks=2 places=0 nudges=0"
  expect_count "$TABLE_LOG" 1 "'place' abortado (fail_stage)"
  node_alive abort_place
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [2, 3, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: ['tag_5', 'tag_1']"
  expect_containers 2 abort_place
}

case_timeout_child() {
  # Mesa lenta (9 s por pick, 3 s por estagio) e op_timeout_s 2 s no no: o
  # 1o pick estoura o prazo, o no cancela o goal filho (o fake responde
  # CANCELED em "approach", tag_5 fica na mesa) e responde pick_falhou com
  # SUCCEEDED — sem canceled() fora de CANCELING (que mataria o no). Prova
  # de vida: um 2o goal observe_only no mesmo no termina com sucesso.
  start_bench timeout_child "$LAYOUT_MAIN" "-p latency_sec:=9.0" "-p op_timeout_s:=2.0" || { settle; return; }
  send_goal timeout_child "$GOAL"
  expect_exit "$GOAL_RC" 0 "send_goal"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: false"
  expect_count "$GOAL_LOG" 1 "partial: true"
  expect_count "$GOAL_LOG" 1 "fail_reason: pick_falhou"
  expect_count "$GOAL_LOG" 1 "picks: 0"
  expect_count "$GOAL_LOG" 1 "places: 0"
  expect_count "$NODE_LOG" 1 "[AMT1] op_1/6_pick_5 falhou: pick_falhou — op_1/6_pick_5: pick estourou o prazo (cancelado): cancelado (fake)"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=true fail_reason='pick_falhou' observed=[5, 2, 3, 1, 4, 6] final=[5, 2, 3, 1, 4, 6] missing=[] picks=0 places=0 nudges=0"
  expect_count "$NODE_LOG" 0 "recover_onboard"
  expect_count "$NODE_LOG" 0 "terminate"
  expect_count "$TABLE_LOG" 1 "cancel recebido em pick (approach)"
  node_alive "timeout_child, apos o goal que estourou"
  send_goal timeout_child_2 "${GOAL/observe_only: false/observe_only: true}"
  expect_exit "$GOAL_RC" 0 "2o send_goal (observe_only)"
  expect_count "$GOAL_LOG" 1 "Goal finished with status: SUCCEEDED"
  expect_count "$GOAL_LOG" 1 "success: true"
  expect_count "$NODE_LOG" 1 "observe_only=true"
  expect_count "$NODE_LOG" 2 "[AMT1] plano: 6 op(s), 3 pick(s), 3 place(s), buffer 3: pick_5 pick_1 place_1_slot_0 pick_4 place_4_slot_3 place_5_slot_4"
  node_alive "timeout_child, apos o 2o goal"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [5, 2, 3, 1, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: []"
  containers_all_empty timeout_child
}

# Cliente proprio para o caso de cancel: o `ros2 action send_goal` do Jazzy
# quebra ao cancelar por SIGINT ("Executor is already spinning"). Manda o
# goal, cancela quando o feedback chega ao estagio <cancel_on> e imprime o
# result no mesmo formato do CLI.
# cancel_client <log> <goal yaml> <estagio que dispara o cancel>
cancel_client() {
  local log="$1" goal="$2" cancel_on="$3"
  timeout 180 python3 - "$goal" "$cancel_on" > "$log" 2>&1 <<'EOF'
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rosidl_runtime_py import message_to_yaml, set_message_fields

from my_robot_msgs.action import Amt1Sort

goal_dict = yaml.safe_load(sys.argv[1])
cancel_on = sys.argv[2]
rclpy.init()
node = rclpy.create_node("amt1_cancel_client")
client = ActionClient(node, Amt1Sort, "/amt1_sort")
if not client.wait_for_server(timeout_sec=20.0):
    print("Action server is not available", flush=True)
    sys.exit(2)
goal = Amt1Sort.Goal()
set_message_fields(goal, goal_dict)
state = {"want_cancel": False, "cancel_future": None, "reported": False}


def on_feedback(msg):
    stage = msg.feedback.current_stage
    print(f"Feedback: {stage}", flush=True)
    if stage.startswith(cancel_on):
        state["want_cancel"] = True


print("Sending goal:\n     " + message_to_yaml(goal), flush=True)
goal_future = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, goal_future, timeout_sec=20.0)
handle = goal_future.result()
if handle is None or not handle.accepted:
    print("Goal was rejected.", flush=True)
    sys.exit(3)
print("Goal accepted with ID: " + bytes(handle.goal_id.uuid).hex(), flush=True)
result_future = handle.get_result_async()
deadline = time.monotonic() + 150.0
while not result_future.done() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if state["want_cancel"] and state["cancel_future"] is None:
        print("Canceling goal...", flush=True)
        state["cancel_future"] = handle.cancel_goal_async()
    cf = state["cancel_future"]
    if cf is not None and cf.done() and not state["reported"]:
        state["reported"] = True
        resp = cf.result()
        if resp is not None and len(resp.goals_canceling) == 1:
            print("Goal canceled.", flush=True)
        else:
            print("Failed to cancel goal", flush=True)
if not result_future.done():
    print("Timed out waiting for result", flush=True)
    sys.exit(4)
wrapped = result_future.result()
names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
         GoalStatus.STATUS_ABORTED: "ABORTED"}
print("Result:\n    " + message_to_yaml(wrapped.result), flush=True)
print("Goal finished with status: " + names.get(wrapped.status, "UNKNOWN"), flush=True)
node.destroy_node()
rclpy.shutdown()
EOF
  return $?
}

case_cancel() {
  # Mesa lenta (2 s por pick/place); o cancel e disparado pelo feedback
  # "op_2/6_pick_1:grasp" (2o pick, no meio da acao filha): o no propaga ao
  # goal filho, NAO recupera (cancelado) e responde CANCELED. tag_5 fica a
  # bordo, tag_1 fica na mesa.
  start_bench cancel "$LAYOUT_MAIN" "-p latency_sec:=2.0" "" || { settle; return; }
  GOAL_LOG="$LOG_DIR/cancel.goal.log"
  cancel_client "$GOAL_LOG" "$GOAL" "op_2/6_pick_1:grasp"
  expect_exit $? 0 "cliente de cancel"
  expect_count "$GOAL_LOG" 1 "Feedback: op_2/6_pick_1:grasp"
  expect_count "$GOAL_LOG" 1 "Canceling goal..."
  expect_count "$GOAL_LOG" 1 "Goal canceled."
  expect_count "$GOAL_LOG" 1 "Goal finished with status: CANCELED"
  expect_count "$GOAL_LOG" 1 "fail_reason: cancelado"
  expect_count "$GOAL_LOG" 1 "picks: 1"
  expect_count "$GOAL_LOG" 1 "places: 0"
  expect_count "$NODE_LOG" 1 "[AMT1] cancelamento pedido — propagando ao goal filho e parando o braco."
  expect_count "$NODE_LOG" 1 "[AMT1] op_2/6_pick_1 falhou: cancelado — op_2/6_pick_1 cancelado"
  expect_count "$NODE_LOG" 1 "fim: success=false partial=false fail_reason='cancelado' observed=[5, 2, 3, 1, 4, 6] final=[0, 2, 3, 1, 4, 6] missing=[] picks=1 places=0"
  expect_count "$NODE_LOG" 1 "stage=canceled"
  expect_count "$NODE_LOG" 0 "recover_onboard"
  expect_count "$TABLE_LOG" 1 "cancel recebido em pick (grasp)"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [2, 3, 1, 4, 6]"
  expect_count "$TABLE_LOG" 1 "a bordo: ['tag_5']"
}

case_mission() {
  start_bench mission "$LAYOUT_MAIN" "" "" || { settle; return; }
  run_executor mission "$ACTIONS" ""
  expect_exit "$CASE_RC" 0 "bt_yaml_executor amt1_actions.yaml (no real)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ordenar 6 tag(s) [1,2,3,4,5,6] na mesa Mesa10, left_to_right, max 3 a bordo; prazo 1800 s (watchdog de feedback 300 s)..."
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] estagio: op_1/6_pick_5 (0/6 ops)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] estagio: done (6/6 ops)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ORDENACAO CONCLUIDA: ordem inicial [5,2,3,1,4,6] -> final [1,2,3,4,5,6]; 3 pick(s), 3 place(s), 0 nudge(s)"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
  expect_count "$NODE_LOG" 1 "[AMT1] goal aceito: ws=PP_1 table_pose=Mesa10 expected=[1, 2, 3, 4, 5, 6] direction=left_to_right max_onboard=3 observe_only=false"
  expect_count "$NODE_LOG" 1 "fim: success=true partial=false fail_reason=''"
  expect_count "$LOG_DIR/mission.fake_nav.log" 1 "/fake_nav/station = 'PP1'"
  settle
  expect_count "$TABLE_LOG" 1 "ordem final: [1, 2, 3, 4, 5, 6]"
  containers_all_empty mission
}

# ---------------------------------------------------------------- so executor
# start_sort_bench <caso> <params fake_sort> <params fake_nav>
start_sort_bench() {
  local name="$1" sort_params="$2" nav_params="${3:-}"
  echo "=== caso: $name ==="
  start_fake_nav "$name" "$nav_params"
  start_fake_sort "$name" "$sort_params"
  if ! wait_for_action "/align_to_dock" || ! wait_for_action "/amt1_sort"; then
    fail "fakes nao apareceram no grafo (ROS_DOMAIN_ID=99)"
    return 1
  fi
  sleep 1
  return 0
}

case_bench_ok() {
  start_sort_bench bench_ok "-p latency_sec:=3.0" || { settle; return; }
  run_executor bench_ok "$ACTIONS" ""
  expect_exit "$CASE_RC" 0 "bt_yaml_executor (fake_amt1_sort ok)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ordenar 6 tag(s) [1,2,3,4,5,6] na mesa Mesa10, left_to_right, max 3 a bordo; prazo 1800 s (watchdog de feedback 300 s)..."
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] estagio: starting (0/6 ops)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] estagio: op_6/6_place_5_slot_4 (5/6 ops)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] estagio: done (6/6 ops)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ORDENACAO CONCLUIDA: ordem inicial [5,2,3,1,4,6] -> final [1,2,3,4,5,6]; 3 pick(s), 3 place(s), 0 nudge(s), deslocamento total 0.000 m"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
  expect_count "$SORT_LOG" 1 "goal aceito: ws=PP_1 table_pose=Mesa10 expected=[1, 2, 3, 4, 5, 6] direction='left_to_right' max_onboard=3 observe_only=False"
  expect_count "$SORT_LOG" 1 "goal concluido com sucesso (fake)"
  settle
}

case_bench_fail() {
  start_sort_bench bench_fail "-p latency_sec:=1.0 -p fail_stage:=abort" || { settle; return; }
  run_executor bench_fail "$ACTIONS" ""
  expect_exit "$CASE_RC" 1 "bt_yaml_executor (fake aborta)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] goal /amt1_sort ABORTADO: pick abortou (fake) (fail_reason=pick_falhou)"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with FAILURE"
  expect_count "$CASE_LOG" 0 "ORDENACAO CONCLUIDA"
  expect_count "$SORT_LOG" 1 "goal ABORTADO (fail_stage:=abort)"
  settle
}

case_bench_result() {
  start_sort_bench bench_result "-p latency_sec:=1.0 -p fail_stage:=result" || { settle; return; }
  run_executor bench_result "$ACTIONS" ""
  expect_exit "$CASE_RC" 0 "bt_yaml_executor (fake devolve success=false)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ordenacao INCOMPLETA (parcial, fail_reason=fora_de_alcance): op_2/6_pick_1: fora de alcance (fake)"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
  expect_count "$CASE_LOG" 0 "ORDENACAO CONCLUIDA"
  settle
}

case_bench_reject() {
  start_sort_bench bench_reject "-p reject:=true" || { settle; return; }
  run_executor bench_reject "$ACTIONS" ""
  expect_exit "$CASE_RC" 1 "bt_yaml_executor (fake rejeita)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] goal /amt1_sort REJEITADO pelo server."
  expect_count "$CASE_LOG" 1 "Behavior tree finished with FAILURE"
  expect_count "$SORT_LOG" 1 "goal REJEITADO (reject:=true)"
  settle
}

case_bench_watchdog() {
  start_sort_bench bench_watchdog "-p silent:=true -p latency_sec:=60.0" || { settle; return; }
  run_executor bench_watchdog "$ACTIONS" "-p amt1_stage_timeout:=4.0"
  expect_exit "$CASE_RC" 1 "bt_yaml_executor (server mudo, watchdog 4 s)"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] WATCHDOG: 4 s sem feedback do /amt1_sort (ultimo estagio: <nenhum>) — server pendurado. Cancelando o goal."
  expect_count "$CASE_LOG" 0 "server nao confirmou o cancelamento"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with FAILURE"
  expect_count "$SORT_LOG" 1 "cancel recebido em"
  settle
}

case_bench_deadline() {
  # Achado 8: o prazo total (amt1_timeout) so AVISA uma vez e o goal segue
  # ate o fim (fake com ~3 s de latencia e prazo de 2 s). Cancelar no meio
  # de um pick/place seria pior que esperar.
  start_sort_bench bench_deadline "-p latency_sec:=3.0" || { settle; return; }
  run_executor bench_deadline "$ACTIONS" "-p amt1_timeout:=2.0"
  expect_exit "$CASE_RC" 0 "bt_yaml_executor (prazo total 2 s estourado, fake continua)"
  expect_count "$CASE_LOG" 1 "max 3 a bordo; prazo 2 s (watchdog de feedback 300 s)..."
  expect_count "$CASE_LOG" 1 "prazo total de 2 s esgotado (ultimo estagio: "
  expect_count "$CASE_LOG" 1 "— NAO cancelando: o server ainda manda feedback; o goal so sera cancelado se o feedback parar por 300 s (watchdog)."
  expect_count "$CASE_LOG" 0 "WATCHDOG"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] ORDENACAO CONCLUIDA: ordem inicial [5,2,3,1,4,6] -> final [1,2,3,4,5,6]"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
  expect_count "$SORT_LOG" 0 "cancel recebido"
  expect_count "$SORT_LOG" 1 "goal concluido com sucesso (fake)"
  settle
  # Fake mudo com o prazo total ja estourado: quem cancela e SO o watchdog
  # de feedback, e a mensagem dele ganha o sufixo do prazo.
  start_sort_bench bench_deadline_wd "-p silent:=true -p latency_sec:=60.0" || { settle; return; }
  run_executor bench_deadline_wd "$ACTIONS" "-p amt1_timeout:=2.0 -p amt1_stage_timeout:=4.0"
  expect_exit "$CASE_RC" 1 "bt_yaml_executor (server mudo, prazo 2 s + watchdog 4 s)"
  expect_count "$CASE_LOG" 1 "prazo total de 2 s esgotado (ultimo estagio: <nenhum>) — NAO cancelando"
  expect_count "$CASE_LOG" 1 "[Amt1Sort PP_1] WATCHDOG: 4 s sem feedback do /amt1_sort (ultimo estagio: <nenhum>) — server pendurado. Cancelando o goal. Prazo total de 2 s tambem ja estava esgotado."
  expect_count "$CASE_LOG" 1 "Behavior tree finished with FAILURE"
  expect_count "$SORT_LOG" 1 "cancel recebido em"
  settle
}

run_all() {
  case_planner; case_regression; case_dry_run
  case_main; case_two_cycles; case_sorted; case_nudge; case_b1; case_search
  case_search_budget; case_search_pick_view; case_short_course; case_unseen
  case_fail_place; case_abort_place; case_timeout_child; case_cancel; case_mission
  case_bench_ok; case_bench_fail; case_bench_result; case_bench_reject; case_bench_watchdog
  case_bench_deadline
}

case "$MODE" in
  planner) case_planner ;;
  regression) case_regression ;;
  dry_run) case_dry_run ;;
  main) case_main ;;
  two_cycles) case_two_cycles ;;
  sorted) case_sorted ;;
  nudge) case_nudge ;;
  b1) case_b1 ;;
  search) case_search ;;
  search_budget) case_search_budget ;;
  search_pick_view) case_search_pick_view ;;
  short_course) case_short_course ;;
  unseen) case_unseen ;;
  fail_place) case_fail_place ;;
  abort_place) case_abort_place ;;
  timeout_child) case_timeout_child ;;
  cancel) case_cancel ;;
  mission) case_mission ;;
  bench_ok) case_bench_ok ;;
  bench_fail) case_bench_fail ;;
  bench_result) case_bench_result ;;
  bench_reject) case_bench_reject ;;
  bench_watchdog) case_bench_watchdog ;;
  bench_deadline) case_bench_deadline ;;
  all) run_all ;;
  *)
    echo "modo desconhecido: $MODE (all|planner|regression|dry_run|main|two_cycles|sorted|nudge|b1|search|search_budget|search_pick_view|short_course|unseen|fail_place|abort_place|timeout_child|cancel|mission|bench_ok|bench_fail|bench_result|bench_reject|bench_watchdog|bench_deadline)"
    exit 2 ;;
esac

if [ "$FAILURES" -eq 0 ]; then
  echo "=== TUDO OK (logs em $LOG_DIR) ==="
  exit 0
fi
echo "=== $FAILURES verificacao(oes) falharam (logs em $LOG_DIR) ==="
exit 1
