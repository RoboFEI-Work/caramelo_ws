#!/usr/bin/env bash
# Bancada da AMT refeita (2026-11-16), SEM robo: arquivo da competicao ->
# task_planner -> bt_yaml_executor -> amt_sort_node (o no' de verdade) ->
# mesa falsa (fake_amt_mesa.py: camera sintetica, pick/place/nudge).
# Confere a ordem FINAL dos blocos na mesa. Uso: run_amt_scenario.sh
# NAO rodar com o robo no ar: usa os mesmos nomes de action, em ROS_DOMAIN_ID=99.
export ROS_DOMAIN_ID=99
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

MAP="$WS_ROOT/src/caramelo_mapping/maps/arenalegal_2026"
BRINGUP="$WS_ROOT/src/manipulation/manip_bringup"
TASK="$WS_ROOT/missions/tasks_geradas3/AMT_seed401715_equipe.yaml"
LOG_DIR="${AMT_LOG_DIR:-/tmp/amt_scenario}"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null; done
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] || continue
    for _ in 1 2 3 4 5 6; do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -KILL "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
  done
  PIDS=()
}
trap cleanup EXIT

FAILURES=0
fail() { echo "  [FALHOU] $*"; FAILURES=$((FAILURES + 1)); }
ok() { echo "  [ok] $*"; }
wait_for_action() {
  local tries=30
  while [ $tries -gt 0 ]; do
    ros2 action list 2>/dev/null | grep -q -F -- "$1" && return 0
    tries=$((tries - 1)); sleep 1
  done
  return 1
}

ACTIONS="$LOG_DIR/amt_actions.yaml"
if ! ros2 run manip_bt task_planner "$TASK" "$ACTIONS" \
    "$BRINGUP/config/tags_36h11.yaml" "$MAP/ws_table_mapping.yaml" > "$LOG_DIR/planner.log" 2>&1; then
  fail "task_planner nao traduziu $TASK"; cat "$LOG_DIR/planner.log"; exit 1
fi
grep -q "kind: amt1_sort" "$ACTIONS" && grep -q "expected_tags" "$ACTIONS" &&
  ok "planner: goto PP + amt1_sort ($(grep -F 'AMT1:' "$LOG_DIR/planner.log" | cut -c16-110))" ||
  fail "plano sem amt1_sort"

# caso <nome> <arranjo inicial> <estado final esperado> [parametros extras da mesa falsa]
caso() {
  local nome="$1" arranjo="$2" esperado="$3"; shift 3
  [ -n "${AMT_SO:-}" ] && [ "$AMT_SO" != "$nome" ] && return   # AMT_SO=<nome>: so' esse caso
  echo "== $nome: $arranjo -> $esperado"
  python3 "$HERE/fake_amt_mesa.py" --ros-args -p arranjo:="'$arranjo'" \
    -p potes_yaml:="$LOG_DIR/potes_$nome.yaml" "$@" \
    > "$LOG_DIR/mesa_$nome.log" 2>&1 &
  PIDS+=("$!")
  # busca_passo_m: a mesa falsa so' tem a vista de pegar_obj (5 dos 7 slots); no robo quem
  # cobre a fileira sao as olhadas laterais do braco e a busca fica desligada (0).
  python3 "$BRINGUP/scripts/amt_sort_node.py" --ros-args -p observar_s:=1.0 -p camera_espera_s:=6.0 \
    -p busca_passo_m:=0.15 \
    -p pausa_entre_ops_s:=0.1 -p container_state_file:="$LOG_DIR/potes_$nome.yaml" \
    > "$LOG_DIR/amt_$nome.log" 2>&1 &
  PIDS+=("$!")
  if ! wait_for_action "/amt1_sort" || ! wait_for_action "/place_tag"; then
    fail "$nome: servidores nao apareceram"; cleanup; return
  fi
  sleep 1
  timeout 300 ros2 run manip_bt bt_yaml_executor "$ACTIONS" --ros-args \
    -p map_folder:="$MAP" -p simulate_navigation:=true -p startup_home:=false \
    -p home_before_dock:=false -p finish_dock_id:="''" -p amt1_look_poses:="''" \
    -p server_wait_timeout:=20.0 -p pose_gate_enabled:=false -p stop_check_enabled:=false \
    > "$LOG_DIR/executor_$nome.log" 2>&1
  local rc=$?
  cleanup
  local fim
  fim=$(grep -o "ESTADO [0-9,]* bordo=\[[0-9, ]*\]" "$LOG_DIR/mesa_$nome.log" | tail -1)
  [ "$fim" = "ESTADO $esperado bordo=[]" ] && ok "$nome: $fim" || fail "$nome: terminou em '$fim'"
  [ $rc -eq 0 ] && ok "$nome: executor terminou com sucesso" || fail "$nome: executor saiu com $rc"
  grep -F "[AMT] ordem final" "$LOG_DIR/amt_$nome.log" | tail -1 | sed 's/.*\[AMT\]/    no:/'
  # 2026-11-17: o no' so' pode dizer "conferi" com a mesa em ordem (antes dizia que ordenou sem ter).
  local conferiu=nao
  grep -q -F "Conferi, a mesa" "$LOG_DIR/amt_$nome.log" && conferiu=sim
  [ "$conferiu" = "$([ "$esperado" = "1,2,3,4,5,6,0" ] && echo sim || echo nao)" ] &&
    ok "$nome: conferido pela camera = $conferiu ($(grep -c -F 'ordem final' "$LOG_DIR/amt_$nome.log") rodada(s))" ||
    fail "$nome: conferido pela camera = $conferiu com a mesa em '$fim'"
}

caso misturado "5,2,6,1,4,3,0" "1,2,3,4,5,6,0"
caso pontas    "6,2,3,4,5,0,1" "1,2,3,4,5,6,0"
caso deslocado "0,3,1,2,6,5,4" "1,2,3,4,5,6,0"
caso em_ordem  "1,2,3,4,5,6,0" "1,2,3,4,5,6,0"
# place do bloco 1 falha 2 vezes: ele fica no pote (nunca vai para outro slot), o resto segue, e a
# rodada seguinte entrega no slot certo.
caso place_falha "2,1,3,4,5,6,0" "1,2,3,4,5,6,0" -p "fail_places:=['tag_1:2']"
# perna da mesa: passo de base acima de 16 cm e' recusado inteiro; o no' tem de ir de 15 e 10 cm.
caso perna "5,2,6,1,4,3,0" "1,2,3,4,5,6,0" -p nudge_recusa_acima:=0.16
# sem /nudge_base a mesa falsa so' mostra os 5 slots do meio, sem tag de slot para ancorar a
# fila: o no' nao pode mexer em bloco com a posicao no chute (17/11: saia 1 slot deslocado).
caso sem_nudge "5,2,6,1,4,3,0" "5,2,6,1,4,3,0" -p sem_nudge:=true
# missao anterior interrompida (17/11): blocos 1 e 6 ja' nos potes de bordo, mesa meio ordenada.
caso a_bordo "0,2,3,4,5,0,0" "1,2,3,4,5,6,0" -p bordo:="'1,6'"

echo
if [ "$FAILURES" -eq 0 ]; then echo "=== TUDO OK (logs em $LOG_DIR) ==="; else echo "=== $FAILURES FALHA(S) (logs em $LOG_DIR) ==="; fi
exit "$FAILURES"
