#!/usr/bin/env bash
# Cenario de bancada da FILA DE ALCANCE (2026-08-28), sem robo e sem MoveIt.
#
# Roda em ROS_DOMAIN_ID=99 (nunca encosta no robo real): sobe os fakes de
# navegacao (fake_nav_actions.py, com nudge_base) e de manipulacao
# (fake_pick_place.py, com tags "fora de alcance"), roda o bt_yaml_executor
# com navegacao REAL (GoToWS/NudgeBase falando com os fakes) sobre
# reach_queue_actions.yaml e confere por grep as linhas esperadas do
# ReachQueue. Codigo de saida 0 so se TODOS os casos baterem.
#
# Uso: run_reach_queue_scenario.sh [all|main|fail_pick|unseen|fail_nudge]
#   main       : ADIADO -> AJUSTE +0.20 -> ok -> AJUSTE +0.25 -> PASSAGEM FINAL -> SUCCESS
#   fail_pick  : fail_stage:=pick no fake -> FAILURE (exit 1), como hoje
#   unseen     : so tags nunca vistas -> pular sem ADIADO (comportamento antigo)
#   fail_nudge : fail_stage:=nudge -> PASSAGEM FINAL sem mover e SUCCESS
# Logs: $REACH_QUEUE_LOG_DIR (default /tmp/reach_queue_scenario).
export ROS_DOMAIN_ID=99
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
# Os setup.bash do ROS usam variaveis nao definidas: sourcear ANTES do set -u.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

MAPS="$WS_ROOT/src/caramelo_mapping/maps"
MAP="$MAPS/arena3_520"
[ -d "$MAP" ] || MAP="$MAPS/arenalegal_2026"
LOG_DIR="${REACH_QUEUE_LOG_DIR:-/tmp/reach_queue_scenario}"
mkdir -p "$LOG_DIR"
ACTIONS="$HERE/reach_queue_actions.yaml"
MODE="${1:-all}"

FAKE_PIDS=()
# SIGINT (nao SIGTERM): os fakes fecham o rclpy direito e o participante DDS
# anuncia a saida. Matar seco deixava locators velhos no FastDDS e o caso
# seguinte via goals duplicados / respostas de "rejeitado" fantasmas.
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
  got=$(grep -c -F -- "$text" "$log")
  if [ "$got" -eq "$want" ]; then
    ok "$want x '$text'"
  else
    fail "esperava $want x '$text', achei $got"
  fi
}

wait_for_action() {
  local name="$1" tries=30
  while [ $tries -gt 0 ]; do
    if ros2 action list 2>/dev/null | grep -q -F -- "$name"; then
      return 0
    fi
    tries=$((tries - 1))
    sleep 1
  done
  return 1
}

# run_case <nome> <exit esperado> <params fake_nav> <params fake_pick_place>
run_case() {
  local name="$1" want_exit="$2" nav_params="$3" pp_params="$4"
  local log="$LOG_DIR/$name.log"
  echo "=== caso: $name ==="
  # shellcheck disable=SC2086
  python3 "$HERE/fake_nav_actions.py" --ros-args -p latency_sec:=0.3 $nav_params \
    > "$LOG_DIR/$name.fake_nav.log" 2>&1 &
  FAKE_PIDS+=("$!")
  # shellcheck disable=SC2086
  python3 "$HERE/fake_pick_place.py" --ros-args -p latency_sec:=0.3 $pp_params \
    > "$LOG_DIR/$name.fake_pp.log" 2>&1 &
  FAKE_PIDS+=("$!")
  if ! wait_for_action "/nudge_base" || ! wait_for_action "/place_tag"; then
    fail "fakes nao apareceram no grafo (ROS_DOMAIN_ID=99)"
    cleanup
    return
  fi
  sleep 1
  timeout 240 ros2 run manip_bt bt_yaml_executor "$ACTIONS" --ros-args \
    -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
    -p finish_dock_id:="''" -p server_wait_timeout:=20.0 -p nudge_timeout:=10.0 \
    > "$log" 2>&1
  local rc=$?
  cleanup
  sleep 2   # deixa a descoberta DDS assentar antes do proximo caso
  if [ "$rc" -eq "$want_exit" ]; then
    ok "codigo de saida $rc"
  else
    fail "codigo de saida $rc (esperava $want_exit) — veja $log"
  fi
  CASE_LOG="$log"
}

MAIN_PP="-p unreachable_picks:=['tag_7:0.20','tag_8:0.60'] -p unreachable_places:=['tag_1:-0.15'] -p unseen_tags:=['tag_9']"

case_main() {
  run_case main 0 "-p nudge_slip:=0.85" "$MAIN_PP"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] ADIADO pick_2: fora de alcance, sugestao +0.20 m"
  expect_count "$CASE_LOG" 3 "[ReachQueue WS_1] ADIADO pick_3: fora de alcance, sugestao"
  expect_count "$CASE_LOG" 0 "ADIADO pick_4"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] AJUSTE LATERAL 1/2: +0.20 m (pick_2)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] AJUSTE LATERAL 2/2: +0.25 m (pick_3)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] PASSAGEM FINAL: 1 acao(oes) pendente(s)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] concluida (ajustes=2, total=+0.45 m)"
  expect_count "$CASE_LOG" 1 "PICK PULADO (causa: tag_nao_vista)"
  expect_count "$CASE_LOG" 1 "PICK PULADO (causa: fora_de_alcance_final)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] ADIADO place_6: fora de alcance, sugestao -0.15 m"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] AJUSTE LATERAL 1/2: -0.15 m (place_6)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] concluida (ajustes=1, total=-0.15 m)"
  expect_count "$CASE_LOG" 0 "[ReachQueue WS_2] PASSAGEM FINAL"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
}

case_fail_pick() {
  run_case fail_pick 1 "" "-p fail_stage:=pick $MAIN_PP"
  expect_count "$CASE_LOG" 1 "PICK action finished with non-success result code"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] pick_2 falhou (FAILURE)"
  expect_count "$CASE_LOG" 0 "AJUSTE LATERAL"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with FAILURE"
}

case_unseen() {
  run_case unseen 0 "" "-p unseen_tags:=['tag_7','tag_8','tag_9']"
  expect_count "$CASE_LOG" 0 "ADIADO"
  expect_count "$CASE_LOG" 0 "AJUSTE LATERAL"
  expect_count "$CASE_LOG" 0 "PASSAGEM FINAL"
  expect_count "$CASE_LOG" 3 "PICK PULADO (causa: tag_nao_vista)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] concluida (ajustes=0, total=+0.00 m)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] concluida (ajustes=0, total=+0.00 m)"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
}

case_fail_nudge() {
  run_case fail_nudge 0 "-p fail_stage:=nudge" "$MAIN_PP"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] AJUSTE LATERAL 1/2: +0.20 m (pick_2)"
  expect_count "$CASE_LOG" 0 "AJUSTE LATERAL 2/2"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] ajuste lateral de +0.20 m FALHOU"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] PASSAGEM FINAL: 2 acao(oes) pendente(s)"
  expect_count "$CASE_LOG" 2 "PICK PULADO (causa: fora_de_alcance_final)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_1] concluida (ajustes=0, total=+0.00 m)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] PASSAGEM FINAL: 1 acao(oes) pendente(s)"
  expect_count "$CASE_LOG" 1 "[ReachQueue WS_2] concluida (ajustes=0, total=+0.00 m)"
  expect_count "$CASE_LOG" 1 "Behavior tree finished with SUCCESS"
}

case "$MODE" in
  main) case_main ;;
  fail_pick) case_fail_pick ;;
  unseen) case_unseen ;;
  fail_nudge) case_fail_nudge ;;
  all) case_main; case_fail_pick; case_unseen; case_fail_nudge ;;
  *) echo "modo desconhecido: $MODE (all|main|fail_pick|unseen|fail_nudge)"; exit 2 ;;
esac

if [ "$FAILURES" -eq 0 ]; then
  echo "=== TUDO OK (mapa $(basename "$MAP"), logs em $LOG_DIR) ==="
  exit 0
fi
echo "=== $FAILURES verificacao(oes) falharam (logs em $LOG_DIR) ==="
exit 1
