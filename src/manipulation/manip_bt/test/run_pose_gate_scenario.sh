#!/usr/bin/env bash
# Cenario de bancada do GATE DE POSE + PARADA do GoToWS (2026-10-26), sem
# robo e sem MoveIt. Roda em ROS_DOMAIN_ID=99 (nunca encosta no robo real):
# sobe os fakes de navegacao (fake_nav_actions.py, com pose ruim nos
# primeiros aligns e /odom falso) e de manipulacao (fake_pick_place.py), roda
# o bt_yaml_executor com navegacao REAL sobre pose_gate_actions.yaml e
# confere por grep as linhas esperadas. Codigo de saida 0 so se TUDO bater.
#
# Uso: run_pose_gate_scenario.sh [all|good|retry|strict|lenient|moving|no_odom|disabled]
#   good     : poses boas -> "pose validada", "base parada", SUCCESS
#   retry    : 1o align ruim (WS3 de 23/10) -> re-dock -> 2o bom -> SUCCESS
#   strict   : align SEMPRE ruim, strict -> re-dock, depois pula a WS_1
#              (Fallback do executor) e segue para a WS_2 -> SUCCESS da missao
#              (exit 0) SEM o place da WS_1
#   lenient  : idem com pose_gate_strict:=false -> aviso e place acontece
#   moving   : /odom em movimento por 5 s apos o align (> timeout 3 s) ->
#              "AINDA EM MOVIMENTO", estacao pulada
#   no_odom  : fake sem /odom -> "sem odometria fresca", estacao pulada
#   disabled : pose_gate_enabled:=false + stop_check_enabled:=false ->
#              comportamento antigo (manipula com a pose ruim)
# Logs: $POSE_GATE_LOG_DIR (default /tmp/pose_gate_scenario).
export ROS_DOMAIN_ID=99
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

MAPS="$WS_ROOT/src/caramelo_mapping/maps"
MAP="$MAPS/arena3_520"
[ -d "$MAP" ] || MAP="$MAPS/arenalegal_2026"
LOG_DIR="${POSE_GATE_LOG_DIR:-/tmp/pose_gate_scenario}"
mkdir -p "$LOG_DIR"
ACTIONS="$HERE/pose_gate_actions.yaml"
MODE="${1:-all}"

FAKE_PIDS=()
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

# run_case <nome> <exit esperado> <params fake_nav> <params executor>
run_case() {
  local name="$1" want_exit="$2" nav_params="$3" exec_params="$4"
  local log="$LOG_DIR/$name.log"
  echo "=== caso: $name ==="
  # shellcheck disable=SC2086
  python3 "$HERE/fake_nav_actions.py" --ros-args -p latency_sec:=0.3 $nav_params \
    > "$LOG_DIR/$name.fake_nav.log" 2>&1 &
  FAKE_PIDS+=("$!")
  python3 "$HERE/fake_pick_place.py" --ros-args -p latency_sec:=0.3 \
    > "$LOG_DIR/$name.fake_pp.log" 2>&1 &
  FAKE_PIDS+=("$!")
  if ! wait_for_action "/align_to_dock" || ! wait_for_action "/place_tag"; then
    fail "fakes nao apareceram no grafo (ROS_DOMAIN_ID=99)"
    cleanup
    return
  fi
  sleep 1
  # shellcheck disable=SC2086
  timeout 240 ros2 run manip_bt bt_yaml_executor "$ACTIONS" --ros-args \
    -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
    -p finish_dock_id:="''" -p server_wait_timeout:=20.0 $exec_params \
    > "$log" 2>&1
  local rc=$?
  cleanup
  sleep 2
  if [ "$rc" -eq "$want_exit" ]; then
    ok "codigo de saida $rc"
  else
    fail "codigo de saida $rc (esperava $want_exit) — veja $log"
  fi
  CASE_LOG="$log"
}

case_good() {
  run_case good 0 "" ""
  expect_count "$CASE_LOG" 2 "pose validada (x/y/yaw dentro dos limites)"
  expect_count "$CASE_LOG" 2 "base parada:"
  expect_count "$CASE_LOG" 2 "pronto para manipulacao."
  expect_count "$CASE_LOG" 0 "POSE FORA DOS LIMITES"
  expect_count "$CASE_LOG" 1 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 1 "PICK concluido com sucesso"
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_retry() {
  run_case retry 0 "-p align_bad_count:=1" ""
  expect_count "$CASE_LOG" 1 "[WS1] POSE FORA DOS LIMITES (y +0.304 m > +0.200 m; yaw +0.319 rad (+18.3 graus) > +0.200 rad (+11.5 graus)): undock + nova navegacao ate a staging + align (re-dock 0 restante(s) depois deste)."
  # 1 undock do re-dock da WS1 + 1 undock normal ao sair da WS1 para a WS2.
  expect_count "$CASE_LOG" 2 "undock (dock_type='caramelo_ws1_dock')..."
  expect_count "$CASE_LOG" 1 "[WS1] alinhamento fino holonomico (docking 2)..."
  expect_count "$CASE_LOG" 2 "pose validada (x/y/yaw dentro dos limites)"
  expect_count "$CASE_LOG" 1 "[WS1] pronto para manipulacao."
  expect_count "$CASE_LOG" 1 "dockings 2;"
  expect_count "$CASE_LOG" 1 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 1 "PICK concluido com sucesso"
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_strict() {
  run_case strict 0 "-p align_bad_count:=-1" ""
  # As duas estacoes recebem a pose ruim: cada uma re-docka uma vez e pula.
  expect_count "$CASE_LOG" 2 "undock + nova navegacao ate a staging + align (re-dock 0 restante(s) depois deste)."
  expect_count "$CASE_LOG" 1 "[WS1] POSE FORA DOS LIMITES apos 2 docking(s) (y +0.304 m > +0.200 m; yaw +0.319 rad (+18.3 graus) > +0.200 rad (+11.5 graus)) — pose_gate_strict=true: nao manipulo aqui."
  expect_count "$CASE_LOG" 0 "pronto para manipulacao."
  expect_count "$CASE_LOG" 0 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 0 "PICK concluido com sucesso"
  # WS1: undock do re-dock + undock ao sair (docked=true apos a falha na mesa);
  # WS2: undock do proprio re-dock.
  expect_count "$CASE_LOG" 2 "undock (dock_type='caramelo_ws1_dock')..."
  expect_count "$CASE_LOG" 1 "undock (dock_type='caramelo_ws2_dock')..."
  # linha de ERROR + linha de resumo, por estacao
  expect_count "$CASE_LOG" 4 "pose_gate_strict=true: nao manipulo aqui."
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_lenient() {
  run_case lenient 0 "-p align_bad_count:=-1" "-p pose_gate_strict:=false"
  expect_count "$CASE_LOG" 2 "pose_gate_strict=false: SIGO para a manipulacao"
  expect_count "$CASE_LOG" 2 "pronto para manipulacao."
  expect_count "$CASE_LOG" 1 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 1 "PICK concluido com sucesso"
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_moving() {
  run_case moving 0 "-p odom_moving_s:=5.0" "-p stop_check_timeout_s:=2.0"
  # linha de ERROR + linha de resumo, por estacao
  expect_count "$CASE_LOG" 4 "base AINDA EM MOVIMENTO 2.0 s apos o align"
  expect_count "$CASE_LOG" 0 "base parada:"
  expect_count "$CASE_LOG" 0 "pronto para manipulacao."
  expect_count "$CASE_LOG" 0 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_no_odom() {
  run_case no_odom 0 "-p odom_publish:=false" "-p stop_check_timeout_s:=2.0"
  # linha de ERROR + linha de resumo, por estacao
  expect_count "$CASE_LOG" 4 "sem odometria fresca em '/odom' por 2.0 s (0 amostras)"
  expect_count "$CASE_LOG" 0 "pronto para manipulacao."
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
  run_case no_odom_lenient 0 "-p odom_publish:=false" \
    "-p stop_check_timeout_s:=2.0 -p stop_check_require_odom:=false"
  expect_count "$CASE_LOG" 2 "parada NAO verificada; sigo (stop_check_require_odom=false)"
  expect_count "$CASE_LOG" 2 "pronto para manipulacao."
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case_disabled() {
  run_case disabled 0 "-p align_bad_count:=-1 -p odom_publish:=false" \
    "-p pose_gate_enabled:=false -p stop_check_enabled:=false"
  expect_count "$CASE_LOG" 2 "gate de pose DESLIGADO (pose_gate_enabled=false)"
  expect_count "$CASE_LOG" 2 "pronto para manipulacao."
  expect_count "$CASE_LOG" 0 "base parada:"
  expect_count "$CASE_LOG" 1 "PLACE concluido com sucesso"
  expect_count "$CASE_LOG" 1 "Arvore terminou com SUCESSO"
}

case "$MODE" in
  good) case_good ;;
  retry) case_retry ;;
  strict) case_strict ;;
  lenient) case_lenient ;;
  moving) case_moving ;;
  no_odom) case_no_odom ;;
  disabled) case_disabled ;;
  all) case_good; case_retry; case_strict; case_lenient; case_moving; case_no_odom; case_disabled ;;
  *) echo "modo desconhecido: $MODE"; exit 2 ;;
esac

if [ "$FAILURES" -eq 0 ]; then
  echo "=== TUDO OK (mapa $(basename "$MAP"), logs em $LOG_DIR) ==="
  exit 0
fi
echo "=== $FAILURES verificacao(oes) falharam (logs em $LOG_DIR) ==="
exit 1
