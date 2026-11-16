#!/usr/bin/env bash
# Cenario de bancada do REPLANEJAMENTO NO MEIO DA MISSAO (2026-11-16), sem
# robo e sem MoveIt, em ROS_DOMAIN_ID=99 (nunca encosta no robo real). Reproduz
# a queda da run_20261116_110428: um place falha de vez com o bloco de volta a
# bordo. Sobe os fakes de navegacao e de manipulacao (fail_places: os FAIL_N
# primeiros places da tag_1 falham), roda o bt_yaml_executor sobre
# replan_actions.yaml e confere por grep. Codigo de saida 0 so se tudo bater.
#
# Uso: run_replan_scenario.sh [FAIL_N]
#   1 (default): a tag_1 falha uma vez, fica para depois e e' entregue no fim
#   9: a tag_1 nunca sai; a missao entrega o resto e desiste so' dela
# Logs: $REPLAN_LOG_DIR (default /tmp/replan_scenario).
export ROS_DOMAIN_ID=99
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

MAP="$WS_ROOT/src/caramelo_mapping/maps/arena_extendida"   # WS1..WS3 com pose gravada
LOG_DIR="${REPLAN_LOG_DIR:-/tmp/replan_scenario}"
mkdir -p "$LOG_DIR"
FAIL_N="${1:-1}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null
  done
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] || continue
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
  done
  PIDS=()
}
trap cleanup EXIT

FAILURES=0
fail() { echo "  [FALHOU] $*"; FAILURES=$((FAILURES + 1)); }
ok() { echo "  [ok] $*"; }
expect_count() {
  local log="$1" want="$2" text="$3" got
  got=$(grep -c -F -- "$text" "$log")
  if [ "$got" -eq "$want" ]; then ok "$want x '$text'"; else fail "esperava $want x '$text', achei $got"; fi
}
wait_for_action() {
  local name="$1" tries=30
  while [ $tries -gt 0 ]; do
    ros2 action list 2>/dev/null | grep -q -F -- "$name" && return 0
    tries=$((tries - 1)); sleep 1
  done
  return 1
}

python3 "$HERE/fake_nav_actions.py" --ros-args -p latency_sec:=0.3 \
  > "$LOG_DIR/fake_nav.log" 2>&1 &
PIDS+=("$!")
python3 "$HERE/fake_pick_place.py" --ros-args -p latency_sec:=0.3 -p track_onboard:=true \
  -p "fail_places:=['tag_1:$FAIL_N']" > "$LOG_DIR/fake_pp.log" 2>&1 &
PIDS+=("$!")
if ! wait_for_action "/align_to_dock" || ! wait_for_action "/place_tag"; then
  fail "fakes nao apareceram no grafo (ROS_DOMAIN_ID=99)"
  exit 1
fi
sleep 1
LOG="$LOG_DIR/executor.log"
timeout 300 ros2 run manip_bt bt_yaml_executor "$HERE/replan_actions.yaml" --ros-args \
  -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
  -p home_before_dock:=false -p finish_dock_id:="''" -p server_wait_timeout:=20.0 \
  -p pose_gate_enabled:=false -p stop_check_enabled:=false > "$LOG" 2>&1
RC=$?
cleanup

# Capacidade: com a tag_1 presa a bordo, nunca mais de 3 objetos no robo.
MAX_ONBOARD=$(grep -o "'\(pick\|place\)' concluido" "$LOG_DIR/fake_pp.log" |
  awk '/pick/ {n++} /place/ {n--} n > m {m = n} END {print m + 0}')
[ "$MAX_ONBOARD" -le 3 ] && ok "no maximo $MAX_ONBOARD objetos a bordo" ||
  fail "$MAX_ONBOARD objetos a bordo (capacidade 3)"

expect_count "$LOG" 1 "Arvore terminou com FALHA"
expect_count "$LOG" 1 "[replano] tag_1 falhou em WS_2: fica para depois; replanejando o resto."
expect_count "$LOG" 1 "[re-tentativa] fora desta rodada: tag_1"
# O resto da missao sai mesmo com a tag_1 parada.
for t in tag_2 tag_3 tag_4 tag_5 tag_6; do
  expect_count "$LOG_DIR/fake_pp.log" 1 "'place' concluido com sucesso (fake). tag=$t "
done
if [ "$FAIL_N" -le 1 ]; then
  [ "$RC" -eq 0 ] && ok "codigo de saida 0" || fail "codigo de saida $RC — veja $LOG"
  expect_count "$LOG" 1 "[re-tentativa] plano: WS_2: place tag_1"
  expect_count "$LOG_DIR/fake_pp.log" 1 "'place' concluido com sucesso (fake). tag=tag_1 "
  expect_count "$LOG" 1 "apos a re-tentativa: [re-tentativa] nada pendente."
  # "Mais tarde": a tag_1 so' e' entregue depois de todo o resto.
  LAST=$(grep -F "'place' concluido com sucesso" "$LOG_DIR/fake_pp.log" | tail -1)
  case "$LAST" in *"tag=tag_1 "*) ok "tag_1 entregue por ultimo" ;; *) fail "tag_1 nao foi a ultima entrega" ;; esac
else
  [ "$RC" -eq 1 ] && ok "codigo de saida 1 (ficou pendencia)" || fail "codigo de saida $RC — veja $LOG"
  expect_count "$LOG" 1 "[replano] tag_1 falhou de novo em WS_2: desisto desse objeto e sigo com o resto."
  expect_count "$LOG" 1 "apos a re-tentativa: [re-tentativa] pendente: tag_1 (a bordo + place)"
  expect_count "$LOG_DIR/fake_pp.log" 2 "'place' FALHOU: tag_1"
fi
echo "logs em $LOG_DIR"
[ "$FAILURES" -eq 0 ] && echo "RESULTADO: OK" || echo "RESULTADO: $FAILURES falha(s)"
[ "$FAILURES" -eq 0 ]
