#!/usr/bin/env bash
# Cenario de bancada da RE-TENTATIVA NO FIM DA MISSAO (2026-11-05), sem robo
# e sem MoveIt, em ROS_DOMAIN_ID=99 (nunca encosta no robo real). Sobe os
# fakes de navegacao (fake_nav_actions.py: o align da WS2 falha nas primeiras
# FAIL_N vezes) e de manipulacao (fake_pick_place.py com track_onboard), roda
# o bt_yaml_executor com navegacao REAL (GoToWS falando com os fakes) sobre
# retry_actions.yaml e confere por grep. Codigo de saida 0 so se tudo bater.
#
# Uso: run_retry_scenario.sh [FAIL_N]   (default 2 = aligns da 1a ida a WS2)
# Logs: $RETRY_LOG_DIR (default /tmp/retry_scenario).
export ROS_DOMAIN_ID=99
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$HERE/../../../.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"
set -u

MAPS="$WS_ROOT/src/caramelo_mapping/maps"
MAP="$MAPS/arena_extendida"   # WS1..WS3 com pose gravada
LOG_DIR="${RETRY_LOG_DIR:-/tmp/retry_scenario}"
mkdir -p "$LOG_DIR"
FAIL_N="${1:-2}"

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
  -p "fail_align_docks:=['WS2:$FAIL_N']" > "$LOG_DIR/fake_nav.log" 2>&1 &
PIDS+=("$!")
python3 "$HERE/fake_pick_place.py" --ros-args -p latency_sec:=0.3 -p track_onboard:=true \
  > "$LOG_DIR/fake_pp.log" 2>&1 &
PIDS+=("$!")
if ! wait_for_action "/align_to_dock" || ! wait_for_action "/place_tag"; then
  fail "fakes nao apareceram no grafo (ROS_DOMAIN_ID=99)"
  exit 1
fi
sleep 1
LOG="$LOG_DIR/executor.log"
timeout 300 ros2 run manip_bt bt_yaml_executor "$HERE/retry_actions.yaml" --ros-args \
  -p map_folder:="$MAP" -p simulate_navigation:=false -p startup_home:=false \
  -p home_before_dock:=false -p finish_dock_id:="''" -p server_wait_timeout:=20.0 \
  -p pose_gate_enabled:=false -p stop_check_enabled:=false > "$LOG" 2>&1
RC=$?
cleanup
[ "$RC" -eq 0 ] && ok "codigo de saida 0" || fail "codigo de saida $RC — veja $LOG"
expect_count "$LOG" 1 "[re-tentativa] pendente: tag_1 (a bordo + place), tag_3 (pick + place)"
expect_count "$LOG" 1 "[re-tentativa] plano: WS_2: place tag_1 pick tag_3 | WS_3: place tag_3"
expect_count "$LOG" 1 "apos a re-tentativa: [re-tentativa] nada pendente."
expect_count "$LOG_DIR/fake_pp.log" 1 "'place' PULADO: tag_3 nao esta a bordo"
echo "logs em $LOG_DIR"
[ "$FAILURES" -eq 0 ] && echo "RESULTADO: OK" || echo "RESULTADO: $FAILURES falha(s)"
[ "$FAILURES" -eq 0 ]
