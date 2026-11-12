#!/usr/bin/env bash
# Grava o rosbag de um tour / sessao de docking ate o Ctrl-C.
#
# Desde 2026-11-11 e' um atalho para o modo CLI do caramelo_bag_common.py, o
# mesmo modulo que o run_mission e o mission_server usam por corrida: mcap +
# zstd_fast, split 500 MB, cache 16 MB, lista MINIMA de topicos (nunca os de
# visualizacao do controller: /marker, /evaluation, /trajectories...),
# corrida.yaml com metadados e limpeza automatica de ~/bags. A versao antiga
# (2026-10-26, diagnostico secao 8.1) gravava sem compressao e com
# /particle_cloud e /global_costmap/costmap sempre: hoje isso e' --extra.
#
# Uso (robo no ar, ANTES de mandar o tour_docks):
#   ros2 run caramelo_navigation gravar_bag_tour                 # ~/bags/<stamp>_tour_sem_mapa
#   ros2 run caramelo_navigation gravar_bag_tour ws6_3x          # ~/bags/<stamp>_ws6_3x_sem_mapa
#   ros2 run caramelo_navigation gravar_bag_tour ws6_3x --mapa arenalegal_2026 --extra
#   BAG_DIR=/media/pen/bags ros2 run caramelo_navigation gravar_bag_tour
# Ctrl-C encerra e fecha o bag (SIGINT no record, nunca SIGKILL); no fim
# imprime `ros2 bag info`. Demais opcoes: --camera, --nav-profile dwb|mppi.
#
# Nao e' lancado pelo launch de proposito: o operador decide quando gravar.
set -euo pipefail

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULO="${AQUI}/caramelo_bag_common.py"   # instalado no mesmo lib/ (install(FILES))
if [ ! -f "${MODULO}" ]; then
  echo "[gravar_bag_tour] nao achei ${MODULO}; rebuild do caramelo_navigation?" >&2
  exit 1
fi

ARGS=()
# Primeiro argumento sem '--' = rotulo da sessao (compatibilidade com a
# versao antiga: `gravar_bag_tour ws6_3x`).
if [ "$#" -gt 0 ] && [[ "$1" != --* ]]; then
  ARGS+=(--task "$1"); shift
else
  ARGS+=(--task tour)
fi
if [ -n "${BAG_DIR:-}" ]; then
  ARGS+=(--bag-dir "${BAG_DIR}")
fi

echo "[gravar_bag_tour] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-nao definido}"
exec python3 "${MODULO}" "${ARGS[@]}" "$@"
