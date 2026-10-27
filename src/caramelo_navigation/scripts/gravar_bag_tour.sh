#!/usr/bin/env bash
# Grava o rosbag do tour / dos dockings (2026-10-26, diagnostico secao 8.1).
#
# Por que: nos dois tours de 2026-10-26 nao havia bag nenhum. Os nos C++ do
# Nav2 deixam log por PID em ~/.ros/log (F1), mas nenhum deles loga pose, e o
# AMCL nao loga /amcl_pose. Sem /scan, /tf, /odom e /amcl_pose nao da para
# separar "base ainda girando" de "AMCL atualizando tarde" (C1/C2), nem ver o
# que o LiDAR enxerga na WS6/WS2 (C4), nem a folga real nas passagens (C8).
#
# Uso (robo no ar, ANTES de mandar o tour_docks / run_mission):
#   ros2 run caramelo_navigation gravar_bag_tour            # ~/bags/tour_<data>
#   ros2 run caramelo_navigation gravar_bag_tour ws6_3x     # ~/bags/tour_<data>_ws6_3x
#   BAG_DIR=/media/pen/bags ros2 run caramelo_navigation gravar_bag_tour
# Ctrl-C encerra e fecha o bag. mcap se o plugin existir, senao sqlite3.
#
# Nao e lancado pelo launch de proposito: o operador decide quando gravar
# (o bag de um tour tem ~1-2 GB por causa dos costmaps e do /particle_cloud).
set -euo pipefail

SUFIXO="${1:-}"
BAG_DIR="${BAG_DIR:-$HOME/bags}"
STAMP="$(date +%Y-%m-%d_%H%M)"
SAIDA="${BAG_DIR}/tour_${STAMP}${SUFIXO:+_$SUFIXO}"
mkdir -p "${BAG_DIR}"

# mcap e mais rapido para gravar e le em qualquer PC com o plugin; o sqlite3 e
# o default do Jazzy e sempre existe. "ros2 pkg prefix" so consulta o ament
# index (nao cria no, nao toca o dominio).
STORAGE="sqlite3"
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  STORAGE="mcap"
fi

# Topicos da secao 8.1 do diagnostico. /scan_raw = LiDAR cru (driver na Pi,
# 360 graus); /scan = saida do scan_normalizer (so os 180 graus frontais, e' o
# que o Nav2 e o dock_align_node usam). /odom/wheel e /imu/data_raw sao as
# entradas do EKF (ekf.yaml do caramelo_hardware_ws), /odom a saida.
# Topico inexistente nao quebra o record, so nao aparece no bag.
# /align_to_dock/_action/* e /navigate_to_pose/_action/* sao "hidden topics"
# (prefixo _action) -> --include-hidden-topics obrigatorio.
TOPICOS=(
  /scan
  /scan_raw
  /tf
  /tf_static
  /odom
  /odom/wheel
  /imu/data_raw
  /amcl_pose
  /particle_cloud
  /mecanum_controller/reference
  /cmd_vel_nav
  /cmd_vel_smoothed
  /cmd_vel
  /local_costmap/costmap
  /local_costmap/published_footprint
  /global_costmap/costmap
  /plan
  /collision_monitor_state
  /joint_states
  /align_to_dock/_action/feedback
  /align_to_dock/_action/status
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
)

# QoS: /scan, /scan_raw e /particle_cloud saem best_effort (SensorDataQoS) e
# /tf_static transient_local. NAO se passa --qos-profile-overrides-path: o
# record do Jazzy descobre cada topico quando ele aparece e adota o QoS que o
# publisher oferece, inclusive se o publisher subir DEPOIS do record. A primeira
# versao deste script (2026-10-26) gerava um yaml de override so com
# reliability/durability e o ros2 bag record o REJEITAVA antes de abrir o bag
# ("History and/or depth settings are required", ros2bag.api
# convert_yaml_to_qos_profile), ou seja, nao gravava nada. Se um dia precisar
# de override, cada perfil exige history: keep_last + depth (ou keep_all).
CMD=(ros2 bag record
  --storage "${STORAGE}"
  --output "${SAIDA}"
  --include-hidden-topics
  "${TOPICOS[@]}")

echo "[gravar_bag_tour] saida: ${SAIDA} (storage ${STORAGE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-nao definido})"
echo "[gravar_bag_tour] comando:"
printf '  %q' "${CMD[@]}"; echo
echo "[gravar_bag_tour] Ctrl-C para encerrar. Depois: ros2 bag info ${SAIDA}"
exec "${CMD[@]}"
