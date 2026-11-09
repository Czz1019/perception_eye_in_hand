#!/usr/bin/env bash
set -eo pipefail
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$IN_HAND_ROOT/env.sh"
if [[ "${IN_HAND_SOURCE_BUILD:-0}" == 1 ]]; then source "$IN_HAND_ROOT/install_source/local_setup.bash"; fi
case "${1:-mesh}" in
  esdf|diagnostic) IN_HAND_RVIZ_CONFIG=diagnostic.rviz ;;
  mesh) IN_HAND_RVIZ_CONFIG=mesh.rviz ;;
  2d) IN_HAND_RVIZ_CONFIG=official_2d.rviz ;;
  *) echo 'Usage: ./rviz.sh [mesh|2d|diagnostic] (esdf is a diagnostic alias)' >&2; exit 2 ;;
esac
exec rviz2 -d "$IN_HAND_ROOT/src/nvblox_in_hand/rviz/$IN_HAND_RVIZ_CONFIG" --ros-args -p "use_sim_time:=${IN_HAND_USE_SIM_TIME:-true}"
