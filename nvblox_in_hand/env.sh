#!/usr/bin/env bash
# Source this file in every monitoring terminal. Never source the parent install tree.
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IN_HAND_REFERENCE="${NVBLOX_ROS_WS:-/home/czz/workspaces/isaac_ros-dev}"
export ROS_HOME="${ROS_HOME:-$IN_HAND_ROOT/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$IN_HAND_ROOT/log/ros}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-127}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11457}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$IN_HAND_ROOT/src/nvblox_in_hand/config/robot/cyclonedds_local.xml}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
if [[ ! -f "$IN_HAND_REFERENCE/install/setup.bash" ]]; then
  echo "Missing official dependency environment: $IN_HAND_REFERENCE/install/setup.bash" >&2
  return 1
fi
source "$IN_HAND_REFERENCE/install/setup.bash"
# The installed Humble RViz ships only GLSL120 resources. The upstream
# plugin also initializes its voxel material when receiving a Mesh, and would
# otherwise request the absent rviz/glsl150/box.geom and abort.
case "${IN_HAND_RVIZ_MATERIAL_MODE:-auto}" in
  auto)
    IN_HAND_RVIZ_MEDIA="$(ros2 pkg prefix rviz_rendering)/share/rviz_rendering/ogre_media/materials"
    if [[ ! -f "$IN_HAND_RVIZ_MEDIA/glsl150/box.geom" || ! -d "$IN_HAND_RVIZ_MEDIA/scripts150" ]]; then
      export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
    fi ;;
  fallback) export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1 ;;
  native) unset ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL ;;
  *) echo 'IN_HAND_RVIZ_MATERIAL_MODE must be auto, fallback or native' >&2; return 1 ;;
esac
if [[ -f "$IN_HAND_ROOT/install/local_setup.bash" ]]; then
  source "$IN_HAND_ROOT/install/local_setup.bash"
else
  # ament package-level hooks are also valid after a direct CMake install.
  for IN_HAND_PACKAGE in ur7e_d435_gazebo nvblox_in_hand; do
    IN_HAND_HOOK="$IN_HAND_ROOT/install/$IN_HAND_PACKAGE/share/$IN_HAND_PACKAGE/local_setup.bash"
    if [[ -f "$IN_HAND_HOOK" ]]; then source "$IN_HAND_HOOK"; fi
  done
fi
# The port stores GXF libraries under share paths. Reproduce its resolved runtime dependencies.
IN_HAND_GXF_DIRS=$(/usr/bin/python3 - "$IN_HAND_REFERENCE/install/nvblox_ros/lib/nvblox_ros/nvblox_node" <<'PY'
import re, subprocess, sys
from pathlib import Path
out=subprocess.check_output(['ldd',sys.argv[1]],text=True)
print(':'.join(sorted({str(Path(p).parent) for p in re.findall(r'=> (/\S+)',out) if Path(p).exists()})))
PY
) || return 1
export LD_LIBRARY_PATH="$IN_HAND_GXF_DIRS:${LD_LIBRARY_PATH:-}"
