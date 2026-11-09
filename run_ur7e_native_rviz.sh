#!/usr/bin/env bash
set -eo pipefail
NATIVE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_REFERENCE="${NVBLOX_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$NATIVE_REFERENCE/install/setup.bash"
exec rviz2 -d "${NVBLOX_RVIZ_CONFIG:-$NATIVE_REPO/src/ur7e_nvblox_native/rviz/native.rviz}" \
  --ros-args -p use_sim_time:="${ESDF_USE_SIM_TIME:-true}" "$@"
