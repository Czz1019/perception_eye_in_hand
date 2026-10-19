#!/usr/bin/env bash
set -eo pipefail
MAPPING_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX QT_QPA_PLATFORM_PLUGIN_PATH
source "$MAPPING_WS/source_gazebo_env.sh"
exec /opt/ros/humble/bin/ros2 launch realtime_nvblox gazebo_mapping.launch.py "$@"
