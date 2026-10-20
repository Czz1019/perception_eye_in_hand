#!/usr/bin/env bash
set -eo pipefail
PP_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX QT_QPA_PLATFORM_PLUGIN_PATH
source /opt/ros/humble/setup.bash
source "$PP_WS/install/local_setup.bash"
exec /opt/ros/humble/bin/ros2 launch ur7e_perception_planning bringup.launch.py "$@"
