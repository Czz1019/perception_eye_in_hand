#!/usr/bin/env bash
set -eo pipefail
SAGE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX QT_QPA_PLATFORM_PLUGIN_PATH
source /opt/ros/humble/setup.bash
if [[ ! -f "$SAGE_WS/install/ur7e_sage_planner/share/ur7e_sage_planner/package.xml" ]]; then
    echo "Build first: $SAGE_WS/build_ur7e_sage_planner.sh" >&2
    exit 1
fi
source "$SAGE_WS/install/local_setup.bash"
exec /opt/ros/humble/bin/ros2 launch ur7e_sage_planner planning.launch.py "$@"
