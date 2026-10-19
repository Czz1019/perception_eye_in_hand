#!/usr/bin/env bash
# Use the same clean ROS environment as the automatic scanner launcher.
set -eo pipefail
AUTO_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source "$AUTO_WS/install/local_setup.bash"
case "${1:-}" in
  prepare|start|cancel)
    exec /opt/ros/humble/bin/ros2 service call "/ur7e_auto_esdf_scan/$1" std_srvs/srv/Trigger '{}'
    ;;
  status)
    exec /opt/ros/humble/bin/ros2 topic echo /ur7e_auto_esdf_scan/status std_msgs/msg/String
    ;;
  *) echo "Usage: $0 {prepare|start|cancel|status}" >&2; exit 2 ;;
esac
