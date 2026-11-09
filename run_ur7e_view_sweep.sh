#!/usr/bin/env bash
set -eo pipefail
VIEW_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-127}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && -z "${CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI="file://$VIEW_REPO/src/ur7e_nvblox_native/config/cyclonedds_local.xml"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 -B "$VIEW_REPO/src/ur7e_d435_gazebo/scripts/view_sweep.py" "$@"
