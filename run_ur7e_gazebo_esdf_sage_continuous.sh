#!/usr/bin/env bash
set -eo pipefail
SIM_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/local_setup.bash"
export ROS_DOMAIN_ID="${UR7E_SIM_DOMAIN_ID:-88}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${UR7E_SIM_GAZEBO_PORT:-11388}"
if [[ "$ROS_DOMAIN_ID" == 0 ]]; then echo 'Refusing default real-robot ROS domain 0' >&2; exit 2; fi
export GAZEBO_MODEL_DATABASE_URI=""
exec /opt/ros/humble/bin/ros2 launch ur7e_gazebo_esdf_sage_continuous bringup.launch.py "$@"
