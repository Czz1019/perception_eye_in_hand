#!/usr/bin/env bash
set -eo pipefail
SIM_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/local_setup.bash"
cd "$SIM_WS"
exec /usr/bin/colcon build --symlink-install --base-paths "$SIM_WS/src/ur7e_gazebo_esdf_sage" --packages-select ur7e_gazebo_esdf_sage --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
