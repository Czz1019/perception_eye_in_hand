#!/usr/bin/env bash
set -eo pipefail
PP_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source "$PP_WS/install/local_setup.bash"
cd "$PP_WS"
exec /usr/bin/colcon build --symlink-install --packages-select ur7e_perception_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
