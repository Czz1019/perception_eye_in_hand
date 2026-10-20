#!/usr/bin/env bash
set -eo pipefail
ESDF_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
cd "$ESDF_WS"
# Build the dependency from its unchanged source; CUDA is loaded only at runtime.
/usr/bin/python3 -m colcon build --packages-select realtime_nvblox ur7e_nvblox
