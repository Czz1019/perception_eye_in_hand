#!/usr/bin/env bash
set -eo pipefail
RECON_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
cd "$RECON_WS"
/usr/bin/python3 -m colcon build --packages-select realtime_nvblox ur7e_nvblox ur7e_reconstruction
