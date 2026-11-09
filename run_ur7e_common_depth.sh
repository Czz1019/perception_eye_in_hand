#!/usr/bin/env bash
set -eo pipefail
DEPTH_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$DEPTH_REPO/src/ur7e_nvblox:$DEPTH_REPO/src/realtime_nvblox:${PYTHONPATH:-}"
exec "${ESDF_PYTHON:-/usr/bin/python3}" -B -m ur7e_nvblox.depth_adapter "$@"
