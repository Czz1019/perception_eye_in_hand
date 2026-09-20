#!/usr/bin/env bash
set -eo pipefail
ESDF_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ESDF_CONDA_ROOT="${ESDF_CONDA_ROOT:-$HOME/miniconda3}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
source "$ESDF_CONDA_ROOT/etc/profile.d/conda.sh"
conda activate whole_control
source /opt/ros/humble/setup.bash
source "$ESDF_WS/install/local_setup.bash"
# Only the mapper uses conda Python; Gazebo runs in the existing scene launcher.
exec /opt/ros/humble/bin/ros2 launch ur7e_nvblox mapping.launch.py \
    "python_executable:=$CONDA_PREFIX/bin/python" "$@"
