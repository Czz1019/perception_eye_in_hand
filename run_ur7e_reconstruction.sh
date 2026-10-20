#!/usr/bin/env bash
set -eo pipefail
RECON_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECON_CONDA_ROOT="${RECON_CONDA_ROOT:-$HOME/miniconda3}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
unset QT_QPA_PLATFORM_PLUGIN_PATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
source "$RECON_CONDA_ROOT/etc/profile.d/conda.sh"
conda activate whole_control
source /opt/ros/humble/setup.bash
source "$RECON_WS/install/local_setup.bash"
exec /opt/ros/humble/bin/ros2 launch ur7e_reconstruction reconstruction.launch.py \
    "python_executable:=$CONDA_PREFIX/bin/python" "$@"
