#!/usr/bin/env bash
set -eo pipefail
ESDF_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ESDF_CONDA_ROOT="${ESDF_CONDA_ROOT:-$HOME/miniconda3}"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
export PYTHONDONTWRITEBYTECODE=1
if [[ -n "${ESDF_PYTHON:-}" ]]; then
    ESDF_RUNTIME_PYTHON="$ESDF_PYTHON"
elif [[ -x "$ESDF_CONDA_ROOT/envs/whole_control/bin/python" ]]; then
    source "$ESDF_CONDA_ROOT/etc/profile.d/conda.sh"
    conda activate whole_control
    ESDF_RUNTIME_PYTHON="$CONDA_PREFIX/bin/python"
else
    # The current machine installs Torch/nvblox_torch in the system user's site.
    unset PYTHONNOUSERSITE
    ESDF_RUNTIME_PYTHON=/usr/bin/python3
fi
source /opt/ros/humble/setup.bash
unset COLCON_CURRENT_PREFIX
source "$ESDF_WS/install/realtime_nvblox/share/realtime_nvblox/package.bash"
unset COLCON_CURRENT_PREFIX
source "$ESDF_WS/install/ur7e_nvblox/share/ur7e_nvblox/package.bash"
# Gazebo and RViz retain their ROS environment; only the mapper selects Python.
exec /opt/ros/humble/bin/ros2 launch ur7e_nvblox mapping.launch.py \
    "python_executable:=$ESDF_RUNTIME_PYTHON" "$@"
