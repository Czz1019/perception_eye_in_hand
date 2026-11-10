#!/usr/bin/env bash
# Official workspace only; Gazebo is started separately from the UR7e environment.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/config/cyclonedds_local.xml
export ROS_LOG_DIR=/home/czz/perception_eye_in_hand/artifacts/workspaces_lab_reference_20261110/ros_logs
REFERENCE_GXF_DIRS=$(python3 - <<'PY'
import re,subprocess
from pathlib import Path
s=subprocess.check_output(['ldd','/home/czz/workspaces/isaac_ros-dev/install/nvblox_ros/lib/nvblox_ros/nvblox_node'],text=True)
print(':'.join(sorted({str(Path(p).parent) for p in re.findall(r'=> (/\S+)',s) if Path(p).exists()})))
PY
) || return 1
export LD_LIBRARY_PATH="$REFERENCE_GXF_DIRS:${LD_LIBRARY_PATH:-}"

# Humble RViz material fallback used by the locally installed official plugin.
if [[ ! -f /opt/ros/humble/share/rviz_rendering/ogre_media/materials/glsl150/box.geom ]]; then
  export ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL=1
fi
