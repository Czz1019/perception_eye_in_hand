#!/usr/bin/env bash
set -eo pipefail
SCENE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Keep Torch/CUDA wheels and unrelated ROS overlays out of this Gazebo process.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX QT_QPA_PLATFORM_PLUGIN_PATH
source /opt/ros/humble/setup.bash
if [[ ! -f "$SCENE_WS/install/ur7e_d435_gazebo/share/ur7e_d435_gazebo/package.xml" ]]; then
    echo "Build first: $SCENE_WS/build_ur7e_d435_scene.sh" >&2
    exit 1
fi
source "$SCENE_WS/install/local_setup.bash"
exec /opt/ros/humble/bin/ros2 launch ur7e_reconstruction scan_scene.launch.py "$@"
