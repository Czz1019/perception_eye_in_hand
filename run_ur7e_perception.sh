#!/usr/bin/env bash
set -eo pipefail
PERCEPTION_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Match the orbit scanner's simulation defaults in fresh terminals.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-127}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11457}"
# Fail before creating a scene if source-only changes have not been built.
"$PERCEPTION_REPO/run_ur7e_native.sh" --check-build
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX QT_QPA_PLATFORM_PLUGIN_PATH
source /opt/ros/humble/setup.bash
for PERCEPTION_PREFIX in "$PERCEPTION_REPO/install/ur7e_d435_gazebo" "$PERCEPTION_REPO/install_native/ur7e_nvblox_native"; do
  if [[ ! -f "$PERCEPTION_PREFIX/share/$(basename "$PERCEPTION_PREFIX")/package.bash" ]]; then
    echo "Build the project first: ./build_ur7e_d435_scene.sh && ./build_ur7e_native.sh" >&2
    exit 1
  fi
  source "$PERCEPTION_PREFIX/share/$(basename "$PERCEPTION_PREFIX")/package.bash"
done
export UR7E_PERCEPTION_ROOT="$PERCEPTION_REPO"
# Humble's installed Cyclone DDS defaults to just 10 automatic participant
# indices. A full scene's transient spawners plus mapper/RViz can exceed that.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && -z "${CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI="file://$PERCEPTION_REPO/src/ur7e_nvblox_native/config/cyclonedds_local.xml"
fi
exec ros2 launch ur7e_nvblox_native perception.launch.py "$@"
