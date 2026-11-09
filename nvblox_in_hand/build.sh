#!/usr/bin/env bash
set -eo pipefail
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$IN_HAND_ROOT"
source "$IN_HAND_ROOT/env.sh"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export CUDACXX="${CUDACXX:-/usr/local/cuda/bin/nvcc}"
case "${1:-}" in
  '') /usr/bin/colcon build --base-paths src --build-base build --install-base install --executor sequential --event-handlers desktop_notification- status- terminal_title- \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES="${IN_HAND_CUDA_ARCH:-89}" -DBUILD_TESTING=OFF ;;
  --from-source) /usr/bin/colcon build --base-paths src upstream/nvblox_ros upstream/nvblox_ros_common upstream/nvblox_msgs upstream/nvblox_rviz_plugin upstream/nvblox_ros_python_utils \
      --build-base build_source --install-base install_source --executor sequential --event-handlers desktop_notification- status- terminal_title- --allow-overriding nvblox_ros nvblox_ros_common nvblox_msgs nvblox_rviz_plugin nvblox_ros_python_utils \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES="${IN_HAND_CUDA_ARCH:-89}" -DBUILD_TESTING=OFF ;;
  *) echo 'Usage: ./build.sh [--from-source]' >&2; exit 2 ;;
esac
