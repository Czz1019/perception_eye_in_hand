#!/usr/bin/env bash
set -eo pipefail
NATIVE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_REFERENCE="${NVBLOX_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
NATIVE_BUILD_GEOMETRY="${ESDF_BUILD_GEOMETRY_TEST:-OFF}"
case "$NATIVE_BUILD_GEOMETRY" in
  ON|OFF) ;;
  *) echo "ESDF_BUILD_GEOMETRY_TEST must be ON or OFF" >&2; exit 1 ;;
esac
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$NATIVE_REFERENCE/install/setup.bash"
cd "$NATIVE_REPO"
export CUDACXX="${CUDACXX:-/usr/local/cuda/bin/nvcc}"
/usr/bin/colcon build --base-paths src/ur7e_nvblox_native \
  --build-base build_native --install-base install_native --packages-select ur7e_nvblox_native \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES="${ESDF_CUDA_ARCHITECTURES:-89}" \
    -DBUILD_TESTING=OFF -DUR7E_BUILD_NATIVE_GEOMETRY_TEST="$NATIVE_BUILD_GEOMETRY"
# A binary need not be relinked when only a diagnostic or CMake metadata changes.
# Record successful completion of the WHOLE build/install, rather than comparing
# every source's mtime with an unchanged executable's mtime.
NATIVE_BUILD_SHARE="$NATIVE_REPO/install_native/ur7e_nvblox_native/share/ur7e_nvblox_native"
touch "$NATIVE_BUILD_SHARE/native_mapper.build_complete"
if [[ "$NATIVE_BUILD_GEOMETRY" == ON ]]; then
  touch "$NATIVE_BUILD_SHARE/native_geometry_test.build_complete"
else
  rm -f "$NATIVE_BUILD_SHARE/native_geometry_test.build_complete"
fi
echo "Native build/install complete. Geometry executable: $NATIVE_BUILD_GEOMETRY (not executed)."
