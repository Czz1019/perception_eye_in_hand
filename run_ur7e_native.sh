#!/usr/bin/env bash
set -eo pipefail
NATIVE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_REFERENCE="${NVBLOX_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
NATIVE_CONFIG="${NVBLOX_NATIVE_CONFIG:-$NATIVE_REPO/src/ur7e_nvblox_native/config/gazebo.yaml}"
NATIVE_BINARY="$NATIVE_REPO/install_native/ur7e_nvblox_native/lib/ur7e_nvblox_native/${ESDF_NATIVE_EXECUTABLE:-native_mapper}"
NATIVE_BUILD_STAMP="$NATIVE_REPO/install_native/ur7e_nvblox_native/share/ur7e_nvblox_native/${ESDF_NATIVE_EXECUTABLE:-native_mapper}.build_complete"
if [[ ! -x "$NATIVE_BINARY" ]]; then
  echo "Native binary missing. Build this repository: ./build_ur7e_native.sh" >&2
  exit 1
fi
if [[ ! -f "$NATIVE_BUILD_STAMP" || "$NATIVE_BINARY" -nt "$NATIVE_BUILD_STAMP" ]]; then
  echo "No successful build/install marker. Run ./build_ur7e_native.sh first." >&2
  if [[ "${ESDF_NATIVE_EXECUTABLE:-native_mapper}" == native_geometry_test ]]; then
    echo "For the optional diagnostic: ESDF_BUILD_GEOMETRY_TEST=ON ./build_ur7e_native.sh" >&2
  fi
  exit 1
fi
NATIVE_MAIN_SOURCE="$NATIVE_REPO/src/ur7e_nvblox_native/src/${ESDF_NATIVE_EXECUTABLE:-native_mapper}.cpp"
for NATIVE_SOURCE in "$NATIVE_MAIN_SOURCE" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/block_export.h" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/block_export.cu" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/mesh_visualization.h" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/mesh_view.h" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/mesh_view.cpp" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/src/gpu_timings.h" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/CMakeLists.txt" \
                     "$NATIVE_REPO/src/ur7e_nvblox_native/package.xml" \
                     "$NATIVE_REPO"/src/ur7e_nvblox_native/launch/*.py; do
  if [[ -f "$NATIVE_SOURCE" && "$NATIVE_SOURCE" -nt "$NATIVE_BUILD_STAMP" ]]; then
    echo "Native source changed; installed binary is stale. Run ./build_ur7e_native.sh before starting." >&2
    exit 1
  fi
done
if [[ "${1:-}" == --check-build ]]; then exit 0; fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$NATIVE_REFERENCE/install/setup.bash"
source "$NATIVE_REPO/install_native/local_setup.bash"
# Isaac ROS stores some GXF libraries below package share directories. Inherit
# the resolved dependency directories of the matching official executable;
# sourcing its overlay alone does not reproduce the executable's embedded RPATH.
NATIVE_LIBRARY_DIRS=$(/usr/bin/python3 - "$NATIVE_REFERENCE/install/nvblox_ros/lib/nvblox_ros/nvblox_node" <<'PY'
import re, subprocess, sys
from pathlib import Path
output=subprocess.check_output(['ldd', sys.argv[1]], text=True)
paths={str(Path(p).parent) for p in re.findall(r'=> (/\S+)', output) if Path(p).exists()}
print(':'.join(sorted(paths)))
PY
)
export LD_LIBRARY_PATH="$NATIVE_LIBRARY_DIRS:${LD_LIBRARY_PATH:-}"
# Execute the project binary directly so the launch system signals the mapper
# itself, rather than a ros2-run parent that can leave its child behind.
exec "$NATIVE_BINARY" --ros-args --params-file "$NATIVE_CONFIG" \
  -r camera_0/depth/image:=/esdf_input/depth \
  -r camera_0/depth/camera_info:=/esdf_input/camera_info \
  -r "camera_0/color/image:=${NVBLOX_COLOR_TOPIC:-/camera/color/image_raw}" \
  -r "camera_0/color/camera_info:=${NVBLOX_COLOR_INFO_TOPIC:-/camera/color/camera_info}" "$@"
