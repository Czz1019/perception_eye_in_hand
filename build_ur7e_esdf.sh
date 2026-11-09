#!/usr/bin/env bash
set -eo pipefail
ESDF_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
cd "$ESDF_WS"
# Migrated install trees can contain broken links into the former user's home.
# Remove only broken generated symlinks in the two packages being rebuilt.
for ESDF_PACKAGE in realtime_nvblox ur7e_nvblox; do
    if [[ -d "install/$ESDF_PACKAGE" ]]; then
        find "install/$ESDF_PACKAGE" -xtype l -delete
    fi
done
# A historical backup marker is not a ROS package ("~" is invalid in names).
ESDF_LEGACY_MARKER=install/realtime_nvblox/share/ament_index/resource_index/packages/realtime_nvblox~upstream_main
if [[ -f "$ESDF_LEGACY_MARKER" ]]; then
    mkdir -p build/realtime_nvblox
    mv "$ESDF_LEGACY_MARKER" build/realtime_nvblox/legacy-package-marker
fi
# Build the dependency from its unchanged source; CUDA is loaded only at runtime.
/usr/bin/python3 -m colcon build \
  --base-paths src/realtime_nvblox src/ur7e_nvblox \
  --packages-select realtime_nvblox ur7e_nvblox
