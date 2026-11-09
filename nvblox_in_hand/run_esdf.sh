#!/usr/bin/env bash
# Official Mapper through the diagnostic adapter, with native observed 3D export.
set -eo pipefail
IN_HAND_ESDF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$IN_HAND_ESDF_ROOT/run.sh" mapper_backend:=diagnostic esdf_mode:=3d enable_esdf_view:=true rviz_mode:=diagnostic "$@"
