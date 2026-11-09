#!/usr/bin/env bash
set -eo pipefail
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$IN_HAND_ROOT/env.sh"
if [[ "${IN_HAND_SOURCE_BUILD:-0}" == 1 ]]; then source "$IN_HAND_ROOT/install_source/local_setup.bash"; fi
exec /usr/bin/python3 -B -u "$IN_HAND_ROOT/src/ur7e_d435_gazebo/scripts/orbit_scan.py" "$@"
