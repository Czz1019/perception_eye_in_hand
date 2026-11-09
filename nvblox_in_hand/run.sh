#!/usr/bin/env bash
set -eo pipefail
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$IN_HAND_ROOT/env.sh"
if [[ "${IN_HAND_SOURCE_BUILD:-0}" == 1 ]]; then
  [[ -f "$IN_HAND_ROOT/install_source/local_setup.bash" ]] || { echo 'Run ./build.sh --from-source first' >&2; exit 1; }
  source "$IN_HAND_ROOT/install_source/local_setup.bash"
fi
[[ "$(ros2 pkg prefix nvblox_in_hand)" == "$IN_HAND_ROOT/"* ]] || { echo 'Build this new workspace first: ./build.sh' >&2; exit 1; }
exec ros2 launch nvblox_in_hand realsense_example.launch.py "$@"
