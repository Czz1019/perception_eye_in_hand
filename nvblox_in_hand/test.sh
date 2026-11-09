#!/usr/bin/env bash
set -eo pipefail
IN_HAND_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$IN_HAND_ROOT/env.sh"
cd "$IN_HAND_ROOT"
mkdir -p artifacts
/usr/bin/c++ -std=c++17 -I src/nvblox_in_hand/include tests/esdf_view_policy_test.cpp -o artifacts/esdf_view_policy_test
artifacts/esdf_view_policy_test
/usr/bin/python3 -B -m unittest discover -s tests -p 'test_*.py' -v
/usr/bin/python3 -B -m unittest discover -s src/ur7e_d435_gazebo/test -p test_orbit_geometry.py -v
