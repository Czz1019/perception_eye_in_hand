#!/usr/bin/env bash
# Repeat this simulation-only experiment in a NEW directory, preserving original evidence.
set -eo pipefail
TEST_TEMPLATE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_PROJECT="$(cd "$TEST_TEMPLATE/../.." && pwd)"
export ROS_DOMAIN_ID=134 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11563
export DISPLAY="${DISPLAY:-:1}"
source "$TEST_PROJECT/env.sh"
TEST_OUTPUT="$(mktemp -d "$TEST_TEMPLATE/retest_XXXXXXXX")"
for TEST_FILE in run_measurement.py view_sweep_isolated.py capture_rviz_composite.py motion_plan.json analyze_test.py plot_saved_mesh.py; do
  cp "$TEST_TEMPLATE/$TEST_FILE" "$TEST_OUTPUT/$TEST_FILE"
done
cp "$TEST_TEMPLATE/"*.rviz "$TEST_OUTPUT/"
echo "New test evidence: $TEST_OUTPUT"
/usr/bin/python3 "$TEST_OUTPUT/run_measurement.py" > "$TEST_OUTPUT/measurement.log" 2>&1
/usr/bin/python3 "$TEST_OUTPUT/analyze_test.py" > "$TEST_OUTPUT/analysis.log"
MPLCONFIGDIR=/tmp/nvblox-mesh-test-mpl /usr/bin/python3 -s "$TEST_OUTPUT/plot_saved_mesh.py"
echo "Finished. Read $TEST_OUTPUT/analysis.json and test_results.csv"
