#!/usr/bin/env bash
# filepath: scripts/run_all_tests.sh
# Every test, on a machine with the workspace built and sourced (the Jetson):
# the ROS-free tests, then the node tests in an isolated loopback domain, then
# the C++ tests. The XRCE agent must be stopped: node tests publish on /fmu.
#
#   source install/setup.bash && bash scripts/run_all_tests.sh
set -euo pipefail
cd "$(dirname "$0")/.."
if pgrep -x MicroXRCEAgent >/dev/null; then
  echo "REFUSED: MicroXRCEAgent is running. Stop it first (node tests publish /fmu topics)." >&2
  exit 1
fi
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/src/drone_bringup/config/cyclonedds_bench.xml"
bash scripts/run_pure_tests.sh
node() { echo "== $2"; PYTHONPATH="$1:$PYTHONPATH" python3 -m pytest -p no:cacheprovider -q "$2"; }
node src/px4_odom_bridge src/px4_odom_bridge/test
node src/geofence_watchdog src/geofence_watchdog/test
node src/planner_shim src/planner_shim/test
node src/frontier_explorer:src/planner_shim src/frontier_explorer/test/test_closed_loop.py
colcon test --packages-select map_interface as2_platform_pixhawk --event-handlers console_cohesion- >/dev/null
colcon test-result --test-result-base build/map_interface
ros2 daemon stop >/dev/null 2>&1 || true
echo "ALL TESTS PASSED (the rig scenarios are separate: src/sim_bringup/README.md)"
