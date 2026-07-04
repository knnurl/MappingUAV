#!/usr/bin/env bash
# filepath: scripts/verify_gate1.sh
# Pass criteria: /fmu/out/* topics visible, timesync converged (<1 ms), stable 10 min.
set -e
source ~/colcon_ws/install/setup.bash

echo "--- topic discovery (expect /fmu/out/vehicle_status etc.) ---"
ros2 topic list | grep /fmu/ || { echo "FAIL: no /fmu topics"; exit 1; }

echo "--- timesync (watch observed_offset converge; must settle < 1e6 ns) ---"
timeout 20 ros2 topic echo /fmu/out/timesync_status --once

echo "--- rates ---"
timeout 15 ros2 topic hz /fmu/out/vehicle_attitude   # expect ~50-100 Hz [VERIFY per dds_topics.yaml rate]

echo "--- QoS reminder: subscribing to /fmu/out requires BEST_EFFORT; ros2 CLI handles this."
echo "--- Stability: leave 'ros2 topic hz /fmu/out/vehicle_status' running 10 min, zero dropouts."
