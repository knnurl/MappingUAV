#!/usr/bin/env bash
# filepath: scripts/verify_gate1.sh
# Gate 1: uXRCE-DDS link between the Pixhawk 6C (TELEM2) and the Jetson.
# Pass criteria: /fmu/out/* topics visible; timesync converged (estimated_offset
# steady to well under 1 ms, and `uxrce_dds_client status` on the FC reports
# "timesync converged: true"); stable for 10 min with zero dropouts.
#
# Prerequisite: the agent is running, e.g.
#   ros2 launch drone_bringup gate1_bridge.launch.py serial_dev:=/dev/serial/by-id/<adapter>
# The default serial_dev is the 40-pin UART /dev/ttyTHS1; a USB-serial adapter
# appears under /dev/serial/by-id/. ROS_DOMAIN_ID on the Jetson must equal the
# FC's UXRCE_DDS_DOM_ID (PX4 default 0).
#
# Usage: verify_gate1.sh              discovery, timesync, 15 s rates
#        verify_gate1.sh --stability  the above plus the 10 min stability run
set -e
source ~/colcon_ws/install/setup.bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "--- topic discovery (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}) ---"
# --no-daemon with a spin time: a cold daemon answers before discovery finishes.
n=$(ros2 topic list --no-daemon --spin-time 5 | grep -c '^/fmu/' || true)
if [ "$n" -eq 0 ]; then
  echo "FAIL: no /fmu topics. Is the agent running? Does PX4 UXRCE_DDS_DOM_ID equal ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}?"
  exit 1
fi
echo "OK: $n /fmu topics"

echo "--- timesync (estimated_offset must be steady; round_trip_time is link latency in us) ---"
timeout 20 ros2 topic echo --qos-reliability best_effort /fmu/out/timesync_status 2>/dev/null \
  | grep -E "estimated_offset|round_trip_time" | head -10

echo "--- rates, 15 s (ros2 topic hz cannot subscribe BEST_EFFORT; expect ~100 / 2 / 2 Hz) ---"
python3 "$HERE/fmu_rate_probe.py" --duration 15 \
  /fmu/out/vehicle_attitude /fmu/out/vehicle_status /fmu/out/timesync_status

if [ "${1:-}" = "--stability" ]; then
  echo "--- stability, 10 min: zero dropouts required ---"
  python3 "$HERE/fmu_rate_probe.py" --duration 600 --report-every 60 \
    /fmu/out/vehicle_status /fmu/out/vehicle_attitude
else
  echo "--- stability: run 'verify_gate1.sh --stability' for the 10 min run (zero dropouts required)."
fi
echo "--- On the FC (QGC MAVLink console): uxrce_dds_client status  ->  'timesync converged: true'"
