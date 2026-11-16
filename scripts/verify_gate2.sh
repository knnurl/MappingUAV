#!/usr/bin/env bash
# filepath: scripts/verify_gate2.sh
# Gate 2: Mid-360 driver + FAST-LIO2 bench test (hand-carry).
# Pass criteria: 10 Hz cloud, ~200 Hz IMU (Mid-360 alternates 4/6 ms), odometry alive,
# return-to-start error < 5 cm, no z drift over 5 min stationary.
#
# Prerequisites: the Jetson Ethernet port carries 192.168.1.5/24 (the host IP in
# MID360_config.json), the Mid-360 (192.168.1.123) is powered and cabled, and the
# stack is up:  ros2 launch drone_bringup gate2_lio.launch.py
# The Livox driver ignores Ctrl-C after a failed start; launch kills it after ~15 s.
# DDS environment: this must match the running stack. If the bench stack was started
# loopback-only (ROS_LOCALHOST_ONLY=1 with CYCLONEDDS_URI unset, e.g. because the WiFi
# roamed and CycloneDDS bound a stale address), run this the same way:
#   env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 scripts/verify_gate2.sh
# Otherwise it discovers nothing and silently records or measures nothing.
set -e
source ~/colcon_ws/install/setup.bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# `timeout` exits 124 when it stops a command, which `set -e` treats as a failure:
# the script used to abort after the first check. The Livox driver and FAST-LIO
# publish RELIABLE, so `ros2 topic hz` works here (unlike PX4's /fmu/out topics).
# Output goes to a file, not a pipe: `ros2 topic hz` block-buffers into a pipe, so
# everything it printed is lost when `timeout` kills it. stdbuf -oL keeps the file
# current. Do not use `timeout -s INT` here: the INT reaches this script too.
rate() {  # rate <seconds> <topic>
  local out; out="$(mktemp)"
  timeout "$1" stdbuf -oL ros2 topic hz "$2" > "$out" 2>/dev/null || true
  grep -E 'average rate|min:' "$out" | tail -2 || echo "  no messages in $1 s"
  rm -f "$out"
}

echo "--- driver streams ---"
echo "/livox/lidar (expect 10 Hz):";  rate 15 /livox/lidar
echo "/livox/imu (expect ~200 Hz):"; rate 15 /livox/imu

echo "--- LIO output ---"
echo "/Odometry (expect ~10 Hz):"; rate 15 /Odometry
timeout 10 ros2 topic echo /Odometry --once || echo "FAIL: no /Odometry message within 10 s"

echo "--- CPU/memory budget (record numbers in the gate log) ---"
top -b -n 1 | grep -E "fastlio|laserMapping|livox" || true
free -h

cat <<EOF
MANUAL TEST (human):
1. Mark start pose on the bench. Power on, let LIO initialize stationary 10 s.
2. In a second terminal, start the bench recording (masterplan §5: Gate 2/3 bench
   sessions record raw clouds for replay tests; bench only, never in flight):
     $HERE/record_bench_bag.sh gate2 walk
3. Hand-carry the airframe a loop around the room (~2 min), return EXACTLY to mark.
4. ros2 topic echo /Odometry --once  -> position must be within 0.05 m of zero.
5. Leave stationary 5 min: z must not drift. Then stop the recording (Ctrl-C).
6. While recording, re-run the rate checks above: /Odometry must stay ~10 Hz. If the
   recording drops it, record only /livox/lidar and /livox/imu (FAST-LIO replays them).
7. Paste the entry the recorder prints into test_data/manifest.yaml.
EOF
