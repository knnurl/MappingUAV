#!/usr/bin/env bash
# filepath: scripts/verify_gate2.sh
# Pass criteria: 10 Hz cloud, ~200 Hz IMU (Mid-360 alternates 4/6 ms), odometry alive,
# return-to-start error < 5 cm, no z drift over 5 min stationary.
set -e
source ~/colcon_ws/install/setup.bash

echo "--- driver streams ---"
timeout 15 ros2 topic hz /livox/lidar    # expect 10 Hz
timeout 15 ros2 topic hz /livox/imu      # expect ~200 Hz

echo "--- LIO output ---"
timeout 15 ros2 topic hz /Odometry       # expect ~10 Hz (topic name verified in laserMapping.cpp)
ros2 topic echo /Odometry --once

echo "--- CPU/memory budget (record numbers in the gate log) ---"
top -b -n 1 | grep -E "fastlio|laserMapping|livox" || true
free -h

cat <<'EOF'
MANUAL TEST (human):
1. Mark start pose on the bench. Power on, let LIO initialize stationary 10 s.
2. Hand-carry the airframe a loop around the room (~2 min), return EXACTLY to mark.
3. ros2 topic echo /Odometry --once  -> position must be within 0.05 m of zero.
4. Leave stationary 5 min: z must not drift.
5. ONLY for this test, a bag may be recorded:
   ros2 bag record -o gate2_walk /Odometry
   (never /livox/lidar unless diagnosing a failure, and never in later gates)
EOF
