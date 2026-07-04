#!/usr/bin/env bash
# filepath: scripts/verify_gate3.sh
# Pass criteria: PX4 local position tracks LIO within noise, correct signs,
# no EKF resets, correct frame conversion verified by directional walk test.
set -e
source ~/colcon_ws/install/setup.bash

echo "--- bridge node alive and publishing ---"
timeout 15 ros2 topic hz /fmu/in/vehicle_visual_odometry   # expect ~10 Hz

echo "--- PX4 estimator output (BEST_EFFORT QoS via CLI) ---"
timeout 15 ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_odometry --once

cat <<'EOF'
MANUAL TESTS (human, FC powered, props OFF):

A. Sign/axis test. This is the single most failure-prone step in the whole build:
   1. Carry airframe +1 m in the direction the FC arrow points (body forward, level).
      -> /fmu/out/vehicle_odometry position[0] (N) increases ~ +1.0.
   2. Carry +1 m to the vehicle's right.
      -> position[1] (E) increases ~ +1.0.
   3. Lift +0.5 m.
      -> position[2] (D) DECREASES ~ -0.5. If it increases, the z negation is wrong.
   4. Yaw the vehicle 90 deg clockwise viewed from above.
      -> heading (from q) increases by +pi/2 in NED convention.
   Any sign error => fix odom_bridge_node, do NOT compensate in EKF2 params.

B. PX4 MAVLink console checks (QGC):
   listener vehicle_visual_odometry   # message arriving in uORB, sane values
   listener estimator_status_flags    # cs_ev_pos and cs_ev_yaw must be true [VERIFY flag names]
   listener vehicle_local_position    # xy_valid, z_valid true; values track LIO
   ekf2 status

C. Log review: download the .ulg, load in Flight Review (logs.px4.io).
   - Zero "EKF reset" events during a 5 min hand-carry.
   - Innovations bounded; no EV timeout warnings.
   - Use the log to refine EKF2_EV_DELAY (cross-correlate EV vs IMU-derived motion).
EOF
