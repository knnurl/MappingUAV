#!/usr/bin/env bash
# filepath: scripts/verify_gate3.sh
# Pass criteria: PX4 local position tracks LIO within noise, correct signs,
# no EKF resets, correct frame conversion verified by directional walk test.
set -e
source ~/colcon_ws/install/setup.bash

echo "--- bridge node alive and publishing ---"
timeout 15 ros2 topic hz /fmu/in/vehicle_visual_odometry   # expect ~10 Hz

echo "--- sole EV writer (expect 'Publisher count: 1', node px4_odom_bridge) ---"
ros2 topic info -v /fmu/in/vehicle_visual_odometry

echo "--- PX4 estimator output (BEST_EFFORT: ros2 topic hz cannot subscribe to it, use the probe) ---"
python3 "$(dirname "$0")/fmu_rate_probe.py" --duration 15 /fmu/out/vehicle_odometry   # expect ~100 Hz
ros2 topic echo /fmu/out/vehicle_odometry --once

cat <<'EOF'
MANUAL TESTS (human, FC powered, props OFF):

A. Sign/axis test. This is the single most failure-prone step in the whole build.
   Convention (pinned by src/px4_odom_bridge/test/test_bridge_core.py): FAST-LIO's
   world x axis is the vehicle's STARTUP forward direction, and the bridge maps it
   to PX4 EAST. PX4 North = startup LEFT. Heading reads ~ +pi/2 right after LIO
   starts. Keep the airframe at its startup heading for steps 1-3.
   1. Carry airframe +1 m in the direction the FC arrow points (body forward, level).
      -> /fmu/out/vehicle_odometry position[1] (E) increases ~ +1.0; position[0] ~ unchanged.
   2. Carry +1 m to the vehicle's right.
      -> position[0] (N) DECREASES ~ -1.0; position[1] ~ unchanged.
   3. Lift +0.5 m.
      -> position[2] (D) DECREASES ~ -0.5. If it increases, the z negation is wrong.
   4. Yaw the vehicle 90 deg clockwise viewed from above.
      -> heading (from q) increases by +pi/2 (from ~ +pi/2 to ~ pi) in NED convention.
   A result that differs from THESE expectations is a real sign error => fix
   odom_bridge_node (and its unit tests), do NOT compensate in EKF2 params.

B. PX4 MAVLink console checks (QGC):
   listener vehicle_visual_odometry   # message arriving in uORB, sane values;
                                      # timestamp - timestamp_sample = small POSITIVE
                                      # LIO processing latency (tens of ms). ~0 or
                                      # negative => timestamp_sample path is wrong.
   listener estimator_status_flags    # cs_ev_pos and cs_ev_yaw must be true [VERIFY flag names]
   listener vehicle_local_position    # xy_valid, z_valid true; values track LIO
   ekf2 status

C. Log review: download the .ulg, load in Flight Review (logs.px4.io).
   - Zero "EKF reset" events during a 5 min hand-carry.
   - Innovations bounded; no EV timeout warnings.
   - Use the log to refine EKF2_EV_DELAY (cross-correlate EV vs IMU-derived motion).
     It is a RESIDUAL now (Ethernet + driver arrival latency): the bridge already
     stamps timestamp_sample with the LIO scan time. Expect single-digit ms.
EOF
