#!/usr/bin/env bash
# filepath: scripts/run_perception.sh
# systemd wrapper: perception = Mid-360 driver + LIO + odometry bridge. The XRCE
# agent is its own unit, so a perception crash never drops the FC link.
source "$(dirname "$0")/drone_env.sh"
exec ros2 launch drone_bringup perception.launch.py lio:="$DRONE_LIO"
