#!/usr/bin/env bash
# filepath: scripts/run_autonomy.sh
# systemd wrapper: geofence watchdog always; map, planner and explorer per
# /etc/default/drone-stack. Aerostack2 is drone-as2.service.
source "$(dirname "$0")/drone_env.sh"
exec ros2 launch drone_bringup autonomy.launch.py lio:="$DRONE_LIO" \
  mapping:="$DRONE_MAPPING" explorer:="$DRONE_EXPLORER" save_map:="$DRONE_SAVE_MAP"
