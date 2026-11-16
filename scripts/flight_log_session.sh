#!/usr/bin/env bash
# filepath: scripts/flight_log_session.sh
# Flight-session rosbag recorder (WP-G). Policy [FIXED]:
#  - bounded topic set, NEVER raw clouds in flight (/livox/lidar,
#    /cloud_registered and /map_interface/* are not here on purpose);
#  - one directory per session, rotated by timestamp;
#  - refuses to start below 4 GB free disk;
#  - PX4 .ulg on the FC remains the primary flight record, this is auxiliary.
# The environment (ROS_DOMAIN_ID, CYCLONEDDS_URI, ROS_LOCALHOST_ONLY) is taken
# from the calling shell so the recorder always shares the stack's graph.
# Topic names confirmed against the running stack in the rig, 2026-11-16.
set -euo pipefail

NS="${AS2_NAMESPACE:-drone0}"
SESSION_ROOT="$HOME/flight_logs"
MIN_FREE_KB=$((4 * 1024 * 1024))   # 4 GB

TOPICS=(
  # estimation
  /Odometry
  /fmu/out/vehicle_odometry
  /fmu/out/vehicle_local_position
  /fmu/out/vehicle_status
  /fmu/out/vehicle_control_mode
  /px4_odom_bridge/status
  /px4_odom_bridge/state
  /lio_health/state
  /lio_health/alert
  # safety
  /geofence/alert
  /geofence/state
  /geofence/hover_request
  /lio_health/hover_request
  /go_to_with_avoidance/hover_request
  # Aerostack2
  "/$NS/platform/info"
  "/$NS/self_localization/pose"
  "/$NS/motion_reference/pose"
  "/$NS/motion_reference/twist"
  "/$NS/controller/info"
  "/$NS/sensor_measurements/battery"
  "/$NS/alert_event"
  # planner and explorer
  /go_to_with_avoidance/goal
  /go_to_with_avoidance/status
  /go_to_with_avoidance/alert
  /frontier_explorer/state
  /frontier_explorer/status
  /frontier_explorer/alert
)

if [ "${1:-}" = "--list" ]; then printf '%s\n' "${TOPICS[@]}"; exit 0; fi

free_kb=$(df --output=avail -k "$HOME" | tail -1 | tr -d ' ')
if [ "$free_kb" -lt "$MIN_FREE_KB" ]; then
  echo "FATAL: $(df -h --output=avail "$HOME" | tail -1 | tr -d ' ') free < 4G required. Not recording." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
: "${ROS_DOMAIN_ID:=0}"            # = PX4 UXRCE_DDS_DOM_ID default
: "${CYCLONEDDS_URI:=file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml}"
export ROS_DOMAIN_ID CYCLONEDDS_URI

session="$SESSION_ROOT/session_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION_ROOT"
echo "Recording ${#TOPICS[@]} topics to $session (bag split at 512 MB), domain $ROS_DOMAIN_ID"
exec ros2 bag record --output "$session" --max-bag-size 536870912 "${TOPICS[@]}"
