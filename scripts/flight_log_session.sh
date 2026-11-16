#!/usr/bin/env bash
# filepath: scripts/flight_log_session.sh
# Flight-session rosbag recorder (WP-G). Policy [FIXED]:
#  - bounded topic set, NEVER raw clouds in flight;
#  - one directory per session, rotated by timestamp;
#  - refuses to start below 4 GB free disk;
#  - PX4 .ulg on the FC remains the primary flight record, this is auxiliary.
set -euo pipefail

SESSION_ROOT="$HOME/flight_logs"
MIN_FREE_KB=$((4 * 1024 * 1024))   # 4 GB

TOPICS=(
  /Odometry
  /fmu/out/vehicle_odometry
  /fmu/out/vehicle_local_position
  /geofence/alert
  /geofence/state
  # behavior states topic joins here at WP-B integration:
  # /as2/behavior_status
)

free_kb=$(df --output=avail -k "$HOME" | tail -1 | tr -d ' ')
if [ "$free_kb" -lt "$MIN_FREE_KB" ]; then
  echo "FATAL: $(df -h --output=avail "$HOME" | tail -1 | tr -d ' ') free < 4G required. Not recording." >&2
  exit 1
fi

# (sourcing after the disk guard; set -u is safe here because the guard above
# uses only variables this script defines)
set +u
source /opt/ros/humble/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml"
export ROS_DOMAIN_ID=0   # = PX4 UXRCE_DDS_DOM_ID default

session="$SESSION_ROOT/session_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION_ROOT"

echo "Recording ${#TOPICS[@]} topics to $session (bag split at 512 MB)"
exec ros2 bag record --output "$session" --max-bag-size 536870912 "${TOPICS[@]}"
