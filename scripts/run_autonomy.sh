#!/usr/bin/env bash
# filepath: scripts/run_autonomy.sh
# systemd wrapper: autonomy layer. TODAY this starts ONLY the geofence
# watchdog; AS2 and the planner join here when WP-B/WP-D integrate (gate5
# token). The watchdog-first ordering is deliberate: it must be up before
# anything can command motion.
# set -u only after sourcing: ROS setup.bash references unbound variables.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml"
export ROS_DOMAIN_ID=0   # = PX4 UXRCE_DDS_DOM_ID default

exec ros2 run geofence_watchdog watchdog_node --ros-args \
  --params-file "$HOME/colcon_ws/src/drone_bringup/config/geofence.yaml"
