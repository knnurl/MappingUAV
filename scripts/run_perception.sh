#!/usr/bin/env bash
# filepath: scripts/run_perception.sh
# systemd wrapper: perception pipeline = gate2 (livox + FAST-LIO) + odom
# bridge. Not gate3_fusion.launch.py: the XRCE agent is a separate unit.
# set -u only after sourcing: ROS setup.bash references unbound variables.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml"
export ROS_DOMAIN_ID=42

exec ros2 launch drone_bringup perception.launch.py
