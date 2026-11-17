# filepath: scripts/drone_env.sh
# Common environment for the systemd wrappers. Sourced, not executed.
# set -u only after sourcing: ROS setup.bash references unbound variables.
set -eo pipefail
[ -f /etc/default/drone-stack ] && . /etc/default/drone-stack
source /opt/ros/humble/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
# The fork's packages, when it is the selected LIO stack.
if [ "${DRONE_LIO:-upstream}" = fork ] && [ -f "$HOME/fastlio2_ws/install/setup.bash" ]; then
  source "$HOME/fastlio2_ws/install/setup.bash"
fi
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/colcon_ws/src/drone_bringup/config/cyclonedds.xml"
export ROS_DOMAIN_ID=0          # = PX4 UXRCE_DDS_DOM_ID default
export PYTHONNOUSERSITE=1       # ~/.local numpy 2 breaks the system scipy
DRONE_LIO="${DRONE_LIO:-upstream}"
DRONE_MAPPING="${DRONE_MAPPING:-false}"
DRONE_EXPLORER="${DRONE_EXPLORER:-false}"
DRONE_SAVE_MAP="${DRONE_SAVE_MAP:-true}"
