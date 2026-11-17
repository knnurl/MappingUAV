#!/usr/bin/env bash
# filepath: scripts/restart_as2.sh
# Restart Aerostack2 between flights and after ANY Offboard exit or disarm: the
# platform forwards no Aerostack2 command again until it is restarted (offboard
# guard, gate5 runbook K2). Perception, the watchdog and the map keep running,
# so the LIO origin and the map survive.
#
# REFUSES while the vehicle is armed.
set -euo pipefail
set +u; source /opt/ros/humble/setup.bash; source "$HOME/colcon_ws/install/setup.bash"; set -u
armed=$(timeout 5 ros2 topic echo --once --qos-reliability best_effort \
          /fmu/out/vehicle_status px4_msgs/msg/VehicleStatus --field arming_state 2>/dev/null | head -1 || true)
if [ "$armed" != "1" ]; then
  echo "REFUSED: vehicle_status arming_state is '${armed:-unknown}' (1 = disarmed). Land and disarm first." >&2
  exit 1
fi
sudo systemctl restart drone-as2.service
sleep 5
systemctl is-active drone-as2.service
echo "Aerostack2 restarted. Run scripts/preflight_check.py --as2 before arming."
