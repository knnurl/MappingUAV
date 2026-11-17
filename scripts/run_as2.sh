#!/usr/bin/env bash
# filepath: scripts/run_as2.sh
# systemd wrapper: Aerostack2 (patched platform, estimator, controller, behaviours).
source "$(dirname "$0")/drone_env.sh"
exec ros2 launch drone_bringup gate5_as2.launch.py
