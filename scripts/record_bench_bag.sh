#!/usr/bin/env bash
# filepath: scripts/record_bench_bag.sh
# Bench recording for Gate 2/3 replay tests (masterplan §5). BENCH ONLY: never during
# flight (flight-time logging is flight_log_session.sh on wp-g-ops).
#
# Usage: record_bench_bag.sh <gate2|gate3> <label>      e.g. record_bench_bag.sh gate2 walk
# Writes test_data/<YYYY-MM-DD>_<gate>_<label>/ (git-ignored) and, on Ctrl-C, prints the
# entry to paste into test_data/manifest.yaml (tracked).
#
# Topics: raw driver streams (replay input) plus FAST-LIO outputs and TF (reference);
# gate3 adds the PX4 estimate, the bridge output and timesync status. About 1 GB per
# 5 min, dominated by /livox/lidar.
# Rule conflict, resolved in favour of masterplan §5: the Gates 1-4 spec forbids
# recording raw /livox/lidar while other pipeline components run; masterplan §5
# (bench-specific) instructs it for Gate 2/3 bench sessions. Check /Odometry stays
# ~10 Hz while recording; if it doesn't, record only /livox/lidar and /livox/imu.
# DDS environment: this must match the running stack. If the bench stack was started
# loopback-only (ROS_LOCALHOST_ONLY=1 with CYCLONEDDS_URI unset, e.g. because the WiFi
# roamed and CycloneDDS bound a stale address), run this the same way:
#   env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 scripts/record_bench_bag.sh gate2 walk
# Otherwise it discovers nothing and silently records or measures nothing.
set -eo pipefail
source ~/colcon_ws/install/setup.bash
set -u   # only after sourcing: ROS setup.bash references unbound variables

gate="${1:-}"
label="${2:-}"
case "$gate" in
  gate2|gate3) ;;
  *) echo "usage: $0 <gate2|gate3> <label>" >&2; exit 2 ;;
esac
[[ "$label" =~ ^[a-z0-9_-]+$ ]] || { echo "label must match [a-z0-9_-]+" >&2; exit 2; }

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name="$(date +%F)_${gate}_${label}"
out="$ROOT/test_data/$name"
[ -e "$out" ] && { echo "already exists: $out" >&2; exit 1; }
free_gb=$(df -BG --output=avail "$ROOT" | tail -1 | tr -dc '0-9')
[ "$free_gb" -ge 10 ] || { echo "only ${free_gb} GB free; need at least 10 GB" >&2; exit 1; }
mkdir -p "$ROOT/test_data"

topics=(/livox/lidar /livox/imu /Odometry /cloud_registered /tf /tf_static)
if [ "$gate" = gate3 ]; then
  topics+=(/fmu/out/vehicle_odometry /fmu/in/vehicle_visual_odometry /fmu/out/timesync_status)
fi

echo "Recording to $out  (Ctrl-C to stop)"
echo "Topics: ${topics[*]}"
started="$(date -Iseconds)"
trap 'true' INT   # a handler (not ignore) so ros2 bag still gets Ctrl-C and this script continues
ros2 bag record -o "$out" "${topics[@]}" || true
trap - INT

cat <<EOF

Paste under 'bags:' in test_data/manifest.yaml, then fill in description and notes:
  - id: $name
    gate: $gate
    path: test_data/$name
    started: $started
    stopped: $(date -Iseconds)
    size: $(du -sh "$out" 2>/dev/null | cut -f1)
    topics: [$(IFS=,; echo "${topics[*]}")]
    description: ""   # e.g. hand-carry loop, return to mark, 5 min stationary
    notes: ""         # lidar mount state, FC warm-up time, anything unusual
EOF
