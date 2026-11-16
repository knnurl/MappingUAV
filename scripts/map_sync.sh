#!/usr/bin/env bash
# filepath: scripts/map_sync.sh
# Mirrors the in-flight map checkpoints (map_saver_node chunks and the octree
# autosave) off the vehicle while it flies. Run it on the Jetson:
#
#   scripts/map_sync.sh <destination> [period_s]
#
# <destination> is anything rsync accepts: a ground-station path over ssh
# (user@host:/data/maps/), a mounted USB or NAS path, or a cloud bucket mounted
# with rclone. Unfinished files (*.tmp) are never copied, so every file at the
# destination is a complete checkpoint. A dropped link only delays the copy:
# the next pass catches up.
set -euo pipefail

DEST="${1:?usage: map_sync.sh <destination> [period_s]}"
PERIOD="${2:-15}"
SRC="${MAP_EXPORT_DIR:-$HOME/map_exports}"

command -v rsync >/dev/null || { echo "FATAL: rsync not installed" >&2; exit 1; }
mkdir -p "$SRC"
echo "Mirroring $SRC/ -> $DEST every ${PERIOD}s (Ctrl-C to stop)"
while true; do
  if ! rsync -a --partial --exclude '*.tmp' --timeout=20 "$SRC"/ "$DEST"; then
    echo "$(date +%T) sync failed; retrying in ${PERIOD}s" >&2
  fi
  sleep "$PERIOD"
done
