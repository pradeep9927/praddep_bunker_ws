#!/usr/bin/env bash
set -euo pipefail
NAME="${1:-office}"
DIR="${2:-$HOME/bunker_maps}"
mkdir -p "$DIR"

echo "==> Exporting 2D + 3D maps from /map and /rtabmap/cloud_map"
ros2 run bunker_aubo_slam export_maps --ros-args -p output_dir:="$DIR" -p name:="$NAME" -p use_sim_time:=true

echo "==> Writing evaluation report (if slam_evaluator is running)"
timeout 20 ros2 service call /slam/save_evaluation std_srvs/srv/Trigger "{}" || echo "   (evaluator not running, skipped)"

DB=""
if ros2 node list 2>/dev/null | grep -q "^/rtabmap/rtabmap$"; then
  DB=$(ros2 param get /rtabmap/rtabmap database_path | sed 's/^String value is: //')
  echo
  echo "RTAB-Map is still running. It writes the database completely when it shuts down."
  echo "Stop the SLAM launch now (Ctrl+C in its terminal). Waiting ..."
  while ros2 node list 2>/dev/null | grep -q "^/rtabmap/rtabmap$"; do sleep 1; done
  sleep 2
fi
DB="${DB:-$HOME/bunker_maps/rtabmap.db}"
if [ -f "$DB" ]; then
  if [ "$(realpath "$DB")" != "$(realpath -m "$DIR/$NAME.db")" ]; then
    cp "$DB" "$DIR/$NAME.db"
  fi
  echo "==> Database saved: $DIR/$NAME.db ($(du -h "$DIR/$NAME.db" | cut -f1))"
else
  echo "!! database $DB not found"
fi
echo
echo "Reload later with:"
echo "  ros2 launch bunker_aubo_slam slam.launch.py localization:=true database_path:=$DIR/$NAME.db"
echo "  ros2 launch bunker_aubo_slam load_map.launch.py map:=$DIR/$NAME.yaml"
ls -lh "$DIR"
