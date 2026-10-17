#!/usr/bin/env bash
# Records the run-time deliverables while you drive the robot from the GUI:
#   TF tree with SLAM running, ROS 2 bag, saved 2D/3D maps + RTAB-Map database,
#   accuracy report against Gazebo ground truth, filled docs/results/README.md.
#
# Start the system first (tools/run_bunker.sh, or sim + gui + slam launch files),
# then in a new terminal:
#   tools/collect_deliverables.sh [bag_seconds] [map_name] [output_dir]
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SECONDS_BAG="${1:-300}"
NAME="${2:-office}"
OUT="${3:-$HOME/bunker_deliverables}"
MAPS="$HOME/bunker_maps"            # slam_evaluator writes to $MAPS/evaluation by default
mkdir -p "$OUT" "$MAPS"

pause() { read -r -p "$1 [Enter] " _; }

if ! ros2 node list 2>/dev/null | grep -q "^/rtabmap/rtabmap$"; then
  echo "SLAM is not running. Start it first (tools/run_bunker.sh or slam.launch.py)."
  exit 1
fi

echo "==> 1/4 TF tree (with map -> odom)"
TMP="$(mktemp -d)"
(cd "$TMP" && ros2 run tf2_tools view_frames >/dev/null 2>&1) || true
PDF="$(ls "$TMP"/frames_*.pdf 2>/dev/null | head -n1 || true)"
if [ -n "$PDF" ]; then
  cp "$PDF" "$REPO/docs/images/tf_view_frames.pdf"
  cp "${PDF%.pdf}.gv" "$REPO/docs/images/tf_view_frames.gv" 2>/dev/null || true
  echo "    saved docs/images/tf_view_frames.pdf"
else
  echo "    view_frames failed, run it by hand later"
fi
rm -rf "$TMP"

echo "==> 2/4 Bag (${SECONDS_BAG}s). Drive the loop with the GUI now and come back to the start rug."
pause "Ready to start recording?"
(cd "$OUT" && ros2 run bunker_aubo_bringup record_bag.sh "$SECONDS_BAG" "bags/bunker_aubo")
BAG="$(ls -d "$OUT"/bags/bunker_aubo_* 2>/dev/null | tail -n1 || true)"

echo "==> 3/4 Saving maps to $MAPS (stop the SLAM launch with Ctrl+C when asked)"
ros2 run bunker_aubo_slam save_map.sh "$NAME" "$MAPS"

echo "==> 4/4 Copying maps + evaluation into docs/results"
read -r -p "Link to the demo video (leave empty to add later): " VIDEO
read -r -p "Link to the uploaded bag (leave empty to use the local path): " BAGLINK
ros2 run bunker_aubo_slam fill_results --repo "$REPO" --maps "$MAPS" --name "$NAME" \
  --video "$VIDEO" --bag "${BAGLINK:-$BAG}"

echo
echo "Done. Check docs/results/README.md, then commit:"
echo "  cd $REPO && git add docs && git commit -m 'Add results of the final run'"
