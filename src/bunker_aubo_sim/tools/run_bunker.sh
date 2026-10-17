#!/usr/bin/env bash
# Build the workspace and start simulation + operator GUI + SLAM in one go (demo.launch.py).
#
#   tools/run_bunker.sh                 build, then launch
#   tools/run_bunker.sh --no-build      launch only
#   tools/run_bunker.sh --clean         remove build/ install/ log/ first
#   tools/run_bunker.sh rviz:=false     anything else is passed to the launch file
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$REPO/../.." && pwd)"
CLEAN=0
BUILD=1
ARGS=()
for a in "$@"; do
  case "$a" in
    --clean) CLEAN=1 ;;
    --no-build) BUILD=0 ;;
    *) ARGS+=("$a") ;;
  esac
done

echo "==> Stopping leftovers of a previous run"
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros2 launch bunker_aubo" 2>/dev/null || true
pkill -f "rtabmap" 2>/dev/null || true
pkill -f "operator_gui" 2>/dev/null || true
sleep 1

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true

cd "$WS"
if [ "$CLEAN" = 1 ]; then
  echo "==> Removing build/ install/ log/ in $WS"
  rm -rf build install log
fi
if [ "$BUILD" = 1 ]; then
  echo "==> colcon build in $WS"
  colcon build --symlink-install
fi
# shellcheck disable=SC1091
source install/setup.bash

echo "==> ros2 launch bunker_aubo_bringup demo.launch.py ${ARGS[*]}"
exec ros2 launch bunker_aubo_bringup demo.launch.py "${ARGS[@]}"
