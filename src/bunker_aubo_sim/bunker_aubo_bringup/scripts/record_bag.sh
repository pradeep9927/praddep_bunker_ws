#!/usr/bin/env bash
set -euo pipefail
DURATION="${1:-90}"
NAME="${2:-bags/bunker_aubo}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$NAME")"

TOPICS=(
  /clock /tf /tf_static /robot_description
  /joint_states /odom /odometry/filtered /cmd_vel
  /imu/data /lidar/points
  /base_camera/color/image_raw /base_camera/depth/image_rect_raw
  /base_camera/color/camera_info /base_camera/depth/camera_info
  /wrist_camera/color/image_raw /wrist_camera/depth/image_rect_raw
  /wrist_camera/color/camera_info /wrist_camera/depth/camera_info
  /ground_truth/odom
)

echo "Recording ${#TOPICS[@]} topics for ${DURATION}s -> ${NAME}"
timeout --signal=INT "${DURATION}" \
  ros2 bag record -s mcap --storage-preset-profile zstd_fast -o "$NAME" "${TOPICS[@]}" || true
ros2 bag info "$NAME" || true
