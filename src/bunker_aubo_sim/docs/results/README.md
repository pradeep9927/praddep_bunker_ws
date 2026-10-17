# Results

Outputs of the final recorded run in the `indoor_office` world. The numbers below are copied
from `evaluation.json`, written by `slam_evaluator` against the Gazebo ground truth pose.

## Recordings

| What | Where |
|---|---|
| Demo video (5 to 10 min) | add link |
| ROS 2 bag (mcap, zstd) | add link |
| RTAB-Map database `office.db` | add link |

## Saved maps

* `maps/office.pgm`, `maps/office.yaml`: 2D occupancy grid (load with `load_map.launch.py`)
* `maps/office_cloud.ply`: registered 3D cloud (MeshLab / CloudCompare)
* `maps/office.db`: RTAB-Map database, reload with `slam.launch.py localization:=true database_path:=<path>/office.db`
* `maps/trajectory.png`, `maps/evaluation.md`: trajectory plot and full report

## Accuracy against Gazebo ground truth

| Metric | Value |
|---|---|
| Path length | not measured m |
| Loop closures / proximity detections | not measured / not measured |
| SLAM ATE RMSE (anchored at start) | not measured m |
| SLAM final position error | not measured m (not measured % of path) |
| SLAM heading error, mean / max | not measured / not measured deg |
| EKF odometry only, ATE RMSE (anchored) | not measured m |
| EKF odometry only, final error | not measured m |
| Largest trajectory jump beyond ground truth | not measured m |
| Map: median distance of occupied cells to true obstacles | not measured m |
| Map: occupied cells within 10 cm | not measured % |
| Map: wall coverage (15 cm tolerance) | not measured % |

No `evaluation.json` was found, so no accuracy numbers are reported yet.
