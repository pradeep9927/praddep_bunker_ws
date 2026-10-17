# Multi-sensor SLAM: design, configuration, failure handling

```
ros2 launch bunker_aubo_slam slam.launch.py                        # mapping
ros2 launch bunker_aubo_slam slam.launch.py localization:=true \
     database_path:=~/bunker_maps/office.db                        # reuse a saved map
```

## 1. Why this architecture

**Choice: robot_localization EKF (tracks + IMU) as the odometry front-end, and RTAB-Map as the
graph-SLAM back-end, fed by the VLP-16 and both D435 cameras.**

| Requirement | How it is met |
|---|---|
| Use IMU, 3D LiDAR, both RGB-D cameras (+ tracks) | every source has a concrete role (table below). The cameras contribute features, registration and map points, not just visualisation |
| Loop closure | RTAB-Map visual bag-of-words (appearance) + LiDAR proximity detection (geometry) |
| 2D map for navigation + 3D map | one node produces both from the same optimised graph |
| Saved map that reloads | the RTAB-Map SQLite database holds the whole graph with raw data. Localization mode reloads it. `pgm/yaml` + `ply` exports are also written |
| Off-the-shelf, ROS 2 Jazzy binaries | `ros-jazzy-rtabmap-ros`, `ros-jazzy-robot-localization` (apt) |

Alternatives considered:

* **slam_toolbox (2D):** robust and simple, but 2D-laser only. It cannot use the 3D LiDAR fully
  and cannot use the cameras. Rejected.
* **LiDAR-inertial odometry (FAST-LIO2, LIO-SAM) + a separate loop closer:** very accurate
  LiDAR odometry, but no RGB-D fusion, no Jazzy binaries, and LIO-SAM needs a 9-axis IMU
  with specific extrinsics. It would make a good *front-end* upgrade later.
* **ORB-SLAM3 / VINS (visual-inertial):** strong with cameras, but it ignores the LiDAR and does
  not produce an occupancy grid.

RTAB-Map is one of the few maintained packages that registers LiDAR and **several** RGB-D
cameras in the same graph. It ships in ROS 2 Jazzy and builds 2D + 3D maps with loop closure
out of the box.

## 2. Role of each sensor

| Source | Consumer | What it contributes | Weakness it covers |
|---|---|---|---|
| Track encoders → `/odom` | EKF | forward velocity, zero lateral velocity | high rate, no drift when driving straight |
| IMU gyro → `/imu/data` | EKF | yaw rate (heading) | track slip makes encoder yaw wrong in turns |
| IMU gravity | RTAB-Map | gravity links in the graph (`Optimizer/GravitySigma`) | keeps the map level. With `Reg/Force3DoF` on a flat floor this is only a consistency term |
| VLP-16 | RTAB-Map | ICP scan-matching on every new node, ICP refinement of loop closures, proximity loop closures, obstacles for `/map` and `cloud_map` | long range, lighting-independent geometry |
| Base D435 | RTAB-Map | visual words for place recognition, 3D-2D (PnP) registration, depth points (coloured) for the grid and 3D map | appearance: distinguishes geometrically similar corridors |
| Wrist D435 | RTAB-Map | the same, from a second, arm-mounted viewpoint. Its extrinsic changes with the arm and is read from TF per image | extra field of view (it looks sideways/backwards when the arm turns); redundancy if the base camera fails |

## 3. Frames and conventions (REP-105)

`map → odom → base_footprint → base_link → sensors`

* `odom→base_footprint` (EKF): continuous and smooth, drifts slowly. Controllers can use it.
* `map→odom` (RTAB-Map): jumps when a loop closes and removes the accumulated drift.
* SLAM pose = `map→base_footprint`, published continuously at 20 Hz (+ 50 Hz odometry).
* `diff_drive_controller` has `enable_odom_tf: false`, so only one node publishes
  `odom→base_footprint`.

## 4. Calibration (extrinsics) and why it matters

All extrinsics come from the URDF ([sensor_mounting.md](sensor_mounting.md)). In simulation they
are exact by construction. Errors on hardware would appear as follows:

* **LiDAR ↔ base:** a yaw error of δ rotates every scan. ICP then "corrects" the odometry by δ
  whenever the robot turns, so the map bends. A 1° error gives ~17 cm of lateral error at 10 m.
* **Camera ↔ base / wrist:** visual loop closures produce transforms offset by the calibration
  error. They then conflict with LiDAR links, and the optimiser either rejects them
  (`RGBD/OptimizeMaxError`) or distorts the graph. Depth points land in the wrong place, so
  walls appear doubled in the grid.
* **Wrist camera:** the chain runs through 6 arm joints. Encoder offsets or a hand-eye error move
  the camera. This needs a hand-eye calibration (e.g. `easy_handeye2`, AX = XB) on the real robot.
* **IMU ↔ base:** a mis-rotated IMU puts a fraction of the yaw rate on the wrong axis. In 2D mode
  the heading is then scaled by cos(tilt) (small). A tilted mount also puts gravity on x/y. That
  is why IMU acceleration is not fused.

## 5. Time synchronisation

| Level | Mechanism |
|---|---|
| Clock | one clock: Gazebo sim time (`/clock`, `use_sim_time: true` everywhere) |
| RGB ↔ depth (same camera) | rendered together, so identical stamps; `rgbd_sync` (approx, ≤ 20 ms) |
| Camera ↔ camera | `multi_rgbd_sync`: closest stamps ≤ 50 ms. Both cameras render on the same sim step, so they are normally equal |
| Cameras ↔ LiDAR | RTAB-Map approximate-time sync (`approx_sync`, queue 30) |
| Sensor ↔ robot pose | TF lookup at the data stamp. Motion between the LiDAR and camera stamps is compensated through odometry. The wrist camera uses the arm configuration at its own stamp |
| Odometry sources | EKF with message history (out-of-order tolerant), 0.2 s sensor timeout |

On hardware the same strategy works with hardware timestamps (PTP/GPS-PPS for the VLP-16,
`global_time_enabled` for RealSense). Clock offsets would show up directly as a pose error of
velocity × offset.

## 6. Failure handling

| Failure | Behaviour |
|---|---|
| **One camera stops** | after 0.5 s `multi_rgbd_sync` substitutes a blank frame (zero depth, black, same intrinsics). RTAB-Map keeps running on LiDAR + the other camera. Recovery is automatic. Test: `ros2 service call /slam/wrist_camera/enable std_srvs/srv/SetBool "{data: false}"` |
| **Both cameras stop** | the group is not published, so RTAB-Map's synchroniser stalls, but TF `map→odom` stays latched and the EKF keeps odometry alive. Mapping resumes when the cameras return. (Extension: a LiDAR-only fall-back rtabmap instance.) |
| **LiDAR stops** | RTAB-Map pauses (the LiDAR is the geometric backbone). The EKF continues. |
| **IMU stops** | EKF `sensor_timeout` → prediction + encoder yaw (worse in turns). The `/imu/data` row in the GUI turns STALE. |
| **Wrong loop closure** | rejected by `RGBD/OptimizeMaxError` (error ratio > 3σ after optimisation) and by `Vis/MinInliers` / `Icp/CorrespondenceRatio` gates |
| **Wheels spinning against a wall** | EKF rejects odometry twists that disagree with the prediction by > 5σ (`odom0_twist_rejection_threshold`); ICP refinement also corrects the node pose |
| **Operator loses connection** | `diff_drive_controller` `cmd_vel_timeout` = 0.5 s stops the base. The GUI publishes zero on release, on E-STOP and when the window loses focus |

## 7. Key parameters (config/rtabmap.yaml)

| Parameter | Value | Why |
|---|---|---|
| `Reg/Strategy` | 2 | visual estimate as the initial guess, then LiDAR ICP refinement. This is the LiDAR ↔ RGB-D registration |
| `Reg/Force3DoF` | true | flat indoor floor: x, y, yaw only (more robust) |
| `RGBD/NeighborLinkRefining` | true | ICP corrects odometry between consecutive nodes |
| `RGBD/ProximityBySpace` | true | geometric loop closures when appearance is ambiguous |
| `Grid/Sensor` | 2 | occupancy from LiDAR **and** both depth cameras |
| `Grid/Footprint*` | 1.10 × 0.85 × 1.6 m | removes the robot's own mast/arm/gripper from the map |
| `Icp/RangeMin` | 0.8 m | ignores self-hits on the arm |
| `RGBD/LinearUpdate` / `AngularUpdate` | 0.1 m / 0.1 rad | a new node only after real motion (no map growth while standing) |
| `Rtabmap/DetectionRate` | 1 Hz | bounds CPU on a WSL laptop/desktop |
| `RGBD/OptimizeMaxError` | 3 | rejects inconsistent loop closures |

## 8. Demonstrating loop closure

1. Start at the spawn pose (west corridor, facing north).
2. Drive north, turn east along the north corridor, south down the east corridor, west through
   the open-plan office, and back north to the start (about 40 m, 2–3 min at 0.3–0.4 m/s).
3. When the robot comes back to the start rug and posters, RTAB-Map reports
   `LOOP CLOSURE #1: node N <-> node M` (slam_evaluator log). The GUI counter increases, and in
   RViz the trajectory (`/rtabmap/mapPath`) visibly snaps back and doubled walls merge.
4. Continue a second lap for more loop and proximity closures.

## 9. Accuracy against ground truth

`slam_evaluator` (on by default) compares the SLAM pose (TF `map→base_footprint`) and the EKF
odometry with `/ground_truth/odom` (Gazebo's true model pose). The ground truth is never
used by the estimator.

* **Anchored ATE:** the map frame is tied to the true start pose once, and every later pose is
  compared directly. This is the honest number.
* **Aligned ATE:** the best rigid 2D fit (the standard ATE definition, e.g. TUM/KITTI tools).
* **Map accuracy:** each occupied cell of `/map` is transformed into the world. Its distance to
  the nearest true obstacle (walls and furniture parsed from the SDF, 0.1–2.0 m height band) is
  measured. Reported as mean / median / 95th percentile, share within 5/10 cm, and wall
  coverage.

Reports: `~/bunker_maps/evaluation/{evaluation.md,json, trajectory.csv, trajectory.png, loop_closures.csv}`.
Offline: `ros2 run bunker_aubo_slam evaluate_map ~/bunker_maps/office.yaml --world <sdf> --start -7.5 -1.0 1.5708`.

## 10. From simulation to the real robot

| Simulated | Real |
|---|---|
| Gazebo + gz_ros2_control | AgileX BUNKER ROS 2 driver over CAN (`ugv_sdk`; base `/cmd_vel` + `/odom`), AUBO's ROS 2 driver (arm, FollowJointTrajectory), gripper vendor driver |
| gpu_lidar | `velodyne_driver` + `velodyne_pointcloud` → `/lidar/points` (remap) |
| rgbd_camera ×2 | two `realsense2_camera` nodes (namespaces `base_camera`, `wrist_camera`, `align_depth.enable: true`, distinct serials) |
| IMU sensor | BUNKER-internal or external IMU driver → `/imu/data` |
| sim time | wall/hardware time (`use_sim_time:=false`) |

**Needs recalibration or re-tuning on hardware:** all sensor extrinsics (LiDAR/camera/IMU to
`base_link`; hand-eye for the wrist camera), camera intrinsics and depth→colour extrinsic (factory
values), IMU biases and noise densities (Allan variance), track odometry scale and
`wheel_separation_multiplier` (floor-dependent), EKF covariances, time offsets between sensors,
and the RTAB-Map range/footprint filters for the real robot geometry.

## Self-filtering and ground segmentation

`lidar_self_filter` (started by `slam.launch.py`) subscribes to `/lidar/points`, `/joint_states`
and `/robot_description`, computes the arm's collision boxes in the LiDAR frame (inflated by 6 cm)
and republishes the cloud without those points on `/lidar/points_filtered`, which RTAB-Map uses.
Until the URDF and joint states arrive it passes clouds through unchanged.

The occupancy grid separates ground from obstacles by surface normal (`Grid/NormalsSegmentation`,
`Grid/MaxGroundAngle 30°`, ground below 0.20 m, range 6 m). With a pure height threshold of 8 cm,
less than one degree of body pitch while driving lifted floor hits at 6–8 m above the threshold and
they appeared as obstacles around the path.
