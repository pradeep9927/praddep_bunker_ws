# Sensor mounting transforms

Generated from the URDF by `ros2 run bunker_aubo_description print_mounts.py`.
Translation in metres, rotation as roll/pitch/yaw in degrees (fixed-axis XYZ, as in URDF `<origin rpy>`).

| Frame | Parent | xyz [m] | rpy [deg] |
|---|---|---|---|
| `imu_link` | `base_link` | +0.0000 +0.0000 +0.1000 | +0.0000 -0.0000 +0.0000 |
| `lidar_link` | `base_link` | +0.3600 +0.0000 +0.8609 | +0.0000 -0.0000 +0.0000 |
| `base_camera_link` | `base_link` | +0.4050 +0.0000 +0.5450 | +0.0000 +10.0000 +0.0000 |
| `base_camera_color_optical_frame` | `base_camera_link` | +0.0000 +0.0000 +0.0000 | -90.0000 -0.0000 -90.0000 |
| `base_camera_depth_optical_frame` | `base_camera_link` | +0.0000 +0.0000 +0.0000 | -90.0000 -0.0000 -90.0000 |
| `wrist_camera_link` | `tool0` | -0.0550 +0.0000 +0.0400 | +0.0000 -90.0000 +0.0000 |
| `wrist_camera_color_optical_frame` | `wrist_camera_link` | +0.0000 +0.0000 +0.0000 | -90.0000 -0.0000 -90.0000 |
| `wrist_camera_depth_optical_frame` | `wrist_camera_link` | +0.0000 +0.0000 +0.0000 | -90.0000 -0.0000 -90.0000 |
| `aubo_base_link` | `base_link` | -0.1000 +0.0000 +0.2980 | +0.0000 -0.0000 +0.0000 |
| `gripper_tcp` | `tool0` | +0.0000 +0.0000 +0.1320 | -0.0000 +0.0000 +0.0000 |
| `base_link` | `base_footprint` | +0.0000 +0.0000 +0.0750 | +0.0000 -0.0000 +0.0000 |

## Height above ground (base_footprint), arm stowed

| Frame | x [m] | y [m] | z [m] |
|---|---|---|---|
| `imu_link` | 0.000 | 0.000 | 0.175 |
| `lidar_link` | 0.360 | 0.000 | 0.936 |
| `base_camera_link` | 0.405 | 0.000 | 0.620 |
| `wrist_camera_link` | 0.083 | -0.121 | 0.784 |
| `gripper_tcp` | 0.165 | -0.121 | 0.716 |

## Notes

* `base_link` is at wheel-axle height (0.075 m above `base_footprint`). x points forward, y left, z up.
* Camera `*_link` frames use the body convention (x = viewing direction). The
  `*_optical_frame`s use REP-103 optical axes (z forward, x right, y down); image headers carry
  `*_color_optical_frame`.
* In simulation the depth imager coincides with the colour imager (registered depth). On the
  real D435 the depth-to-colour offset is ~15 mm along y (factory calibration).
* The wrist camera is attached to `tool0`, so its pose in `base_link` depends on the arm joints.
  The table above gives it relative to `tool0`. The height table gives it for the stowed pose.
* The LiDAR's lowest beam (−15°) meets the floor 3.5 m from the sensor. Everything below the
  cone that falls inside the robot footprint (the stowed arm) is invisible to it, and self-hits
  closer than 0.8 m are filtered in RTAB-Map anyway (`Icp/RangeMin`, `Grid/Footprint*`).
