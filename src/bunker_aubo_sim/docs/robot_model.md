# Robot model: design notes

## Files

| File | Content |
|---|---|
| `urdf/bunker_aubo.urdf.xacro` | assembly and xacro arguments. Loads `config/sensors.yaml` and `config/initial_positions.yaml` |
| `urdf/common/inertial_macros.xacro` | solid box / cylinder inertia macros |
| `urdf/bunker/bunker_base.xacro` | chassis, 8 road wheels, sensor mast, arm mounting plate |
| `urdf/aubo/aubo_i5.xacro` | AUBO-i5 (manufacturer kinematics + inertias) |
| `urdf/gripper/parallel_gripper.xacro` | two-finger gripper + `gripper_tcp` |
| `urdf/sensors/{imu,vlp16,d435}.xacro` | sensor links + Gazebo sensor definitions (parametrised) |
| `urdf/control/ros2_control.xacro` | hardware interfaces (only when `sim_gazebo:=true`) |
| `urdf/gazebo.xacro` | gz_ros2_control plugin and ground-truth odometry publisher |

View it without Gazebo: `ros2 launch bunker_aubo_description display.launch.py`.

## Tracks: skid-steer approximation

The BUNKER is a rubber-track vehicle. Gazebo Harmonic has a `TrackedVehicle` system that fakes
track motion with contact-surface velocities. It only works with specific collision set-ups, it
does not produce ros2_control joint states, and it does not give wheel/encoder odometry of the kind
the real BUNKER reports. For this project the standard approximation was chosen:

* **4 road wheels per side** (r = 75 mm, 120 mm wide, 170 mm spacing) inside each track, driven as
  one group per side by `diff_drive_controller` (`left_wheel_names`/`right_wheel_names`).
* The **visual** is the real BUNKER body mesh. The **collision** of chassis and tracks starts
  above the ground (90 mm clearance under the chassis, 160 mm under the upper track shell), so
  only the wheels touch the floor.
* **Anisotropic friction** on the wheels: μ1 (lateral) = 0.6 and μ2 (longitudinal) = 1.0.
  The friction direction `fdir1` is set to the axle. DART reads `fdir1` in the *collision
  shape* frame, and the wheel cylinder is rotated by rpy (π/2, 0, 0), so the value is
  `0 0 1` (the cylinder axis). That direction does not rotate with the wheel. A tracked vehicle turns by
  dragging its tracks sideways. Lower lateral friction reproduces that, so the robot turns
  without hopping.
* **Consequence for odometry:** during a turn the tracks slip, so the rotation measured from
  wheel speeds (`(vR − vL) / B`) is larger than the true rotation. The effective track width is
  bigger than the geometric one. Two measures deal with this:
  1. `wheel_separation_multiplier` in `controllers.yaml` can be calibrated. Command a pure
     rotation, compare `/odom` yaw with `/ground_truth/odom`, and set multiplier = ω_odom / ω_true.
  2. The EKF gives encoder yaw rate a large variance and trusts the IMU gyro for heading.

## Mass and inertia

| Part | Mass [kg] | Source |
|---|---|---|
| Chassis + tracks | 110 | BUNKER datasheet (~130 kg total incl. wheels/battery) |
| Road wheels | 8 × 2.0 | estimate |
| AUBO-i5 | 9.9 | official URDF (real arm 24 kg; the manufacturer URDF values are kept) |
| Gripper | 0.7 | estimate |
| Sensors, mast, mount plate | 3.8 | datasheets (VLP-16 0.83 kg, D435 0.072 kg) + estimate |
| **Total** | **~140** | `test_total_mass_is_plausible` |

Inertia tensors come from solid-primitive formulas (box/cylinder) or the manufacturer's values
(arm). A test checks that every tensor is positive definite and satisfies the triangle
inequality. The chassis centre of mass is low (0.19 m above ground), which keeps the robot stable
when the arm is extended.

## Joint limits

| Joint | Lower | Upper | Velocity | Effort |
|---|---|---|---|---|
| shoulder / upperArm / foreArm | −3.04 rad | 3.04 rad | 2.618 rad/s (150°/s) | 150 Nm |
| wrist1 / wrist2 / wrist3 | −3.04 rad | 3.04 rad | 3.142 rad/s (180°/s) | 28 Nm |
| gripper fingers (each) | 0 m | 0.04 m | 0.10 m/s | 60 N |
| road wheels | continuous | | 22 rad/s (1.65 m/s) | 120 Nm |

Limits are enforced in three places: the GUI clamps every target to the URDF limits, Gazebo
enforces the URDF limits physically, and the diff_drive controller limits base velocity and
acceleration.

## Named arm poses (rad; shoulder, upperArm, foreArm, wrist1, wrist2, wrist3)

| Pose | Values | Purpose |
|---|---|---|
| home | 0, 0, 0, 0, 0, 0 | AUBO zero (arm straight up) |
| stowed | 0, 1.0, 2.75, 1.6, 1.571, −1.571 | spawn/transport: folded over the deck, below the LiDAR, wrist camera upright and looking forward (≈9° down, 12 cm beside the mast). The earlier pose (0, 1.2, 2.77, −1.571, −1.571, 0) pushed wrist2/wrist3 about 2 cm into the upper arm |
| ready | 0, 0.3, 1.85, 0, 1.571, 0 | tool pointing down in front of the arm |
| inspect | 0, 0.5, 1.6, 1.0, 1.571, 0 | wrist camera looking forward at ≈ 1.1 m |

All four poses, and every straight joint-space move between them, are collision-free according to
`bunker_aubo_description/self_collision.py` (tested in `test/test_self_collision.py`).

## Collision model and arm guard

* **Tracks:** each track shell is three boxes measured from the BUNKER mesh: the upper run
  (0.16–0.39 m) and the sloped front and rear ends. The lowest corner is 2 cm above the floor,
  so only the eight road wheels touch the ground and the drive behaviour is unchanged.
* **Arm guard:** Gazebo simulates the model with self-collision off (URDF default), so physics
  does not stop the arm from passing through the tracks, the deck, the mast or itself. The
  operator GUI therefore checks every arm command first with `SelfCollisionChecker`
  (collision boxes from the URDF, area-sampled link surfaces, floor clearance 3 cm, 2.5 cm margin
  to the body). A target or path that collides is refused and the colliding links are named in the
  status bar; the arm panel also shows live whether the current slider target is free.

## Sensors

Mounting transforms: [sensor_mounting.md](sensor_mounting.md). Rates and noise:
`config/sensors.yaml`.

* **IMU:** inside the chassis on the centre-line. Gaussian white noise and bias on gyro and
  accelerometer; the orientation is relative to the start pose (no magnetometer).
* **VLP-16:** on top of the front mast (optical centre 0.936 m above the ground). 16 channels,
  ±15° vertical, 900 azimuth samples, 10 Hz, σ = 1 cm range noise, 0.4 m minimum range. The
  mast is in front of the arm, so the stowed arm is behind and below the beam cone.
* **Base D435:** front face of the mast, 0.62 m high, pitched down 10°, so it sees both the floor
  and wall posters.
* **Wrist D435:** on a bracket beside the gripper, looking along the tool axis. It sees the
  fingertips at the image edge, like a real eye-in-hand set-up.
* Both cameras: 640×480 @ 15 Hz, 69.4° HFoV, depth 0.105–10 m, RGB noise σ = 0.007. Depth is
  registered to colour (single Gazebo `rgbd_camera`), so the `*_depth_optical_frame` equals the
  `*_color_optical_frame` in simulation. On the real camera, replace the zero offset with the
  factory depth→colour extrinsic, or use `align_depth` in `realsense2_camera`.
