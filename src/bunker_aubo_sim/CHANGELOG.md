# Changelog

## 1.2.1

* Repository files for GitHub: README overview, CONTRIBUTING, CITATION.cff, issue and pull
  request templates, .gitattributes, CI on push to main / pull requests / manual run,
  `tools/publish_to_github.sh`.

## 1.2.0

* New package `bunker_aubo_navigation`: Nav2 with Smac Planner 2D, Regulated Pure Pursuit, 3D voxel
  costmaps from the self-filtered VLP-16, velocity smoother and the default recoveries.
  Localization by RTAB-Map (live SLAM or a saved database) or AMCL on a saved 2D map.
* `nav_demo.launch.py` (sim + GUI + localization + Nav2), `navigation.rviz`, named goals and the
  `go_to` tool (single goal, x y yaw, or a patrol lap). No changes to the other packages.

## 1.1.1

* Gripper: the slider now moves the gripper when it is released (or shortly after a click or
  arrow key), Open/Close keep the slider in step, and the slider starts at the measured opening.
* Gripper action server runs only the newest command (a new one preempts the running one), times
  the move in sim time, and reports reached / holding object / did not reach correctly. Results of
  replaced commands no longer overwrite the GUI status.

## 1.1.0

* Operator GUI: the 3D LiDAR view now shows the robot model (URDF meshes posed from
  `/joint_states`), and the cloud is drawn in `base_footprint` so it lines up with the robot.
* SLAM helper-node parameters moved from `slam.launch.py` to `config/slam_nodes.yaml`.
* `tools/run_bunker.sh` and `tools/collect_deliverables.sh`, `fill_results` report writer.
* TF tree and architecture diagrams in `docs/images`.

## 1.0.1

* **Arm collision.** The spawn/stowed pose had the wrist inside the upper arm; it is now
  `0, 1.0, 2.75, 1.6, 1.571, −1.571` (collision-free, camera upright and forward). The GUI refuses
  arm targets or paths that would hit the tracks, deck, mast, LiDAR, cameras, floor or the arm itself
  (`bunker_aubo_description/self_collision.py`).
* **Track collision.** Each track now has an upper run plus sloped front/rear collision boxes
  matching the mesh, instead of one box that left the track ends without collision.
* **Map artefacts while driving.**
  * Ground is segmented by surface normals (`Grid/NormalsSegmentation`), so small body pitch
    while driving no longer turns far floor hits into ghost obstacles.
  * The EKF no longer fuses the track-odometry yaw rate (skid-steer slip made it wrong and smeared
    walls on every turn); heading comes from the IMU gyro.
  * `lidar_self_filter` removes LiDAR hits on the arm before RTAB-Map (`/lidar/points_filtered`),
    so an extended arm does not leave a trail in the map.
  * `multi_rgbd_sync` uses sim time, so a slow simulation no longer blanks healthy cameras.
  * The SLAM RViz no longer draws the drifting EKF-odometry arrow trail in the map frame.
* **Tracks look like they move.** The BUNKER mesh (body + tracks) is one rigid visual, so the belts
  could not move. Each road wheel now has a hub cap with orange spokes on the outside of the track
  (visible turning in Gazebo and RViz), and `track_visualizer` draws tread bars on `/track_treads`
  that run around each track with the measured wheel rotation (RViz).
* `/map` is republished on every RTAB-Map update (`map_always_update`), so it never goes stale
  while the robot stands still.


## 1.0.0

First complete version: simulation, operator GUI and multi-sensor SLAM.
