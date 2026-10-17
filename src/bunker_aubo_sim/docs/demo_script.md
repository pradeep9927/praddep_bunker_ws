# Demo video plan (5–10 min)

Recording: OBS (display capture of the WSLg windows) or Game Bar (`Win + Alt + R`), 1080p 30 fps.
Terminal on one side, GUI / RViz / Gazebo on the other.

| # | Time | Action | What to say / show |
|---|---|---|---|
| 1 | 0:00 | Open a **new** terminal. `cd ~/bunker_ws && source install/setup.bash` | clean start, no running nodes: `ros2 node list` is empty |
| 2 | 0:20 | `ros2 launch bunker_aubo_bringup sim.launch.py` | Gazebo opens the office world; the robot spawns with the arm stowed; the controllers load (`ros2 control list_controllers`) |
| 3 | 1:00 | RViz: enable the **TF** display. Second terminal: `ros2 run tf2_tools view_frames`, open the PDF | complete tree `odom→base_footprint→…→sensors`; optical frames; wrist camera under `tool0` |
| 4 | 1:40 | Third terminal: `ros2 launch bunker_aubo_gui gui.launch.py`. Drive with the pad and W/A/S/D, change the speed slider, hit **E-STOP** and release it | base moves, measured velocity follows the command, E-STOP holds zero |
| 5 | 2:30 | GUI arm panel: **Ready**, **Inspect**, one joint slider + *Move to target*, then **Stowed**. Gripper **Close**/**Open** | joint readouts follow; targets clamp at the limits (try moving a slider to the end) |
| 6 | 3:10 | Point at both camera views; tick *show depth* for one | both raw RGB streams live at the same time |
| 7 | 3:20 | Point at the 3D LiDAR view (rotate it with the mouse) and the status table | rates, OK states |
| 8 | 3:40 | Fourth terminal: `ros2 launch bunker_aubo_slam slam.launch.py`. Drive the loop (north corridor → east → open office → back west) | `/map` grows in RViz, the 3D cloud builds, the trajectory is drawn |
| 8b | (optional) | `ros2 service call /slam/wrist_camera/enable std_srvs/srv/SetBool "{data: false}"`, keep driving, then `true` | GUI and `/slam/camera_status` show the drop-out; mapping continues |
| 9 | 6:30 | Return to the start rug | `LOOP CLOSURE #1` in the log, GUI counter increases, the path snaps back in RViz |
| 10 | 7:00 | `ros2 run bunker_aubo_slam save_map.sh office` → Ctrl+C the SLAM launch when asked | pgm/yaml/ply/db written; show `evaluation.md` (ATE, map accuracy) and `trajectory.png` |
| 11 | 8:00 | Ctrl+C everything. Restart `sim.launch.py` (+ GUI), then `ros2 launch bunker_aubo_slam slam.launch.py localization:=true database_path:=~/bunker_maps/office.db` | the saved map appears at once; drive a little and the robot localises (`/rtabmap/localization_pose`) |
| 12 | 9:00 | (optional) `ros2 launch bunker_aubo_slam load_map.launch.py map:=~/bunker_maps/office.yaml` | the same 2D map served by nav2 map_server |

Bag for the deliverables (during step 8, in another terminal):
`ros2 run bunker_aubo_bringup record_bag.sh 90`
