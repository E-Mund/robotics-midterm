# team1_nav — TurtleBot3 two-goal navigation (ROS 2 Humble)

Namespaced TurtleBot3 **waffle_pi** in the Gazebo House world. The robot drives to `goal1`, then `goal2`, avoiding static and dynamic obstacles. That includes small objects on the floor, which the LIDAR cannot see; an OpenCV camera detector handles those.

Every topic is under the team namespace (default `team1`), including TF: `/team1/odom`, `/team1/scan`, `/team1/camera/image_raw`, `/team1/cmd_vel`, `/team1/tf`, `/team1/map`, `/team1/goal1`, `/team1/goal2`, and so on.

## Architecture

```
Gazebo (house world) ── /team1/scan, /team1/odom, /team1/camera/*, /team1/tf
        │
        ├─ slam_toolbox (online async) ──> /team1/map, map->odom TF
        ├─ floor_obstacle_detector (OpenCV) ──> /team1/camera_obstacles/{points,free}
        │                                         └─> Nav2 costmap "camera_layer"
        ├─ Nav2: planner (NavFn, plans through unknown space), controller (DWB),
        │        behaviors (spin/backup), bt_navigator (replans at 1 Hz), velocity_smoother
        ├─ mission_manager ── /team1/goal1, /team1/goal2 (geometry_msgs/Point) ──> NavigateToPose
        └─ cmd_vel_gate ── vetoes forward motion if LIDAR/camera sees something right ahead ──> /team1/cmd_vel
```

| Node | Role |
|---|---|
| `mission_manager` | State machine IDLE → NAV_GOAL1 → WAIT_GOAL2 → NAV_GOAL2 → DONE. It publishes `/team1/mission_state`. |
| `floor_obstacle_detector` | Appearance-based floor segmentation, with a column-wise contact point projected onto the ground plane, published as PointCloud2. |
| `cmd_vel_gate` | Last-resort safety stop. It is the only publisher of `/team1/cmd_vel`. It sends a single zero command when Nav2 goes idle. |

### Required behaviours

| Input | Result |
|---|---|
| No goals | Nothing is sent to Nav2 and the robot stays put. |
| Only `/goal1` | The robot drives to goal1, then holds position until goal2 arrives. |
| Only `/goal2` | The goal is stored and the robot does not move until goal1 arrives. |
| Both | The robot goes to goal1, then goal2. |
| New goal1 at any time | Pre-empts the current motion and restarts the sequence. |
| Identical goal re-published | Ignored, so a finished mission is not restarted. |
| Nav2 failure | Costmaps are cleared and the goal is retried (up to `max_retries`). |

### Dynamic obstacles / small floor objects
* LIDAR feeds the `obstacle_layer` of both costmaps, with marking and clearing, so moving objects get cleared when they leave.
* The camera feeds a **separate** `camera_layer`. It marks from `camera_obstacles/points` and clears from `camera_obstacles/free` (rays to the visible free floor). It is a separate layer because LIDAR rays pass *over* small objects and would otherwise erase them.
* The BT replans at 1 Hz and DWB avoids locally. The camera can't see the floor closer than about 0.28 m, so `cmd_vel_gate` keeps camera detections in an odometry-based memory for a few seconds to cover that blind zone.

## Build & run

```bash
sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-slam-toolbox \
                 ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge python3-opencv
mkdir -p ~/ws/src && cp -r team1_nav ~/ws/src/ && cd ~/ws
colcon build --symlink-install && source install/setup.bash

# everything (House world, robot at (-2, 1)):
ros2 launch team1_nav bringup.launch.py namespace:=team1
# other world / spawn pose for evaluation:
ros2 launch team1_nav bringup.launch.py world:=/path/to/other.world x_pose:=0.0 y_pose:=0.0
# or in two terminals: sim_house.launch.py, then navigation.launch.py
```

Send goals in Gazebo world coordinates. Pick free points in your world; these are examples only:
```bash
ros2 topic pub --once /team1/goal1 geometry_msgs/msg/Point "{x: 0.5, y: 1.0, z: 0.0}"
ros2 topic pub --once /team1/goal2 geometry_msgs/msg/Point "{x: 3.0, y: 2.0, z: 0.0}"
ros2 topic echo /team1/mission_state
```

To watch in RViz (TF is namespaced):
```bash
rviz2 --ros-args -r /tf:=/team1/tf -r /tf_static:=/team1/tf_static -p use_sim_time:=true
```
Useful displays are `/team1/map`, `/team1/global_costmap/costmap`, `/team1/local_costmap/costmap`, `/team1/camera_obstacles/points`, and the image `/team1/camera_obstacles/debug`.

## Configuration notes
* **Goal topic names.** The defaults are relative (`goal1` resolves to `/team1/goal1`), as the namespace requirement implies. If evaluators publish on a global `/goal1`, set `goal1_topic: /goal1` in `config/team_nav_params.yaml`.
* **Goal frame.** The Gazebo diff-drive plugin reports odometry in world coordinates, so `odom` equals the world frame and goals need no offset. To verify, right after spawning, `ros2 topic echo /team1/odom --once` should show the spawn pose, e.g. (-2, 1). If it shows (0, 0) instead, set `world_origin_x/y/yaw` to the spawn pose.
* **Namespace.** Change it with `namespace:=teamX`. The parameter template substitutes `TEAM_NS` automatically.
* **Detector tuning.** Use `backproject_threshold`, `lightness_threshold`, `min_run`, and `max_range`. Check `/team1/camera_obstacles/debug`; red is the obstacle mask and green dots are contact points.
* **Model.** `waffle_pi` is required, because burger has no camera.

## Status / limitations
* The vision pipeline was unit-tested offline. A synthetic 5 cm box at 0.80 m was detected at 0.83 m, with no false positives on a textured floor. The goal state machine was tested against all four required behaviours with Nav2 mocked. The full stack still needs to be run in Gazebo before submission, and Nav2/detector parameters may need tuning there.
* Appearance-based detection assumes the floor directly in front of the robot is free when the model is learned. Floors with strong colour patterns, or obstacles the same colour as the floor, can cause misses or false detections. The model re-learns when the floor changes and the LIDAR confirms free space.

## Sources
* TurtleBot3 simulation (Humble): ROBOTIS `turtlebot3_simulations` — house world, waffle_pi model and launch files. https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/ and https://github.com/ROBOTIS-GIT/turtlebot3_simulations
* Nav2 (Humble) documentation and `nav2_bringup` params/launch files (namespacing, `/tf` remaps, costmap observation sources, DWB config). https://docs.nav2.org
* slam_toolbox, `mapper_params_online_async.yaml`. https://github.com/SteveMacenski/slam_toolbox
* gazebo_ros plugin `<ros><namespace>/<remapping>` conventions. https://github.com/ros-simulation/gazebo_ros_pkgs/wiki
* I. Ulrich & I. Nourbakhsh, "Appearance-Based Obstacle Detection with Monocular Color Vision", AAAI 2000. This is the basis of the floor-obstacle detector.
* OpenCV histogram back-projection tutorial. https://docs.opencv.org/4.x/dc/df6/tutorial_py_histogram_backprojection.html
* ROS 2 rclpy action client tutorial (Humble). https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
* Code written with the assistance of Claude (Anthropic AI assistant).
