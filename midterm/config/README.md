# House navigation configuration

`nav2_params.yaml` uses a rotation shim around DWB and matching 1 m inflation
gradients in both costmaps. The stopped threshold is 0.05 m/s and forward speed
remains capped at 0.26 m/s. Stationary path alignment triggers at a 1.2 rad heading
error, samples the path 0.8 m ahead, and turns at up to 1.0 rad/s. This reduces
the previous profile's frequent, slow alignment turns. `frontier_explorer.yaml` uses the Waffle's 3.5 m LiDAR
range and 0.26 m/s speed estimate. These settings aim to reduce stalls and improve
path tracking; they do not guarantee continuous motion or eliminate revisits.

From the workspace root, in each terminal, activate Pixi and the local overlay:

```bash
pixi shell
source install/local_setup.bash
```

Then run one command per terminal. Start the subscriber after the frontier service
exists and Nav2 reports that its managed nodes are active.

```bash
# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-2.5 y_pose:=3.5
# Terminal 2
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
# Terminal 3
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=src/midterm/midterm/config/nav2_params.yaml
# Terminal 5: before terminal 4
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true autostart:=false control_service_enabled:=true params_file:=src/midterm/midterm/config/frontier_explorer.yaml
# Terminal 4
ros2 run midterm sub --ros-args -p use_sim_time:=true
```

These commands load source YAML directly, without helper scripts. Editing config
files does not affect already running nodes: restart the launches to apply edits.
The original commands without `params_file` continue to use upstream defaults.

`ros2 launch midterm test_env.launch.py` instead loads the installed copies and
also starts the human detector and the house world containing a human. After
editing source YAML, update its installed copies from the workspace root:

```bash
pixi shell
source install/local_setup.bash
colcon build --build-base build/separated --base-paths src/midterm --packages-select midterm --executor sequential
```

Check the active settings:

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.trans_stopped_velocity
ros2 param get /frontier_explorer goal_preemption_lidar_range_m
```

Expected: `nav2_rotation_shim_controller::RotationShimController`, `0.05`, `3.5`.

## Initial validation — 2026-10-20

- PASS: YAML values match the previously trialed house profiles; installed YAML
  matches source after the targeted `midterm` build above.
- PASS: live controller reports RotationShimController and 0.05 m/s stopped
  threshold. Bounded simulation from (-2.5, 3.5) ran 120.36 simulation seconds,
  reached 14 frontier goals with zero failed goals, progress failures, or invalid
  trajectories. Odometry distance was 14.22 m; known map area was 23.85 m².
- NOT TESTED: elimination of triangular routes and full-house completion.
  This run does **not** demonstrate increased exploration speed: the earlier
  default run reached 15 goals, traveled 17.07 m, and mapped 30.62 m² in the same
  duration. Different frontier choices and one run per configuration limit the
  comparison; rotation alignment still involves stopping to turn.

Generated trial logs and active parameter query:
`log/midterm-config-navigation-20261020/` at the workspace root.

## Follow-up: reduce stationary alignment

Changed only three shim settings: angular threshold 0.6 → 1.2 rad, heading
sampling distance 0.4 → 0.8 m, rotation velocity 0.6 → 1.0 rad/s. No forward
speed limits, obstacle checks, goal tolerances, or frontier settings changed.
Parameter semantics were checked against the
[Humble shim implementation](https://api.nav2.org/nav2-humble/html/nav2__rotation__shim__controller_8cpp_source.html).

PASS: the revised profile completed 120.36 simulation seconds from the same spawn
with 15 successful frontier goals and zero failed goals, progress failures, or
invalid-trajectory messages. Travel increased from 14.22 to 16.00 m and known
map area from 23.85 to 27.54 m². Turning with nearly zero forward command occurred
in 26/99 one-second samples after startup, versus 35/99 previously.

This is a bounded improvement over the previous shim profile, not proof of
uniformly faster exploration or superiority to upstream defaults. Routes differ,
and each profile has one run. The minimum sampled laser range was 0.354 m versus
0.483 m previously; this is sensor range, not measured body clearance. Nav2 still
logged action-result errors during goal halts, though all completed goals succeeded.
Full-house completion and elimination of the triangular route remain NOT TESTED.

Logs: `log/navigation-motion-tuning/less-rotation-trial/`. The temporary candidate
YAML used for that trial was checked for semantic equality with the final source.

Restart Nav2 to apply changes. Verify after it becomes active:

```bash
ros2 param get /controller_server FollowPath.angular_dist_threshold
ros2 param get /controller_server FollowPath.forward_sampling_distance
ros2 param get /controller_server FollowPath.rotate_to_heading_angular_vel
```

Expected: `1.2`, `0.8`, `1.0`.
