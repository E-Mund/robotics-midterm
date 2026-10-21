# House parameters — 2026-10-21

The coverage profiles use the default ROS namespace. Wildcard node selectors
and `/roborob/scan` references from the namespace migration were reverted.
The explorer still filters against the map-frame global costmap; Nav2 retains
its separate odom-frame local costmap.

Two tuning changes were retained for the simulated Waffle house workflow:

- SLAM Toolbox `do_loop_closing: false`: the original run developed distorted
  walls, failed navigation near the southeast doorway, and reported completion
  while that room was still unmapped. Disabling loop closure preserved the
  upper-middle room and allowed the robot to enter the southeast room. Local
  scan matching remains enabled. This is a simulation setting; real-robot
  odometry drift and long mapping sessions need separate validation.
- DWB `FollowPath.BaseObstacle.scale: 0.08`, previously `0.02`: gives obstacle
  clearance more influence. A no-loop-only run contacted a café table's low
  base and developed a false wall in the southeast room. The more aggressive
  weight `0.5` caused repeated stalls and was rejected. The retained `0.08`
  run had no recorded obstacle contacts or navigation/progress failures.

The explorer's completion signal means its search found no eligible frontiers;
it does not prove every room or every observable floor cell was mapped. With
return-to-start disabled, its completion latch stops further goal selection.
No new recovery or completion logic was added.

## Validation

| Check | Status | Result |
| --- | --- | --- |
| Default namespace and active profiles | PASS | SLAM and both costmaps received `/scan`; no `roborob` topics; all queried nodes used simulation time. |
| Midterm rebuild | PASS | Rebuilt the stale installation; installed subscriber matches the current source. |
| Original tuning explores the entire house | FAIL | 85.04 m² known map cells at 392.70 simulation seconds; southeast room unmapped despite completion. |
| Loop closure disabled, original obstacle weight: room reach | PASS | All six main rooms reached; 98.73 m² known at 450.84 s. |
| Loop closure disabled, original obstacle weight: complete fix | FAIL | Remaining southeast patch and table-base contact prevented acceptance as a complete fix. |
| Retained settings: loop closure off, weight 0.08 | PASS | Fresh 450.84 s run, 27 reached goals, zero aborted Nav2 goals, zero progress failures, zero recorded obstacle contacts, 95.86 m² known map cells. |
| Full exploration completion with retained settings | NOT TESTED | The 450 s test horizon ended while exploration was still active in the southeast room. Further verification was stopped at the user's request. |
| Exhaustive observable-cell coverage / repeatability | NOT TESTED | Known-map area includes occupied cells and is not a floor coverage percentage. |

Evidence remains in the parent workspace under
`log/coverage-default-namespace-rebuilt-20261021/`,
`log/coverage-no-loop-20261021/`, `log/coverage-clearance-20261021/` and
`log/coverage-final-20261021/`. Raw logs/environments are not committed here.
The benchmark now starts the explorer before `sub`, preserves completion events
on interruption, and counts the installed explorer's aborted-action messages.

## Run commands

From `~/ros2ws`, stop previous launches first. Use one terminal per command in
the order below. Wait for Nav2's `Managed nodes are active` message before
starting terminal 5. No extra namespace or remapping arguments are needed.
The YAML files are read directly from source; restart nodes to apply changes.

```bash
# Terminal 1
pixi run run ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-2.5 y_pose:=3.5

# Terminal 2
pixi run run ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=src/midterm/midterm/config/slam_coverage.yaml

# Terminal 3
pixi run run ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=src/midterm/midterm/config/nav2_coverage.yaml

# Terminal 4
pixi run frontier params_file:=src/midterm/midterm/config/frontier_coverage.yaml

# Terminal 5
pixi run run ros2 run midterm sub --ros-args -p use_sim_time:=true
```
