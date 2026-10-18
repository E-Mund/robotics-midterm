# Midterm Programming Assignment
## Running instructions

From the workspace root (this directory):

    ./run.sh

This builds the workspace and starts the whole sim: Gazebo (House world, TurtleBot3
waffle), slam_toolbox, Nav2, RViz and `sub`. Ctrl-C stops everything.

Options:

    ./run.sh --scenario both           # also send goals once Nav2 is ready
    ./run.sh --no-rviz                 # skip RViz
    ./run.sh --no-build                # skip colcon build
    ./run.sh x_pose:=1.0 y_pose:=2.0   # other args go to the launch file (spawn point)

Scenarios (`midterm/midterm/goal_publisher.py`): `goal1_only`, `goal2_only`, `both`,
`goal2_then_goal1`, `emergency`. Default goals are reachable from the default spawn.

To send goals by hand from another terminal (after `source install/setup.bash`):

    ros2 topic pub --once /goal1 geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -3.8, y: -1.0}}"

Goals outside the area SLAM has mapped so far fail to plan (Nav2's global costmap is
only as big as the current map). The House world spans about x = -7.6..7.6, y = -5.4..5.4.

Requires ROS 2 Humble with `turtlebot3_gazebo`, `slam_toolbox` and `nav2_bringup`.
The script sources `/opt/ros/humble/setup.bash` if ROS isn't already sourced.
Nav2 and slam_toolbox settings are in `midterm/config/`.
