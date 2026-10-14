# Midterm Programming Assignment
## running instructions:
To run:
  within workspace:
    colcon build

  ALL TERMINALS:
    source /opt/ros/humble/setup.bash
    source install/setup.bash

  Terminal 1:
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

  Terminal 2:
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

  Terminal 3:
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

  Terminal 4:
    rviz2

  Terminal 5:
    ros2 run midterm sub

  Terminal 6:
    ros2 topic pub --once /goal1 geometry_msgs/msg/PointStamped   "{header: {frame_id: map}, point: {x: 0.0, y: 5.0, z: 0.0}}"
