# Midterm Programming Assignment
## Running instructions

### Terminal 1: Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-2.5 y_pose:=3.5

### Terminal 2: SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

### Terminal 3: Navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

### Terminal 4: Your node
ros2 run midterm sub --ros-args -p use_sim_time:=true

### Terminal 5: Frontier exploration
ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true autostart:=false control_service_enabled:=true

To send goals by hand from another terminal (after `source install/setup.bash`):

    ros2 topic pub --once /goal1 geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: -3.8, y: -1.0}}"

Goals outside the area SLAM has mapped so far fail to plan (Nav2's global costmap is
only as big as the current map). The House world spans about x = -7.6..7.6, y = -5.4..5.4.

Requires ROS 2 Humble with `turtlebot3_gazebo`, `slam_toolbox` and `nav2_bringup`.
The script sources `/opt/ros/humble/setup.bash` if ROS isn't already sourced.
Nav2 and slam_toolbox settings are in `midterm/config/`.

## Navigation - Roberto Molina
Runs frontier navigation algorithm to explore the enviornment and implements custom goal logic.

## Human detection - Ethan Mund

`human_detector` runs YOLOX on the camera image with OpenCV's `cv2.dnn`, projects the
bottom of the person's box onto the floor to get their map position, and publishes it
once on `human_location` after 5 frames in a row agree. `show_detector:=true` opens a
camera window with the boxes.

## Goal logic - Nathanael Hazard
Worked out goal logics in tandem with Roberto to accept the goals and start movements.

Sources:

- Model: [OpenCV Zoo YOLOX](https://huggingface.co/opencv/opencv_zoo/tree/main/models/object_detection_yolox) (`object_detection_yolox_2022nov.onnx`, Apache-2.0)
- Pre/post-processing based on the OpenCV Zoo demo: [`yolox.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/yolox.py) (anchor grid decoding, NMS) and [`demo.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/demo.py) (letterbox / unletterbox)
