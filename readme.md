# Midterm Programming Assignment
## Running instructions

From the top-level folder, run `./run.sh`.

Requires ROS 2 Humble with Gazebo Classic, `gazebo_ros`, `turtlebot3_gazebo`, `slam_toolbox`, `nav2_bringup`, `frontier_exploration_ros2`, `cv_bridge`, OpenCV and NumPy.
Runtime settings use `midterm/config/*_coverage.yaml`.

## Navigation - Roberto Molina
Runs frontier navigation to explore the environment and implements custom goal logic. Send goals in another terminal:

```bash
ros2 topic pub --once /goal1 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -3.8, y: 3.5, z: 0.0}}'
ros2 topic pub --once /goal2 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -6.3, y: 3.0, z: 0.0}}'
ros2 topic pub --once /goal3 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -2.5, y: 3.5, z: 0.0}}'
```

## Human detection - Ethan Mund

`human_detector` subscribes to the camera and runs YOLOX with OpenCV.

## Goal logic - Nathanael Hazard
Worked out goal logics in tandem with Roberto to accept the goals and start movements.

Sources:

- Frontier navigation: [frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)
- Model: [OpenCV Zoo YOLOX](https://huggingface.co/opencv/opencv_zoo/tree/main/models/object_detection_yolox) (`object_detection_yolox_2022nov.onnx`, Apache-2.0)
- Pre/post-processing based on the OpenCV Zoo demo: [`yolox.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/yolox.py) (anchor grid decoding, NMS) and [`demo.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/demo.py) (letterbox / unletterbox)
