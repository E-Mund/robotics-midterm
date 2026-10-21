# Midterm Programming Assignment
## Running instructions

Run Script: ./run.sh

Requires ROS 2 Humble with `turtlebot3_gazebo`, `slam_toolbox` and `nav2_bringup`.
Nav2 and slam_toolbox settings are in `midterm/config/`.

## Navigation - Roberto Molina
Runs frontier navigation algorithm to explore the enviornment and implements custom goal logic.

Goal1 Script: pixi run run ros2 topic pub --once /goal1 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -3.8, y: 3.5, z: 0.0}}'
Goal2 Script: pixi run run ros2 topic pub --once /goal2 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -6.3, y: 3.0, z: 0.0}}'
Goal3 Script: pixi run run ros2 topic pub --once /goal3 geometry_msgs/msg/PointStamped \
  '{header: {frame_id: map}, point: {x: -2.5, y: 3.5, z: 0.0}}' 

## Human detection - Ethan Mund

human_detector subcribes to the camera and feeds it into YoloX with OpenCV.

## Goal logic - Nathanael Hazard
Worked out goal logics in tandem with Roberto to accept the goals and start movements.

Sources:

- Model: [OpenCV Zoo YOLOX](https://huggingface.co/opencv/opencv_zoo/tree/main/models/object_detection_yolox) (`object_detection_yolox_2022nov.onnx`, Apache-2.0)
- Pre/post-processing based on the OpenCV Zoo demo: [`yolox.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/yolox.py) (anchor grid decoding, NMS) and [`demo.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/demo.py) (letterbox / unletterbox)
