# Midterm Programming Assignment
## Running instructions

Requires ROS 2 Humble with `turtlebot3_gazebo`, `slam_toolbox` and `nav2_bringup`.
The script sources `/opt/ros/humble/setup.bash` if ROS isn't already sourced.
Nav2 and slam_toolbox settings are in `midterm/config/`.

## Navigation - Roberto Molina
Runs frontier navigation algorithm to explore the enviornment and implements custom goal logic.

## Human detection - Ethan Mund

human_detector subcribes to the camera and feeds it into YoloX with OpenCV.

## Goal logic - Nathanael Hazard
Worked out goal logics in tandem with Roberto to accept the goals and start movements.

Sources:

- Model: [OpenCV Zoo YOLOX](https://huggingface.co/opencv/opencv_zoo/tree/main/models/object_detection_yolox) (`object_detection_yolox_2022nov.onnx`, Apache-2.0)
- Pre/post-processing based on the OpenCV Zoo demo: [`yolox.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/yolox.py) (anchor grid decoding, NMS) and [`demo.py`](https://huggingface.co/opencv/opencv_zoo/blob/main/models/object_detection_yolox/demo.py) (letterbox / unletterbox)
