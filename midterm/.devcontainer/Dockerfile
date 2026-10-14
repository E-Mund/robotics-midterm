FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations \
      ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
      ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-sensor-msgs-py \
      python3-opencv python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
