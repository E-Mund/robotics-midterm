#!/usr/bin/env bash
# Start the eight processes in the manual house simulation workflow.
# Requires an installed ROS 2 Humble environment and built workspace packages.
# Usage: ./run.sh
# Publish goals separately. Ctrl-C stops the started processes.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ros2 >/dev/null; then
    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        echo "ROS 2 Humble is unavailable. Install it and its dependencies, or source your ROS environment first." >&2
        exit 1
    fi
    source /opt/ros/humble/setup.bash
fi

# Load the built packages, as the workspace's run wrapper does.
OVERLAY="$WS/install/local_setup.bash"
if [[ ! -f "$OVERLAY" && "$(basename "$(dirname "$WS")")" == src ]]; then
    OVERLAY="$WS/../../install/local_setup.bash"
fi
if [[ -f "$OVERLAY" ]]; then
    source "$OVERLAY"
fi

export TURTLEBOT3_MODEL=waffle
pids=()

start() {
    "$@" &
    pids+=("$!")
}
cleanup() {
    if ((${#pids[@]})); then
        kill "${pids[@]}" 2>/dev/null || true
        wait || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start ros2 launch gazebo_ros gazebo.launch.py \
    world:="$WS/midterm/worlds/turtlebot3_house_with_human.world"
start ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py \
    use_sim_time:=true
start ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py \
    x_pose:=-2.5 y_pose:=3.5
start ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    slam_params_file:="$WS/midterm/config/slam_coverage.yaml"
start ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    params_file:="$WS/midterm/config/nav2_coverage.yaml"
start ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
    use_sim_time:=true autostart:=false control_service_enabled:=true \
    params_file:="$WS/midterm/config/frontier_coverage.yaml"
start ros2 run midterm human_detector --ros-args -p use_sim_time:=true -p show:=true
start ros2 run midterm sub --ros-args -p use_sim_time:=true

wait
