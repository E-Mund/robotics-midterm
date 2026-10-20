#!/usr/bin/env bash
# Build the workspace and start the full sim: Gazebo (House world with a human),
# slam_toolbox, Nav2, frontier exploration, RViz, sub and human_detector.
# Optionally sends a goal scenario once Nav2 is ready.
#
#   ./run.sh                        # sim + RViz
#   ./run.sh --scenario emergency   # ...and send goal1, goal2, then goal3 (see goal_publisher.py)
#   ./run.sh --no-rviz --no-build   # extra flags; anything else goes to ros2 launch,
#   ./run.sh x_pose:=1.0 y_pose:=2.0  # e.g. a different spawn point
#
# Ctrl-C stops everything.
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

RVIZ=true
BUILD=true
SCENARIO=""
LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-rviz) RVIZ=false ;;
        --no-build) BUILD=false ;;
        --scenario) SCENARIO="$2"; shift ;;
        -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
        *) LAUNCH_ARGS+=("$1") ;;
    esac
    shift
done

if ! command -v ros2 >/dev/null; then
    source /opt/ros/humble/setup.bash
fi
export TURTLEBOT3_MODEL=waffle

if pgrep -x gzserver >/dev/null; then
    echo "A Gazebo server (gzserver) is already running; stop it first." >&2
    exit 1
fi

# sub.py needs frontier_exploration_ros2 (not packaged for Humble); fetch it
# into deps/ once, and the workspace build below builds it with everything else
EXPLORER="$WS/deps/src/frontier_exploration_ros2"
if [[ ! -d "$EXPLORER" ]]; then
    git clone https://github.com/mertgulerx/frontier_exploration_ros2.git "$EXPLORER"
    git -C "$EXPLORER" checkout --quiet 0d61180
    touch "$EXPLORER/plugin/COLCON_IGNORE"  # RViz plugin, not needed
fi

if $BUILD; then
    # Explicit Python so message packages (project_status) build under newer CMake
    CMAKE_ARGS=(-DPython_EXECUTABLE="$(command -v python3)" -DBUILD_TESTING=OFF)
    if [[ -n "$CONDA_PREFIX" ]]; then
        # pixi/conda ROS: link C++ against the env's libstdc++, not the system one
        LINK="-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib"
        CMAKE_ARGS+=("-DCMAKE_EXE_LINKER_FLAGS=$LINK" "-DCMAKE_SHARED_LINKER_FLAGS=$LINK")
    fi
    colcon build --cmake-args "${CMAKE_ARGS[@]}"
fi
source "$WS/install/setup.bash"

if [[ -n "$SCENARIO" ]]; then
    # goal_publisher waits for sub.py and Nav2 itself before publishing
    ros2 run midterm goal_publisher --ros-args -p scenario:="$SCENARIO" &
    GOALS_PID=$!
    trap 'kill "$GOALS_PID" 2>/dev/null' EXIT
fi

ros2 launch midterm test_env.launch.py rviz:="$RVIZ" "${LAUNCH_ARGS[@]}"
