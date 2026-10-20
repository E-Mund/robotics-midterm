"""
Full test environment: Gazebo House world with a human (worlds/), slam_toolbox,
Nav2, sub.py and human_detector.

    ros2 launch midterm test_env.launch.py
    ros2 launch midterm test_env.launch.py rviz:=true x_pose:=1.0 y_pose:=2.0
    ros2 launch midterm test_env.launch.py show_detector:=true
    ros2 launch midterm test_env.launch.py world:=/path/to/other.world

Starts are staggered so Gazebo's /clock and TF exist before SLAM and Nav2
come up. Send goals from another terminal once Nav2 is active, e.g.:

    ros2 run midterm goal_publisher --ros-args -p scenario:=both
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, **args):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    # Scoped group: Humble includes otherwise leak their launch arguments into
    # each other (gazebo_ros declares params_file:='' which breaks Nav2).
    return GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path), launch_arguments=args.items()
        )
    ])


def generate_launch_description():
    share = get_package_share_directory("midterm")
    config = os.path.join(share, "config")
    world = os.path.join(share, "worlds", "turtlebot3_house_with_human.world")
    slam_params = os.path.join(config, "slam_toolbox.yaml")
    nav2_params = os.path.join(config, "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("show_detector", default_value="false",
                              description="Open the detector's camera window"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument("world", default_value=world,
                              description="Gazebo world file"),

        # Same pieces as turtlebot3_house.launch.py, which hardcodes its world
        include("gazebo_ros", "gzserver.launch.py",
                world=LaunchConfiguration("world")),
        include("gazebo_ros", "gzclient.launch.py"),
        include("turtlebot3_gazebo", "robot_state_publisher.launch.py",
                use_sim_time="true"),
        include(
            "turtlebot3_gazebo", "spawn_turtlebot3.launch.py",
            x_pose=LaunchConfiguration("x_pose"),
            y_pose=LaunchConfiguration("y_pose"),
        ),
        TimerAction(period=12.0, actions=[
            include(
                "slam_toolbox", "online_async_launch.py",
                use_sim_time="true", slam_params_file=slam_params,
            ),
        ]),
        TimerAction(period=18.0, actions=[
            include(
                "nav2_bringup", "navigation_launch.py",
                use_sim_time="true", params_file=nav2_params,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch", "rviz_launch.py")),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]),
        TimerAction(period=25.0, actions=[
            Node(
                package="midterm", executable="sub", output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="midterm", executable="human_detector", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "show": ParameterValue(
                        LaunchConfiguration("show_detector"), value_type=bool),
                }],
            ),
        ]),
    ])
