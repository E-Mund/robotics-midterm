"""
Full test environment: Gazebo House world, slam_toolbox, Nav2 and sub.py.

    ros2 launch midterm test_env.launch.py
    ros2 launch midterm test_env.launch.py rviz:=true x_pose:=1.0 y_pose:=2.0

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
    config = os.path.join(get_package_share_directory("midterm"), "config")
    slam_params = os.path.join(config, "slam_toolbox.yaml")
    nav2_params = os.path.join(config, "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),

        include(
            "turtlebot3_gazebo", "turtlebot3_house.launch.py",
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
        ]),
    ])
