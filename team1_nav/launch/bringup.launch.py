"""One-shot bringup: House world simulation + namespaced navigation stack (started after a delay
so Gazebo plugins and TF are up before Nav2 configures)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('team1_nav'), 'launch')
    args = ['namespace', 'world', 'x_pose', 'y_pose', 'yaw', 'gui']
    defaults = {'namespace': 'team1', 'world': '', 'x_pose': '-2.0', 'y_pose': '1.0', 'yaw': '0.0',
                'gui': 'true'}
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim_house.launch.py')),
        launch_arguments={a: LaunchConfiguration(a) for a in args}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
        launch_arguments={'namespace': LaunchConfiguration('namespace'), 'use_sim_time': 'true'}.items())

    return LaunchDescription(declared + [sim, TimerAction(period=8.0, actions=[nav])])
