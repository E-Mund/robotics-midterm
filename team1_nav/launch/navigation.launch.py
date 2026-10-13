"""
Namespaced navigation stack: slam_toolbox (online async) + Nav2 servers + team nodes.

Velocity pipeline (only cmd_vel_gate publishes the robot command):
  controller_server --cmd_vel_nav--> velocity_smoother --cmd_vel_gate_in--> cmd_vel_gate --> <ns>/cmd_vel
  behavior_server (spin/backup recoveries) ------------cmd_vel_gate_in-----^

Nodes are listed explicitly (instead of including nav2_bringup/navigation_launch.py) so the velocity
topics can be routed through the safety gate. Pattern taken from nav2_bringup navigation_launch.py
(Humble): namespaced nodes, '/tf'->'tf' remaps, lifecycle manager with autostart.

Args: namespace:=team1 use_sim_time:=true params_file:=<template yaml>
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server',
                        'behavior_server', 'bt_navigator', 'velocity_smoother']


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    template = LaunchConfiguration('params_file').perform(context)

    # Fill in namespace and nest everything under it so names match /<ns>/<node>.
    with open(template, 'r') as f:
        data = yaml.safe_load(f.read().replace('TEAM_NS', ns))
    params_file = os.path.join(tempfile.gettempdir(), f'{ns}_nav_params.yaml')
    with open(params_file, 'w') as f:
        yaml.safe_dump({ns: data}, f, default_flow_style=False)

    params = [params_file, {'use_sim_time': use_sim_time}]
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def node(pkg, exe, name, remaps=()):
        return Node(package=pkg, executable=exe, name=name, namespace=ns, output='screen',
                    parameters=params, remappings=tf + list(remaps))

    return [
        # SLAM -> /<ns>/map and map->odom TF
        node('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox',
             [('/map', 'map'), ('/map_metadata', 'map_metadata'), ('/map_updates', 'map_updates')]),
        # Nav2
        node('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_smoother', 'smoother_server', 'smoother_server'),
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_gate_in')]),
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_gate_in')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=ns, output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': NAV2_LIFECYCLE_NODES}]),
        # Team nodes
        node('team1_nav', 'floor_obstacle_detector', 'floor_obstacle_detector'),
        node('team1_nav', 'cmd_vel_gate', 'cmd_vel_gate'),
        node('team1_nav', 'mission_manager', 'mission_manager'),
    ]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('team1_nav'), 'config', 'team_nav_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='team1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        OpaqueFunction(function=launch_setup),
    ])
