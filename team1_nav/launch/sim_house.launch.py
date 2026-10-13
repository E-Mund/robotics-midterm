"""
Gazebo (classic) + TurtleBot3 waffle_pi in the House world, fully namespaced.

All gazebo_ros plugins of the robot get <namespace>/<ns></namespace> and their /tf, /tf_static
are remapped to <ns>/tf, <ns>/tf_static, so EVERY robot topic is prefixed:
  /team1/odom  /team1/scan  /team1/camera/image_raw  /team1/cmd_vel  /team1/imu  /team1/tf ...

Adapted from turtlebot3_simulations (turtlebot3_gazebo/launch/turtlebot3_house.launch.py,
spawn_turtlebot3.launch.py, robot_state_publisher.launch.py), ROS 2 Humble branch.
The model must be waffle_pi (burger has no camera).

Args: namespace:=team1 world:=<path to .world> x_pose:=-2.0 y_pose:=1.0 yaw:=0.0 gui:=true
"""
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODEL = 'waffle_pi'


def patch_sdf(src, dst, ns):
    """Put every gazebo_ros plugin into namespace `ns` and namespace its TF topics."""
    tree = ET.parse(src)
    for plugin in tree.getroot().iter('plugin'):
        if 'gazebo_ros' not in plugin.get('filename', ''):
            continue
        ros = plugin.find('ros')
        if ros is None:
            ros = ET.SubElement(plugin, 'ros')
        for old in ros.findall('namespace'):
            ros.remove(old)
        ns_el = ET.Element('namespace')
        ns_el.text = '/' + ns
        ros.insert(0, ns_el)
        for topic in ('/tf', '/tf_static'):
            remap = ET.SubElement(ros, 'remapping')
            remap.text = f'{topic}:={topic[1:]}'
    tree.write(dst)
    return dst


def find_urdf():
    candidates = []
    for pkg in ('turtlebot3_gazebo', 'turtlebot3_description'):
        try:
            candidates.append(os.path.join(get_package_share_directory(pkg), 'urdf', f'turtlebot3_{MODEL}.urdf'))
        except Exception:  # noqa: BLE001
            pass
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise RuntimeError(f'turtlebot3_{MODEL}.urdf not found in {candidates}')


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world').perform(context)
    if not world:
        world = os.path.join(tb3_gazebo, 'worlds', 'turtlebot3_house.world')

    models_dir = os.path.join(tb3_gazebo, 'models')
    sdf_src = os.path.join(models_dir, f'turtlebot3_{MODEL}', 'model.sdf')
    sdf_dst = patch_sdf(sdf_src, os.path.join(tempfile.gettempdir(), f'{ns}_{MODEL}.sdf'), ns)

    with open(find_urdf(), 'r') as f:
        robot_description = f.read()

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    model_path = models_dir + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')

    return [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', MODEL),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen', remappings=tf_remaps,
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', f'{ns}_{MODEL}', '-file', sdf_dst,
                       '-x', LaunchConfiguration('x_pose').perform(context),
                       '-y', LaunchConfiguration('y_pose').perform(context),
                       '-z', '0.01',
                       '-Y', LaunchConfiguration('yaw').perform(context)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='team1'),
        DeclareLaunchArgument('world', default_value='', description='World file; empty = TurtleBot3 House'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
