from glob import glob
from setuptools import setup

package_name = 'team1_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team1',
    maintainer_email='team1@example.com',
    description='Namespaced TurtleBot3 two-goal navigation with Nav2 + OpenCV obstacle detection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_manager = team1_nav.mission_manager:main',
            'floor_obstacle_detector = team1_nav.floor_obstacle_detector:main',
            'cmd_vel_gate = team1_nav.cmd_vel_gate:main',
        ],
    },
)
