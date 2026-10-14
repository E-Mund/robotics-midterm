#!/usr/bin/env bash
set -e
mkdir -p /root/ws/src
ln -sfn "$(pwd)/team1_nav" /root/ws/src/team1_nav
source /opt/ros/humble/setup.bash
cd /root/ws && colcon build --symlink-install
grep -q team1_setup /root/.bashrc || cat >> /root/.bashrc << 'RC'
# team1_setup
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /root/ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
RC
