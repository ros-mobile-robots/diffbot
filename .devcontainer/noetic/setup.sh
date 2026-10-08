#!/usr/bin/env bash
# Runs once when the dev container is created: fetches the other repositories,
# installs any missing dependencies and builds the catkin workspace.
set -eo pipefail

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws

# rplidar_ros and remo_description (its STL meshes are empty placeholders, see its README)
vcs import --skip-existing src < src/diffbot/diffbot_dev.repos

sudo apt-get update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

catkin config --extend /opt/ros/noetic
catkin build

grep -qxF "source ~/catkin_ws/devel/setup.bash" ~/.bashrc \
    || echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
