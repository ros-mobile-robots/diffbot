#!/usr/bin/env bash
# Runs once when the dev container is created: fetches the other repositories,
# installs any missing dependencies and builds the catkin workspace.
set -eo pipefail

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws

# rplidar_ros and remo_description (its STL meshes are empty placeholders, see its README)
vcs import --skip-existing src < src/diffbot/diffbot_dev.repos

# Dependencies of packages that were added since the image was built
sudo apt-get update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Build on top of the ROS installation in /opt/ros/noetic
catkin config --extend /opt/ros/noetic
catkin build

# Source the workspace in every new terminal; added only once, so running setup.sh again is safe
grep -qxF "source ~/catkin_ws/devel/setup.bash" ~/.bashrc \
    || echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
