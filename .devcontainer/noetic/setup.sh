#!/usr/bin/env bash
# Runs once when the dev container is created: fetches the other repositories,
# installs any missing dependencies and builds the catkin workspace.
set -eo pipefail

# Load ROS, so rosdep and catkin work, and go to the workspace
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws

# Clone the other repositories listed in diffbot_dev.repos into src: rplidar_ros and
# remo_description (its STL meshes are empty placeholders, see its README). Folders that
# already exist are kept.
vcs import --skip-existing src < src/diffbot/diffbot_dev.repos

# Install dependencies of packages that were added since the image was built
sudo apt-get update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Build the workspace on top of the ROS installation in /opt/ros/noetic
catkin config --extend /opt/ros/noetic
catkin build

# Load the workspace in every new terminal. The line is added only once, so running
# setup.sh again is safe.
grep -qxF "source ~/catkin_ws/devel/setup.bash" ~/.bashrc \
    || echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
