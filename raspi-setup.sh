#!/usr/bin/env bash
# Sets up the robot's Raspberry Pi 4 B with Ubuntu MATE 20.04 in one go: I2C, ROS Noetic, rosdep,
# and the catkin workspace ~/catkin_ws with this repository and the robot's other repositories.
# The docs explain each step: https://ros-mobile-robots.com/processing_units/ros-setup/ and
# https://ros-mobile-robots.com/packages/packages-setup/
#
# Run it on the robot, for Remo (the default) or DiffBot:
#   bash raspi-setup.sh [remo|diffbot]
# Running it again is safe: steps that are already done are skipped or repeated without harm.

# Stop at the first command that fails (-e), also inside a pipe (pipefail)
set -eo pipefail

# Which robot: remo_robot.repos or diffbot_robot.repos lists its other repositories
robot="${1:-remo}"
case "$robot" in
    remo|diffbot) ;;
    *) echo "Usage: bash $0 [remo|diffbot]" >&2; exit 1 ;;
esac

# Robots set up with the old instructions have the ROS source in ros-latest.list, with the key
# that expired in June 2025: apt update fails while it's there, so remove it and the old key first
if [ -f /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo rm /etc/apt/sources.list.d/ros-latest.list
    sudo apt-key del "C1CF 6E31 E6BA DE88 68B1 72B4 F42E D6FB AB17 C654" || true
fi

sudo apt update
sudo apt -y upgrade
sudo apt -y install i2c-tools curl git

# Enable I2C for the motor driver and the sensors: uncomment dtparam=i2c0=on in the boot config
if [ -f /boot/firmware/config.txt ]; then
    sudo sed -i '/dtparam=i2c0=on/s/^#//g' /boot/firmware/config.txt
fi

# The ROS package source and its signing key, with the ros-apt-source package, as the ROS signing
# key migration guide recommends since the key expired in June 2025:
# https://discourse.openrobotics.org/t/ros-signing-key-migration-guide/43937
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
sudo dpkg -i /tmp/ros-apt-source.deb

# ROS for the robot, without the GUI tools (they run on the PC, in the dev container), and the
# tools to set up and build the workspace
sudo apt update
sudo apt -y install ros-noetic-robot python3-rosdep python3-catkin-tools python3-vcstool \
    build-essential python-is-python3

# rosdep: init once, then download the package index. A plain "rosdep update" skips Noetic,
# which is past its end of life, so name it.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update --rosdistro noetic

# Load ROS in every new interactive Bash shell (added only once), and here: this script runs in
# its own shell, which doesn't read ~/.bashrc
grep -qxF "source /opt/ros/noetic/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

# The workspace: this repository, and the robot's other repositories with vcstool (the paths in
# the .repos files start with src/). Folders that already exist are kept.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
if [ ! -d src/diffbot ]; then
    git clone https://github.com/ros-mobile-robots/diffbot.git src/diffbot
fi
vcs import --skip-existing < "src/diffbot/${robot}_robot.repos"

# Install the dependencies the packages list in their package.xml, then build on top of
# (extending) the ROS installation in /opt/ros/noetic
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin config --extend /opt/ros/noetic
catkin build

# Load the workspace in every new interactive Bash shell (added only once)
grep -qxF "source ~/catkin_ws/devel/setup.bash" ~/.bashrc \
    || echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
