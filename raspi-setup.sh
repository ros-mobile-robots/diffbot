#!/bin/sh
# update & upgrade #
sudo apt update
sudo apt upgrade

sudo apt -y install i2c-tools curl

# Add i2c support to /boot/firmware/config.txt
# dtparam=i2c0=on
sudo sed -i '/dtparam=i2c0=on/s/^#//g' /boot/firmware/config.txt

# ROS Noetic setup
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt -y update
sudo apt -y install ros-noetic-ros-base

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool \
    python3-rosdep python3-catkin-tools build-essential

sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ..
catkin build
