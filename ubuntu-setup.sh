#!/bin/sh
# update & upgrade #
sudo apt update
sudo apt upgrade

sudo apt -y install i2c-tools

# Add i2c support to /boot/firmware/config.txt
# dtparam=i2c0=on
sudo sed -i '/dtparam=i2c0=on/s/^#//g' /boot/firmware/config.txt

# ROS Noetic setup
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt -y update
sudo apt -y install ros-noetic-desktop-full

sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Catkin_tools
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt -y update
sudo apt -y install python-catkin-tools


# git setup
git config --global credential.helper store

echo "Enter your user.email for git config" 
read email

git config --global user.email \"$email\"

echo "Enter your user.name for git config"
read name

git config --global user.name \"$name\"

