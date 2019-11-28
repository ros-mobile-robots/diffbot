# 2wd-robot
Autonomous 2wd robot using ROS on RPi 4 B

## ROS Installation

The robot setup is supposed to run on Ubuntu 18.04 Bionic. [ROS Melodic]() is intended to run with this Ubuntu version.

From the [catkin tutorial](https://wiki.ros.org/catkin/Tutorials) here are the commands used to create the workspace:
Use [`catkin build`](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) instead of [`catkin_make`](https://wiki.ros.org/catkin/commands/catkin_make).
[Here is why](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).

```
$ mkdir -p ~/ros/src
$ cd ~/ros/
$ catkin build
``` 
