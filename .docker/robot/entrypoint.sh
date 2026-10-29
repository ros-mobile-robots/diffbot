#!/bin/bash
# Entry point of the robot image: load ROS and the robot's workspace, then run the command,
# by default the bringup (Dockerfile's CMD, or the command in compose.yaml).
set -e

# ROS reads ROS_IP and ROS_HOSTNAME when they are set, even to an empty string, and roscpp
# rejects an empty one. compose.yaml passes them through, so remove them when they are empty.
[ -n "$ROS_IP" ] || unset ROS_IP
[ -n "$ROS_HOSTNAME" ] || unset ROS_HOSTNAME

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
# "$@" is the command that this script got: compose.yaml's command, the Dockerfile's CMD, the
# rest of a docker run line, or what follows /entrypoint.sh when docker exec runs it explicitly.
# exec runs it in place of this script, in the same process. For the container's main process,
# that means the command gets Docker's stop signal itself and can shut its nodes down cleanly.
exec "$@"
