# Robot image for DiffBot and Remo on ROS 1 Noetic (Ubuntu 20.04), for the robot's computer:
# a Raspberry Pi 4 B (arm64) with a current host OS, for example Ubuntu Server 24.04, and
# Docker Engine. It also builds on amd64, for CI and for trying it on a PC.
# Unlike the dev container (.devcontainer/noetic), it holds only what the robot runs, already
# built: the hardware interface, the controllers, rosserial for the Teensy, the robot model and
# the lidar driver. No Gazebo, no navigation, no desktop tools.
# Build it from the repository's root folder (compose.yaml does the same):
#   docker build -f .docker/robot/Dockerfile -t diffbot-robot:noetic .

# Stage 1, named manifests (ROS calls a package.xml the package manifest): copy only the
# package.xml files out of the repository, as the dev container does. The step that installs
# the dependencies then reruns only when a package.xml changes, not on every code change.
FROM ubuntu:20.04 AS manifests
COPY . /src
RUN mkdir /manifests \
    && cd /src \
    && find . -name package.xml -exec cp --parents {} /manifests \;

# Stage 2: the official ROS image with ROS Noetic's base packages, for arm64 and amd64
FROM ros:noetic-ros-base

# Let apt install packages without asking questions
ENV DEBIAN_FRONTEND=noninteractive

# The build tools: catkin tools to build the workspace, vcstool to fetch the other repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        python3-osrf-pycommon \
        python3-vcstool \
        git \
    && rm -rf /var/lib/apt/lists/*

# The user ros, with UID and GID 1000, the usual IDs of the first user on Linux, so that files it
# writes to a mounted host folder (logs, bags) belong to that user on the host. No sudo: the
# robot's processes don't need root.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME

# The packages to build: the ones that the bringup and the lidar need, for both robots (the image
# has DiffBot's and Remo's models); diffbot_gazebo, the navigation and SLAM packages stay out.
ARG PACKAGES="diffbot_bringup diffbot_base diffbot_control diffbot_description diffbot_msgs remo_description rplidar_ros"
ENV DIFFBOT_PACKAGES=$PACKAGES

# Fetch the other repositories with vcstool: remo_robot.repos has the lidar driver and Remo's
# model. (diffbot_robot.repos would add the legacy camera driver, which this image can't use,
# see #111.) The paths in the file start with src/. This layer reruns only when it changes.
USER $USERNAME
WORKDIR /home/$USERNAME/catkin_ws
COPY --chown=$USERNAME remo_robot.repos /tmp/robot.repos
RUN mkdir src && vcs import --input /tmp/robot.repos .

# Install the dependencies of the packages to build with rosdep, from their package.xml files:
# the diffbot packages from stage 1, the fetched repositories from src. The base image already
# set up rosdep. Then delete apt's package lists and the copied files.
USER root
COPY --from=manifests /manifests /tmp/manifests
RUN paths="src" \
    && for p in $PACKAGES; do if [ -d /tmp/manifests/$p ]; then paths="$paths /tmp/manifests/$p"; fi; done \
    && apt-get update \
    && rosdep install --from-paths $paths --ignore-src --rosdistro noetic -y \
    && rm -rf /var/lib/apt/lists/* /tmp/manifests

# Copy the repository into the workspace and build the packages. The workspace stays at this
# path, with its devel space, because the packages have no complete install rules yet.
USER $USERNAME
COPY --chown=$USERNAME . src/diffbot
RUN . /opt/ros/noetic/setup.sh \
    && catkin config --extend /opt/ros/noetic \
    && catkin build --no-status $PACKAGES

# Load ROS and the workspace before every command, also when Docker starts the container
COPY .docker/robot/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# By default, bring up Remo; compose.yaml sets the model and the devices
CMD ["roslaunch", "diffbot_bringup", "bringup.launch", "model:=remo"]
