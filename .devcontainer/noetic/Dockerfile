# Development image for DiffBot and Remo on ROS 1 Noetic (Ubuntu 20.04).
# Used by the dev container (.devcontainer/noetic/devcontainer.json) and by CI.
# It contains ROS, Gazebo 11 and the dependencies of all DiffBot packages, but not the
# workspace itself: that is built when the container is created (setup.sh).

# Stage 1: collect only the package.xml files, so the dependency layer below is rebuilt
# only when dependencies change, not on every source change.
FROM ubuntu:20.04 AS manifests
COPY . /src
RUN mkdir /manifests \
    && cd /src \
    && find . -name package.xml -not -path './.git/*' -exec cp --parents {} /manifests \;

# Stage 2: the development image
FROM osrf/ros:noetic-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

# Build tools (catkin tools, vcstool) and a few tools for the shell
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-catkin-tools \
        python3-osrf-pycommon \
        python3-vcstool \
        git \
        sudo \
        bash-completion \
        less \
        nano \
        iputils-ping \
        net-tools \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with passwordless sudo; the dev container maps its UID to the host user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Dependencies of all DiffBot packages, resolved by rosdep from their package.xml files
COPY --from=manifests /manifests /tmp/manifests
RUN apt-get update \
    && rosdep install --from-paths /tmp/manifests --ignore-src --rosdistro noetic -y \
    && rm -rf /var/lib/apt/lists/* /tmp/manifests

# rosdep's package index for the user, the workspace folder, and ROS in every new shell
USER $USERNAME
RUN rosdep update --rosdistro noetic \
    && mkdir -p /home/$USERNAME/catkin_ws/src \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/catkin_ws
