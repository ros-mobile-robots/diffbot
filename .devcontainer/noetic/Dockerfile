# Development image for DiffBot and Remo on ROS 1 Noetic (Ubuntu 20.04).
# Used by the dev container (.devcontainer/noetic/devcontainer.json) and by CI.
# It contains ROS, Gazebo 11 and the dependencies of all DiffBot packages, but not the
# workspace itself: that is built when the container is created (setup.sh).

# Stage 1: copy only the package.xml files out of the repository. The step that installs the
# dependencies (stage 2) then reruns only when a package.xml changes, not on every code change.
FROM ubuntu:20.04 AS manifests
COPY . /src
RUN mkdir /manifests \
    && cd /src \
    && find . -name package.xml -not -path './.git/*' -exec cp --parents {} /manifests \;

# Stage 2: start from the official ROS image with ROS Noetic, Gazebo 11 and RViz (desktop-full)
FROM osrf/ros:noetic-desktop-full

# Name and IDs of the user in the image; the dev container changes the IDs to match yours
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Let apt install packages without asking questions
ENV DEBIAN_FRONTEND=noninteractive

# Install the build tools (catkin tools, vcstool) and a few tools for the shell
# (git, sudo, an editor, ping), then delete apt's package lists to keep the image small
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

# Create the user ros, who may use sudo without a password. The dev container changes the
# user's UID to yours, so files created in the container belong to you on the host.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --create-home --shell /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install what the DiffBot packages need: rosdep reads their dependencies from the
# package.xml files of stage 1 and installs the matching Ubuntu and ROS packages
COPY --from=manifests /manifests /tmp/manifests
RUN apt-get update \
    && rosdep install --from-paths /tmp/manifests --ignore-src --rosdistro noetic -y \
    && rm -rf /var/lib/apt/lists/* /tmp/manifests

# As the user ros: download rosdep's package index, create the workspace folder, and load ROS
# in every new interactive Bash shell
USER $USERNAME
RUN rosdep update --rosdistro noetic \
    && mkdir -p /home/$USERNAME/catkin_ws/src \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

# Start in the workspace
WORKDIR /home/$USERNAME/catkin_ws
