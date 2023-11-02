# syntax=docker/dockerfile:1
ARG ROS_DISTRO="noetic"
FROM osrf/ros:noetic-desktop-full AS upstream
# Restate for later use
ARG ROS_DISTRO
ARG REPO

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install development tools
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
        apt-utils \
        ccache \
        clang \
        cmake \
        git \
        lld \
        llvm \
        python3-catkin-tools \
        python3-colcon-mixin \
        python3-colcon-common-extensions \
        python3-colcon-lcov-result \
        python3-colcon-coveragepy-result \
        python3-colcon-mixin \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        wget \
    && rm -rf /var/lib/apt/lists/*


# copy source to install repo dependencies
WORKDIR /ws
COPY . ./src/${REPO}
# install repo dependencies
RUN rosdep update && apt-get update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM upstream AS development

ARG UID
ARG GID
ARG USER

# fail build if args are missing
# hadolint ignore=SC2028
RUN if [ -z "$UID" ]; then echo '\nERROR: UID not set. Run \n\n \texport UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$GID" ]; then echo '\nERROR: GID not set. Run \n\n \texport GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# install developer tools
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        clang-format \
        clang-tidy \
        git \
        git-lfs \
        openssh-client \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    pre-commit==3.0.4

# Setup user home directory
# --no-log-init helps with excessively long UIDs
RUN groupadd --gid $GID $USER \
    && useradd --no-log-init --uid $GID --gid $UID -m $USER --groups sudo \
    && echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.profile \
    && touch /home/${USER}/.bashrc \
    && chown -R ${GID}:${UID} /home/${USER}

USER $USER
ENV SHELL /bin/bash
ENTRYPOINT []

# Setup mixin
WORKDIR /home/${USER}/ws
