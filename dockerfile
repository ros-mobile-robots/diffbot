# Command to invoke this Docker image build:
# docker build --file dockerfile -t diffbot_dev_image:latest .

# After build run containers with:
# docker run -it --rm --net=host --privileged --add-host=devhost:<your devhost IP> -v /tmp/.X11-unix:/tmp/.X11-unix --name=roscore diffbot_dev_image:latest'
# docker run -it --rm --net=host --privileged --add-host=devhost:<your devhost IP> -v /tmp/.X11-unix:/tmp/.X11-unix --name=rosnav  diffbot_dev_image:latest'

# Run following command on devhost before starting any process with graphic output (rviz, rqt, etc.)
# xhost +local:

# This build is based on official ROS Noetic (includes Ubuntu 20.04LTS) Docker image
FROM ros:noetic
USER root

# Upgrade Ubuntu and install helpful utilities
RUN apt-get update
RUN apt-get upgrade -y
RUN apt install -y \
	python3-catkin-tools \
	python3-vcstool \
    wget \
    git \
# Following are optional but nice for debugging your image and containers
# Just remember that containers are immutable so any changes are ephemeral
# For permanent changes update this dockerfile and rebuild the image
	net-tools \
	nano \
	iputils-ping

# Update apt repos so we can install ROS packages
RUN sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# Install ROS utilities
RUN apt update
RUN apt install -y \
      ros-noetic-roslint \
      ros-noetic-geographic-msgs \
      libgeographic-dev \
      ros-noetic-rqt \
      ros-noetic-rqt-robot-plugins

# Set location of our container's catkin workspace
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# I had problems when set to localhost, passing devhost IP via 'docker run' command fixes comms
ENV ROS_MASTER_URI http://devhost:11311/

# Do this ---->
# Copy our customized/cloned catkin_ws/src into the image
# Note: catkin_ws/src MUST be in same directory as this dockerfile, symbolic links won't work
# COPY catkin_ws/src $ROS_WS/src

# OR this ---->
# To build from the official diffbot repo on github, instead of your 
# custom catkin_ws, just comment-out the COPY above and then uncomment the following two commands
RUN git -C src clone \
    -b noetic-devel \  
    https://github.com/ros-mobile-robots/diffbot.git

RUN vcs import < $ROS_WS/src/diffbot/diffbot_dev.repos

# Install diffbot ROS package dependencies
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Build all ROS packages from source code
RUN catkin config --extend /opt/ros/$ROS_DISTRO
RUN catkin build

# Source our diffbot environment
RUN sed --in-place --expression \
      '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

# Tell container that UI output goes to host X11 server display 0
ENV DISPLAY=:0
# Important: devhost MUST execute 'xhost +local:' command BEFORE rviz tries to open display (at OS prompt, not container prompt)   

# Clean up
RUN rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]

# Suggestion: Run one container for roscore and another for navigation (see 'docker run' examples above.)
