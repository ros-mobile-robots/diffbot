# Dev container for ROS 1 Noetic

A Docker image with Ubuntu 20.04, ROS Noetic, Gazebo 11, RViz and the Ubuntu and ROS packages that the DiffBot packages need, for development on Linux and on Windows with WSL 2. CI builds the same image.

| File | When | What it does |
|:-----|:-----|:-------------|
| [`devcontainer.json`](devcontainer.json) | Read first | Tells VS Code and the Dev Container CLI how to build and run the container: the image, your clone of this repository mounted at `~/catkin_ws/src/diffbot`, the host network, the display access, and which scripts run when. |
| [`host-x11.sh`](host-x11.sh) | On the host, before each container start | Copies your display's X11 cookie into `.x11/` in your clone, so RViz and Gazebo in the container can open windows on your screen. |
| [`Dockerfile`](Dockerfile) | When the image is built | Builds the Docker image: the official ROS Noetic image with Gazebo 11 and RViz, plus build tools (catkin tools, vcstool) and every Ubuntu and ROS package that the DiffBot packages need. The workspace itself isn't in the image; it's built in the container. |
| [`setup.sh`](setup.sh) | Once, in each new container | Clones the other repositories `rplidar_ros` and `remo_description`, installs any missing dependencies with rosdep, and builds the workspace with catkin. |

To start it, open your clone of this repository in VS Code (on Windows: the clone in WSL) and choose **Reopen in Container**, or use the [Dev Container CLI](https://github.com/devcontainers/cli) from the repository root:

```console
devcontainer up --workspace-folder . --config .devcontainer/noetic/devcontainer.json
devcontainer exec --workspace-folder . --config .devcontainer/noetic/devcontainer.json bash
```

Documentation:

- [Use the Dev Container](https://ros-mobile-robots.com/development/dev-container/): requirements, usage on Linux and Windows, updating, troubleshooting
- [How the Dev Container Works](https://ros-mobile-robots.com/development/dev-container-internals/): the image, the container, VS Code, the network and the display access
