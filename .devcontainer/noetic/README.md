# Dev container for ROS 1 Noetic

A Docker image with Ubuntu 20.04, ROS Noetic, Gazebo 11, RViz and the dependencies of all DiffBot packages, for development on Linux and on Windows with WSL 2. CI builds the same image.

| File | Purpose |
|:-----|:--------|
| [`Dockerfile`](Dockerfile) | The image: ROS, Gazebo, tools and the packages' dependencies |
| [`devcontainer.json`](devcontainer.json) | How the container runs: mounts, network, display, what runs when |
| [`setup.sh`](setup.sh) | Runs once in a new container: fetches repositories, installs dependencies, builds the workspace |
| [`host-x11.sh`](host-x11.sh) | Runs on the host before the container starts: prepares the display access |

To start it, open the repository in VS Code and choose **Reopen in Container**, or use the [Dev Container CLI](https://github.com/devcontainers/cli) from the repository root:

```console
devcontainer up --workspace-folder . --config .devcontainer/noetic/devcontainer.json
devcontainer exec --workspace-folder . --config .devcontainer/noetic/devcontainer.json bash
```

Documentation:

- [Use the Dev Container](https://ros-mobile-robots.com/development/dev-container/): requirements, usage on Linux and Windows, updating, troubleshooting
- [How the Dev Container Works](https://ros-mobile-robots.com/development/dev-container-internals/): the image, the container, VS Code, the network and the display access
