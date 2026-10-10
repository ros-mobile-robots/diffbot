# Workflows

| Workflow | File | What it checks |
|:---------|:-----|:---------------|
| CI | [`diffbot_ci_action.yml`](diffbot_ci_action.yml) | Builds and tests the ROS packages on Noetic with [industrial_ci](https://github.com/ros-industrial/industrial_ci), once with the released ROS packages (`main`) and once with those about to be released (`testing`) |
| Dev container | [`devcontainer.yml`](devcontainer.yml) | Builds the [dev container](../../.devcontainer/noetic) and runs the workspace's tests in it |
| Build base controller | [`build_base_controller.yml`](build_base_controller.yml) | Builds the Teensy firmware with PlatformIO, for Teensy 4.0 and Teensy 3.1/3.2 |

How they work, and how to run the checks locally: [Testing and CI](https://ros-mobile-robots.com/development/ci/).
