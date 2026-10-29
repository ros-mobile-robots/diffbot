# Workflows

The four workflows are independent: each runs on its own runner, they run in parallel, and none uses the result of another.

| Workflow | File | Runs on | What it checks |
|:---------|:-----|:--------|:---------------|
| CI | [`diffbot_ci_action.yml`](diffbot_ci_action.yml) | Every push and pull request | Builds and tests the ROS packages on Noetic with [industrial_ci](https://github.com/ros-industrial/industrial_ci), in its own ROS container, once with the released ROS packages (`main`) and once with those about to be released (`testing`) |
| Dev container | [`devcontainer.yml`](devcontainer.yml) | Pull requests and pushes to `noetic-devel` | Builds the [dev container](../../.devcontainer/noetic) image, creates the container (which builds the workspace) and runs the workspace's tests inside it |
| Firmware | [`firmware.yml`](firmware.yml) | Every push and pull request | Builds the Teensy firmware in [`diffbot_firmware`](../../diffbot_firmware) with PlatformIO, in the firmware's container ([`.docker/firmware`](../../.docker/firmware)), for Teensy 4.0 and Teensy 3.1/3.2 |
| Robot container | [`robot_container.yml`](robot_container.yml) | Pushes and pull requests that change the robot image or its packages | Checks the settings template, builds the [robot image](../../.docker/robot) natively on GitHub's arm64 and amd64 runners and runs its smoke test, without hardware |

How they work, and how to run the checks locally: [Testing and CI](https://ros-mobile-robots.com/development/ci/).
