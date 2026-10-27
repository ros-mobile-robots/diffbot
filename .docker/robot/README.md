# Robot container

ROS 1 Noetic and the robot's packages in a Docker image, for the robot's computer: a Raspberry Pi 4 B with a current host OS, such as Ubuntu Server 24.04, instead of a native Ubuntu 20.04 installation. The page [Robot Container](https://ros-mobile-robots.com/processing_units/robot-container/) explains the setup step by step; this is the short version.

| File | What it's for |
|:--|:--|
| [`Dockerfile`](Dockerfile) | The image: `ros:noetic-ros-base`, the packages that the bringup and the lidar need (`remo_robot.repos` by default), built in `~/catkin_ws` |
| [`compose.yaml`](compose.yaml) | Runs the bringup, and with `--profile lidar` the lidar, with the host's network and the serial devices |
| [`99-diffbot-robot.rules`](99-diffbot-robot.rules) | udev rules for the host: `/dev/teensy` and `/dev/rplidar` |
| [`entrypoint.sh`](entrypoint.sh) | Loads ROS and the workspace before each command |
| [`smoke_test.sh`](smoke_test.sh) | The test without hardware that [CI](../../.github/workflows/robot_container.yml) runs in the image |

On the robot's host, with Docker Engine and this repository cloned:

```bash
sudo cp .docker/robot/99-diffbot-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
docker compose -f .docker/robot/compose.yaml up --build                # Remo's bringup
docker compose -f .docker/robot/compose.yaml --profile lidar up        # with the lidar
```

To use the PC's ROS master, set `ROS_MASTER_URI` and `ROS_IP` in `.docker/robot/.env`, as on [ROS Network Setup](https://ros-mobile-robots.com/processing_units/ros-network-setup/); `compose.yaml` lists the other settings. Without them, roslaunch starts a master on the robot.

What it doesn't do yet: the camera (the Raspberry Pi camera driver needs the legacy camera stack, #111), and starting at boot. If the Teensy is reset, reflashed or unplugged, stop the containers and start them again (`docker compose ... down`, then `up`): a running container keeps the device it got at its start. Flash the Teensy from the PC, as [CONTRIBUTING](../../CONTRIBUTING.md) describes.
