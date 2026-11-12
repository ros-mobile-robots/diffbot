# Robot container

ROS 1 Noetic and the robot's packages in a Docker image, for the robot's computer: a Raspberry Pi 4 B with a current host OS, such as Ubuntu Server 24.04, instead of a native Ubuntu 20.04 installation. The page [Robot Container](https://ros-mobile-robots.com/processing_units/robot-container/) explains the setup step by step; this is the short version.

**Experimental:** CI builds the image for arm64 and amd64 and tests it without hardware, but it hasn't driven a robot yet: the test with teleop and the lidar on a Raspberry Pi with Remo is still open ([#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

| File | What it's for |
|:--|:--|
| [`Dockerfile`](Dockerfile) | The image: `ros:noetic-ros-base`, the packages that the bringup and the lidar need for both robots, built in `~/catkin_ws` |
| [`compose.yaml`](compose.yaml) | Runs the bringup, and with the `lidar` profile the lidar, with the host's network and the serial devices |
| [`env.example`](env.example) | The template for `.env`, the robot's settings, which `docker compose` reads from this folder |
| [`99-diffbot-robot.rules`](99-diffbot-robot.rules) | udev rules for the host: `/dev/teensy` and `/dev/rplidar` |
| [`entrypoint.sh`](entrypoint.sh) | Loads ROS and the workspace before each command |
| [`smoke_test.sh`](smoke_test.sh) | The test without hardware that [CI](../../.github/workflows/robot_container.yml) runs inside a container of the image |
| [`publish_image.sh`](publish_image.sh) | Publishes the tested images for both architectures, in CI on `noetic-devel` |

CI publishes the image that passed the smoke test, for arm64 and amd64, as [`ghcr.io/ros-mobile-robots/diffbot-robot`](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot): `noetic` is the newest from `noetic-devel`, and `noetic-<commit>` stays the same. The robot downloads it, so it doesn't build anything.

On the robot's host, with Docker Engine (with Compose and Buildx) and this repository cloned to `~/diffbot`:

```bash
sudo cp ~/diffbot/.docker/robot/99-diffbot-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir -p ~/robot-data      # ROS logs and bags; as the first user (UID 1000, like ros in the container)
cd ~/diffbot/.docker/robot
cp env.example .env        # then edit .env: the robot model, the lidar, the ROS network
docker compose up -d --no-build   # download the image if needed, start the bringup in the background
docker compose logs -f            # follow the output; Ctrl+C stops following, the robot keeps running
docker compose down               # stop
docker compose pull --policy always && docker compose up -d --no-build   # update the image
```

`--no-build` makes a failed download an error. Without it, Compose would build the image on the robot instead, which takes long. To run your own changes, set `ROBOT_IMAGE=diffbot-robot:local` in `.env` and build on the robot with `docker compose up -d --build --pull never`. Remove the line again to go back to the published image.

[`env.example`](env.example) explains each setting. To use the PC's ROS master, set `ROS_MASTER_URI` and `ROS_IP` in `.env`, as on [ROS Network Setup](https://ros-mobile-robots.com/processing_units/ros-network-setup/). Without them, roslaunch starts a master on the robot. `COMPOSE_PROFILES=lidar` starts the lidar too.

What it doesn't do yet: the camera (the Raspberry Pi camera driver needs the legacy camera stack, [#111](https://github.com/ros-mobile-robots/diffbot/issues/111)), and starting at boot. If the Teensy is reset, reflashed or unplugged, stop the containers and start them again (`docker compose down`, then `docker compose up -d --no-build`): a running container keeps the device it got at its start. The Teensy's firmware isn't in the image: flash it from the PC, as [Microcontroller](https://ros-mobile-robots.com/processing_units/teensy-mcu/#teensy-setup) describes.
