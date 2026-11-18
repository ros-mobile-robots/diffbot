# Robot container

ROS 1 Noetic and the robot's packages in a Docker image, for the robot's computer: a Raspberry Pi 4 B with a current host OS, such as Ubuntu Server 24.04, instead of a native Ubuntu 20.04 installation. The page [Robot Container](https://ros-mobile-robots.com/processing_units/robot-container/) explains the setup step by step; this is the short version.

**Experimental:** CI builds the image for arm64 and amd64 and tests it without hardware. On Remo's Raspberry Pi 4 B, an image built on that Raspberry Pi started the bringup and turned the wheels by teleop, with the wheels off the ground ([the bench record](https://github.com/ros-mobile-robots/diffbot/issues/160#issuecomment-6550353204)). Still open on a robot: the published image's download, the lidar's scans and a drive on the floor ([#109](https://github.com/ros-mobile-robots/diffbot/issues/109)).

| File | What it's for |
|:--|:--|
| [`Dockerfile`](Dockerfile) | The image: `ros:noetic-ros-base`, the packages that the bringup and the lidar need for both robots, built in `~/catkin_ws` |
| [`compose.yaml`](compose.yaml) | Runs the bringup, and with the `lidar` profile the lidar, with the host's network and the serial devices |
| [`compose.build.yaml`](compose.build.yaml) | Builds the image from your clone, as `diffbot-robot:local`, to run your own changes |
| [`env.example`](env.example) | The template for `.env`, the robot's settings, which `docker compose` reads from this folder |
| [`99-diffbot-robot.rules`](99-diffbot-robot.rules) | udev rules for the host: `/dev/teensy` and `/dev/rplidar` |
| [`entrypoint.sh`](entrypoint.sh) | Loads ROS and the workspace before each command |
| [`smoke_test.sh`](smoke_test.sh) | The test without hardware that [CI](../../.github/workflows/robot_container.yml) runs inside a container of the image |
| [`publish_image.sh`](publish_image.sh) | Publishes the tested images for both architectures, in CI on `noetic-devel` |
| [`test_publish_image.sh`](test_publish_image.sh) | Tests what `publish_image.sh` writes for each answer of the registry, without a registry. CI runs it for every pull request |

CI publishes the image that passed the smoke test, for arm64 and amd64, as [`ghcr.io/ros-mobile-robots/diffbot-robot`](https://github.com/orgs/ros-mobile-robots/packages/container/package/diffbot-robot): `noetic` is the newest from `noetic-devel`, and `noetic-<commit>` stays the same. The robot downloads it, so it doesn't build anything.

On the robot's host, with Docker Engine (with Compose and Buildx) and this repository cloned to `~/diffbot`:

```bash
sudo cp ~/diffbot/.docker/robot/99-diffbot-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir -p ~/robot-data      # ROS logs and bags; as the first user (UID 1000, like ros in the container)
cd ~/diffbot/.docker/robot
cp env.example .env        # then edit .env: the robot model, the lidar, the ROS network
docker compose up -d       # download the image if needed, start the bringup in the background
docker compose logs -f     # follow the output. Ctrl+C stops following, the robot keeps running
docker compose down        # stop
docker compose pull --policy always && docker compose up -d   # update the image
```

`compose.yaml` only runs an image and never builds one, so a failed download is an error. To run your own changes, build the image on the robot with `docker compose -f compose.yaml -f compose.build.yaml build`. The build gets a name of its own, `diffbot-robot:local`, never the published image's name. Then set `ROBOT_IMAGE=diffbot-robot:local` in `.env` and start the build with `docker compose up -d --pull never`. Remove the line again to go back to the published image.

[`env.example`](env.example) explains each setting. The ROS master runs in one of two places:

- **On your PC:** set `ROS_MASTER_URI` and `ROS_IP` in `.env`, as on [ROS Network Setup](https://ros-mobile-robots.com/processing_units/ros-network-setup/), and start `roscore` on the PC first. `COMPOSE_PROFILES=lidar` then starts the lidar too.
- **On the robot:** without `ROS_MASTER_URI`, roslaunch starts a ROS master in the bringup container. Run the bringup alone then, without `COMPOSE_PROFILES=lidar`. Started together, the bringup and the lidar container can each start a ROS master, and one of the two containers then stops. In a test without hardware, that happened in each of three starts. [Without any PC](https://ros-mobile-robots.com/processing_units/robot-container/usage/#without-any-pc) has the steps.

What it doesn't do yet: the camera (the Raspberry Pi camera driver needs the legacy camera stack, [#111](https://github.com/ros-mobile-robots/diffbot/issues/111)), and starting at boot. If the Teensy is reset, reflashed or unplugged, stop the containers and start them again (`docker compose down`, then `docker compose up -d`): a running container keeps the device it got at its start. The Teensy's firmware isn't in the image: flash it from the PC, as [Microcontroller](https://ros-mobile-robots.com/processing_units/teensy-mcu/#teensy-setup) describes.
