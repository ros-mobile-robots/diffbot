# DiffBot 

![CI](https://github.com/ros-mobile-robots/diffbot/workflows/CI/badge.svg?branch=noetic-devel)
[![Documentation CI](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/actions/workflows/ci.yml/badge.svg)](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/actions/workflows/ci.yml)

DiffBot is an autonomous differential drive robot with two wheels. Its main processing unit is a Raspberry Pi 4 B running Ubuntu Mate 20.04 and the ROS 1 (ROS Noetic) middleware. This respository contains ROS driver packages, ROS Control Hardware Interface for the real robot and configurations for simulating DiffBot. The formatted documentation can be found at: https://ros-mobile-robots.com.

| DiffBot | Lidar SLAMTEC RPLidar A2 | 
|:-------:|:-----------------:|
|  [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/diffbot/diffbot-front.png" width="700">](https://youtu.be/IcYkQyzUqik) | [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/diffbot/rplidara2.png" width="700">](https://fjp.at/projects/diffbot/) |

If you are looking for a 3D printable modular base, see the [`remo_description`](https://github.com/ros-mobile-robots/remo_description) repository. You can use it directly with the software of this `diffbot` repository.

| Remo | Gazebo Simulation | RViz |
|:-------:|:-----------------:|:----:|
|  [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo_front_side.jpg" width="700">](https://youtu.be/IcYkQyzUqik) | [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/remo-gazebo.png" width="700">](https://github.com/fjp/diffbot) | [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/remo/camera_types/oak-d.png?raw=true" width="700">](https://github.com/ros-mobile-robots/diffbot) |

It provides mounts for different camera modules, such as Raspi Cam v2, OAK-1, OAK-D and you can even design your own if you like. There is also support for different single board computers (Raspberry Pi and Nvidia Jetson Nano) through two changable decks. You are agin free to create your own.

## Demonstration

### SLAM and Navigation

| Real robot | Gazebo Simulation  | 
|:-------:|:-----------------:|
|  [<img src="https://img.youtube.com/vi/IcYkQyzUqik/hqdefault.jpg" width="250">](https://youtu.be/IcYkQyzUqik) | [<img src="https://img.youtube.com/vi/gLlo5V-BZu0/hqdefault.jpg" width="250">](https://youtu.be/gLlo5V-BZu0) [<img src="https://img.youtube.com/vi/2SwFTrJ1Ofg/hqdefault.jpg" width="250">](https://youtu.be/2SwFTrJ1Ofg) |

## :package: Package Overview

- [`diffbot_base`](./diffbot_base): ROS Control hardware interface including [`controller_manager`](http://wiki.ros.org/controller_manager) control loop for the real robot. The [`scripts` folder](./diffbot_base/scripts) of this package contains the low-level `base_controller` that is running on the Teensy microcontroller.
- [`diffbot_bringup`](./diffbot_bringup): Launch files to bring up the hardware drivers (camera, lidar, imu, ultrasonic, ...) for the real DiffBot robot.
- [`diffbot_control`](./diffbot_control): Configurations for the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller) of ROS Control used in Gazebo simulation and the real robot.
- [`diffbot_description`](./diffbot_description): URDF description of DiffBot including its sensors.
- [`diffbot_gazebo`](./diffbot_gazebo): Simulation specific launch and configuration files for DiffBot.
- [`diffbot_msgs`](./diffbot_msgs): Message definitions specific to DiffBot, for example the message for encoder data.
- [`diffbot_navigation`](./diffbot_navigation): Navigation based on [`move_base` package](http://wiki.ros.org/move_base); launch and configuration files.
- [`diffbot_slam`](./diffbot_slam): Simultaneous localization and mapping using different implementations (e.g., [gmapping](http://wiki.ros.org/gmapping)) to create a map of the environment

## Installation

The packages are written for and tested with [ROS 1 Noetic](http://wiki.ros.org/noetic) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/).
For the real robot [Ubuntu Mate 20.04](https://ubuntu-mate.org/download/arm64/focal/) for arm64 is installed on the [Raspberry Pi 4 B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) with 4GB. The communication between the mobile robot and the work pc is done by configuring the [ROS Network](http://wiki.ros.org/ROS/NetworkSetup), see also the [documentation](./docs/ros-network-setup.md).

### Dependencies

The required Ubuntu packages are listed in the [documentation](./docs). Other ROS catkin packages such as [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) need to be cloned into the catkin workspace. It is planned to use [`vcstool`](https://github.com/dirk-thomas/vcstool) in the future to automate the dependency installtions.

### :hammer: How to Build

To build the packages in this repository including the Remo robot follow these steps:

1. Clone this repository in the `src` folder of your ROS Noetic [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

   ```console
   catkin_ws/src$ git clone https://github.com/fjp/diffbot.git
   ```
   
2. Execute the `vcs import` command from the root of the catkin workspace and pipe in the `diffbot_dev.repos` or `remo_robot.repos` YAML file, depending on where you execute the command, either the development PC or the SBC of Remo to clone the listed dependencies. Run the following command only on your development machine:

   ```
   vcs import < src/diffbot/diffbot_dev.repos
   ```

   Run the next command on Remo robot's SBC:

   ```
   vcs import < src/diffbot/remo_robot.repos
   ```
   
3. Install the requried binary dependencies of all packages in the catkin workspace using the following [`rosdep` command](http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace):

   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. After installing the required dependencies build the catkin workspace, either with [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make):

   ```console
   catkin_ws$ catkin_make
   ```
   or using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/):

   ```console
   catkin_ws$ catkin build
   ```
   
5. Finally, source the newly built packages with the `devel/setup.*` script, depending on your used shell:

   For bash use:

   ```console
   catkin_ws$ source devel/setup.bash
   ```

   For zsh use:

   ```console
   catkin_ws$ source devel/setup.zsh
   ```

## Usage

The following sections describe how to run the robot simulation and how to make use of the real hardware using the available package launch files.

### Gazebo Simulation with ROS Control

Control the robot inside Gazebo and view what it sees in RViz using the following launch file:

```console
roslaunch diffbot_control diffbot.launch
```

This will launch the default diffbot world `db_world.world`.

To run the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/models/turtlebot3_world) 
make sure to download it to your `~/.gazebo/models/` folder, because the `turtlebot3_world.world` file references the `turtlebot3_world` model.
After that you can run it with the following command:

```console
roslaunch diffbot_control diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

| `db_world.world` | `turtlebot3_world.world` | 
|:-------------------------------------:|:--------------------------------:|
| ![corridor-world](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/control/diffbot_world_control.png) | ![turtlebot3-world](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/control/diffbot-turtlebot3-world.png) |

#### Navigation

To navigate the robot in the Gazebo simulator in `db_world.world` run the command:

```console
roslaunch diffbot_navigation diffbot.launch
```

This uses a previously mapped map of `db_world.world` (found in [`diffbot_navigation/maps`](./diffbot_navigation/maps/)) that is served by
the [`map_server`](http://wiki.ros.org/map_server). With this you can use the [2D Nav Goal in RViz](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#A2D_Nav_Goal) directly to let the robot drive autonomously in the `db_world.world`.

[![DiffBot navigation](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/raw/main/docs/resources/navigation/db_world-nav.gif)](https://youtu.be/2SwFTrJ1Ofg)

To run the `turtlebot3_world.world` (or your own stored world and map) use the same `diffbot_navigation/launch/diffbot.launch` file but change
the `world_name` and `map_file` arguments to your desired world and map yaml files:

```console
roslaunch diffbot_navigation diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world' map_file:='$(find diffbot_navigation)/maps/map.yaml'
```

[![DiffBot navigation](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/raw/main/docs/resources/navigation/diffbot-navigation-gazebo-turtlebot3-world-small.gif)](https://youtu.be/2SwFTrJ1Ofg)

#### SLAM

To map a new simulated environment using slam gmapping, first run

```console
roslaunch diffbot_gazebo diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

and in a second terminal execute

```console
roslaunch diffbot_slam diffbot_slam.launch slam_method:=gmapping
```

Then explore the world with the [`teleop_twist_keyboard`](http://wiki.ros.org/teleop_twist_keyboard) or with the already launched [`rqt_robot_steering`](https://wiki.ros.org/rqt_robot_steering) GUI plugin:

[![DiffBot slam](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/slam/diffbot-slam.gif)](https://youtu.be/gLlo5V-BZu0)

When you finished exploring the new world, use the [`map_saver`](http://wiki.ros.org/map_server#map_saver) node from the [`map_server`](http://wiki.ros.org/map_server) package to store the mapped enviornment:

```console
rosrun map_server map_saver -f ~/map
```


### RViz

View just the `diffbot_description` in RViz.

```console
roslaunch diffbot_description view_diffbot.launch
```
![DiffBot RViz](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/rviz_diffbot_meshes.png)


### Navigating and Mapping on the real Robot

The following video shows how to map a new environment and navigate in it

[<img src="https://img.youtube.com/vi/IcYkQyzUqik/hqdefault.jpg" width="250">](https://youtu.be/IcYkQyzUqik)

Start by setting up the ROS Network, by making the development PC the rosmaster (set the `ROS_MASTER_URI` environment variable accordingly, see [ROS Network Setup](https://ros-mobile-robots.com/ros-network-setup/) for more details), 
Then follow the steps listed below to run the real Diffbot or Remo robot hardware:

1. First, brinup the robot hardware including its laser with the following launch file from the [`diffbot_bringup`](./diffbot_bringup) package.
Make sure to run this on the real robot (e.g. connect to it via `ssh`):

   ```console
   roslaunch diffbot_bringup bringup_with_laser.launch
   ```

2. Then, in a new terminal on your remote/work development machine (not the single board computer) run the slam gmapping with the same command as in the simulation:

   ```console
   roslaunch diffbot_slam diffbot_slam.launch slam_method:=gmapping
   ```

   As you can see in the video, this should open up RViz and the [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering) plugin.

3. Next, steer the robot around manually either using the `keyboard_teleop` node or using the `rqt_robot_steering` node 
and save the map with the following command when you are done exploring:

   ```console
   rosrun map_server map_saver -f office
   ```

After the mapping process it is possible to use the created map for navigation, after running the following launch files:

1. On the single board computer (e.g. Raspberry Pi) make sure that the following is launched:

   ```console
   roslaunch diffbot_bringup bringup_with_laser.launch
   ```

2. Then on the work/remote development machine run the `diffbot_hw.lauch` from the `diffbot_navigation` package:

   ```console
   roslaunch diffbot_navigation diffbot_hw.lauch
   ```

   Among other essential navigation and map server nodes, this will also launch an instance of RViz on your work pc where you can use its tools to: 

   1. Localize the robot with the "2D Pose Estimate" tool (green arrow) in RViz
   2. Use the "2D Nav Goal" tool in RViz (red arrow) to send goals to the robot

## :construction: Future Work

Contributions to these tasks are welcome, see also the [contribution section](./README.md#contributions) below.

### ROS 2

- Migrate from ROS 1 to ROS 2

### Drivers, Odometry and Hardware Interface

- Add `diffbot_driver` package for ultrasonic ranger, imu and motor driver node code.
- Make use of the imu odometry data to improve the encoder odometry using [`robot_pose_ekf`](http://wiki.ros.org/robot_pose_ekf).
- The current implementation of the ROS Control `hardware_interface::RobotHW` uses a high level PID controller. This is working but also
test a low level PID on the Teensy 3.2 mcu using the [Arduino library of the Grove i2c motor driver](https://github.com/Seeed-Studio/Grove_I2C_Motor_Driver_v1_3). -> This is partly implemented (see `diffbot_base/scripts/base_controller`)
Also replace `Wire.h` with the improved [`i2c_t3`](https://github.com/nox771/i2c_t3) library.

### Navigation

- Test different global and local planners and add documentation
- Add `diffbot_mbf` package using [`move_base_flex`](http://wiki.ros.org/move_base_flex), the improved version of [`move_base`](http://wiki.ros.org/move_base).

### Perception

To enable object detection or semantic segmentation with the RPi Camera the Raspberry Pi 4 B will be upated with a Google Coral USB Accelerator.
Possible useful packages:

- [MSeg](https://github.com/mseg-dataset/mseg-semantic)

![Mseg Example](https://user-images.githubusercontent.com/62491525/83895683-094caa00-a721-11ea-8905-2183df60bc4f.gif)

### Teleoperation

- Use the generic [`teleop_twist_keyboard`](http://wiki.ros.org/teleop_twist_keyboard) and/or [`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) package to drive the real robot and in simulation.
- Playstation controller

### Tooling

- [`vcstool`](https://github.com/dirk-thomas/vcstool) to simplify external dependency installation
- Adding instructions how to use [`rosdep`](http://wiki.ros.org/rosdep) to install required system dependencies
- Use [clang format](https://clang.llvm.org/docs/ClangFormat.html) together with [`.clang-format`](https://github.com/PickNikRobotics/roscpp_code_format) file for `roscpp` to comply with [ROS C++ Style Guidelines](http://wiki.ros.org/CppStyleGuide)


## Part List Diffbot

| SBC RPi 4B | MCU Teensy 3.2 | IMU Bosch |
|:-------:|:-----------------:|:----:|
|  [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/diffbot/sbc-rpi-4b.png" width="700">](https://ros-mobile-robots.com/) | [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/diffbot/mcu-teensy32.png" width="700">](https://github.com/ros-mobile-robots/diffbot) | [<img src="https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/resources/diffbot/imu.png" width="700">](https://github.com/ros-mobile-robots/diffbot) |

| Part                    | Store |
|:------------------------|:---------------------------------------------------------------------------:|
| Raspberry Pi 4 B (4 Gb) | [Amazon.com](https://amzn.to/3ltuJUo), [Amazon.de](https://amzn.to/2IchIAc) |
| SanDisk 64 GB SD Card Class 10 | [Amazon.com](https://amzn.to/2GLOyr0), [Amazon.de](https://amzn.to/3dcFmYE) |
|Robot Smart Chassis Kit  | [Amazon.com](https://amzn.to/34GXNAK), [Amazon.de](https://amzn.to/2Gy3CJ4) |
| SLAMTEC RPLidar A2M8 (12 m) | [Amazon.com](https://amzn.to/3lthTFz), [Amazon.de](https://amzn.to/30MyImR) |
| Grove Ultrasonic Ranger | [Amazon.com](https://amzn.to/36M9TLS), [Amazon.de](https://amzn.to/34GZmyC) |
| Raspi Camera Module V2, 8 MP, 1080p | [Amazon.com](https://amzn.to/2Ib9fgG), [Amazon.de](https://amzn.to/2FdVDQF) |
| Grove Motor Driver | [seeedstudio.com](https://www.seeedstudio.com/Grove-I2C-Motor-Driver-with-L298.html), [Amazon.de](https://amzn.to/36M8O6M) |
| I2C Hub | [seeedstudio.com](https://www.seeedstudio.com/Grove-I2C-Hub.html), [Amazon.de](https://amzn.to/34CGEbz) |
| Teensy 4.0 or 3.2 | [PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html), [PJRC Teensy 3.2](https://www.pjrc.com/store/teensy32.html) |
| Hobby Motor with Encoder - Metal Gear (DG01D-E) | [Sparkfun](https://www.sparkfun.com/products/16413) |

## Part List Remo

| Part                    | Store |
|:------------------------|:---------------------------------------------------------------------------:|
| Raspberry Pi 4 B (4 Gb) | [Amazon.com](https://amzn.to/3ltuJUo), [Amazon.de](https://amzn.to/2IchIAc) |
| SanDisk 64 GB SD Card Class 10 | [Amazon.com](https://amzn.to/2GLOyr0), [Amazon.de](https://amzn.to/3dcFmYE) |
| Remo Base  | 3D printable, see [`remo_description`](https://github.com/ros-mobile-robots/remo_description) |
| SLAMTEC RPLidar A2M8 (12 m) | [Amazon.com](https://amzn.to/3lthTFz), [Amazon.de](https://amzn.to/30MyImR) |
| Raspi Camera Module V2, 8 MP, 1080p | [Amazon.com](https://amzn.to/2Ib9fgG), [Amazon.de](https://amzn.to/2FdVDQF) |
| Adafruit DC Motor (+ Stepper) FeatherWing  | [adafruit.com](https://www.adafruit.com/product/2927), [Amazon.de](https://amzn.to/3km5KF3) |
| Teensy 4.0 or 3.2 | [PJRC Teensy 4.0](https://www.pjrc.com/store/teensy40.html), [PJRC Teensy 3.2](https://www.pjrc.com/store/teensy32.html) |
| Hobby Motor with Encoder - Metal Gear (DG01D-E) | [Sparkfun](https://www.sparkfun.com/products/16413) |
| Powerbank (e.g 15000 mAh) | [Amazon.de](https://amzn.to/3kmkx2t) This Powerbank from Goobay is close to the maximum possible size LxWxH: 135.5x70x18 mm) |
| Battery pack (for four or eight batteries) | [Amazon.de](https://amzn.to/3kiX8PH) |


## Additional (Optional) Equipment

| Part                                   | Store |
|:---------------------------------------|:------------------------------------:|
| PicoScope 3000 Series Oscilloscope 2CH | [Amazon.de](https://amzn.to/33I5tUb) |
| VOLTCRAFT PPS-16005                    | [Amazon.de](https://amzn.to/3iKsI4a) |
| 3D Printer for Remo's parts            | [Prusa](https://shop.prusa3d.com/en/17-3d-printers), [Ultimaker](https://ultimaker.com/), etc. or use a local print service or an online one such as [Sculpteo](https://www.sculpteo.com/) |

## Hardware Architecture and Wiring

<details>
  <summary><b>DiffBot</b></summary>

![Hardware Architecture and Wiring](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/fritzing/diffbot_architecture.svg)

</details>

<details open>
<summary><b>Remo</b></summary>

![Hardware Architecture and Wiring](https://raw.githubusercontent.com/ros-mobile-robots/ros-mobile-robots.github.io/main/docs/fritzing/remo_architecture.svg)

</details>

## :handshake: Acknowledgment

- [Louis Morandy-Rapiné](https://louisrapine.com/) for his great work on [REMO robot](https://github.com/ros-mobile-robots/remo_description) and designing it in [Fusion 360](https://www.autodesk.com/products/fusion-360/overview).
- [Lentin Joseph](https://lentinjoseph.com/) and the participants of [ROS Developer Learning Path](https://robocademy.com/2020/06/25/enroll-in-robot-operating-system-learning-path-by-lentin-joseph/)
- The configurable `diffbot_description` using yaml files (see [ROS Wiki on xacro](http://wiki.ros.org/xacro#YAML_support)) is part of [`mobile_robot_description`](https://github.com/pxalcantara/mobile_robot_description) from [@pxalcantara](https://github.com/pxalcantara).
- Thanks to [@NestorDP](https://github.com/NestorDP) for help with the meshes (similar to [`littlebot`](https://github.com/NestorDP/littlebot)), see also [issue #1](https://github.com/fjp/diffbot/issues/1)
- [`dfki-ric/mir_robot`](https://github.com/dfki-ric/mir_robot)
- [`eborghi10/my_ROS_mobile_robot`](https://github.com/eborghi10/my_ROS_mobile_robot)
- [`husky`](https://github.com/husky/husky)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [Linorobot](https://github.com/linorobot/linorobot)


## :wrench: Contributing

Your contributions are most welcome. These can be in the form of raising issues, creating PRs to correct or add documentation and of course solving existing issues or adding new features.


## :pencil: License

`diffbot` is licenses under the [BSD 3-Clause](./LICENSE).
See also [open-source-license-acknowledgements-and-third-party-copyrights.md](open-source-license-acknowledgements-and-third-party-copyrights.md).
The [documentation](https://ros-mobile-robots.com/) is licensed differently,
visit its [license text](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io#license) to learn more.
