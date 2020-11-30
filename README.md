# DiffBot ![travis ci](https://travis-ci.com/fjp/diffbot.svg?branch=master)

DiffBot is an autonomous differential drive robot with two wheels. Its main processing unit is a Raspberry Pi 4 B running Ubuntu Mate 20.04 and the ROS 1 (ROS Noetic) middleware. This respository contains ROS driver packages, ROS Control Hardware Interface for the real robot and configurations for simulating DiffBot. The formatted documentation can be found at: https://fjp.at/projects/diffbot/

| DiffBot | Lidar SLAMTEC RPLidar A2 | 
|:-------:|:-----------------:|
|  [<img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot/diffbot-front.png" width="700">](https://fjp.at/projects/diffbot/) | [<img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot/rplidara2.png" width="700">](https://github.com/fjp/diffbot) |

## Package Overview

- [`diffbot_base`](./diffbot_base): ROS Control hardware interface including `controller_manager` control loop for the real robot
- [`diffbot_bringup`](./diffbot_bringup): Launch files to bring up the hardware drivers (camera, lidar, imu, ultrasonic, ...) for the real DiffBot robot
- [`diffbot_control`](./diffbot_control): Configurations for the `diff_drive_controller` of ROS Control used in Gazebo simulation and the real robot
- [`diffbot_description`](./diffbot_description): URDF description of DiffBot including its sensors
- [`diffbot_gazebo`](./diffbot_gazebo): Simulation specific launch and configuration files for DiffBot
- [`diffbot_navigation`](./diffbot_navigation): Navigation based on `move_base` launch and configuration files
- [`diffbot_slam`](./diffbot_slam): Simultaneous localization and mapping using different implementations to create a map of the environment

## Dependencies and Installation

The packages are written for and tested with [ROS 1 Noetic](http://wiki.ros.org/noetic) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/).
For the real robot [Ubuntu Mate 20.04](https://ubuntu-mate.org/download/arm64/focal/) for arm64 is installed on the [Raspberry Pi 4 B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) with 4GB. The communication between the mobile robot and the work pc is done by configuring the [ROS Network](http://wiki.ros.org/ROS/NetworkSetup), see also the [documentation](./docs).

### Dependencies

The required Ubuntu packages are listed in the [documentation](./docs). Other ROS catkin packages such as [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) need to be cloned into the catkin workspace. It is planned to use [`vcstool`](https://github.com/dirk-thomas/vcstool) in the future to automate the dependency installtions.

### Installation

To build the packages in this repository, clone it in the `src` folder of your ROS Noetic catkin workspace:

```console
catkin_ws/src$ git clone https://github.com/fjp/diffbot.git
```

After installing the required dependencies build the catkin workspace:

```console
catkin_ws$ catkin_make
```
or using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/):

```console
catkin_ws$ catkin build
```


## Usage

The following sections describe how to run the robot simulation and how to make use of the real hardware using the available package launch files.

### Gazebo Simulation with ROS Control

Control the robot inside Gazebo and view what it sees in RViz using the following launch file:

```console
roslaunch diffbot_control diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/corridor.world'
```

To run the [turtlebot3_world](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/models/turtlebot3_world) 
make sure to download it to your `~/.gazebo/models/` folder, because the `turtlebot3_world.world` file references the `turtlebot3_world` model.

| `corridor.world` | `turtlebot3_world.world` | 
|:-------------------------------------:|:--------------------------------:|
| ![corridor-world](https://github.com/fjp/diffbot/blob/master/docs/resources/control/diffbot-rplidar.png) | ![turtlebot3-world](https://github.com/fjp/diffbot/blob/master/docs/resources/control/diffbot-turtlebot3-world.png) |

#### Navigation

To navigate the robot in the simulation run this command:

```console
roslaunch diffbot_navigation diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

Navigate the robot in a known map from the running [`map_server`](http://wiki.ros.org/map_server) using the [2D Nav Goal in RViz](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#A2D_Nav_Goal).

[![DiffBot navigation](https://github.com/fjp/diffbot/blob/master/docs/resources/navigation/diffbot-navigation-gazebo-turtlebot3-world-small.gif)](https://youtu.be/2SwFTrJ1Ofg)

#### SLAM

To map a new environment using slam gmapping first run

```console
roslaunch diffbot_gazebo diffbot.launch world_name:='$(find diffbot_gazebo)/worlds/turtlebot3_world.world'
```

and in a second terminal execute

```console
roslaunch diffbot_slam diffbot_slam.launch
```

Then explore the world with the [`teleop_twist_keyboard`](http://wiki.ros.org/teleop_twist_keyboard) or with the already launched [`rqt_robot_steering`](https://wiki.ros.org/rqt_robot_steering) GUI plugin:

[![DiffBot slam](https://github.com/fjp/diffbot/blob/master/docs/resources/slam/diffbot-slam.gif)](https://youtu.be/gLlo5V-BZu0)

### DiffBot Control in Gazebo

```console
roslaunch diffbot_control diffbot.launch
```

![DiffBot Gazebo](docs/resources/gazebo/diffbot.png)

### RViz

View just the `diffbot_description` in RViz.

```console
roslaunch diffbot_description view_diffbot.launch
```
![DiffBot RViz](docs/resources/rviz_diffbot_meshes.png)

## Future Work

Contributions to these tasks are welcome, see also the [contribution section](./README.md#contributions) below.

### Drivers, Odometry and Hardware Interface

- Add `diffbot_driver` package for ultrasonic ranger, imu and motor driver node code.
- Include the [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros) package repository as git submodule.
- Make use of the imu odometry data to improve the encoder odometry using [`robot_pose_ekf`](http://wiki.ros.org/robot_pose_ekf).
- The current implementation of the ROS Control `hardware_interface::RobotHW` uses a high level PID controller. This is working but also
test a low level PID on the Teensy 3.2 mcu using the [Arduino library of the Grove i2c motor driver](https://github.com/Seeed-Studio/Grove_I2C_Motor_Driver_v1_3). 
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


## Part List

| SBC RPi 4B | MCU Teensy 3.2 | IMU Bosch |
|:-------:|:-----------------:|:----:|
|  [<img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot/sbc-rpi-4b.png" width="700">](https://fjp.at/projects/diffbot/) | [<img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot/mcu-teensy32.png" width="700">](https://github.com/fjp/diffbot) | [<img src="https://raw.githubusercontent.com/fjp/diffbot/master/docs/resources/diffbot/imu.png" width="700">](https://github.com/fjp/diffbot) |

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


## Additional (Optional) Equipment

| Part                                   | Store |
|:---------------------------------------|:------------------------------------:|
| PicoScope 3000 Series Oscilloscope 2CH | [Amazon.de](https://amzn.to/33I5tUb) |
| VOLTCRAFT PPS-16005                    | [Amazon.de](https://amzn.to/3iKsI4a) |

## Acknowledgment

- [Lentin Joseph](https://lentinjoseph.com/) and the participants of [ROS Developer Learning Path](https://robocademy.com/2020/06/25/enroll-in-robot-operating-system-learning-path-by-lentin-joseph/)
- The configurable `diffbot_description` using yaml files (see [ROS Wiki on xacro](http://wiki.ros.org/xacro#YAML_support)) is part of [`mobile_robot_description`](https://github.com/pxalcantara/mobile_robot_description) from [@pxalcantara](https://github.com/pxalcantara).
- Thanks to [@NestorDP](https://github.com/NestorDP) for help with the meshes (similar to [`littlebot`](https://github.com/NestorDP/littlebot)), see also [issue #1](https://github.com/fjp/diffbot/issues/1)
- [`dfki-ric/mir_robot`](https://github.com/dfki-ric/mir_robot)
- [`eborghi10/my_ROS_mobile_robot`](https://github.com/eborghi10/my_ROS_mobile_robot)
- [`husky`](https://github.com/husky/husky)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)


## Contributions

Your contributions are more than welcome. These can be in the form of raising issues, creating PRs to correct or add documentation and of course solving existing issues or adding new features.


## License

Licensed under GNU GPLv3, see [LICENSE](./LICENSE)

Feel free to reuse or modify this work but please cite it in case you do so:

> Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
