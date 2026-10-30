# Teensy firmware

The firmware for the robots' Teensy microcontroller: it reads the wheel encoders, runs a velocity PID for each wheel, and drives the motors. It talks to the Raspberry Pi over USB with rosserial, which passes its messages on to `diffbot_base`'s hardware interface. It's a [PlatformIO](https://platformio.org/) project, not a ROS package, so a catkin build of the workspace skips it.

| Folder | What it's for |
|:--|:--|
| [`src`](src) | `main.cpp`: the control loop |
| [`lib`](lib) | The base controller, encoders, motor driver, PID, configuration, and the rosserial message headers in `ros_lib` |
| [`test`](test) | Checks that run on the Teensy |
| [`sketches`](sketches) | Older Arduino sketches: the encoders alone over rosserial, the same with the BNO055 IMU, and a check of the encoders by hand |

Build the firmware for your robot's Teensy in the firmware's container, [`.docker/firmware`](../.docker/firmware). It needs only Docker. Our script [`.docker/firmware/pio`](../.docker/firmware/pio) builds the image and runs PlatformIO's command `pio` in the container as your user. From the repository's root folder, run the line for your robot:

```bash
.docker/firmware/pio run -e teensy40   # Remo: Teensy 4.0
.docker/firmware/pio run -e teensy31   # DiffBot: Teensy 3.2
```

When you change the firmware, build both, as CI does, because the change has to work on both boards.

The firmware files land in `.pio/build/<board>/` in this folder, for example `firmware.hex`, which PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html) can flash. With PlatformIO installed on your PC ([CONTRIBUTING](../CONTRIBUTING.md)), `pio run -e teensy40` or `pio run -e teensy31` in this folder builds it too.

[Low-Level Base Controller](https://ros-mobile-robots.com/packages/diffbot_base/low-level/) on ros-mobile-robots.com explains the code, and [Microcontroller](https://ros-mobile-robots.com/processing_units/teensy-mcu/) the Teensy.
