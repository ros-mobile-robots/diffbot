# Teensy firmware

The firmware for the robots' Teensy microcontroller: it reads the wheel encoders, runs a velocity PID for each wheel, and drives the motors. It talks to the Raspberry Pi over USB with rosserial, which passes its messages on to `diffbot_base`'s hardware interface. It's a [PlatformIO](https://platformio.org/) project, not a ROS package, so a catkin build of the workspace skips it.

| Folder | What it's for |
|:--|:--|
| [`src`](src) | `main.cpp`: the control loop |
| [`lib`](lib) | The wheel control, command timeout, base controller, encoders, motor driver, IMU, PID, configuration, and the rosserial message headers in `ros_lib` |
| [`test`](test) | Unit tests that run on the PC: Unity suites in `test_*`, GoogleTest suites in `googletest/test_*`, and fakes for the hardware in `fakes`. Also checks that run on the Teensy |
| [`sketches`](sketches) | Older Arduino sketches: the encoders alone over rosserial, the same with the BNO055 IMU, and a check of the encoders by hand |

Build the firmware for your robot's Teensy in the firmware's container, [`.docker/firmware`](../.docker/firmware). It needs only Docker. Our script [`.docker/firmware/run.sh`](../.docker/firmware/run.sh) builds the image and runs a command in the container as your user, here PlatformIO's `pio`. From the repository's root folder, run the line for your robot:

```bash
.docker/firmware/run.sh pio run -e teensy40   # Remo: Teensy 4.0
.docker/firmware/run.sh pio run -e teensy31   # DiffBot: Teensy 3.2
```

When you change the firmware, build both and run the unit tests, `.docker/firmware/run.sh pio test -e native -e native_googletest`, as CI does, because the change has to work on both boards.

The unit tests use two frameworks, and [`platformio.ini`](platformio.ini) has an environment for each:

| Framework | Suites | Environment | Used for |
|:--|:--|:--|:--|
| [Unity](https://github.com/ThrowTheSwitch/Unity), which can also run on a Teensy | [`test/test_*`](test) | `native` | The tests of the rosserial code, and the suites of the core that are still written for Unity |
| [GoogleTest](https://google.github.io/googletest/), which PlatformIO offers for the PC and not for a Teensy | [`test/googletest/test_*`](test/googletest) | `native_googletest` | New tests of the core. A Unity suite of the core moves here when its code changes |

[`test/googletest/test_wire_bus`](test/googletest/test_wire_bus/test_wire_bus.cpp) shows the pattern for a GoogleTest suite: a fixture for what the tests share, and a parameterized test for a test that runs with several values. [Firmware unit tests](https://ros-mobile-robots.com/development/ci/#firmware-unit-tests) on ros-mobile-robots.com explains both frameworks and how to write a test.

The firmware files land in `.pio/build/<board>/` in this folder, for example `firmware.hex`, which PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html) can flash. With PlatformIO installed on your PC ([CONTRIBUTING](../CONTRIBUTING.md)), `pio run -e teensy40` or `pio run -e teensy31` in this folder builds it too.

## Ports and adapters

The firmware is moving to Alistair Cockburn's [ports and adapters](https://alistair.cockburn.us/hexagonal-architecture/) structure, also called hexagonal architecture ([#148](https://github.com/ros-mobile-robots/diffbot/issues/148)). It separates the logic from the libraries and the hardware, so that the unit tests can run the logic on a PC. [Ports and adapters](https://ros-mobile-robots.com/packages/diffbot_base/low-level/#ports-and-adapters) on ros-mobile-robots.com shows it in diagrams.

- The **core** contains the logic that we want to test independently. The firmware's portable core includes no Arduino, rosserial or vendor library headers.
- A **port** defines an interaction across the core's boundary: the functions, their arguments and their required behavior. A port can be ordinary public functions, such as `WheelControl`'s `receiveCommand()` and `step()`. A secondary port can also be an abstract class, such as `I2cBus`, or requirements on a template argument, such as `Controller`.
- An **adapter** connects a port to a particular environment, such as ROS, Arduino, a motor driver or a test harness.

The two kinds of ports differ in who calls whom:

| | Primary port | Secondary port |
|:--|:--|:--|
| Who calls the functions | an adapter | the core |
| Who implements the functions | the core | an adapter |
| Other names | driving, inbound | driven, outbound |

Where the firmware stands today:

| Core | Primary ports, with the adapter that calls each | Secondary ports, with the adapter that implements each |
|:--|:--|:--|
| [`WheelControl`](lib/wheel_control/wheel_control.h) | `receiveCommand()`: [`RosWheelCommandSubscriber`](lib/ros_wheel_commands/ros_wheel_command_subscriber.h)<br>`step()`: `loop()` in [`main.cpp`](src/main.cpp) | `Controller`: [`BaseController`](lib/base_controller/base_controller.h)<br>`LogTimeout`: `RosTimeoutLogger` in [`main.cpp`](src/main.cpp) |
| [`Bno055`](lib/imu/bno055.h), the portable sensor driver | `update()`, `read()`, `restart()`, `state()`: [`ImuPublisher`](lib/imu/imu_publisher.h) | [`I2cBus`](lib/imu/i2c_bus.h): [`WireBusT`](lib/imu/wire_bus.h) |

`main.cpp` creates the objects and connects them. `CommandTimeout` is internal to `WheelControl` and has no separate port across this boundary. `BaseController` supplies the `Controller` operations, and also contains PID logic and dependencies on rosserial, the encoder library and the motor library. The work planned in [#148](https://github.com/ros-mobile-robots/diffbot/issues/148) separates those responsibilities.

## More

[Low-Level Base Controller](https://ros-mobile-robots.com/packages/diffbot_base/low-level/) on ros-mobile-robots.com explains the code, and [Microcontroller](https://ros-mobile-robots.com/processing_units/teensy-mcu/) the Teensy.
