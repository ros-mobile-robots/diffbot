# IMU: Bosch BNO055 on the Teensy

The firmware reads a BNO055 on the Teensy's I2C bus (`Wire`: SDA pin 18, SCL pin 19 on the Teensy 4.0 and 3.2, the same bus as the Adafruit motor FeatherWing) and publishes it over rosserial. Its settings are typed values in [`diffbot_base_config.h`](../config/diffbot_base_config.h): `IMU_BNO055_ADDRESS`, and `IMU_PUBLISHER_CONFIG` with the fields of [`imu_config.h`](imu_config.h). For a robot without an IMU, set its `enabled` to `false`. Not tested on the robot yet: the bench checks below are still open.

## What it publishes

`/diffbot/imu/data_raw`, `sensor_msgs/Imu`, at 50 Hz:

| Field | Content |
|:--|:--|
| `header.frame_id` | `imu_link`, the sensor's own axes. The robot model has to place this frame as the board sits on the robot |
| `header.stamp` | ROS time when the read begins, published only while rosserial's clock is synchronized with the host: after the host's time reply for the current connection. It estimates the sample's time. The BNO055 doesn't tell when it measured, and the read returns its registers' latest values |
| `angular_velocity` | rad/s, the gyroscope, corrected for its offset by the chip's fusion (see below) |
| `linear_acceleration` | m/s², the specific force including gravity (about +9.8 m/s² on the upward axis at rest), corrected for its offset by the chip's fusion |
| `orientation` | not provided: `orientation_covariance[0] = -1`, as REP 145 says for a missing field |
| covariances | diagonal starting values from the datasheet, not measured yet |

A sample is published only after a read that succeeded and took at most `read_budget_us`, and only while the sensor reports that its fusion runs (`SYS_STATUS` 5), so a stalled, starting or unplugged IMU stops the topic instead of repeating old values. If the sensor resets itself or reads fail ten times in a row, the firmware starts it again. A missing IMU is looked for again after `retry_ms`, without blocking the motor control loop. The 50 Hz period counts on the Teensy's own clock (`millis()`), so the rate doesn't change when ROS time jumps, for example after a reconnect to a host whose clock is earlier.

| File | What it does |
|:--|:--|
| [`bno055.h`](bno055.h) | The driver: start-up, configuration and reads, through [`i2c_bus.h`](i2c_bus.h) |
| [`wire_bus.h`](wire_bus.h) | The I2C bus on Arduino's Wire library |
| [`synced_node_handle.h`](synced_node_handle.h) | rosserial's node handle, plus whether its clock is synchronized |
| [`imu_publisher.h`](imu_publisher.h) | The rate, the checks above and the message |
| [`imu_config.h`](imu_config.h) | The publisher's settings, as one typed value |

The unit tests run all four on the PC, with a fake sensor, Wire library, serial port and clock, and rosserial's own parser: `.docker/firmware/run.sh pio test -e native`, from the repository's root, runs [`test/test_bno055`](../../test/test_bno055), [`test_wire_bus`](../../test/test_wire_bus), [`test_synced_node_handle`](../../test/test_synced_node_handle) and [`test_imu_publisher`](../../test/test_imu_publisher). They check the logic, not the hardware: the bench checks below are still needed.

## The fusion mode

The BNO055 has a processor of its own, which runs Bosch's sensor fusion. In the mode this driver selects, IMUPLUS, it combines the accelerometer and the gyroscope into a relative orientation, without the magnetometer ([datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf), rev. 1.8, section 3.3.3.1). It also calibrates both sensors in the background and corrects their data registers for the offsets (sections 3.6.5.1 and 3.6.5.3). The driver reads only these corrected rates and accelerations, not the orientation: the EKF on the robot's computer fuses the rate with the wheel odometry. A mode without fusion, AMG or ACCGYRO, would give uncorrected values. Corrected doesn't mean bias-free: the bench checks measure the remaining bias. The calibration status, `CALIB_STAT`, comes with each sample, but doesn't hold samples back.

## Why a driver of its own

- The [Adafruit BNO055 library](https://github.com/adafruit/Adafruit_BNO055), used in earlier pull requests (#37 and #66), waits with `delay()` during its start-up, and after the sensor's reset it polls for the chip without a time limit. A sensor that doesn't answer again would stop the firmware, and with it the motor control. Its vector reads also don't report failed transfers: a failed read returns zeros, and zero rates look like a robot at rest.
- Bosch's [BNO055 Sensor API](https://github.com/boschsensortec/BNO055_SensorAPI) takes functions for the bus and for waiting, but its mode changes call the waiting function synchronously, and it keeps its device in one global pointer.
- This driver does only what the robot needs: it starts the sensor step by step without waiting, checks the sample transfers and the configuration's read-back, and leaves a sample unchanged when a read fails. The unit tests check each of these through [`i2c_bus.h`](i2c_bus.h). The cost is that the register sequence and its timings, from the datasheet, are maintained here.

## Bench checks on the robot

With the bringup running and the wheels off the ground:

1. **Rate and stamps:** `rostopic hz /diffbot/imu/data_raw` shows about 50 Hz, and the stamps in `rostopic echo` are current ROS time.
2. **Axes and signs:** at rest, the acceleration's upward axis reads about +9.8 m/s². Turning the robot counterclockwise by hand (seen from above) gives a positive angular velocity about the upward axis. That decides how `imu_link` has to sit in the URDF.
3. **Noise and bias:** a stationary recording of a few minutes (`rosbag record /diffbot/imu/data_raw`) gives the gyroscope's bias and variance at 50 Hz, to replace the covariances' starting values.
4. **Faults:** unplugging the IMU stops the topic within a sample, logs a warning, and the wheels keep responding to commands. Plugging it back in restarts it within `retry_ms`. The command timeout keeps working: after `COMMAND_TIMEOUT_MS` without a wheel command, it stops the motors.
5. **Timing:** the control loop keeps its rate with the IMU running (for example `rostopic hz /diffbot/measured_joint_states`).
