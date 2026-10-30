# IMU: Bosch BNO055 on the Teensy

The firmware reads a BNO055 on the Teensy's I2C bus (`Wire`: SDA pin 18, SCL pin 19 on the Teensy 4.0 and 3.2, the same bus as the Adafruit motor FeatherWing) and publishes it over rosserial. `IMU_ENABLED`, `IMU_BNO055_ADDRESS` and the other settings are in [`diffbot_base_config.h`](../config/diffbot_base_config.h). Not tested on the robot yet: the bench checks below are still open.

## What it publishes

`/diffbot/imu/data_raw`, `sensor_msgs/Imu`, at 50 Hz:

| Field | Content |
|:--|:--|
| `header.frame_id` | `imu_link`, the sensor's own axes. The robot model has to place this frame as the board sits on the robot |
| `header.stamp` | ROS time before the read, published only while rosserial's clock is synchronized with the host: after the host's time reply for the current connection |
| `angular_velocity` | rad/s, the gyroscope (the BNO055 in IMUPLUS mode: accelerometer and gyroscope, no magnetometer) |
| `linear_acceleration` | m/s², the raw specific force, including gravity (about +9.8 m/s² on the upward axis at rest) |
| `orientation` | not provided: `orientation_covariance[0] = -1`, as REP 145 says for a missing field |
| covariances | diagonal starting values from the datasheet, not measured yet |

A sample is published only after a read that succeeded and took at most `IMU_READ_BUDGET_US`, and only while the sensor reports that its fusion runs (`SYS_STATUS` 5), so a stalled, starting or unplugged IMU stops the topic instead of repeating old values. If the sensor resets itself or reads fail ten times in a row, the firmware starts it again. A missing IMU is looked for every `IMU_RETRY_PERIOD` seconds, without blocking the motor control loop. The 50 Hz period counts on the Teensy's own clock (`millis()`), so the rate doesn't change when ROS time jumps, for example after a reconnect to a host whose clock is earlier.

| File | What it does |
|:--|:--|
| [`bno055.h`](bno055.h) | The driver: start-up, configuration and reads, through [`i2c_bus.h`](i2c_bus.h) |
| [`wire_bus.h`](wire_bus.h) | The I2C bus on Arduino's Wire library |
| [`synced_node_handle.h`](synced_node_handle.h) | rosserial's node handle, plus whether its clock is synchronized |
| [`imu_publisher.h`](imu_publisher.h) | The rate, the checks above and the message |

The unit tests run all four on the PC, with a fake sensor, Wire library, serial port and clock, and rosserial's own parser: `.docker/firmware/run.sh pio test -e native`, from the repository's root, runs [`test/test_bno055`](../../test/test_bno055), [`test_wire_bus`](../../test/test_wire_bus), [`test_synced_node_handle`](../../test/test_synced_node_handle) and [`test_imu_publisher`](../../test/test_imu_publisher). They check the logic, not the hardware: the bench checks below are still needed.

## Bench checks on the robot

With the bringup running and the wheels off the ground:

1. **Rate and stamps:** `rostopic hz /diffbot/imu/data_raw` shows about 50 Hz, and the stamps in `rostopic echo` are current ROS time.
2. **Axes and signs:** at rest, the acceleration's upward axis reads about +9.8 m/s². Turning the robot counterclockwise by hand (seen from above) gives a positive angular velocity about the upward axis. That decides how `imu_link` has to sit in the URDF.
3. **Noise and bias:** a stationary recording of a few minutes (`rosbag record /diffbot/imu/data_raw`) gives the gyroscope's bias and variance at 50 Hz, to replace the covariances' starting values.
4. **Faults:** unplugging the IMU stops the topic within a sample, logs a warning, and the wheels keep responding to commands. Plugging it back in restarts it within `IMU_RETRY_PERIOD` seconds. The command timeout keeps working: after 5 s without a wheel command, it sets the target speeds to zero, which doesn't guarantee yet that the motors stop (#146).
5. **Timing:** the control loop keeps its rate with the IMU running (for example `rostopic hz /diffbot/measured_joint_states`).
