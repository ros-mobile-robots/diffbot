#!/usr/bin/env python3
"""Turn the simulated IMU into what the real one publishes.

Gazebo's IMU plugin publishes sensor_msgs/Imu on imu/gazebo, with an orientation from the
simulation. The real robot's firmware (diffbot_firmware/lib/imu) publishes
imu/data_raw: the angular velocity and the acceleration, with the orientation marked unavailable
(orientation_covariance[0] = -1, REP 145), because the EKF fuses only the angular velocity. This
node copies each message from imu/gazebo to imu/data_raw with the orientation removed, and keeps
its stamp, frame, values and covariances, so the simulation and the robot give the EKF the same
input.
"""

import rospy
from sensor_msgs.msg import Imu

INPUT_TOPIC = 'imu/gazebo'
OUTPUT_TOPIC = 'imu/data_raw'
QUEUE_SIZE = 10  # messages, 0.2 s at the IMU's 50 Hz
# REP 145: the first element -1 marks the orientation as unavailable, the others are unused
ORIENTATION_UNAVAILABLE = [-1.0] + [0.0] * 8


class ImuRawAdapter:

    def __init__(self) -> None:
        self.publisher = rospy.Publisher(OUTPUT_TOPIC, Imu, queue_size=QUEUE_SIZE)
        rospy.Subscriber(INPUT_TOPIC, Imu, self.callback, queue_size=QUEUE_SIZE)

    def callback(self, message: Imu) -> None:
        """Publish the message without its orientation."""
        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = 0.0
        message.orientation.w = 0.0
        message.orientation_covariance = list(ORIENTATION_UNAVAILABLE)
        self.publisher.publish(message)


if __name__ == '__main__':
    rospy.init_node('imu_raw_adapter')
    ImuRawAdapter()
    rospy.spin()
