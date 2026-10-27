#!/usr/bin/env python3
"""Turns the simulated IMU into what the real one publishes.

Gazebo's IMU plugin publishes sensor_msgs/Imu on imu/gazebo, with an orientation from the
simulation. The real robot's firmware (diffbot_base/scripts/base_controller/lib/imu) publishes
imu/data_raw: the angular velocity and the acceleration, with the orientation marked unavailable
(orientation_covariance[0] = -1, REP 145), because the EKF fuses only the angular velocity. This
node copies each message from imu/gazebo to imu/data_raw with the orientation removed, and keeps
its stamp, frame, values and covariances, so the simulation and the robot give the EKF the same
input.
"""

import rospy
from sensor_msgs.msg import Imu


class ImuRawAdapter:

    def __init__(self):
        self.publisher = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
        rospy.Subscriber("imu/gazebo", Imu, self.callback, queue_size=10)

    def callback(self, message):
        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = 0.0
        message.orientation.w = 0.0
        message.orientation_covariance = [-1.0] + [0.0] * 8
        self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("imu_raw_adapter")
    ImuRawAdapter()
    rospy.spin()
