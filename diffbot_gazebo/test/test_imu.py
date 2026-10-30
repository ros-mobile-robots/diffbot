#!/usr/bin/env python3
"""Checks the simulated IMU's topic /diffbot/imu/data_raw, as the EKF and the real robot's
firmware expect it: its rate, its frame, the orientation marked unavailable, usable variances,
gravity on the upward axis at rest, and a positive yaw rate when the robot turns left
(counterclockwise seen from above, REP 103). rostest starts the simulation of
sim_drive_<model>.test again for each of its tests, so this one waits for the drive controller
itself before it drives.
"""

import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

TOPIC = "/diffbot/imu/data_raw"
COMMAND = "/diffbot/mobile_base_controller/cmd_vel"
ODOMETRY = "/diffbot/mobile_base_controller/odom"
RATE = 50.0  # Hz, the firmware's rate
STARTUP_TIMEOUT = 60.0  # s of wall time, for Gazebo and the controller spawner


class TestImu(unittest.TestCase):

    def setUp(self):
        self.messages = []
        rospy.Subscriber(TOPIC, Imu, self.messages.append, queue_size=100)
        self.command = rospy.Publisher(COMMAND, Twist, queue_size=1)
        rospy.wait_for_message(TOPIC, Imu, timeout=STARTUP_TIMEOUT)
        # Gazebo's IMU publishes before the controller spawner has started the drive controller:
        # wait until the controller publishes its odometry and listens to the commands
        rospy.wait_for_message(ODOMETRY, Odometry, timeout=STARTUP_TIMEOUT)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while self.command.get_num_connections() == 0:
            self.assertLess(time.monotonic(), deadline, "the drive controller doesn't subscribe to " + COMMAND)
            time.sleep(0.1)

    def collect(self, seconds):
        """The messages of the next seconds of simulation time."""
        self.messages.clear()
        rospy.sleep(seconds)
        return list(self.messages)

    def test_contract_at_rest(self):
        self.command.publish(Twist())
        rospy.sleep(1.0)
        messages = self.collect(2.0)
        rate = len(messages) / 2.0
        self.assertGreater(rate, 0.9 * RATE, "{} published at {:.1f} Hz, expected {:.0f} Hz".format(TOPIC, rate, RATE))
        stamps = [m.header.stamp for m in messages]
        for earlier, later in zip(stamps, stamps[1:]):
            self.assertGreater(later, earlier, "each stamp should be later than the one before")
        for message in messages:
            self.assertEqual(message.header.frame_id, "imu_link")
            self.assertEqual(message.orientation_covariance[0], -1.0, "the orientation should be marked unavailable")
            for value in (message.angular_velocity.z, message.linear_acceleration.z):
                self.assertTrue(math.isfinite(value))
            # The EKF needs a positive, finite variance for the yaw rate it fuses
            variance = message.angular_velocity_covariance[8]
            self.assertTrue(math.isfinite(variance) and variance > 0.0,
                            "the yaw rate's variance should be positive, is {}".format(variance))
        gravity = sum(m.linear_acceleration.z for m in messages) / len(messages)
        self.assertAlmostEqual(gravity, 9.81, delta=0.5, msg="at rest, the upward axis should read gravity, read {:.2f} m/s²".format(gravity))

    def test_left_turn_is_a_positive_yaw_rate(self):
        turn = Twist()
        turn.angular.z = 0.5
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < end:  # let the robot speed up
            self.command.publish(turn)
            rate.sleep()
        self.messages.clear()
        end = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < end:
            self.command.publish(turn)
            rate.sleep()
        messages = list(self.messages)
        self.command.publish(Twist())
        yaw_rate = sum(m.angular_velocity.z for m in messages) / len(messages)
        self.assertAlmostEqual(yaw_rate, 0.5, delta=0.15,
                               msg="turning left at 0.5 rad/s, the IMU measured {:.3f} rad/s about z".format(yaw_rate))


if __name__ == "__main__":
    rospy.init_node("test_imu")
    rostest.rosrun("diffbot_gazebo", "test_imu", TestImu)
