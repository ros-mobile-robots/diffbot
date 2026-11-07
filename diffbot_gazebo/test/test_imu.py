#!/usr/bin/env python3
"""Check the simulated IMU's topic /diffbot/imu/data_raw, as the EKF and the firmware expect it.

The checks: its rate, its frame, the orientation marked unavailable, usable variances,
gravity on the upward axis at rest, and a positive yaw rate when the robot turns left
(counterclockwise seen from above, REP 103). rostest starts the simulation of
sim_drive_<model>.test again for each of its tests, so this one waits for the drive controller
itself before it drives.
"""

import math
import time
from typing import List
import unittest

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest
from sensor_msgs.msg import Imu

TOPIC = '/diffbot/imu/data_raw'
COMMAND = '/diffbot/mobile_base_controller/cmd_vel'
ODOMETRY = '/diffbot/mobile_base_controller/odom'
FRAME = 'imu_link'
QUEUE_SIZE = 100  # messages, 2 s at RATE: the test collects them all
COMMAND_QUEUE_SIZE = 1  # only the latest command matters

RATE = 50.0  # Hz, the firmware's rate
RATE_TOLERANCE = 0.9  # the share of RATE that the topic must reach at least
STARTUP_TIMEOUT = 60.0  # s of wall time, for Gazebo and the controller spawner
POLL_INTERVAL = 0.1  # s of wall time between checks for the drive controller's subscription
# The drive controller stops the robot when no command arrives for 0.5 s (diff_drive_controller's
# default cmd_vel_timeout), so the test sends its command again at this rate, as test_drive.py does
COMMAND_RATE = 20  # Hz
SETTLE = 1.0  # s of simulation time at rest before the test measures
SPEED_UP = 1.0  # s of simulation time for the robot to reach the turn rate
MEASURE = 2.0  # s of simulation time that each test collects the IMU's messages for

# REP 145: orientation_covariance[0] = -1 marks the orientation as unavailable
ORIENTATION_UNAVAILABLE = -1.0
# The index of the z-z element in the row-major 3x3 angular velocity covariance, which holds
# the yaw rate's variance
YAW_RATE_VARIANCE_INDEX = 8
GRAVITY = 9.81  # m/s², what the upward axis reads at rest
GRAVITY_TOLERANCE = 0.5  # m/s²
TURN_RATE = 0.5  # rad/s, the commanded left turn
TURN_RATE_TOLERANCE = 0.15  # rad/s


class TestImu(unittest.TestCase):

    def setUp(self) -> None:
        self.messages: List[Imu] = []
        rospy.Subscriber(TOPIC, Imu, self.messages.append, queue_size=QUEUE_SIZE)
        self.command = rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)
        rospy.wait_for_message(TOPIC, Imu, timeout=STARTUP_TIMEOUT)
        # Gazebo's IMU publishes before the controller spawner has started the drive controller:
        # wait until the controller publishes its odometry and listens to the commands
        rospy.wait_for_message(ODOMETRY, Odometry, timeout=STARTUP_TIMEOUT)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while self.command.get_num_connections() == 0:
            self.assertLess(time.monotonic(), deadline,
                            "the drive controller doesn't subscribe to " + COMMAND)
            time.sleep(POLL_INTERVAL)

    def collect(self, seconds: float) -> List[Imu]:
        """Return the messages of the next seconds of simulation time."""
        self.messages.clear()
        rospy.sleep(seconds)
        return list(self.messages)

    def test_imu_publishes_the_firmware_message_at_rest(self) -> None:
        self.command.publish(Twist())
        rospy.sleep(SETTLE)
        messages = self.collect(MEASURE)
        rate = len(messages) / MEASURE
        self.assertGreater(rate, RATE_TOLERANCE * RATE,
                           '{} published at {:.1f} Hz, expected {:.0f} Hz'.format(
                               TOPIC, rate, RATE))
        stamps = [m.header.stamp for m in messages]
        for earlier, later in zip(stamps, stamps[1:]):
            self.assertGreater(later, earlier, 'each stamp should be later than the one before')
        for message in messages:
            self.assertEqual(message.header.frame_id, FRAME)
            self.assertEqual(message.orientation_covariance[0], ORIENTATION_UNAVAILABLE,
                             'the orientation should be marked unavailable')
            for value in (message.angular_velocity.z, message.linear_acceleration.z):
                self.assertTrue(math.isfinite(value))
            # The EKF needs a positive, finite variance for the yaw rate it fuses
            variance = message.angular_velocity_covariance[YAW_RATE_VARIANCE_INDEX]
            self.assertTrue(math.isfinite(variance) and variance > 0.0,
                            "the yaw rate's variance should be positive, is {}".format(variance))
        gravity = sum(m.linear_acceleration.z for m in messages) / len(messages)
        self.assertAlmostEqual(gravity, GRAVITY, delta=GRAVITY_TOLERANCE,
                               msg='at rest, the upward axis should read gravity, '
                                   'read {:.2f} m/s²'.format(gravity))

    def test_imu_measures_a_positive_yaw_rate_in_a_left_turn(self) -> None:
        turn = Twist()
        turn.angular.z = TURN_RATE
        rate = rospy.Rate(COMMAND_RATE)
        end = rospy.Time.now() + rospy.Duration(SPEED_UP)
        while rospy.Time.now() < end:  # let the robot speed up
            self.command.publish(turn)
            rate.sleep()
        self.messages.clear()
        end = rospy.Time.now() + rospy.Duration(MEASURE)
        while rospy.Time.now() < end:
            self.command.publish(turn)
            rate.sleep()
        messages = list(self.messages)
        self.command.publish(Twist())
        yaw_rate = sum(m.angular_velocity.z for m in messages) / len(messages)
        self.assertAlmostEqual(yaw_rate, TURN_RATE, delta=TURN_RATE_TOLERANCE,
                               msg='turning left at {} rad/s, the IMU measured {:.3f} rad/s '
                                   'about z'.format(TURN_RATE, yaw_rate))


if __name__ == '__main__':
    rospy.init_node('test_imu')
    rostest.rosrun('diffbot_gazebo', 'test_imu', TestImu)
