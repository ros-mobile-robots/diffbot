#!/usr/bin/env python3
"""Unit tests of the IMU gate's checks (scripts/imu_gate.py), without a ROS master. catkin runs
them with nose (catkin_add_nosetests in CMakeLists.txt); ekf_synthetic.test checks the gate together
with the EKF.
"""

import math
import os
import sys
import unittest

import rospy
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import imu_gate  # noqa: E402 (after the path to the scripts)

NOW = rospy.Time(100, 0)


def sample(stamp=NOW, frame_id="imu_link", yaw_rate=0.5):
    """A valid sample, as the firmware publishes it."""
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.orientation_covariance[0] = -1.0
    message.angular_velocity.z = yaw_rate
    message.linear_acceleration.z = 9.81
    for i in (0, 4, 8):
        message.angular_velocity_covariance[i] = 4.0e-4
        message.linear_acceleration_covariance[i] = 1.0e-2
    return message


def seconds(value):
    return rospy.Duration.from_sec(value)


class TestImuGate(unittest.TestCase):

    def setUp(self):
        self.gate = imu_gate.ImuGate("imu_link", max_age=0.1, max_future=0.02)

    def test_a_valid_sample_passes(self):
        self.assertIsNone(self.gate.check(sample(), NOW))
        self.assertIsNone(self.gate.check(sample(NOW + seconds(0.02)), NOW + seconds(0.02)))

    def test_no_clock(self):
        # With simulated time, before the first /clock message
        self.assertEqual(imu_gate.NO_CLOCK, self.gate.check(sample(), rospy.Time(0)))

    def test_wrong_frame(self):
        self.assertEqual(imu_gate.WRONG_FRAME, self.gate.check(sample(frame_id="base_link"), NOW))
        self.assertEqual(imu_gate.WRONG_FRAME, self.gate.check(sample(frame_id=""), NOW))

    def test_values_must_be_finite(self):
        for value in (math.nan, math.inf, -math.inf):
            message = sample()
            message.angular_velocity.z = value
            self.assertEqual(imu_gate.NOT_FINITE, self.gate.check(message, NOW))
            message = sample()
            message.linear_acceleration.x = value
            self.assertEqual(imu_gate.NOT_FINITE, self.gate.check(message, NOW))

    def test_angular_velocity_variances_must_be_usable(self):
        unavailable = sample()
        unavailable.angular_velocity_covariance = [-1.0] + [0.0] * 8  # REP 145: no angular velocity
        zero = sample()
        zero.angular_velocity_covariance[8] = 0.0
        not_finite = sample()
        not_finite.angular_velocity_covariance[1] = math.nan
        for message in (unavailable, zero, not_finite):
            self.assertEqual(imu_gate.NO_VARIANCE, self.gate.check(message, NOW))

    def test_age_limits(self):
        self.assertIsNone(self.gate.check(sample(NOW - seconds(0.1)), NOW))
        self.assertEqual(imu_gate.TOO_OLD, self.gate.check(sample(NOW - seconds(0.100001)), NOW))

    def test_future_limits(self):
        self.assertEqual(imu_gate.FROM_THE_FUTURE, self.gate.check(sample(NOW + seconds(0.020001)), NOW))
        self.assertIsNone(self.gate.check(sample(NOW + seconds(0.02)), NOW))

    def test_each_sample_must_be_later_than_the_last(self):
        self.assertIsNone(self.gate.check(sample(NOW), NOW))
        self.assertEqual(imu_gate.NOT_NEWER, self.gate.check(sample(NOW), NOW))
        self.assertEqual(imu_gate.NOT_NEWER, self.gate.check(sample(NOW - seconds(0.01)), NOW))
        self.assertIsNone(self.gate.check(sample(NOW + rospy.Duration(0, 1)), NOW + seconds(0.001)))

    def test_a_dropped_sample_doesnt_count_as_the_last(self):
        self.assertIsNone(self.gate.check(sample(NOW), NOW))
        self.assertEqual(imu_gate.FROM_THE_FUTURE, self.gate.check(sample(NOW + seconds(5)), NOW))
        self.assertIsNone(self.gate.check(sample(NOW + seconds(0.02)), NOW + seconds(0.02)))

    def test_the_order_starts_over_when_the_clock_goes_back(self):
        # For example a reset of Gazebo's world: the clock and the stamps start again near 0
        self.assertIsNone(self.gate.check(sample(NOW), NOW))
        restart = rospy.Time(0, 20000000)
        self.assertIsNone(self.gate.check(sample(restart), restart))
        self.assertEqual(imu_gate.NOT_NEWER, self.gate.check(sample(restart), restart + seconds(0.001)))

    def test_old_samples_still_dropped_after_the_clock_went_back(self):
        self.assertIsNone(self.gate.check(sample(NOW), NOW))
        restart = rospy.Time(50, 0)
        self.assertEqual(imu_gate.FROM_THE_FUTURE, self.gate.check(sample(NOW + seconds(0.02)), restart))
        self.assertEqual(imu_gate.TOO_OLD, self.gate.check(sample(restart - seconds(1)), restart))


class TestHealth(unittest.TestCase):
    """seconds_since_pass(), which the gate's diagnostics report."""

    def setUp(self):
        self.gate = imu_gate.ImuGate("imu_link", max_age=0.1, max_future=0.02)

    def test_none_before_the_first_sample(self):
        self.assertIsNone(self.gate.seconds_since_pass(NOW))

    def test_time_since_the_last_passed_sample(self):
        self.assertIsNone(self.gate.check(sample(), NOW))
        self.assertAlmostEqual(1.5, self.gate.seconds_since_pass(NOW + seconds(1.5)))

    def test_dropped_samples_dont_count(self):
        self.assertIsNone(self.gate.check(sample(), NOW))
        later = NOW + seconds(3)
        self.assertEqual(imu_gate.TOO_OLD, self.gate.check(sample(NOW + seconds(1)), later))
        self.assertAlmostEqual(3.0, self.gate.seconds_since_pass(later))

    def test_clock_back_then_silence(self):
        self.assertIsNone(self.gate.check(sample(), NOW))
        self.assertIsNone(self.gate.seconds_since_pass(rospy.Time(50, 0)))
        self.assertIsNone(self.gate.seconds_since_pass(rospy.Time(55, 0)))

    def test_clock_back_then_only_dropped_samples(self):
        self.assertIsNone(self.gate.check(sample(), NOW))
        restart = rospy.Time(50, 0)
        self.assertEqual(imu_gate.FROM_THE_FUTURE, self.gate.check(sample(NOW + seconds(0.02)), restart))
        self.assertEqual(imu_gate.WRONG_FRAME, self.gate.check(sample(restart, frame_id="base_link"), restart))
        self.assertIsNone(self.gate.seconds_since_pass(restart + seconds(1)))
        self.assertIsNone(self.gate.check(sample(restart + seconds(2)), restart + seconds(2)))
        self.assertAlmostEqual(0.5, self.gate.seconds_since_pass(restart + seconds(2.5)))


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("diffbot_control", "test_imu_gate", TestImuGate)
    rosunit.unitrun("diffbot_control", "test_imu_gate_health", TestHealth)
