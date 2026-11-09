#!/usr/bin/env python3
"""Test the IMU sample validator's checks (scripts/imu_validator.py) without a ROS master.

catkin runs them with nose (catkin_add_nosetests in CMakeLists.txt); ekf_synthetic.test checks the
validator together with the EKF. A rewind is the ROS clock going back, as when Gazebo's world is
reset.
"""

import math
import os
import sys
import unittest

import rospy
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
# After the path to the scripts, so not in the order of the imports above
import imu_validator  # noqa: E402, I100

NOW = rospy.Time(100, 0)  # the ROS time of most checks
FRAME = 'imu_link'
MAX_AGE = 0.1  # s, the node's default
MAX_FUTURE = 0.02  # s, the node's default
# Just beyond the limits: 1 µs more than MAX_AGE and MAX_FUTURE
JUST_TOO_OLD = 0.100001  # s
JUST_TOO_FAR_AHEAD = 0.020001  # s

# A sample as the firmware publishes it: no orientation (REP 145: orientation_covariance[0] = -1),
# a left turn, gravity on the upward axis, and the firmware's variances (diffbot_base_config.h)
ORIENTATION_UNAVAILABLE = -1.0
YAW_RATE = 0.5  # rad/s
GRAVITY = 9.81  # m/s²
ANGULAR_VELOCITY_VARIANCE = 4.0e-4  # (rad/s)²
LINEAR_ACCELERATION_VARIANCE = 1.0e-2  # (m/s²)²
VARIANCE_INDICES = (0, 4, 8)  # the diagonal of a 3x3 covariance stored row by row
# REP 145: angular_velocity_covariance[0] = -1 marks the angular velocity as unavailable
ANGULAR_VELOCITY_UNAVAILABLE = [-1.0] + [0.0] * 8
YAW_RATE_VARIANCE_INDEX = 8  # z-z
OFF_DIAGONAL_INDEX = 1  # x-y


def sample(stamp: rospy.Time = NOW, frame_id: str = FRAME, yaw_rate: float = YAW_RATE) -> Imu:
    """Return a valid sample, as the firmware publishes it."""
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.orientation_covariance[0] = ORIENTATION_UNAVAILABLE
    message.angular_velocity.z = yaw_rate
    message.linear_acceleration.z = GRAVITY
    for index in VARIANCE_INDICES:
        message.angular_velocity_covariance[index] = ANGULAR_VELOCITY_VARIANCE
        message.linear_acceleration_covariance[index] = LINEAR_ACCELERATION_VARIANCE
    return message


def seconds(value: float) -> rospy.Duration:
    """Return a duration of value seconds."""
    return rospy.Duration.from_sec(value)


class TestImuValidator(unittest.TestCase):

    def setUp(self) -> None:
        self.validator = imu_validator.ImuValidator(FRAME, max_age=MAX_AGE, max_future=MAX_FUTURE)

    def test_check_accepts_valid_samples(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        self.assertIsNone(self.validator.check(sample(NOW + seconds(MAX_FUTURE)),
                                               NOW + seconds(MAX_FUTURE)))

    def test_check_drops_a_sample_at_ros_time_zero(self) -> None:
        # With simulated time, before the first /clock message
        self.assertEqual(imu_validator.NO_CLOCK, self.validator.check(sample(), rospy.Time(0)))

    def test_check_drops_a_sample_in_another_frame_or_none(self) -> None:
        self.assertEqual(imu_validator.WRONG_FRAME,
                         self.validator.check(sample(frame_id='base_link'), NOW))
        self.assertEqual(imu_validator.WRONG_FRAME, self.validator.check(sample(frame_id=''), NOW))

    def test_check_drops_a_sample_with_a_nan_or_infinite_value(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            message = sample()
            message.angular_velocity.z = value
            self.assertEqual(imu_validator.NOT_FINITE, self.validator.check(message, NOW))
            message = sample()
            message.linear_acceleration.x = value
            self.assertEqual(imu_validator.NOT_FINITE, self.validator.check(message, NOW))

    def test_check_drops_a_sample_without_usable_angular_velocity_variances(self) -> None:
        unavailable = sample()
        unavailable.angular_velocity_covariance = list(ANGULAR_VELOCITY_UNAVAILABLE)
        zero = sample()
        zero.angular_velocity_covariance[YAW_RATE_VARIANCE_INDEX] = 0.0
        not_finite = sample()
        not_finite.angular_velocity_covariance[OFF_DIAGONAL_INDEX] = math.nan
        for message in (unavailable, zero, not_finite):
            self.assertEqual(imu_validator.NO_VARIANCE, self.validator.check(message, NOW))

    def test_check_accepts_a_sample_max_age_old_and_drops_an_older_one(self) -> None:
        self.assertIsNone(self.validator.check(sample(NOW - seconds(MAX_AGE)), NOW))
        self.assertEqual(imu_validator.TOO_OLD,
                         self.validator.check(sample(NOW - seconds(JUST_TOO_OLD)), NOW))

    def test_check_accepts_a_sample_max_future_ahead_and_drops_a_later_one(self) -> None:
        self.assertEqual(imu_validator.FROM_THE_FUTURE,
                         self.validator.check(sample(NOW + seconds(JUST_TOO_FAR_AHEAD)), NOW))
        self.assertIsNone(self.validator.check(sample(NOW + seconds(MAX_FUTURE)), NOW))

    def test_check_drops_a_sample_not_later_than_the_last_forwarded_one(self) -> None:
        self.assertIsNone(self.validator.check(sample(NOW), NOW))
        self.assertEqual(imu_validator.NOT_NEWER, self.validator.check(sample(NOW), NOW))
        self.assertEqual(imu_validator.NOT_NEWER,
                         self.validator.check(sample(NOW - seconds(0.01)), NOW))
        self.assertIsNone(self.validator.check(sample(NOW + rospy.Duration(0, 1)),
                                               NOW + seconds(0.001)))

    def test_check_compares_with_the_last_forwarded_sample_not_a_dropped_one(self) -> None:
        self.assertIsNone(self.validator.check(sample(NOW), NOW))
        self.assertEqual(imu_validator.FROM_THE_FUTURE,
                         self.validator.check(sample(NOW + seconds(5)), NOW))
        self.assertIsNone(self.validator.check(sample(NOW + seconds(MAX_FUTURE)),
                                               NOW + seconds(MAX_FUTURE)))

    def test_check_starts_the_order_over_after_a_rewind(self) -> None:
        # For example a reset of Gazebo's world: the clock and the stamps start again near 0
        self.assertIsNone(self.validator.check(sample(NOW), NOW))
        restart = rospy.Time(0, 20000000)
        self.assertIsNone(self.validator.check(sample(restart), restart))
        self.assertEqual(imu_validator.NOT_NEWER,
                         self.validator.check(sample(restart), restart + seconds(0.001)))

    def test_check_still_drops_old_samples_after_a_rewind(self) -> None:
        self.assertIsNone(self.validator.check(sample(NOW), NOW))
        restart = rospy.Time(50, 0)
        self.assertEqual(imu_validator.FROM_THE_FUTURE,
                         self.validator.check(sample(NOW + seconds(MAX_FUTURE)), restart))
        self.assertEqual(imu_validator.TOO_OLD,
                         self.validator.check(sample(restart - seconds(1)), restart))


class TestHealth(unittest.TestCase):
    """Test seconds_since_pass(), which the validator's diagnostics report."""

    def setUp(self) -> None:
        self.validator = imu_validator.ImuValidator(FRAME, max_age=MAX_AGE, max_future=MAX_FUTURE)

    def assert_seconds_since_pass(self, expected: float, now: rospy.Time) -> None:
        """Assert that seconds_since_pass(now) is about expected."""
        seconds_since_pass = self.validator.seconds_since_pass(now)
        if seconds_since_pass is None:
            self.fail('seconds_since_pass() is None, expected {}'.format(expected))
        self.assertAlmostEqual(expected, seconds_since_pass)

    def test_seconds_since_pass_is_none_before_the_first_sample(self) -> None:
        self.assertIsNone(self.validator.seconds_since_pass(NOW))

    def test_seconds_since_pass_is_the_time_since_the_last_forwarded_sample(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        self.assert_seconds_since_pass(1.5, NOW + seconds(1.5))

    def test_seconds_since_pass_ignores_dropped_samples(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        later = NOW + seconds(3)
        self.assertEqual(imu_validator.TOO_OLD,
                         self.validator.check(sample(NOW + seconds(1)), later))
        self.assert_seconds_since_pass(3.0, later)

    def test_seconds_since_pass_is_none_after_a_rewind_without_samples(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        self.assertIsNone(self.validator.seconds_since_pass(rospy.Time(50, 0)))
        self.assertIsNone(self.validator.seconds_since_pass(rospy.Time(55, 0)))

    def test_seconds_since_pass_is_none_after_a_short_rewind(self) -> None:
        # A rewind by less than the time since the last sample: still not healthy
        self.assertIsNone(self.validator.check(sample(), NOW))
        self.assert_seconds_since_pass(3.3, NOW + seconds(3.3))
        self.assertIsNone(self.validator.seconds_since_pass(NOW + seconds(0.2)))
        self.assertIsNone(self.validator.seconds_since_pass(NOW + seconds(1.0)))

    def test_seconds_since_pass_is_none_after_a_rewind_that_a_dropped_sample_shows(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        self.assert_seconds_since_pass(3.0, NOW + seconds(3))
        self.assertEqual(imu_validator.TOO_OLD,
                         self.validator.check(sample(NOW - seconds(1)), NOW + seconds(0.2)))
        self.assertIsNone(self.validator.seconds_since_pass(NOW + seconds(0.3)))

    def test_seconds_since_pass_restarts_with_a_sample_forwarded_after_a_rewind(self) -> None:
        self.assertIsNone(self.validator.check(sample(), NOW))
        restart = rospy.Time(50, 0)
        self.assertEqual(imu_validator.FROM_THE_FUTURE,
                         self.validator.check(sample(NOW + seconds(MAX_FUTURE)), restart))
        self.assertEqual(imu_validator.WRONG_FRAME,
                         self.validator.check(sample(restart, frame_id='base_link'), restart))
        self.assertIsNone(self.validator.seconds_since_pass(restart + seconds(1)))
        self.assertIsNone(self.validator.check(sample(restart + seconds(2)), restart + seconds(2)))
        self.assert_seconds_since_pass(0.5, restart + seconds(2.5))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('diffbot_control', 'test_imu_validator', TestImuValidator)
    rosunit.unitrun('diffbot_control', 'test_imu_validator_health', TestHealth)
