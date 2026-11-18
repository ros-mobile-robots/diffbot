#!/usr/bin/env python3
"""Test the IMU sample validator's checks (scripts/imu_validator.py) without a ROS master.

catkin runs them with nose (catkin_add_nosetests in CMakeLists.txt). ekf_synthetic.test checks the
validator together with the EKF. The ROS clock goes back, for example, when Gazebo's world is
reset.
"""

import math
import os
import sys
from typing import Optional
import unittest

from diagnostic_msgs.msg import DiagnosticStatus
import rospy
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
# After the path to the scripts, so not in the order of the imports above
import imu_validator  # noqa: E402, I100

NOW = rospy.Time(100, 0)  # the ROS time of most checks
FRAME = 'imu_link'
MAX_AGE = 0.1  # s
MAX_FUTURE = 0.02  # s
# Steps between two times. A comment where a test uses a step says which condition it meets
ONE_NANOSECOND = rospy.Duration(0, 1)  # the smallest step between two stamps
ONE_MICROSECOND = 1e-6  # s
ONE_MILLISECOND = 0.001  # s
ONE_SECOND = 1.0  # s, more than MAX_AGE
# Just beyond the limits: 1 µs more than MAX_AGE and MAX_FUTURE
JUST_TOO_OLD = MAX_AGE + ONE_MICROSECOND  # s
JUST_TOO_FAR_AHEAD = MAX_FUTURE + ONE_MICROSECOND  # s
# Within MAX_AGE, so that check 4 lets a stamp this old through
SHORTLY_BEFORE = MAX_AGE / 10  # s
# More than MAX_FUTURE, so that check 4 rejects a stamp this far ahead
FAR_AHEAD = 5.0  # s
# With simulated time, the ROS time before the first message on /clock: the validator has no
# clock
ROS_TIME_ZERO = rospy.Time(0)
# ROS times after the clock went back from NOW, neither of them 0: near 0 as after
# /gazebo/reset_simulation, and a time that leaves room for a stamp ONE_SECOND before it
RESTART_NEAR_ZERO = rospy.Time(0, 20000000)  # 0.02 s
RESTART = rospy.Time(50, 0)
# A time without valid samples, longer than ONE_SECOND plus MAX_AGE
SILENCE = 3.0  # s
# A time since the last valid sample, well within the silence limit, and a number of new drops
RECENT = 0.1  # s
NEW_DROP_COUNT = 3

# A sample like the firmware's: no orientation (REP 145: orientation_covariance[0] = -1), a left
# turn, gravity on the upward axis, and variances of the size of the firmware's
# (diffbot_base_config.h). The test doesn't read the firmware's values
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


def make_imu_sample(stamp: rospy.Time = NOW, frame_id: str = FRAME,
                    yaw_rate: float = YAW_RATE) -> Imu:
    """Return an IMU sample like the firmware's, with defaults valid at NOW for a new validator.

    Tests override stamp, frame_id or yaw_rate, or change a field of the returned message, to
    build valid and invalid samples.
    """
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


class ValidatorTestCase(unittest.TestCase):
    """A new validator for each test, and assertions about its decision on one sample.

    Each assertion calls drop_reason_or_record() once, as the node does for each sample. Only
    for an accepted sample, the validator stores the sample's stamp in its attribute
    last_valid_stamp and the ROS time of its arrival in last_valid_arrival_time.
    """

    def setUp(self) -> None:
        self.validator = imu_validator.ImuValidator(FRAME, max_age=MAX_AGE, max_future=MAX_FUTURE)

    def assert_accepted(self, message: Imu, now: rospy.Time, msg: Optional[str] = None) -> None:
        """Assert that the validator accepts message at ROS time now: no drop reason."""
        self.assertIsNone(self.validator.drop_reason_or_record(message, now), msg)

    def assert_rejected(self, reason: imu_validator.DropReason, message: Imu, now: rospy.Time,
                        msg: Optional[str] = None) -> None:
        """Assert that the validator rejects message at ROS time now, for reason."""
        self.assertEqual(reason, self.validator.drop_reason_or_record(message, now), msg)


class TestImuValidator(ValidatorTestCase):
    """Test which samples the validator accepts, and the reason for each one it rejects."""

    def test_validator_accepts_valid_samples(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        self.assert_accepted(make_imu_sample(NOW + seconds(MAX_FUTURE)), NOW + seconds(MAX_FUTURE))

    def test_validator_rejects_a_sample_at_ros_time_zero(self) -> None:
        self.assert_rejected(imu_validator.DropReason.NO_CLOCK, make_imu_sample(), ROS_TIME_ZERO)

    def test_validator_rejects_a_sample_with_a_wrong_or_empty_frame(self) -> None:
        for frame_id in ('base_link', ''):
            self.assert_rejected(imu_validator.DropReason.WRONG_FRAME,
                                 make_imu_sample(frame_id=frame_id), NOW,
                                 'frame "{}"'.format(frame_id))

    def test_validator_rejects_a_sample_with_a_nan_or_infinite_value(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            message = make_imu_sample()
            message.angular_velocity.z = value
            self.assert_rejected(imu_validator.DropReason.NOT_FINITE, message, NOW,
                                 'angular velocity z = {}'.format(value))
            message = make_imu_sample()
            message.linear_acceleration.x = value
            self.assert_rejected(imu_validator.DropReason.NOT_FINITE, message, NOW,
                                 'linear acceleration x = {}'.format(value))

    def test_validator_rejects_a_sample_with_an_invalid_angular_velocity_covariance(self) -> None:
        unavailable = make_imu_sample()
        unavailable.angular_velocity_covariance = list(ANGULAR_VELOCITY_UNAVAILABLE)
        zero_variance = make_imu_sample()
        zero_variance.angular_velocity_covariance[YAW_RATE_VARIANCE_INDEX] = 0.0
        nan_off_the_diagonal = make_imu_sample()
        nan_off_the_diagonal.angular_velocity_covariance[OFF_DIAGONAL_INDEX] = math.nan
        cases = (('marked unavailable', unavailable), ('a variance of zero', zero_variance),
                 ('NaN off the diagonal', nan_off_the_diagonal))
        for case, message in cases:
            self.assert_rejected(imu_validator.DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE,
                                 message, NOW, case)

    def test_validator_accepts_a_sample_max_age_old_and_rejects_an_older_one(self) -> None:
        self.assert_accepted(make_imu_sample(NOW - seconds(MAX_AGE)), NOW)
        self.assert_rejected(imu_validator.DropReason.TOO_OLD,
                             make_imu_sample(NOW - seconds(JUST_TOO_OLD)), NOW)

    def test_validator_accepts_a_sample_max_future_ahead_and_rejects_a_later_one(self) -> None:
        self.assert_rejected(imu_validator.DropReason.FROM_THE_FUTURE,
                             make_imu_sample(NOW + seconds(JUST_TOO_FAR_AHEAD)), NOW)
        self.assert_accepted(make_imu_sample(NOW + seconds(MAX_FUTURE)), NOW)

    def test_validator_rejects_a_sample_not_later_than_the_last_valid_one(self) -> None:
        self.assert_accepted(make_imu_sample(NOW), NOW)
        # The same stamp again
        self.assert_rejected(imu_validator.DropReason.NOT_NEWER, make_imu_sample(NOW), NOW)
        # An earlier stamp within MAX_AGE, so that check 5 drops it and not check 4
        self.assert_rejected(imu_validator.DropReason.NOT_NEWER,
                             make_imu_sample(NOW - seconds(SHORTLY_BEFORE)), NOW)
        # The smallest later stamp is enough. The sample arrives 1 ms after NOW, after its stamp:
        # an arrival before NOW would clear last_valid_stamp
        self.assert_accepted(make_imu_sample(NOW + ONE_NANOSECOND),
                             NOW + seconds(ONE_MILLISECOND))

    def test_validator_accepts_a_new_sample_after_rejecting_a_future_one(self) -> None:
        # The order check compares with last_valid_stamp, the stamp of the last accepted sample,
        # not with a rejected one's. The rejected stamp is later than the next valid one
        self.assert_accepted(make_imu_sample(NOW), NOW)
        self.assert_rejected(imu_validator.DropReason.FROM_THE_FUTURE,
                             make_imu_sample(NOW + seconds(FAR_AHEAD)), NOW)
        self.assert_accepted(make_imu_sample(NOW + seconds(MAX_FUTURE)), NOW + seconds(MAX_FUTURE))

    def test_validator_accepts_a_stamp_once_after_the_clock_goes_back(self) -> None:
        # For example after /gazebo/reset_simulation: the clock and the stamps start again near 0.
        # The order check starts over there, and works again from the first valid sample on
        self.assert_accepted(make_imu_sample(NOW), NOW)
        self.assert_accepted(make_imu_sample(RESTART_NEAR_ZERO), RESTART_NEAR_ZERO)
        # The same stamp again, 1 ms later
        self.assert_rejected(imu_validator.DropReason.NOT_NEWER,
                             make_imu_sample(RESTART_NEAR_ZERO),
                             RESTART_NEAR_ZERO + seconds(ONE_MILLISECOND))

    def test_validator_rejects_stamps_outside_time_limits_after_the_clock_goes_back(self) -> None:
        self.assert_accepted(make_imu_sample(NOW), NOW)
        # A stamp that was valid at NOW, ahead of RESTART by more than MAX_FUTURE
        self.assert_rejected(imu_validator.DropReason.FROM_THE_FUTURE,
                             make_imu_sample(NOW + seconds(MAX_FUTURE)), RESTART)
        # More than MAX_AGE before RESTART
        self.assert_rejected(imu_validator.DropReason.TOO_OLD,
                             make_imu_sample(RESTART - seconds(ONE_SECOND)), RESTART)


class TestSecondsSinceLastValidSample(ValidatorTestCase):
    """Test seconds_since_last_valid_sample(), the silence that the diagnostics report."""

    def assert_silence(self, expected: float, now: rospy.Time) -> None:
        """Assert that seconds_since_last_valid_sample(now) is about expected."""
        silence = self.validator.seconds_since_last_valid_sample(now)
        if silence is None:
            self.fail('seconds_since_last_valid_sample() is None, expected {}'.format(expected))
        self.assertAlmostEqual(expected, silence)

    def test_silence_is_none_before_the_first_valid_sample(self) -> None:
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(NOW))

    def test_silence_is_the_time_since_the_last_valid_sample(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        self.assert_silence(SILENCE, NOW + seconds(SILENCE))

    def test_silence_keeps_growing_while_samples_are_rejected(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        later = NOW + seconds(SILENCE)
        # Later than the accepted sample's stamp, so that check 5 lets it through, but more than
        # MAX_AGE before the ROS time later
        self.assert_rejected(imu_validator.DropReason.TOO_OLD,
                             make_imu_sample(NOW + seconds(ONE_SECOND)), later)
        self.assert_silence(SILENCE, later)

    def test_silence_is_none_after_the_clock_goes_back_without_samples(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(RESTART))
        # Still None while the clock goes on
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(
            RESTART + seconds(SILENCE)))

    def test_silence_is_none_after_the_clock_goes_back_less_than_the_silence(self) -> None:
        # The clock goes back by less than the time since the last valid sample, to a ROS time
        # at which that sample would be RECENT: still no health. Then the clock goes on
        self.assert_accepted(make_imu_sample(), NOW)
        self.assert_silence(SILENCE, NOW + seconds(SILENCE))
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(NOW + seconds(RECENT)))
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(
            NOW + seconds(ONE_SECOND)))

    def test_silence_is_none_when_a_rejected_sample_shows_the_clock_went_back(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        self.assert_silence(SILENCE, NOW + seconds(SILENCE))
        # The clock goes back to NOW + RECENT. The sample's stamp is more than MAX_AGE older
        self.assert_rejected(imu_validator.DropReason.TOO_OLD,
                             make_imu_sample(NOW - seconds(ONE_SECOND)), NOW + seconds(RECENT))
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(
            NOW + seconds(ONE_SECOND)))

    def test_silence_restarts_with_the_first_valid_sample_after_the_clock_goes_back(self) -> None:
        self.assert_accepted(make_imu_sample(), NOW)
        # Two samples that the validator rejects at RESTART: one with a stamp from before the
        # clock went back, one with a wrong frame
        self.assert_rejected(imu_validator.DropReason.FROM_THE_FUTURE,
                             make_imu_sample(NOW + seconds(MAX_FUTURE)), RESTART)
        self.assert_rejected(imu_validator.DropReason.WRONG_FRAME,
                             make_imu_sample(RESTART, frame_id='base_link'), RESTART)
        self.assertIsNone(self.validator.seconds_since_last_valid_sample(
            RESTART + seconds(ONE_SECOND)))
        # The first valid sample after the clock went back, later than the call before
        first_valid = RESTART + seconds(SILENCE)
        self.assert_accepted(make_imu_sample(first_valid), first_valid)
        self.assert_silence(RECENT, first_valid + seconds(RECENT))


class TestHealth(unittest.TestCase):
    """Test health(), the level and the message of the validator's diagnostics status."""

    def test_health_is_an_error_before_the_first_valid_sample(self) -> None:
        self.assertEqual(DiagnosticStatus.ERROR, imu_validator.health(None, 0).level)

    def test_health_is_ok_at_the_silence_limit_and_an_error_beyond_it(self) -> None:
        at_the_limit = imu_validator.health(imu_validator.SILENCE_LIMIT, 0)
        beyond_the_limit = imu_validator.health(imu_validator.SILENCE_LIMIT + ONE_MILLISECOND, 0)
        self.assertEqual(DiagnosticStatus.OK, at_the_limit.level, at_the_limit.message)
        self.assertEqual(DiagnosticStatus.ERROR, beyond_the_limit.level, beyond_the_limit.message)

    def test_health_is_a_warning_with_new_drops(self) -> None:
        with_new_drops = imu_validator.health(RECENT, NEW_DROP_COUNT)
        self.assertEqual(DiagnosticStatus.WARN, with_new_drops.level)
        self.assertIn(str(NEW_DROP_COUNT), with_new_drops.message)

    def test_health_is_ok_without_new_drops(self) -> None:
        self.assertEqual(DiagnosticStatus.OK, imu_validator.health(RECENT, 0).level)

    def test_health_is_an_error_for_silence_also_with_new_drops(self) -> None:
        never = imu_validator.health(None, NEW_DROP_COUNT)
        silent = imu_validator.health(imu_validator.SILENCE_LIMIT + ONE_MILLISECOND,
                                      NEW_DROP_COUNT)
        self.assertEqual(DiagnosticStatus.ERROR, never.level, never.message)
        self.assertEqual(DiagnosticStatus.ERROR, silent.level, silent.message)


class TestSampleCounts(unittest.TestCase):
    """Test SampleCounts, the numbers of a diagnostics report."""

    def setUp(self) -> None:
        self.counts = imu_validator.SampleCounts()

    def test_snapshot_has_the_forwarded_count_and_the_drops_by_reason(self) -> None:
        self.counts.count_forwarded()
        self.counts.count_forwarded()
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        self.counts.count_dropped(imu_validator.DropReason.WRONG_FRAME)
        snapshot = self.counts.take_snapshot(RECENT)
        self.assertEqual(2, snapshot.forwarded_count)
        self.assertEqual({imu_validator.DropReason.TOO_OLD: 2,
                          imu_validator.DropReason.WRONG_FRAME: 1}, snapshot.drops_by_reason)
        self.assertEqual(RECENT, snapshot.seconds_since_last_valid_sample)

    def test_snapshot_counts_a_dropped_sample_as_new_only_once(self) -> None:
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        self.assertEqual(1, self.counts.take_snapshot(RECENT).new_drop_count)
        self.assertEqual(0, self.counts.take_snapshot(RECENT).new_drop_count)
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        self.counts.count_dropped(imu_validator.DropReason.NOT_NEWER)
        self.assertEqual(2, self.counts.take_snapshot(RECENT).new_drop_count)

    def test_snapshot_counts_a_dropped_sample_as_new_once_also_in_an_error_report(self) -> None:
        # No sample was valid yet, so health() reports an error for the first snapshot
        self.counts.count_dropped(imu_validator.DropReason.WRONG_FRAME)
        first = self.counts.take_snapshot(None)
        first_health = imu_validator.health(first.seconds_since_last_valid_sample,
                                            first.new_drop_count)
        self.assertEqual(DiagnosticStatus.ERROR, first_health.level)
        self.assertEqual(1, first.new_drop_count)
        self.assertEqual(0, self.counts.take_snapshot(RECENT).new_drop_count)

    def test_snapshot_keeps_its_numbers_when_more_samples_arrive(self) -> None:
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        snapshot = self.counts.take_snapshot(RECENT)
        self.counts.count_forwarded()
        self.counts.count_dropped(imu_validator.DropReason.TOO_OLD)
        self.assertEqual(0, snapshot.forwarded_count)
        self.assertEqual({imu_validator.DropReason.TOO_OLD: 1}, snapshot.drops_by_reason)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('diffbot_control', 'test_imu_validator', TestImuValidator)
    rosunit.unitrun('diffbot_control', 'test_imu_validator_silence',
                    TestSecondsSinceLastValidSample)
    rosunit.unitrun('diffbot_control', 'test_imu_validator_health', TestHealth)
    rosunit.unitrun('diffbot_control', 'test_imu_validator_sample_counts', TestSampleCounts)
