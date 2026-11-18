#!/usr/bin/env python3
"""Check scripts/imu_noise_at_rest.py: its statistics, its checks, its report and its messages.

The tests of the statistics, the checks and the report use readings with known results. The
tests of the tool as a command write small bags into a temporary folder and run the script on
them. No test needs a ROS master. catkin runs them with nose (catkin_add_nosetests in
CMakeLists.txt).
"""

import gzip
import io
import math
import os
import shutil
import statistics
import struct
import subprocess
import sys
import tempfile
from typing import Any
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import TypeVar
import unittest

import rosbag
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# The tool is a script, not a Python package, so its folder has to be on the module search path
SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, SCRIPTS)
# After the path to the scripts, so not in the order of the imports above
from imu_noise_at_rest import axis_statistics  # noqa: E402, I100
from imu_noise_at_rest import AxisStatistics  # noqa: E402
from imu_noise_at_rest import check_finite_readings  # noqa: E402
from imu_noise_at_rest import check_message_count  # noqa: E402
from imu_noise_at_rest import check_named_frame  # noqa: E402
from imu_noise_at_rest import check_one_frame  # noqa: E402
from imu_noise_at_rest import check_reading_count  # noqa: E402
from imu_noise_at_rest import check_reading_size  # noqa: E402
from imu_noise_at_rest import check_samples  # noqa: E402
from imu_noise_at_rest import check_topic_type  # noqa: E402
from imu_noise_at_rest import DEFAULT_TOPIC  # noqa: E402
from imu_noise_at_rest import field_statistics  # noqa: E402
from imu_noise_at_rest import FieldStatistics  # noqa: E402
from imu_noise_at_rest import format_duration  # noqa: E402
from imu_noise_at_rest import format_report  # noqa: E402
from imu_noise_at_rest import format_table  # noqa: E402
from imu_noise_at_rest import IMU_PERIOD  # noqa: E402
from imu_noise_at_rest import imu_statistics  # noqa: E402
from imu_noise_at_rest import ImuSample  # noqa: E402
from imu_noise_at_rest import ImuStatistics  # noqa: E402
from imu_noise_at_rest import LARGEST_READING  # noqa: E402
from imu_noise_at_rest import LONG_INTERVAL  # noqa: E402
from imu_noise_at_rest import measured_vector  # noqa: E402
from imu_noise_at_rest import MessageError  # noqa: E402
from imu_noise_at_rest import NANOSECONDS_PER_MILLISECOND  # noqa: E402
from imu_noise_at_rest import NANOSECONDS_PER_SECOND  # noqa: E402
from imu_noise_at_rest import NOT_MEASURED  # noqa: E402
from imu_noise_at_rest import stamp_statistics  # noqa: E402
from imu_noise_at_rest import StampStatistics  # noqa: E402
from imu_noise_at_rest import to_sample  # noqa: E402
from imu_noise_at_rest import Vector  # noqa: E402
from imu_noise_at_rest import vector_statistics  # noqa: E402
from imu_noise_at_rest import VectorStatistics  # noqa: E402

TOOL = os.path.join(SCRIPTS, 'imu_noise_at_rest.py')

# The BNO055's output step of the angular velocity: 900 LSB per rad/s
GYROSCOPE_STEP = 1 / 900  # rad/s
# Three yaw rates one output step apart. Their mean is one step, their deviations from it are
# -1, 0 and +1 step, so their sample variance is 2 steps² / (3 - 1) = 1 step²
YAW_RATES_ONE_STEP_APART = (0.0, GYROSCOPE_STEP, 2 * GYROSCOPE_STEP)  # rad/s

FRAME = 'imu_link'
NO_ROTATION = Vector(0.0, 0.0, 0.0)  # rad/s
GRAVITY = 9.81  # m/s^2
LEVEL = Vector(0.0, 0.0, GRAVITY)  # m/s^2, gravity on the upward axis

MILLISECOND = NANOSECONDS_PER_MILLISECOND  # ns
# A header stamp of the size that a recording has: 2023-11-14, 22:13:20 UTC. As a float in
# seconds, a stamp of this size is only exact to about 240 ns
EPOCH = 1700000000 * NANOSECONDS_PER_SECOND  # ns
# Five stamps 20 ms apart, except for one interval of 35 ms
STAMPS_WITH_ONE_LONG_INTERVAL = [EPOCH + offset * MILLISECOND for offset in (0, 20, 40, 75, 95)]
# Five stamps of which the third repeats the second and the fourth is 10 ms earlier. The last
# is 80 ms after the first
STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK = [
    EPOCH + offset * MILLISECOND for offset in (0, 20, 20, 10, 80)]
# Three stamps one period of the firmware apart
REGULAR_STAMPS = [EPOCH + index * IMU_PERIOD for index in range(3)]
# The time that a message takes from its header stamp to the recorder, in the bags of the tests
# that set the recorder's clock back
DELIVERY_TIME = 2 * MILLISECOND  # ns


def sample(stamp: int = EPOCH, frame_id: str = FRAME,
           angular_velocity: Optional[Vector] = NO_ROTATION,
           linear_acceleration: Optional[Vector] = LEVEL) -> ImuSample:
    """Return a sample of a level robot at rest."""
    return ImuSample(stamp, frame_id, angular_velocity, linear_acceleration)


Value = TypeVar('Value')


def known(value: Optional[Value]) -> Value:
    """Return a value that a test expects to be known, and fail the test if it is None."""
    if value is None:
        raise AssertionError('expected a value, got None')
    return value


def message_of(error_context: Any) -> str:
    """Return the text of the exception that an assertRaises() block caught."""
    return str(error_context.exception)


class TestAxisStatistics(unittest.TestCase):
    """axis_statistics() gives the mean, the sample variance and the distinct values."""

    def test_axis_statistics_gives_the_mean(self) -> None:
        """Check that 1, 2, 3 and 6 have the mean 3."""
        self.assertAlmostEqual(axis_statistics([1.0, 2.0, 3.0, 6.0]).mean, 3.0)

    def test_axis_statistics_gives_the_sample_variance(self) -> None:
        """Check that the squared deviations are divided by the number of values minus one.

        The squared deviations of 1, 2, 3 and 4 from their mean 2.5 add up to 5. The sample
        variance is 5 / (4 - 1), not 5 / 4.
        """
        variance = axis_statistics([1.0, 2.0, 3.0, 4.0]).variance
        self.assertAlmostEqual(variance, 5.0 / 3.0)

    def test_axis_statistics_gives_the_standard_deviation_as_the_root_of_the_variance(
            self) -> None:
        """Check that the standard deviation of 1, 2, 3 and 4 is the root of 5 / 3."""
        axis = axis_statistics([1.0, 2.0, 3.0, 4.0])
        self.assertAlmostEqual(axis.standard_deviation, math.sqrt(5.0 / 3.0))

    def test_axis_statistics_counts_each_distinct_value_once(self) -> None:
        """Check that five readings on three output levels count as three distinct values."""
        values = [0.0, GYROSCOPE_STEP, GYROSCOPE_STEP, 0.0, -GYROSCOPE_STEP]
        self.assertEqual(axis_statistics(values).distinct_value_count, 3)

    def test_axis_statistics_gives_a_variance_of_zero_without_noise(self) -> None:
        """Check that five equal readings have a variance of exactly zero."""
        self.assertEqual(axis_statistics([GYROSCOPE_STEP] * 5).variance, 0.0)

    def test_axis_statistics_gives_a_variance_of_zero_for_deviations_too_small_to_square(
            self) -> None:
        """Check that a reading of 7e-304 next to two of zero gives a variance, not an error.

        The squares of such deviations are zero as floats. A changed byte in a number's exponent
        makes such a reading.
        """
        self.assertEqual(axis_statistics([7.0e-304, 0.0, 0.0]).variance, 0.0)

    def test_axis_statistics_fails_with_fewer_than_two_values(self) -> None:
        """Check that one value raises an error: a sample variance needs two."""
        with self.assertRaises(statistics.StatisticsError):
            axis_statistics([1.0])


class TestVectorStatistics(unittest.TestCase):
    """vector_statistics() gives each axis's statistics from that axis's values only."""

    def test_vector_statistics_keeps_the_axes_apart(self) -> None:
        """Check that each axis's mean comes from that axis: 2, 20 and 200."""
        axes = vector_statistics([Vector(1.0, 10.0, 100.0), Vector(3.0, 30.0, 300.0)])
        self.assertAlmostEqual(axes.x.mean, 2.0)
        self.assertAlmostEqual(axes.y.mean, 20.0)
        self.assertAlmostEqual(axes.z.mean, 200.0)


class TestFieldStatistics(unittest.TestCase):
    """field_statistics() counts a field's readings and needs two of them for the statistics."""

    def test_field_statistics_gives_the_reading_count_and_the_axes_statistics(self) -> None:
        """Check that two readings are counted and give each axis's statistics."""
        field = field_statistics([Vector(1.0, 10.0, 100.0), Vector(3.0, 30.0, 300.0)])
        self.assertEqual(field.reading_count, 2)
        self.assertEqual(field.axes, vector_statistics([Vector(1.0, 10.0, 100.0),
                                                        Vector(3.0, 30.0, 300.0)]))

    def test_field_statistics_gives_no_axes_statistics_for_one_reading(self) -> None:
        """Check that one reading is counted, without statistics instead of an error."""
        self.assertEqual(field_statistics([LEVEL]), FieldStatistics(reading_count=1, axes=None))


class TestMeasuredVector(unittest.TestCase):
    """measured_vector() gives a field's reading unless its covariance marks it not measured."""

    READING = Vector(0.1, 0.2, 0.3)

    def test_measured_vector_gives_none_when_the_covariance_starts_with_minus_one(self) -> None:
        """Check that sensor_msgs/Imu's mark for "no measurement" gives no reading."""
        covariance = [NOT_MEASURED] + [0.0] * 8
        self.assertIsNone(measured_vector(self.READING, covariance))

    def test_measured_vector_gives_the_reading_for_a_covariance_of_zeros(self) -> None:
        """Check that an unknown covariance, all zeros, still gives the reading."""
        self.assertEqual(measured_vector(self.READING, [0.0] * 9), self.READING)


class TestToSample(unittest.TestCase):
    """to_sample() takes the stamp, the frame and the measured fields of a sensor_msgs/Imu."""

    def test_to_sample_keeps_one_nanosecond_of_the_header_stamp(self) -> None:
        """Check that a stamp 1 ns after EPOCH stays 1 ns after it, as an integer."""
        message = Imu()
        message.header.stamp = rospy.Time(EPOCH // NANOSECONDS_PER_SECOND, 1)
        self.assertEqual(to_sample(message).stamp, EPOCH + 1)

    def test_to_sample_gives_each_field_by_its_own_covariance(self) -> None:
        """Check that a message without the angular velocity still gives its acceleration."""
        message = Imu()
        message.header.frame_id = FRAME
        message.angular_velocity_covariance[0] = NOT_MEASURED
        message.linear_acceleration.z = GRAVITY
        self.assertEqual(to_sample(message), ImuSample(0, FRAME, None, LEVEL))


class TestStampStatistics(unittest.TestCase):
    """stamp_statistics() gives the header stamps' duration, rate and intervals."""

    def test_stamp_statistics_gives_the_duration_and_the_mean_rate(self) -> None:
        """Check that five stamps over 95 ms have four intervals in 0.095 s."""
        stamps = stamp_statistics(STAMPS_WITH_ONE_LONG_INTERVAL)
        self.assertEqual(stamps.message_count, 5)
        self.assertEqual(stamps.duration, 95 * MILLISECOND)
        self.assertAlmostEqual(known(stamps.rate), 4 / 0.095)

    def test_stamp_statistics_gives_the_shortest_and_the_longest_interval(self) -> None:
        """Check that the intervals of 20, 20, 35 and 20 ms give 20 and 35 ms, to the ns."""
        stamps = stamp_statistics(STAMPS_WITH_ONE_LONG_INTERVAL)
        self.assertEqual(stamps.shortest_interval, 20 * MILLISECOND)
        self.assertEqual(stamps.longest_interval, 35 * MILLISECOND)

    def test_stamp_statistics_counts_no_long_interval_one_nanosecond_below_long_interval(
            self) -> None:
        """Check that an interval of LONG_INTERVAL minus 1 ns isn't long."""
        stamps = stamp_statistics([EPOCH, EPOCH + LONG_INTERVAL - 1])
        self.assertEqual(stamps.long_interval_count, 0)

    def test_stamp_statistics_counts_no_long_interval_at_exactly_long_interval(self) -> None:
        """Check that an interval of exactly LONG_INTERVAL isn't long: only a longer one is."""
        stamps = stamp_statistics([EPOCH, EPOCH + LONG_INTERVAL])
        self.assertEqual(stamps.long_interval_count, 0)

    def test_stamp_statistics_counts_a_long_interval_one_nanosecond_above_long_interval(
            self) -> None:
        """Check that an interval of LONG_INTERVAL plus 1 ns is long."""
        stamps = stamp_statistics([EPOCH, EPOCH + LONG_INTERVAL + 1])
        self.assertEqual(stamps.long_interval_count, 1)

    def test_stamp_statistics_counts_no_repeated_or_earlier_stamp_in_increasing_stamps(
            self) -> None:
        """Check that stamps that all advance count neither as repeated nor as earlier."""
        stamps = stamp_statistics(STAMPS_WITH_ONE_LONG_INTERVAL)
        self.assertEqual((stamps.repeated_stamp_count, stamps.earlier_stamp_count), (0, 0))

    def test_stamp_statistics_counts_a_repeated_and_an_earlier_stamp_apart(self) -> None:
        """Check that one repeat and one step back of 10 ms count once each."""
        stamps = stamp_statistics(STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK)
        self.assertEqual((stamps.repeated_stamp_count, stamps.earlier_stamp_count), (1, 1))
        self.assertEqual(stamps.shortest_interval, -10 * MILLISECOND)

    def test_stamp_statistics_gives_no_rate_when_a_stamp_inside_the_recording_goes_back(
            self) -> None:
        """Check that a step back gives no rate, although the last stamp is after the first.

        Four intervals in 80 ms would look like a healthy 50 Hz.
        """
        stamps = stamp_statistics(STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK)
        self.assertEqual(stamps.duration, 80 * MILLISECOND)
        self.assertIsNone(stamps.rate)

    def test_stamp_statistics_gives_no_rate_when_no_stamp_advances(self) -> None:
        """Check that three stamps of zero, an unset clock, give no rate and two repeats."""
        stamps = stamp_statistics([0, 0, 0])
        self.assertIsNone(stamps.rate)
        self.assertEqual(stamps.repeated_stamp_count, 2)


class BagWithConnections:
    """A stand-in for a bag that has only the connections of one topic, and no messages."""

    def __init__(self, connection_types: Sequence[str]) -> None:
        """Remember the message type of each of the topic's connections."""
        self.connection_types = connection_types

    def read_messages(self, topics: Sequence[str], connection_filter: Any) -> List[Any]:
        """Ask the filter about each connection, as rosbag does, and return no message."""
        for connection_type in self.connection_types:
            connection_filter(topics[0], connection_type, 'checksum', 'definition', {})
        return []


class TestTopicType(unittest.TestCase):
    """check_topic_type() refuses a topic with messages that aren't sensor_msgs/Imu."""

    def test_check_topic_type_passes_a_topic_of_imu_messages(self) -> None:
        """Check that a topic with one connection of sensor_msgs/Imu raises nothing."""
        check_topic_type(BagWithConnections(['sensor_msgs/Imu']), DEFAULT_TOPIC)

    def test_check_topic_type_fails_for_a_topic_with_several_types_and_names_them_all(
            self) -> None:
        """Check that an IMU topic with two more types is refused, with all three names.

        rosbag info shows such a topic with one type only, here sensor_msgs/Imu.
        """
        bag = BagWithConnections(['sensor_msgs/Imu', 'std_msgs/String', 'std_msgs/UInt32'])
        with self.assertRaises(MessageError) as error:
            check_topic_type(bag, DEFAULT_TOPIC)
        self.assertEqual(message_of(error),
                         'the topic holds sensor_msgs/Imu, std_msgs/String, std_msgs/UInt32. '
                         'Every message of the topic must be a sensor_msgs/Imu')


class TestSampleChecks(unittest.TestCase):
    """The checks raise a MessageError for samples that the statistics can't use."""

    def test_check_message_count_fails_for_one_message(self) -> None:
        """Check that one sample is refused: it has no interval to another."""
        with self.assertRaises(MessageError) as error:
            check_message_count([sample()])
        self.assertEqual(message_of(error), '1 message, the statistics need at least two')

    def test_check_named_frame_fails_for_messages_without_a_frame_id(self) -> None:
        """Check that samples with an empty frame_id are refused: their axes aren't known."""
        with self.assertRaises(MessageError) as error:
            check_named_frame([sample(frame_id=''), sample(frame_id='')])
        self.assertEqual(message_of(error),
                         '2 of 2 messages have an empty frame_id. The readings need one named '
                         'frame, which says what their axes are')

    def test_check_one_frame_fails_for_two_frames_and_names_them(self) -> None:
        """Check that samples in imu_link and in other_imu are refused, with both names."""
        with self.assertRaises(MessageError) as error:
            check_one_frame([sample(frame_id='other_imu'), sample(frame_id=FRAME)])
        self.assertEqual(message_of(error),
                         'the messages have more than one frame_id ("imu_link", "other_imu"), '
                         'so their readings have different axes')

    def test_check_finite_readings_fails_for_a_nan_and_names_the_field_and_the_stamp(
            self) -> None:
        """Check that a NaN yaw rate is refused, with its field and its message's stamp."""
        not_a_number = sample(stamp=EPOCH + IMU_PERIOD,
                              angular_velocity=Vector(0.0, 0.0, math.nan))
        with self.assertRaises(MessageError) as error:
            check_finite_readings([sample(), not_a_number])
        self.assertEqual(message_of(error),
                         'the message stamped 1700000000.020000000 has the angular_velocity '
                         "(0.0, 0.0, nan), which isn't finite")

    def test_check_finite_readings_fails_for_an_infinite_acceleration(self) -> None:
        """Check that an infinite acceleration is refused as well."""
        infinite = sample(linear_acceleration=Vector(math.inf, 0.0, GRAVITY))
        with self.assertRaises(MessageError):
            check_finite_readings([sample(), infinite])

    def test_check_reading_size_fails_for_a_reading_beyond_the_largest_reading(self) -> None:
        """Check that an acceleration of twice LARGEST_READING is refused, with its stamp."""
        too_large = sample(linear_acceleration=Vector(0.0, 0.0, 2 * LARGEST_READING))
        with self.assertRaises(MessageError) as error:
            check_reading_size([sample(), too_large])
        self.assertEqual(message_of(error),
                         'the message stamped 1700000000.000000000 has the linear_acceleration '
                         "(0.0, 0.0, 2000000.0). A value beyond 1e+06 isn't an IMU's reading")

    def test_check_reading_size_passes_a_reading_of_the_largest_reading(self) -> None:
        """Check that a reading of exactly LARGEST_READING, on the negative side, passes."""
        check_reading_size([sample(), sample(angular_velocity=Vector(-LARGEST_READING, 0, 0))])

    def test_check_finite_readings_passes_a_sample_without_a_field(self) -> None:
        """Check that a field that a message didn't measure isn't looked at."""
        check_finite_readings([sample(angular_velocity=None), sample()])

    def test_check_reading_count_fails_when_no_field_has_two_readings(self) -> None:
        """Check that two samples with one reading of each field are refused, with the counts."""
        samples = [sample(angular_velocity=None), sample(linear_acceleration=None)]
        with self.assertRaises(MessageError) as error:
            check_reading_count(samples)
        self.assertEqual(message_of(error),
                         'angular_velocity has 1 reading and linear_acceleration has 1 reading. '
                         'At least one of them needs two')

    def test_check_samples_passes_two_samples_at_rest_in_one_frame(self) -> None:
        """Check that two finite samples in one frame raise nothing."""
        check_samples([sample(), sample(stamp=EPOCH + IMU_PERIOD)])


class TestImuStatistics(unittest.TestCase):
    """imu_statistics() takes each statistic from the right field of the samples."""

    def test_imu_statistics_gives_the_stamps_statistics_and_the_frame(self) -> None:
        """Check that two samples one period apart give that duration and their frame."""
        imu = imu_statistics([sample(), sample(stamp=EPOCH + IMU_PERIOD)])
        self.assertEqual(imu.stamps.duration, IMU_PERIOD)
        self.assertEqual(imu.frame_id, FRAME)

    def test_imu_statistics_keeps_the_angular_velocity_and_the_acceleration_apart(
            self) -> None:
        """Check that the yaw rates' mean is 0.4 rad/s and the z accelerations' 10 m/s²."""
        samples = [ImuSample(EPOCH, FRAME, Vector(0.1, 0.2, 0.3), Vector(1.0, 2.0, 9.0)),
                   ImuSample(EPOCH + IMU_PERIOD, FRAME, Vector(0.3, 0.4, 0.5),
                             Vector(3.0, 4.0, 11.0))]
        imu = imu_statistics(samples)
        self.assertAlmostEqual(known(imu.angular_velocity.axes).z.mean, 0.4)
        self.assertAlmostEqual(known(imu.linear_acceleration.axes).z.mean, 10.0)

    def test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration(
            self) -> None:
        """Check that a tilted IMU's mean acceleration (2, 3, 6) has the length 7."""
        samples = [sample(linear_acceleration=Vector(1.0, 3.0, 5.0)),
                   sample(stamp=EPOCH + IMU_PERIOD, linear_acceleration=Vector(3.0, 3.0, 7.0))]
        self.assertAlmostEqual(known(imu_statistics(samples).gravity), 7.0)

    def test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it(self) -> None:
        """Check that a sample without the angular velocity doesn't count as a reading of zero.

        Three samples, the middle one without the angular velocity. The yaw rates 0.2 and
        0.4 rad/s give two readings with the mean 0.3 rad/s, not three with the mean 0.2 rad/s.
        """
        samples = [sample(angular_velocity=Vector(0.0, 0.0, 0.2)),
                   sample(stamp=EPOCH + IMU_PERIOD, angular_velocity=None),
                   sample(stamp=EPOCH + 2 * IMU_PERIOD, angular_velocity=Vector(0.0, 0.0, 0.4))]
        imu = imu_statistics(samples)
        self.assertEqual(imu.angular_velocity.reading_count, 2)
        self.assertAlmostEqual(known(imu.angular_velocity.axes).z.mean, 0.3)
        self.assertEqual(imu.linear_acceleration.reading_count, 3)

    def test_imu_statistics_gives_no_gravity_without_the_acceleration(self) -> None:
        """Check that samples without the acceleration give no gravity, not a length of zero."""
        samples = [sample(linear_acceleration=None),
                   sample(stamp=EPOCH + IMU_PERIOD, linear_acceleration=None)]
        self.assertIsNone(imu_statistics(samples).gravity)


class TestFormatDuration(unittest.TestCase):
    """format_duration() gives seconds from one second on and milliseconds below."""

    def test_format_duration_gives_seconds_with_one_decimal_from_one_second_on(self) -> None:
        """Check that 179.78 s reads "179.8 s"."""
        self.assertEqual(format_duration(179780 * MILLISECOND), '179.8 s')

    def test_format_duration_gives_milliseconds_below_one_second(self) -> None:
        """Check that 40 ms reads "40.0 ms", not "0.0 s"."""
        self.assertEqual(format_duration(40 * MILLISECOND), '40.0 ms')


class TestFormatTable(unittest.TestCase):
    """format_table() aligns the columns."""

    def test_format_table_aligns_the_first_column_left_and_the_others_right(self) -> None:
        """Check that each column is as wide as its widest cell."""
        lines = format_table([['axis', 'mean'], ['x', '+0.5'], ['yy', '-12.25']])
        self.assertEqual(lines, ['axis    mean', 'x       +0.5', 'yy    -12.25'])


class TestFormatReport(unittest.TestCase):
    """format_report() gives the report that the tool prints, line by line."""

    # The same statistics on all three axes of a field, so the tests show the layout
    ROTATION_AXIS = AxisStatistics(mean=0.002, variance=1e-06, distinct_value_count=3)
    ACCELERATION_AXIS = AxisStatistics(mean=-1.0, variance=0.25, distinct_value_count=12)
    ROTATION = FieldStatistics(
        501, VectorStatistics(ROTATION_AXIS, ROTATION_AXIS, ROTATION_AXIS))
    ACCELERATION = FieldStatistics(
        501, VectorStatistics(ACCELERATION_AXIS, ACCELERATION_AXIS, ACCELERATION_AXIS))
    STAMPS = StampStatistics(
        message_count=501, duration=10 * NANOSECONDS_PER_SECOND, rate=50.0,
        shortest_interval=19 * MILLISECOND, longest_interval=31 * MILLISECOND,
        long_interval_count=2, repeated_stamp_count=0, earlier_stamp_count=0)

    def test_format_report_gives_the_stamps_a_table_for_each_vector_field_and_gravity(
            self) -> None:
        """Check every line of a report with both fields' statistics."""
        imu = ImuStatistics(stamps=self.STAMPS, frame_id=FRAME, angular_velocity=self.ROTATION,
                            linear_acceleration=self.ACCELERATION, gravity=9.81)
        self.assertEqual(format_report('/imu', imu), [
            '/imu: 501 messages, frame_id "imu_link"',
            'Header stamps: 10.0 s from the first to the last, 50.00 Hz',
            'Header intervals: shortest 19.0 ms, longest 31.0 ms, 2 longer than 30 ms',
            "Stamps that don't advance: 0 repeated, 0 earlier than the one before",
            '',
            'angular_velocity  mean (bias)     variance  std dev  distinct',
            '501 readings          [rad/s]  [(rad/s)^2]  [rad/s]    values',
            'x                    +0.00200    1.000e-06  0.00100         3',
            'y                    +0.00200    1.000e-06  0.00100         3',
            'z                    +0.00200    1.000e-06  0.00100         3',
            '',
            'linear_acceleration  mean (gravity + bias)     variance  std dev  distinct',
            '501 readings                       [m/s^2]  [(m/s^2)^2]  [m/s^2]    values',
            'x                                 -1.00000    2.500e-01  0.50000        12',
            'y                                 -1.00000    2.500e-01  0.50000        12',
            'z                                 -1.00000    2.500e-01  0.50000        12',
            '',
            'Gravity, the length of the mean acceleration: 9.810 m/s^2',
        ])

    def test_format_report_says_no_rate_and_counts_the_stamps_that_do_not_advance(
            self) -> None:
        """Check the stamps' lines for a recording with a repeated and an earlier stamp."""
        stamps = StampStatistics(
            message_count=5, duration=80 * MILLISECOND, rate=None,
            shortest_interval=-10 * MILLISECOND, longest_interval=70 * MILLISECOND,
            long_interval_count=1, repeated_stamp_count=1, earlier_stamp_count=1)
        imu = ImuStatistics(stamps=stamps, frame_id=FRAME, angular_velocity=self.ROTATION,
                            linear_acceleration=self.ACCELERATION, gravity=9.81)
        self.assertEqual(format_report('/imu', imu)[1:4], [
            "Header stamps: 80.0 ms from the first to the last, no rate (some stamps don't "
            'advance)',
            'Header intervals: shortest -10.0 ms, longest 70.0 ms, 1 longer than 30 ms',
            "Stamps that don't advance: 1 repeated, 1 earlier than the one before",
        ])

    def test_format_report_gives_the_reason_instead_of_a_table_for_a_field_without_statistics(
            self) -> None:
        """Check a report without the acceleration: two lines for it, and no gravity."""
        imu = ImuStatistics(stamps=self.STAMPS, frame_id=FRAME, angular_velocity=self.ROTATION,
                            linear_acceleration=FieldStatistics(reading_count=1, axes=None),
                            gravity=None)
        self.assertEqual(format_report('/imu', imu)[10:], [
            '',
            'linear_acceleration: 1 reading, the statistics need at least two',
            '  (a message without a reading has -1 as linear_acceleration_covariance[0])',
        ])


def to_ros_time(stamp: int) -> rospy.Time:
    """Return a stamp in ns as a ROS time."""
    seconds, nanoseconds = divmod(stamp, NANOSECONDS_PER_SECOND)
    return rospy.Time(seconds, nanoseconds)


def imu_message(stamp: int, yaw_rate: float = 0.0) -> Imu:
    """Return a message of a level robot at rest, without an orientation like the firmware's.

    The covariances of the angular velocity and the acceleration are zeros: not known.
    """
    message = Imu()
    message.header.stamp = to_ros_time(stamp)
    message.header.frame_id = FRAME
    message.orientation_covariance[0] = NOT_MEASURED
    message.angular_velocity.z = yaw_rate
    message.linear_acceleration.z = GRAVITY
    return message


def messages_one_step_apart() -> List[Imu]:
    """Return three messages one period apart, with the yaw rates one output step apart."""
    return [imu_message(stamp, yaw_rate)
            for stamp, yaw_rate in zip(REGULAR_STAMPS, YAW_RATES_ONE_STEP_APART)]


def write_bag(bag_path: str, topic: str, messages: Sequence[Any],
              arrival_stamps: Sequence[int] = REGULAR_STAMPS,
              compression: str = rosbag.Compression.NONE) -> None:
    """Write the messages to a new bag in their order, as a recorder stores them.

    Each message gets the time at which the recorder got it, by the recorder's clock.
    """
    with rosbag.Bag(bag_path, 'w', compression=compression) as bag:
        for message, arrival_stamp in zip(messages, arrival_stamps):
            bag.write(topic, message, to_ros_time(arrival_stamp))


def write_bag_with_a_short_last_message(bag_path: str) -> None:
    """Write three messages one step apart, of which the last lacks the last byte of its data.

    The recorder's raw interface takes a message as its type, its bytes, the type's checksum
    and its class, and stores the bytes as they are.
    """
    messages = messages_one_step_apart()
    serialized_message = io.BytesIO()
    messages[2].serialize(serialized_message)
    short_message = (Imu._type, serialized_message.getvalue()[:-1], Imu._md5sum, Imu)
    with rosbag.Bag(bag_path, 'w') as bag:
        for message, arrival_stamp in zip(messages[:2], REGULAR_STAMPS):
            bag.write(DEFAULT_TOPIC, message, to_ros_time(arrival_stamp))
        bag.write(DEFAULT_TOPIC, short_message, to_ros_time(REGULAR_STAMPS[2]), raw=True)


def damage_first_chunk(bag_path: str) -> None:
    """Invert one byte in the middle of the data of the bag's first chunk.

    A chunk is a record of the bag file: the length of its header, the header, the length of
    its data and the data, which holds the messages. The bag's index lies behind the chunk and
    stays intact, so the bag still opens.
    """
    with rosbag.Bag(bag_path) as bag:
        _, first_record, _ = next(iter(bag.read_messages(raw=True)))
    chunk_position, _ = first_record[3]
    with open(bag_path, 'r+b') as bag_file:
        bag_file.seek(chunk_position)
        header_length, = struct.unpack('<L', bag_file.read(4))
        bag_file.seek(header_length, os.SEEK_CUR)
        data_length, = struct.unpack('<L', bag_file.read(4))
        bag_file.seek(data_length // 2, os.SEEK_CUR)
        damaged_byte = bag_file.read(1)[0] ^ 0xFF
        bag_file.seek(-1, os.SEEK_CUR)
        bag_file.write(bytes([damaged_byte]))


def run_tool(arguments: Sequence[str]) -> Tuple[int, str, str]:
    """Run the tool as a command and return its exit status, its stdout and its stderr."""
    result = subprocess.run([sys.executable, TOOL] + list(arguments), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, universal_newlines=True)
    return result.returncode, result.stdout, result.stderr


class TestToolOnBags(unittest.TestCase):
    """The tool as a command: a report and status 0, or one line on stderr and status 1."""

    def setUp(self) -> None:
        """Make a temporary folder for the test's bag."""
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.bag_path = os.path.join(folder.name, 'imu_still.bag')

    def report_lines(self, arguments: Sequence[str] = ()) -> List[str]:
        """Return the lines of the report that the tool prints for the test's bag."""
        status, stdout, stderr = run_tool([self.bag_path] + list(arguments))
        self.assertEqual((status, stderr), (0, ''))
        return stdout.splitlines()

    def assert_tool_ends_with(self, message: str, arguments: Sequence[str] = ()) -> None:
        """Check that the tool prints only the message, on stderr, and ends with status 1."""
        result = run_tool([self.bag_path] + list(arguments))
        self.assertEqual(result, (1, '', message + '\n'))

    def assert_tool_ends_with_a_reason_after(self, line_start: str) -> None:
        """Check that the tool ends with status 1 and one line: the start, then a reason.

        For reasons whose words come from rosbag or from Python, so that the test doesn't
        depend on them. An empty reason or "None" fails.
        """
        status, stdout, stderr = run_tool([self.bag_path])
        self.assertEqual((status, stdout), (1, ''))
        self.assertEqual(stderr.count('\n'), 1)
        self.assertTrue(stderr.startswith(line_start), stderr)
        self.assertNotIn(stderr[len(line_start):].strip(), ('', 'None'))

    def test_tool_prints_the_report_of_three_readings_one_output_step_apart(self) -> None:
        """Check every line for yaw rates of 0, 1 and 2 steps: a variance of one step squared.

        One step is 1/900 rad/s, so the mean and the standard deviation read 0.00111 rad/s and
        the variance 1.235e-06 (rad/s)^2. The covariances of zeros don't hide the readings.
        """
        write_bag(self.bag_path, DEFAULT_TOPIC, messages_one_step_apart())
        self.assertEqual(self.report_lines(), [
            '/diffbot/imu/data_raw: 3 messages, frame_id "imu_link"',
            'Header stamps: 40.0 ms from the first to the last, 50.00 Hz',
            'Header intervals: shortest 20.0 ms, longest 20.0 ms, 0 longer than 30 ms',
            "Stamps that don't advance: 0 repeated, 0 earlier than the one before",
            '',
            'angular_velocity  mean (bias)     variance  std dev  distinct',
            '3 readings            [rad/s]  [(rad/s)^2]  [rad/s]    values',
            'x                    +0.00000    0.000e+00  0.00000         1',
            'y                    +0.00000    0.000e+00  0.00000         1',
            'z                    +0.00111    1.235e-06  0.00111         3',
            '',
            'linear_acceleration  mean (gravity + bias)     variance  std dev  distinct',
            '3 readings                         [m/s^2]  [(m/s^2)^2]  [m/s^2]    values',
            'x                                 +0.00000    0.000e+00  0.00000         1',
            'y                                 +0.00000    0.000e+00  0.00000         1',
            'z                                 +9.81000    0.000e+00  0.00000         1',
            '',
            'Gravity, the length of the mean acceleration: 9.810 m/s^2',
        ])

    def test_tool_reads_another_topic_with_the_topic_option(self) -> None:
        """Check that --topic picks the messages of a topic with another name."""
        write_bag(self.bag_path, '/imu/data', messages_one_step_apart())
        lines = self.report_lines(['--topic', '/imu/data'])
        self.assertEqual(lines[0], '/imu/data: 3 messages, frame_id "imu_link"')

    def test_tool_gives_the_reason_instead_of_statistics_for_a_field_not_measured(self) -> None:
        """Check a bag whose messages mark the angular velocity as not measured.

        The angular velocity gets the reason instead of a bias and a noise of zero. The
        acceleration keeps its table.
        """
        messages = messages_one_step_apart()
        for message in messages:
            message.angular_velocity_covariance[0] = NOT_MEASURED
        write_bag(self.bag_path, DEFAULT_TOPIC, messages)
        self.assertEqual(self.report_lines()[5:10], [
            'angular_velocity: 0 readings, the statistics need at least two',
            '  (a message without a reading has -1 as angular_velocity_covariance[0])',
            '',
            'linear_acceleration  mean (gravity + bias)     variance  std dev  distinct',
            '3 readings                         [m/s^2]  [(m/s^2)^2]  [m/s^2]    values',
        ])

    def test_tool_takes_the_stamps_from_the_headers_not_from_the_arrival_times(self) -> None:
        """Check that a message that arrives 500 ms late leaves the stamps' lines as they are.

        The headers stay 20 ms apart, so the report can't show the late delivery.
        """
        late = REGULAR_STAMPS[2] + 500 * MILLISECOND
        write_bag(self.bag_path, DEFAULT_TOPIC, messages_one_step_apart(),
                  arrival_stamps=[REGULAR_STAMPS[0], REGULAR_STAMPS[1], late])
        self.assertEqual(self.report_lines()[1:3], [
            'Header stamps: 40.0 ms from the first to the last, 50.00 Hz',
            'Header intervals: shortest 20.0 ms, longest 20.0 ms, 0 longer than 30 ms',
        ])

    def messages_with_a_step_back(self) -> List[Imu]:
        """Return three messages of which the second is stamped 10 ms before the first."""
        messages = messages_one_step_apart()
        messages[1].header.stamp = to_ros_time(EPOCH - 10 * MILLISECOND)
        return messages

    def assert_report_has_no_rate_and_the_yaw_rate_statistics(self) -> None:
        """Check the report of messages_with_a_step_back(): no rate, one earlier stamp."""
        lines = self.report_lines()
        self.assertEqual(lines[1], 'Header stamps: 40.0 ms from the first to the last, no rate '
                                   "(some stamps don't advance)")
        self.assertEqual(lines[3],
                         "Stamps that don't advance: 0 repeated, 1 earlier than the one before")
        self.assertEqual(lines[9], 'z                    +0.00111    1.235e-06  0.00111         3')

    def test_tool_gives_no_rate_and_keeps_the_statistics_when_a_stamp_goes_back(self) -> None:
        """Check a bag whose second header stamp is earlier, while the recorder's clock goes on.

        The report says so instead of a rate, and still has the yaw rate's statistics.
        """
        write_bag(self.bag_path, DEFAULT_TOPIC, self.messages_with_a_step_back())
        self.assert_report_has_no_rate_and_the_yaw_rate_statistics()

    def test_tool_gives_no_rate_when_the_recorder_clock_goes_back_with_the_stamps(self) -> None:
        """Check the same bag when the recorder's clock is set back together with the stamps.

        Each message reaches the recorder DELIVERY_TIME after its stamp, so sorted by the
        recorder's time the stamps would advance. The tool reads them in the order in which
        the bag stored them, and the report is the same as with a recorder's clock that goes on.
        """
        messages = self.messages_with_a_step_back()
        write_bag(self.bag_path, DEFAULT_TOPIC, messages,
                  arrival_stamps=[message.header.stamp.to_nsec() + DELIVERY_TIME
                                  for message in messages])
        self.assert_report_has_no_rate_and_the_yaw_rate_statistics()

    def test_tool_ends_with_a_message_when_the_messages_have_two_frames(self) -> None:
        """Check that a second frame, with gravity on -z, is refused instead of averaged."""
        messages = messages_one_step_apart()
        messages[1].header.frame_id = 'other_imu'
        messages[1].linear_acceleration.z = -GRAVITY
        write_bag(self.bag_path, DEFAULT_TOPIC, messages)
        self.assert_tool_ends_with(
            f'{self.bag_path}, {DEFAULT_TOPIC}: the messages have more than one frame_id '
            '("imu_link", "other_imu"), so their readings have different axes')

    def test_tool_ends_with_a_message_when_the_messages_have_no_frame_id(self) -> None:
        """Check that messages without a frame's name are refused: gravity up, down and up."""
        messages = messages_one_step_apart()
        for message in messages:
            message.header.frame_id = ''
        messages[1].linear_acceleration.z = -GRAVITY
        write_bag(self.bag_path, DEFAULT_TOPIC, messages)
        self.assert_tool_ends_with(
            f'{self.bag_path}, {DEFAULT_TOPIC}: 3 of 3 messages have an empty frame_id. The '
            'readings need one named frame, which says what their axes are')

    def test_tool_ends_with_a_message_when_a_reading_is_not_a_number(self) -> None:
        """Check that a NaN yaw rate is refused, with the stamp of its message."""
        messages = messages_one_step_apart()
        messages[1].angular_velocity.z = math.nan
        write_bag(self.bag_path, DEFAULT_TOPIC, messages)
        self.assert_tool_ends_with(
            f'{self.bag_path}, {DEFAULT_TOPIC}: the message stamped 1700000000.020000000 has '
            "the angular_velocity (0.0, 0.0, nan), which isn't finite")

    def test_tool_ends_with_a_message_when_the_topic_has_another_type(self) -> None:
        """Check that std_msgs/String messages are refused by their type's name."""
        write_bag(self.bag_path, DEFAULT_TOPIC, [String('not an IMU'), String('message')])
        self.assert_tool_ends_with(
            f'{self.bag_path}, {DEFAULT_TOPIC}: the topic holds std_msgs/String. Every message '
            'of the topic must be a sensor_msgs/Imu')

    def test_tool_ends_with_a_message_when_the_topic_has_one_message(self) -> None:
        """Check that one message is refused: the statistics need two."""
        write_bag(self.bag_path, DEFAULT_TOPIC, messages_one_step_apart()[:1])
        self.assert_tool_ends_with(
            f'{self.bag_path}, {DEFAULT_TOPIC}: 1 message, the statistics need at least two')

    def test_tool_ends_with_a_message_when_the_topic_is_not_in_the_bag(self) -> None:
        """Check that a topic without messages in the bag is refused."""
        write_bag(self.bag_path, DEFAULT_TOPIC, messages_one_step_apart())
        self.assert_tool_ends_with(
            f'{self.bag_path}, /other: 0 messages, the statistics need at least two',
            ['--topic', '/other'])

    def test_tool_ends_with_a_message_when_the_file_is_missing(self) -> None:
        """Check that a file name without a file gives the system's reason, not a traceback."""
        self.assert_tool_ends_with(f"{self.bag_path} can't be opened: No such file or directory")

    def test_tool_ends_with_a_message_when_the_file_is_not_a_rosbag(self) -> None:
        """Check that a text file is refused by its first bytes, not with a traceback."""
        with open(self.bag_path, 'w') as text_file:
            text_file.write('not a bag\n')
        self.assert_tool_ends_with(f"{self.bag_path} isn't a rosbag: a bag file starts with "
                                   'the text #ROSBAG')

    def test_tool_ends_with_a_message_when_the_file_is_a_compressed_copy_of_a_bag(self) -> None:
        """Check that a bag packed with gzip, whose first bytes aren't text, is refused."""
        unpacked_path = self.bag_path + '.unpacked'
        write_bag(unpacked_path, DEFAULT_TOPIC, messages_one_step_apart())
        with open(unpacked_path, 'rb') as unpacked, gzip.open(self.bag_path, 'wb') as packed:
            shutil.copyfileobj(unpacked, packed)
        self.assert_tool_ends_with(f"{self.bag_path} isn't a rosbag: a bag file starts with "
                                   'the text #ROSBAG')

    def test_tool_ends_with_a_message_when_the_file_ends_after_the_first_line(self) -> None:
        """Check that a bag cut off after its first line is refused, with what can repair it."""
        with open(self.bag_path, 'w') as cut_off_file:
            cut_off_file.write('#ROSBAG V2.0\n')
        self.assert_tool_ends_with(f'{self.bag_path} has no index, like a recording that was '
                                   'cut off. "rosbag reindex" can repair such a file')

    def test_tool_ends_with_a_message_when_the_messages_data_is_damaged(self) -> None:
        """Check that a bag which opens, but whose compressed messages are damaged, is refused.

        The bag's index is intact. The reason comes from the decompression.
        """
        write_bag(self.bag_path, DEFAULT_TOPIC, messages_one_step_apart(),
                  compression=rosbag.Compression.BZ2)
        damage_first_chunk(self.bag_path)
        self.assert_tool_ends_with_a_reason_after(
            f"{self.bag_path}, {DEFAULT_TOPIC}: a message can't be read from the bag: ")

    def test_tool_ends_with_a_message_when_a_message_lacks_its_last_byte(self) -> None:
        """Check that a message whose data is too short for its type is refused.

        rosbag reads the record. The reason comes from the message's deserialization.
        """
        write_bag_with_a_short_last_message(self.bag_path)
        self.assert_tool_ends_with_a_reason_after(
            f"{self.bag_path}, {DEFAULT_TOPIC}: a message can't be read from the bag: ")


if __name__ == '__main__':
    unittest.main()
