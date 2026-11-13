#!/usr/bin/env python3
"""Check the statistics of scripts/imu_noise_at_rest.py on readings with known results.

The tests need no bag and no ROS master. catkin runs them with nose (catkin_add_nosetests in
CMakeLists.txt).
"""

import math
import os
import statistics
import sys
import unittest

# The tool is a script, not a Python package, so its folder has to be on the module search path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from imu_noise_at_rest import axis_statistics  # noqa: E402
from imu_noise_at_rest import AxisStatistics  # noqa: E402
from imu_noise_at_rest import format_report  # noqa: E402
from imu_noise_at_rest import format_table  # noqa: E402
from imu_noise_at_rest import imu_statistics  # noqa: E402
from imu_noise_at_rest import ImuSample  # noqa: E402
from imu_noise_at_rest import ImuStatistics  # noqa: E402
from imu_noise_at_rest import stamp_statistics  # noqa: E402
from imu_noise_at_rest import StampStatistics  # noqa: E402
from imu_noise_at_rest import Vector  # noqa: E402
from imu_noise_at_rest import vector_statistics  # noqa: E402
from imu_noise_at_rest import VectorStatistics  # noqa: E402

# The BNO055's output step of the angular velocity in rad/s: 900 LSB per rad/s
GYROSCOPE_STEP = 1 / 900  # rad/s


class TestAxisStatistics(unittest.TestCase):
    """axis_statistics() gives the mean, the sample variance and the distinct values."""

    def test_axis_statistics_gives_the_mean(self) -> None:
        self.assertAlmostEqual(axis_statistics([1.0, 2.0, 3.0, 6.0]).mean, 3.0)

    def test_axis_statistics_gives_the_sample_variance(self) -> None:
        # The squared deviations from the mean 2.5 add up to 5. Divided by 4 - 1 values, not by 4
        variance = axis_statistics([1.0, 2.0, 3.0, 4.0]).variance
        self.assertAlmostEqual(variance, 5.0 / 3.0)

    def test_axis_statistics_gives_the_standard_deviation_as_the_root_of_the_variance(
            self) -> None:
        axis = axis_statistics([1.0, 2.0, 3.0, 4.0])
        self.assertAlmostEqual(axis.standard_deviation, math.sqrt(5.0 / 3.0))

    def test_axis_statistics_counts_each_distinct_value_once(self) -> None:
        values = [0.0, GYROSCOPE_STEP, GYROSCOPE_STEP, 0.0, -GYROSCOPE_STEP]
        self.assertEqual(axis_statistics(values).distinct_value_count, 3)

    def test_axis_statistics_gives_a_variance_of_zero_without_noise(self) -> None:
        self.assertEqual(axis_statistics([GYROSCOPE_STEP] * 5).variance, 0.0)

    def test_axis_statistics_fails_with_fewer_than_two_values(self) -> None:
        with self.assertRaises(statistics.StatisticsError):
            axis_statistics([1.0])


class TestVectorStatistics(unittest.TestCase):
    """vector_statistics() gives each axis's statistics from that axis's values only."""

    def test_vector_statistics_keeps_the_axes_apart(self) -> None:
        axes = vector_statistics([Vector(1.0, 10.0, 100.0), Vector(3.0, 30.0, 300.0)])
        self.assertAlmostEqual(axes.x.mean, 2.0)
        self.assertAlmostEqual(axes.y.mean, 20.0)
        self.assertAlmostEqual(axes.z.mean, 200.0)


class TestStampStatistics(unittest.TestCase):
    """stamp_statistics() gives the messages' duration, rate and gaps."""

    # Five stamps 20 ms apart, except for one gap of 35 ms (s)
    STAMPS = [10.0, 10.02, 10.04, 10.075, 10.095]

    def test_stamp_statistics_gives_the_duration_and_the_mean_rate(self) -> None:
        stamps = stamp_statistics(self.STAMPS)
        self.assertEqual(stamps.message_count, 5)
        self.assertAlmostEqual(stamps.duration, 0.095)
        # Four gaps in 0.095 s
        self.assertAlmostEqual(stamps.rate, 4 / 0.095)

    def test_stamp_statistics_gives_the_shortest_and_the_longest_gap(self) -> None:
        stamps = stamp_statistics(self.STAMPS)
        self.assertAlmostEqual(stamps.shortest_gap, 0.02)
        self.assertAlmostEqual(stamps.longest_gap, 0.035)

    def test_stamp_statistics_counts_the_gaps_longer_than_long_gap(self) -> None:
        self.assertEqual(stamp_statistics(self.STAMPS).long_gap_count, 1)
        self.assertEqual(stamp_statistics([10.0, 10.025, 10.05]).long_gap_count, 0)

    def test_stamp_statistics_gives_no_rate_when_the_stamps_do_not_advance(self) -> None:
        self.assertTrue(math.isnan(stamp_statistics([0.0, 0.0, 0.0]).rate))


class TestImuStatistics(unittest.TestCase):
    """imu_statistics() takes each statistic from the right field of the samples."""

    def test_imu_statistics_keeps_the_angular_velocity_and_the_acceleration_apart(
            self) -> None:
        samples = [ImuSample(0.0, Vector(0.1, 0.2, 0.3), Vector(1.0, 2.0, 9.0)),
                   ImuSample(0.02, Vector(0.3, 0.4, 0.5), Vector(3.0, 4.0, 11.0))]
        imu = imu_statistics(samples)
        self.assertAlmostEqual(imu.stamps.duration, 0.02)
        self.assertAlmostEqual(imu.angular_velocity.z.mean, 0.4)
        self.assertAlmostEqual(imu.linear_acceleration.z.mean, 10.0)

    def test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration(
            self) -> None:
        # Tilted, the IMU measures gravity on all three axes: 2, 3 and 6 make a length of 7
        samples = [ImuSample(0.0, Vector(0.0, 0.0, 0.0), Vector(1.0, 3.0, 5.0)),
                   ImuSample(0.02, Vector(0.0, 0.0, 0.0), Vector(3.0, 3.0, 7.0))]
        self.assertAlmostEqual(imu_statistics(samples).gravity, 7.0)


class TestFormatTable(unittest.TestCase):
    """format_table() aligns the columns."""

    def test_format_table_aligns_the_first_column_left_and_the_others_right(self) -> None:
        lines = format_table([['axis', 'mean'], ['x', '+0.5'], ['yy', '-12.25']])
        self.assertEqual(lines, ['axis    mean', 'x       +0.5', 'yy    -12.25'])


class TestFormatReport(unittest.TestCase):
    """format_report() gives the report that the tool prints, line by line."""

    def test_format_report_gives_the_stamps_a_table_for_each_vector_field_and_gravity(
            self) -> None:
        # The same statistics on all three axes of a field, so the test shows the layout
        rotation_axis = AxisStatistics(mean=0.002, variance=1e-06, distinct_value_count=3)
        acceleration_axis = AxisStatistics(mean=-1.0, variance=0.25, distinct_value_count=12)
        imu = ImuStatistics(
            stamps=StampStatistics(message_count=501, duration=10.0, rate=50.0,
                                   shortest_gap=0.019, longest_gap=0.031, long_gap_count=2),
            angular_velocity=VectorStatistics(rotation_axis, rotation_axis, rotation_axis),
            linear_acceleration=VectorStatistics(acceleration_axis, acceleration_axis,
                                                 acceleration_axis),
            gravity=9.81)
        self.assertEqual(format_report('/imu', imu), [
            '/imu: 501 messages in 10.0 s, 50.00 Hz',
            'Gaps between the stamps: shortest 19.0 ms, longest 31.0 ms, 2 longer than 30 ms',
            '',
            'angular_velocity  mean (bias)     variance  std dev  distinct',
            '                      [rad/s]  [(rad/s)^2]  [rad/s]    values',
            'x                    +0.00200    1.000e-06  0.00100         3',
            'y                    +0.00200    1.000e-06  0.00100         3',
            'z                    +0.00200    1.000e-06  0.00100         3',
            '',
            'linear_acceleration  mean (gravity + bias)     variance  std dev  distinct',
            '                                   [m/s^2]  [(m/s^2)^2]  [m/s^2]    values',
            'x                                 -1.00000    2.500e-01  0.50000        12',
            'y                                 -1.00000    2.500e-01  0.50000        12',
            'z                                 -1.00000    2.500e-01  0.50000        12',
            '',
            'Gravity, the length of the mean acceleration: 9.810 m/s^2',
        ])


if __name__ == '__main__':
    unittest.main()
