#!/usr/bin/env python3
"""Print an IMU's bias and noise from a rosbag recorded with the robot at rest.

At rest, an ideal IMU reads an angular velocity of zero and an acceleration of gravity's size.
A real one differs in two ways, which this tool estimates on each axis of the sensor_msgs/Imu
messages:
- The bias is an offset. At rest, the mean of the angular velocity estimates the bias that
  remains in the published readings during the recording. Sensors that correct their own
  offsets, such as the BNO055 in its fusion modes, publish already corrected readings, so this
  isn't the raw sensor's bias. The mean of the acceleration is gravity plus the accelerometer's
  remaining error: one pose at rest can't separate the two. The length of the mean acceleration
  is gravity as the IMU measures it, about 9.81 m/s^2.
- The noise scatters the readings around their mean. The variances estimate the noise at rest.
  They are a baseline for the diagonal of the messages' covariances, which a filter such as
  robot_localization's EKF uses to weigh the readings. Driving can add vibration, so check the
  values while driving too.

The number of distinct values shows the sensor's resolution: when the noise is about one output
step, the readings take only a few values. The gaps between the stamps show missing or late
messages.

Usage: rosrun diffbot_bringup imu_noise_at_rest.py imu_still.bag [--topic /diffbot/imu/data_raw]
"""

import argparse
import math
import statistics
import sys
from typing import Any
from typing import List
from typing import NamedTuple
from typing import Sequence

import rosbag

DEFAULT_TOPIC = '/diffbot/imu/data_raw'
# The firmware publishes every 20 ms (50 Hz), so a gap of more than one and a half periods means
# that a message is missing or late
LONG_GAP = 0.030  # s


class Vector(NamedTuple):
    """A vector reading of the IMU, in the axes of the message's frame."""

    x: float
    y: float
    z: float


class ImuSample(NamedTuple):
    """The stamp and the readings of one sensor_msgs/Imu message."""

    stamp: float  # s
    angular_velocity: Vector  # rad/s
    linear_acceleration: Vector  # m/s^2


class AxisStatistics(NamedTuple):
    """The statistics of one axis's readings."""

    mean: float
    variance: float  # the sample variance, see axis_statistics()
    distinct_value_count: int

    @property
    def standard_deviation(self) -> float:
        """Return the square root of the variance, the noise in the readings' own unit."""
        return math.sqrt(self.variance)


class VectorStatistics(NamedTuple):
    """The statistics of a vector reading's x, y and z axes."""

    x: AxisStatistics
    y: AxisStatistics
    z: AxisStatistics


class StampStatistics(NamedTuple):
    """How regularly the messages came, from their header stamps."""

    message_count: int
    duration: float  # s, from the first stamp to the last
    rate: float  # Hz, the mean over the duration
    shortest_gap: float  # s, between two consecutive stamps
    longest_gap: float  # s
    long_gap_count: int  # the gaps longer than LONG_GAP


class ImuStatistics(NamedTuple):
    """The statistics of all the messages that the report shows."""

    stamps: StampStatistics
    angular_velocity: VectorStatistics  # rad/s
    linear_acceleration: VectorStatistics  # m/s^2
    # The length of the mean acceleration (m/s^2): at rest, gravity as the IMU measures it
    gravity: float


class ImuField(NamedTuple):
    """A vector field of sensor_msgs/Imu, as the report's table names it."""

    name: str
    unit: str
    # What the mean of the field's readings is at rest
    mean_meaning: str


ANGULAR_VELOCITY = ImuField('angular_velocity', 'rad/s', 'bias')
LINEAR_ACCELERATION = ImuField('linear_acceleration', 'm/s^2', 'gravity + bias')


def to_vector(message: Any) -> Vector:
    """Convert a geometry_msgs/Vector3 message to a Vector."""
    return Vector(message.x, message.y, message.z)


def to_sample(message: Any) -> ImuSample:
    """Convert a sensor_msgs/Imu message to an ImuSample.

    rosbag builds the message classes from the definitions stored in the bag, so the message
    isn't a sensor_msgs.msg.Imu, but it has the same fields.
    """
    return ImuSample(stamp=message.header.stamp.to_sec(),
                     angular_velocity=to_vector(message.angular_velocity),
                     linear_acceleration=to_vector(message.linear_acceleration))


def read_samples(bag_path: str, topic: str) -> List[ImuSample]:
    """Return the samples of the sensor_msgs/Imu messages on the topic, in the bag's order."""
    with rosbag.Bag(bag_path) as bag:
        return [to_sample(message) for _, message, _ in bag.read_messages(topics=[topic])]


def axis_statistics(values: Sequence[float]) -> AxisStatistics:
    """Return the mean, the sample variance and the number of distinct values of one axis.

    The sample variance divides by the number of values minus one: the deviations are measured
    from the values' own mean, which lies closer to them than the true mean. It needs at least
    two values.
    """
    return AxisStatistics(mean=statistics.mean(values),
                          variance=statistics.variance(values),
                          distinct_value_count=len(set(values)))


def vector_statistics(vectors: Sequence[Vector]) -> VectorStatistics:
    """Return the statistics of each axis of the vectors."""
    return VectorStatistics(x=axis_statistics([vector.x for vector in vectors]),
                            y=axis_statistics([vector.y for vector in vectors]),
                            z=axis_statistics([vector.z for vector in vectors]))


def stamp_statistics(stamps: Sequence[float]) -> StampStatistics:
    """Return the number of stamps, their duration, their mean rate and the gaps between them.

    The rate counts the gaps per second, because n messages in a row span n - 1 gaps. Stamps that
    don't advance, for example all zero, have no rate: it's NaN. It needs at least two stamps.
    """
    gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
    duration = stamps[-1] - stamps[0]
    return StampStatistics(message_count=len(stamps),
                           duration=duration,
                           rate=len(gaps) / duration if duration > 0.0 else math.nan,
                           shortest_gap=min(gaps),
                           longest_gap=max(gaps),
                           long_gap_count=sum(gap > LONG_GAP for gap in gaps))


def mean_vector_length(axes: VectorStatistics) -> float:
    """Return the length of the vector of the axes' means."""
    return math.hypot(axes.x.mean, axes.y.mean, axes.z.mean)


def imu_statistics(samples: Sequence[ImuSample]) -> ImuStatistics:
    """Return the statistics of the samples' stamps and readings."""
    linear_acceleration = vector_statistics([sample.linear_acceleration for sample in samples])
    return ImuStatistics(
        stamps=stamp_statistics([sample.stamp for sample in samples]),
        angular_velocity=vector_statistics([sample.angular_velocity for sample in samples]),
        linear_acceleration=linear_acceleration,
        gravity=mean_vector_length(linear_acceleration))


def format_stamps(topic: str, stamps: StampStatistics) -> List[str]:
    """Return the lines that describe the messages' count, rate and gaps."""
    return [f'{topic}: {stamps.message_count} messages in {stamps.duration:.1f} s, '
            f'{stamps.rate:.2f} Hz',
            f'Gaps between the stamps: shortest {stamps.shortest_gap * 1000:.1f} ms, '
            f'longest {stamps.longest_gap * 1000:.1f} ms, '
            f'{stamps.long_gap_count} longer than {LONG_GAP * 1000:.0f} ms']


def format_table(rows: Sequence[Sequence[str]]) -> List[str]:
    """Return the rows as lines of columns, the first aligned left and the others right."""
    widths = [max(len(cell) for cell in column) for column in zip(*rows)]
    lines = []
    for row in rows:
        first_cell = row[0].ljust(widths[0])
        other_cells = [cell.rjust(width) for cell, width in zip(row[1:], widths[1:])]
        lines.append('  '.join([first_cell] + other_cells))
    return lines


def format_axis_row(axis_name: str, axis: AxisStatistics) -> List[str]:
    """Return the table cells of one axis."""
    return [axis_name, f'{axis.mean:+.5f}', f'{axis.variance:.3e}',
            f'{axis.standard_deviation:.5f}', str(axis.distinct_value_count)]


def format_vector_table(field: ImuField, axes: VectorStatistics) -> List[str]:
    """Return a table of a vector field's statistics, one row per axis, with the units."""
    header = [field.name, f'mean ({field.mean_meaning})', 'variance', 'std dev', 'distinct']
    units = ['', f'[{field.unit}]', f'[({field.unit})^2]', f'[{field.unit}]', 'values']
    axis_rows = [format_axis_row(axis_name, axis)
                 for axis_name, axis in zip(VectorStatistics._fields, axes)]
    return format_table([header, units] + axis_rows)


def print_report(topic: str, imu: ImuStatistics) -> None:
    """Print the stamps, a table for the angular velocity and the acceleration, then gravity."""
    lines = format_stamps(topic, imu.stamps)
    lines += [''] + format_vector_table(ANGULAR_VELOCITY, imu.angular_velocity)
    lines += [''] + format_vector_table(LINEAR_ACCELERATION, imu.linear_acceleration)
    lines += ['', f'Gravity, the length of the mean acceleration: {imu.gravity:.3f} m/s^2']
    print('\n'.join(lines))


def parse_arguments() -> argparse.Namespace:
    """Return the command line's bag and topic."""
    parser = argparse.ArgumentParser(
        description="Print an IMU's bias and noise from a rosbag recorded at rest.")
    parser.add_argument('bag', help='the rosbag file')
    parser.add_argument('--topic', default=DEFAULT_TOPIC,
                        help='the sensor_msgs/Imu topic (default: %(default)s)')
    return parser.parse_args()


def main() -> None:
    """Read the IMU's messages from the bag and print their statistics."""
    arguments = parse_arguments()
    samples = read_samples(arguments.bag, arguments.topic)
    if len(samples) < 2:
        sys.exit(f'{arguments.bag} has {len(samples)} messages on {arguments.topic}, '
                 'the statistics need at least two')
    print_report(arguments.topic, imu_statistics(samples))


if __name__ == '__main__':
    main()
