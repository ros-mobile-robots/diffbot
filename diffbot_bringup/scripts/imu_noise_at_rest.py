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

The number of distinct values says how many output levels an axis's readings took in this
recording. When the noise is about one output step, they take only a few. The count isn't the
sensor's resolution: that is the size of one step, which the sensor's datasheet gives.

The stamps are the messages' header stamps, which the firmware takes before it reads the IMU. The
intervals between them show how regularly the recorded samples were taken. LONG_INTERVAL is one
and a half periods of the firmware's 50 Hz. A longer interval is worth a look: a sample was taken
late, dropped or lost. The header stamps don't say when a message arrived, so they can't show a
late delivery. A stamp that repeats the one before, or is earlier than it, is counted, and the
recording then has no rate.

A field's statistics come from the messages that measured the field: sensor_msgs/Imu marks a
field without a measurement with -1 as the first element of the field's covariance. A field with
fewer than two readings gets two lines instead of statistics. The tool prints one line and ends
with status 1 instead of a report when the file isn't a readable rosbag, when the topic's messages
have another type, more than one frame_id or a reading that isn't finite, and when fewer than two
messages or readings are left for the statistics.

Usage: rosrun diffbot_bringup imu_noise_at_rest.py imu_still.bag [--topic /diffbot/imu/data_raw]
"""

import argparse
import math
import statistics
import sys
from typing import Any
from typing import Callable
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

import rosbag

DEFAULT_TOPIC = '/diffbot/imu/data_raw'
IMU_TYPE = 'sensor_msgs/Imu'
# sensor_msgs/Imu: a message without a measurement of a field has -1 as the first element of the
# field's covariance
NOT_MEASURED = -1.0

NANOSECONDS_PER_SECOND = 1000000000
NANOSECONDS_PER_MILLISECOND = 1000000
# The firmware takes a sample every 20 ms (50 Hz)
IMU_PERIOD = 20 * NANOSECONDS_PER_MILLISECOND  # ns
# One and a half periods. A longer interval between two header stamps is worth a look: the
# firmware took a sample late or dropped it, or a message was lost
LONG_INTERVAL = 3 * IMU_PERIOD // 2  # ns


class InputError(Exception):
    """A bag or its messages that give no report, with the reason as the error's text."""


class Vector(NamedTuple):
    """A vector reading of the IMU, in the axes of the message's frame."""

    x: float
    y: float
    z: float


class ImuSample(NamedTuple):
    """The header and the readings of one sensor_msgs/Imu message."""

    # The header stamp as an integer, so that the interval between two stamps is exact
    stamp: int  # ns
    frame_id: str
    # None when the message has no measurement of the field
    angular_velocity: Optional[Vector]  # rad/s
    linear_acceleration: Optional[Vector]  # m/s^2


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


class FieldStatistics(NamedTuple):
    """The statistics of one vector field, from the messages that measured it."""

    reading_count: int
    # None with fewer than two readings, which have no variance
    axes: Optional[VectorStatistics]


class StampStatistics(NamedTuple):
    """How regularly the samples were taken, from the messages' header stamps."""

    message_count: int
    duration: int  # ns, from the first stamp to the last
    # The mean over the duration. None when a stamp doesn't advance, see mean_rate()
    rate: Optional[float]  # Hz
    shortest_interval: int  # ns, between two consecutive stamps
    longest_interval: int  # ns
    long_interval_count: int  # the intervals longer than LONG_INTERVAL
    repeated_stamp_count: int  # the stamps equal to the one before
    earlier_stamp_count: int  # the stamps earlier than the one before


class ImuStatistics(NamedTuple):
    """The statistics of all the messages that the report shows."""

    stamps: StampStatistics
    frame_id: str
    angular_velocity: FieldStatistics  # rad/s
    linear_acceleration: FieldStatistics  # m/s^2
    # The length of the mean acceleration: at rest, gravity as the IMU measures it. None without
    # the acceleration's statistics
    gravity: Optional[float]  # m/s^2


class ImuField(NamedTuple):
    """A vector field of sensor_msgs/Imu and of ImuSample, as the report's table names it."""

    name: str
    unit: str
    # What the mean of the field's readings is at rest
    mean_meaning: str


ANGULAR_VELOCITY = ImuField('angular_velocity', 'rad/s', 'bias')
LINEAR_ACCELERATION = ImuField('linear_acceleration', 'm/s^2', 'gravity + bias')
FIELDS = (ANGULAR_VELOCITY, LINEAR_ACCELERATION)


def count_of(count: int, noun: str) -> str:
    """Return the count with its noun, in the plural unless the count is one."""
    return f'{count} {noun}' if count == 1 else f'{count} {noun}s'


def read_messages(bag_path: str, topic: str) -> List[Any]:
    """Return the topic's messages in the order in which the bag recorded them."""
    try:
        with rosbag.Bag(bag_path) as bag:
            return [message for _, message, _ in bag.read_messages(topics=[topic])]
    except OSError as error:
        raise InputError(f"{bag_path} can't be opened: {error.strerror}") from error
    except rosbag.ROSBagException as error:
        raise InputError(f"{bag_path} can't be read as a rosbag: {error}") from error


def check_message_types(messages: Sequence[Any]) -> None:
    """Raise an InputError when a message isn't a sensor_msgs/Imu.

    rosbag builds the message classes from the definitions stored in the bag. Each class has the
    name of its type as _type.
    """
    other_types = sorted({message._type for message in messages} - {IMU_TYPE})
    if other_types:
        raise InputError(
            f'the messages have the type {", ".join(other_types)}, the tool reads {IMU_TYPE}')


def measured_vector(vector: Any, covariance: Sequence[float]) -> Optional[Vector]:
    """Return a field's geometry_msgs/Vector3 as a Vector, or None for a field not measured.

    Only -1 as the covariance's first element marks a field as not measured. A covariance of
    zeros means that the covariance isn't known, and the field's reading counts.
    """
    if covariance[0] == NOT_MEASURED:
        return None
    return Vector(vector.x, vector.y, vector.z)


def to_sample(message: Any) -> ImuSample:
    """Convert a sensor_msgs/Imu message to an ImuSample."""
    return ImuSample(
        stamp=message.header.stamp.to_nsec(),
        frame_id=message.header.frame_id,
        angular_velocity=measured_vector(message.angular_velocity,
                                         message.angular_velocity_covariance),
        linear_acceleration=measured_vector(message.linear_acceleration,
                                            message.linear_acceleration_covariance))


def field_readings(samples: Sequence[ImuSample], field: ImuField) -> List[Vector]:
    """Return the field's readings from the samples that measured the field."""
    readings = [getattr(sample, field.name) for sample in samples]
    return [reading for reading in readings if reading is not None]


def check_message_count(samples: Sequence[ImuSample]) -> None:
    """Raise an InputError for fewer than two messages: they have no interval between them."""
    if len(samples) < 2:
        raise InputError(f'{count_of(len(samples), "message")}, the statistics need at least two')


def check_one_frame(samples: Sequence[ImuSample]) -> None:
    """Raise an InputError when the messages name more than one frame.

    Readings in different frames have different axes, so their mean says nothing about one axis.
    """
    frame_ids = sorted({sample.frame_id for sample in samples})
    if len(frame_ids) > 1:
        quoted_frame_ids = ', '.join(f'"{frame_id}"' for frame_id in frame_ids)
        raise InputError(f'the messages have more than one frame_id ({quoted_frame_ids}), '
                         'so their readings have different axes')


def format_stamp(stamp: int) -> str:
    """Return a stamp in ns as seconds with all nine decimals, as rostopic echo shows it."""
    seconds, nanoseconds = divmod(stamp, NANOSECONDS_PER_SECOND)
    return f'{seconds}.{nanoseconds:09d}'


def first_sample_with_a_nonfinite_reading(samples: Sequence[ImuSample],
                                          field: ImuField) -> Optional[ImuSample]:
    """Return the first sample whose reading of the field has a NaN or an infinite value."""
    for sample in samples:
        reading = getattr(sample, field.name)
        if reading is not None and not all(math.isfinite(value) for value in reading):
            return sample
    return None


def check_finite_readings(samples: Sequence[ImuSample]) -> None:
    """Raise an InputError for a measured reading with a NaN or an infinite value.

    One such value would make the axis's mean and variance NaN or infinite too.
    """
    for field in FIELDS:
        sample = first_sample_with_a_nonfinite_reading(samples, field)
        if sample is not None:
            x, y, z = getattr(sample, field.name)
            raise InputError(f'the message stamped {format_stamp(sample.stamp)} has the '
                             f"{field.name} ({x}, {y}, {z}), which isn't finite")


def check_reading_count(samples: Sequence[ImuSample]) -> None:
    """Raise an InputError when no field has the two readings that a variance needs."""
    if all(len(field_readings(samples, field)) < 2 for field in FIELDS):
        raise InputError('fewer than two messages have a measurement of the angular velocity '
                         'or of the acceleration')


SampleCheck = Callable[[Sequence[ImuSample]], None]
# In this order: the later checks and the statistics rely on the earlier ones
SAMPLE_CHECKS: Tuple[SampleCheck, ...] = (
    check_message_count,
    check_one_frame,
    check_finite_readings,
    check_reading_count,
)


def check_samples(samples: Sequence[ImuSample]) -> None:
    """Raise the InputError of the first check in SAMPLE_CHECKS that the samples fail."""
    for check in SAMPLE_CHECKS:
        check(samples)


def read_samples(bag_path: str, topic: str) -> List[ImuSample]:
    """Return the samples of the topic's messages, in the bag's order, after all the checks.

    An InputError about the messages names the bag and the topic.
    """
    messages = read_messages(bag_path, topic)
    try:
        check_message_types(messages)
        samples = [to_sample(message) for message in messages]
        check_samples(samples)
    except InputError as error:
        raise InputError(f'{bag_path}, {topic}: {error}') from error
    return samples


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


def field_statistics(readings: Sequence[Vector]) -> FieldStatistics:
    """Return the number of a field's readings and, from two readings on, their statistics."""
    axes = vector_statistics(readings) if len(readings) >= 2 else None
    return FieldStatistics(reading_count=len(readings), axes=axes)


def mean_rate(intervals: Sequence[int]) -> Optional[float]:
    """Return the number of intervals per second in Hz, or None when a stamp doesn't advance.

    n messages in a row span n - 1 intervals, so the rate counts the intervals, not the messages.
    The intervals add up to the time from the first stamp to the last. With a repeated or an
    earlier stamp, that time isn't the time the messages span, so they have no rate.
    """
    if min(intervals) <= 0:
        return None
    return len(intervals) * NANOSECONDS_PER_SECOND / sum(intervals)


def stamp_statistics(stamps: Sequence[int]) -> StampStatistics:
    """Return the number of header stamps, their duration, their rate and their intervals.

    The stamps are integers in ns, so the intervals are exact, and so is their comparison with
    LONG_INTERVAL. Seconds as floats can't tell 30 ms from 30 ms and 1 ns at a stamp of today's
    size. It needs at least two stamps.
    """
    intervals = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
    return StampStatistics(
        message_count=len(stamps),
        duration=stamps[-1] - stamps[0],
        rate=mean_rate(intervals),
        shortest_interval=min(intervals),
        longest_interval=max(intervals),
        long_interval_count=sum(interval > LONG_INTERVAL for interval in intervals),
        repeated_stamp_count=sum(interval == 0 for interval in intervals),
        earlier_stamp_count=sum(interval < 0 for interval in intervals))


def mean_vector_length(axes: VectorStatistics) -> float:
    """Return the length of the vector of the axes' means."""
    return math.hypot(axes.x.mean, axes.y.mean, axes.z.mean)


def gravity(linear_acceleration: FieldStatistics) -> Optional[float]:
    """Return the length of the mean acceleration, or None without the field's statistics."""
    if linear_acceleration.axes is None:
        return None
    return mean_vector_length(linear_acceleration.axes)


def imu_statistics(samples: Sequence[ImuSample]) -> ImuStatistics:
    """Return the statistics of the samples' stamps and readings.

    The samples have passed check_samples(): at least two of them, all in one frame.
    """
    linear_acceleration = field_statistics(field_readings(samples, LINEAR_ACCELERATION))
    return ImuStatistics(
        stamps=stamp_statistics([sample.stamp for sample in samples]),
        frame_id=samples[0].frame_id,
        angular_velocity=field_statistics(field_readings(samples, ANGULAR_VELOCITY)),
        linear_acceleration=linear_acceleration,
        gravity=gravity(linear_acceleration))


def format_duration(duration: int) -> str:
    """Return a duration in ns as text: in seconds from one second on, in milliseconds below."""
    if abs(duration) >= NANOSECONDS_PER_SECOND:
        return f'{duration / NANOSECONDS_PER_SECOND:.1f} s'
    return f'{duration / NANOSECONDS_PER_MILLISECOND:.1f} ms'


def format_rate(rate: Optional[float]) -> str:
    """Return the rate in Hz as text, or why the stamps have none."""
    if rate is None:
        return "no rate (some stamps don't advance)"
    return f'{rate:.2f} Hz'


def format_stamps(topic: str, frame_id: str, stamps: StampStatistics) -> List[str]:
    """Return the lines about the messages' count and frame and about their header stamps."""
    return [f'{topic}: {count_of(stamps.message_count, "message")}, frame_id "{frame_id}"',
            f'Header stamps: {format_duration(stamps.duration)} from the first to the last, '
            f'{format_rate(stamps.rate)}',
            f'Header intervals: shortest {format_duration(stamps.shortest_interval)}, '
            f'longest {format_duration(stamps.longest_interval)}, '
            f'{stamps.long_interval_count} longer than '
            f'{LONG_INTERVAL // NANOSECONDS_PER_MILLISECOND} ms',
            f"Stamps that don't advance: {stamps.repeated_stamp_count} repeated, "
            f'{stamps.earlier_stamp_count} earlier than the one before']


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


def format_vector_table(field: ImuField, reading_count: int,
                        axes: VectorStatistics) -> List[str]:
    """Return a table of a field's statistics: the units, the reading count, a row per axis."""
    header = [field.name, f'mean ({field.mean_meaning})', 'variance', 'std dev', 'distinct']
    units = [count_of(reading_count, 'reading'), f'[{field.unit}]', f'[({field.unit})^2]',
             f'[{field.unit}]', 'values']
    axis_rows = [format_axis_row(axis_name, axis)
                 for axis_name, axis in zip(VectorStatistics._fields, axes)]
    return format_table([header, units] + axis_rows)


def format_field(field: ImuField, summary: FieldStatistics) -> List[str]:
    """Return the table of a field's statistics, or two lines that say why the field has none."""
    if summary.axes is None:
        return [f'{field.name}: {count_of(summary.reading_count, "reading")}, '
                'the statistics need at least two',
                f'  (a message without a reading has -1 as {field.name}_covariance[0])']
    return format_vector_table(field, summary.reading_count, summary.axes)


def format_report(topic: str, imu: ImuStatistics) -> List[str]:
    """Return the report's lines: the stamps, each vector field, then gravity if it is known."""
    lines = format_stamps(topic, imu.frame_id, imu.stamps)
    lines += [''] + format_field(ANGULAR_VELOCITY, imu.angular_velocity)
    lines += [''] + format_field(LINEAR_ACCELERATION, imu.linear_acceleration)
    if imu.gravity is not None:
        lines += ['', f'Gravity, the length of the mean acceleration: {imu.gravity:.3f} m/s^2']
    return lines


def parse_arguments() -> argparse.Namespace:
    """Return the command line's bag and topic."""
    parser = argparse.ArgumentParser(
        description="Print an IMU's bias and noise from a rosbag recorded at rest.")
    parser.add_argument('bag', help='the rosbag file')
    parser.add_argument('--topic', default=DEFAULT_TOPIC,
                        help='the sensor_msgs/Imu topic (default: %(default)s)')
    return parser.parse_args()


def main() -> None:
    """Print the statistics of the IMU's messages in the bag, or why the bag gives none."""
    arguments = parse_arguments()
    try:
        samples = read_samples(arguments.bag, arguments.topic)
    except InputError as error:
        sys.exit(str(error))
    print('\n'.join(format_report(arguments.topic, imu_statistics(samples))))


if __name__ == '__main__':
    main()
