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

The stamps are the messages' header stamps, which the firmware takes before it reads the IMU,
in the order in which the bag stored the messages. The intervals between them show how regularly
the recorded samples were stamped. LONG_INTERVAL is one and a half periods of the firmware's
50 Hz. The report counts the longer intervals, because they are worth a look. It can't say why
one is long. With this firmware, a sample that was taken late and a sample that is missing are
two possible reasons. A clock that was set forward is another. A regular topic of 10 Hz has
intervals of 100 ms, so each of them is longer than this firmware's limit. The header stamps
don't say when a message arrived, so they can't show a late delivery. A stamp that repeats the
one before, or is earlier than it, is counted, and the recording then has no rate.

A field's statistics come from the messages that measured the field: sensor_msgs/Imu marks a
field without a measurement with -1 as the first element of the field's covariance. A field with
fewer than two readings gets two lines instead of statistics. The tool prints one line and ends
with status 1 instead of a report when the file isn't a readable rosbag, when the topic's messages
have another type, an empty frame_id, more than one frame_id or a reading that isn't finite or
is larger than any IMU's, and when fewer than two messages or readings are left for the
statistics.

Usage: rosrun diffbot_bringup imu_noise_at_rest.py imu_still.bag [--topic /diffbot/imu/data_raw]
"""

import argparse
from fractions import Fraction
import math
import operator
import statistics
import sys
from typing import Any
from typing import Callable
from typing import Iterator
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Set
from typing import Tuple

import rosbag
from sensor_msgs.msg import Imu

DEFAULT_TOPIC = '/diffbot/imu/data_raw'
# The first bytes of every bag file, before the format's version
BAG_FILE_START = b'#ROS'
# sensor_msgs/Imu: a message without a measurement of a field has -1 as the first element of the
# field's covariance
NOT_MEASURED = -1.0
# This tool's limit for a plausible reading. No IMU measures a rate in rad/s or an acceleration
# in m/s^2 of this size, so the tool takes a larger value as damaged data
LARGEST_READING = 1.0e6

NANOSECONDS_PER_SECOND = 1000000000
NANOSECONDS_PER_MILLISECOND = 1000000
# The firmware takes a sample every 20 ms (50 Hz)
IMU_PERIOD = 20 * NANOSECONDS_PER_MILLISECOND  # ns
# One and a half periods of the firmware. The report counts the longer intervals between two
# header stamps, because they are worth a look. The count doesn't say why an interval is long
LONG_INTERVAL = 3 * IMU_PERIOD // 2  # ns


class InputError(Exception):
    """An input that gives no report. The error's text is the line that the tool prints."""


class MessageError(Exception):
    """Messages that the statistics can't use. The error's text is the reason."""


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


class MessageDefinition(NamedTuple):
    """The name of a message type and the checksum of the type's definition."""

    type_name: str
    checksum: str


# ROS's own definition of the messages that the tool reads
IMU_DEFINITION = MessageDefinition(Imu._type, Imu._md5sum)


class StoredSample(NamedTuple):
    """A sample and the place of its message in the bag file."""

    # The position of the message's chunk in the file and of the message in the chunk. The
    # recorder appends each message, so a later message has a larger position
    position: Tuple[int, int]
    sample: ImuSample


class AxisStatistics(NamedTuple):
    """The statistics of one axis's readings."""

    mean: float
    variance: float  # the sample variance, see sample_variance()
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


def reason_of(error: Exception) -> str:
    """Return what an error says, as the reason in the tool's line.

    rosbag's own errors have a text for the user. Another error keeps the name of its type,
    because its text alone can be a number or empty.
    """
    text = str(error)
    if isinstance(error, rosbag.ROSBagException) and text:
        return text
    return f'{type(error).__name__}: {text}' if text else type(error).__name__


def check_bag_file_start(bag_path: str) -> None:
    """Raise an InputError when the file can't be opened or doesn't start as a bag file does."""
    try:
        with open(bag_path, 'rb') as bag_file:
            file_start = bag_file.read(len(BAG_FILE_START))
    except OSError as error:
        raise InputError(f"{bag_path} can't be opened: {error.strerror}") from error
    if file_start != BAG_FILE_START:
        raise InputError(f"{bag_path} isn't a rosbag: a bag file starts with the text #ROSBAG")


def open_bag(bag_path: str) -> Any:
    """Return the bag, opened for reading, or raise an InputError with the file and the reason.

    rosbag reads the bag's index when it opens the file. On a file that starts like a bag and is
    damaged or cut off, its parser fails with its own errors and with others, such as a
    KeyError. Whatever the error is, the file can't be read as a rosbag.
    """
    check_bag_file_start(bag_path)
    try:
        return rosbag.Bag(bag_path)
    except rosbag.ROSBagUnindexedException as error:
        # The recorder writes the index when it ends, so a recording that was cut off has none
        raise InputError(f'{bag_path} has no index, like a recording that was cut off. '
                         '"rosbag reindex" can repair such a file') from error
    except Exception as error:
        raise InputError(f"{bag_path} can't be read as a rosbag: {reason_of(error)}") from error


def topic_definitions(bag: Any, topic: str) -> Set[MessageDefinition]:
    """Return the message definitions of the topic's connections, without reading a message.

    rosbag asks the connection filter about each connection of the topic before it reads the
    connection's messages. The filter notes the connection's type and checksum and declines the
    connection. An error of rosbag's reader becomes the reason of a MessageError.
    """
    definitions: Set[MessageDefinition] = set()

    def note_definition(_topic: str, type_name: str, checksum: str, _definition: str,
                        _header: Any) -> bool:
        definitions.add(MessageDefinition(type_name, checksum))
        return False

    try:
        for _ in bag.read_messages(topics=[topic], connection_filter=note_definition):
            pass
    except Exception as error:
        raise MessageError(
            f"the topic's message types can't be read from the bag: {reason_of(error)}"
        ) from error
    return definitions


def check_topic_type(definitions: Set[MessageDefinition]) -> None:
    """Raise a MessageError when the topic has messages that aren't named sensor_msgs/Imu.

    The reason names all the topic's types, so that it fits what rosbag info shows for a topic
    with several types: only one of them.
    """
    type_names = sorted({definition.type_name for definition in definitions})
    if any(type_name != IMU_DEFINITION.type_name for type_name in type_names):
        raise MessageError(f'the topic holds {", ".join(type_names)}. '
                           f'Every message of the topic must be a {IMU_DEFINITION.type_name}')


def check_topic_checksum(definitions: Set[MessageDefinition]) -> None:
    """Raise a MessageError when the messages' definition isn't ROS's sensor_msgs/Imu.

    A bag stores each message type's definition. Messages that carry the type's name with
    another definition can lack the fields that the tool reads.
    """
    checksums = sorted({definition.checksum for definition in definitions})
    if any(checksum != IMU_DEFINITION.checksum for checksum in checksums):
        raise MessageError(
            f"the topic's messages are named {IMU_DEFINITION.type_name}, but their definition "
            f"isn't ROS's: its checksum is {', '.join(checksums)} instead of "
            f'{IMU_DEFINITION.checksum}')


def check_topic(bag: Any, topic: str) -> None:
    """Raise a MessageError when the topic's messages aren't ROS's sensor_msgs/Imu.

    Before any message is read: a topic of another type can hold far larger messages, such as a
    camera's images.
    """
    definitions = topic_definitions(bag, topic)
    check_topic_type(definitions)
    check_topic_checksum(definitions)


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


def raw_records(bag: Any, topic: str) -> Iterator[Any]:
    """Yield the raw records of the topic's messages, in the order of rosbag's reader.

    A raw record is the message's type, its bytes, the type's checksum, the message's position
    in the file and the message's class, which rosbag builds from the definition in the bag.

    A bag whose index is intact opens, also when the messages' data is damaged. The reader then
    fails while it reads, with its own errors, with the decompression's and with others.
    Whatever the error is, it becomes the reason of a MessageError.
    """
    try:
        for _, record, _ in bag.read_messages(topics=[topic], raw=True):
            yield record
    except Exception as error:
        raise MessageError(f"a message can't be read from the bag: {reason_of(error)}") from error


def to_stored_sample(record: Any) -> StoredSample:
    """Convert a raw record to the message's sample and its place in the file.

    The message's bytes may not fit the message's type, for example when the last bytes are
    missing. The error of the type's deserialization becomes the reason of a MessageError.
    """
    _, serialized_message, _, position, message_class = record
    message = message_class()
    try:
        message.deserialize(serialized_message)
    except Exception as error:
        raise MessageError(f"a message can't be read from the bag: {reason_of(error)}") from error
    return StoredSample(position, to_sample(message))


def recorded_samples(bag: Any, topic: str) -> List[ImuSample]:
    """Return the samples of the topic's messages in the order in which the bag stored them.

    rosbag's reader gives the messages sorted by the time at which the recorder got each one.
    That isn't the order of the recording when the recorder's clock was set back in between. A
    publisher on the same clock then stamps its messages earlier too, and in the reader's order
    its stamps would seem to advance. Each message is converted when it is read, so the messages
    themselves aren't kept.
    """
    stored_samples = [to_stored_sample(record) for record in raw_records(bag, topic)]
    stored_samples.sort(key=operator.attrgetter('position'))
    return [stored.sample for stored in stored_samples]


def field_readings(samples: Sequence[ImuSample], field: ImuField) -> List[Vector]:
    """Return the field's readings from the samples that measured the field."""
    readings = [getattr(sample, field.name) for sample in samples]
    return [reading for reading in readings if reading is not None]


def check_message_count(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError for fewer than two messages: they have no interval between them."""
    if len(samples) < 2:
        raise MessageError(
            f'{count_of(len(samples), "message")}, the statistics need at least two')


def check_named_frame(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError when a message has an empty frame_id.

    The frame says which axes a message's readings have. Without its name, nothing shows that
    the readings of two messages share their axes.
    """
    unnamed_count = sum(not sample.frame_id for sample in samples)
    if unnamed_count:
        raise MessageError(f'{unnamed_count} of {count_of(len(samples), "message")} have an '
                           'empty frame_id. The readings need one named frame, which says '
                           'what their axes are')


def check_one_frame(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError when the messages name more than one frame.

    Readings in different frames have different axes, so their mean says nothing about one axis.
    """
    frame_ids = sorted({sample.frame_id for sample in samples})
    if len(frame_ids) > 1:
        quoted_frame_ids = ', '.join(f'"{frame_id}"' for frame_id in frame_ids)
        raise MessageError(f'the messages have more than one frame_id ({quoted_frame_ids}), '
                           'so their readings have different axes')


def format_stamp(stamp: int) -> str:
    """Return a stamp in ns as seconds with all nine decimals, as rostopic echo shows it."""
    seconds, nanoseconds = divmod(stamp, NANOSECONDS_PER_SECOND)
    return f'{seconds}.{nanoseconds:09d}'


def is_of_an_imus_size(value: float) -> bool:
    """Return whether a value's size is at most LARGEST_READING."""
    return abs(value) <= LARGEST_READING


def first_sample_with_an_unexpected_reading(samples: Sequence[ImuSample], field: ImuField,
                                            is_expected: Callable[[float], bool]
                                            ) -> Optional[ImuSample]:
    """Return the first sample whose reading of the field has a value that isn't as expected."""
    for sample in samples:
        reading = getattr(sample, field.name)
        if reading is not None and not all(is_expected(value) for value in reading):
            return sample
    return None


def format_reading(sample: ImuSample, field: ImuField) -> str:
    """Return a sample's stamp and its reading of the field, as the start of a reason."""
    x, y, z = getattr(sample, field.name)
    return (f'the message stamped {format_stamp(sample.stamp)} has the {field.name} '
            f'({x}, {y}, {z})')


def check_finite_readings(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError for a measured reading with a NaN or an infinite value.

    One such value would make the axis's mean and variance NaN or infinite too.
    """
    for field in FIELDS:
        sample = first_sample_with_an_unexpected_reading(samples, field, math.isfinite)
        if sample is not None:
            raise MessageError(f"{format_reading(sample, field)}, which isn't finite")


def check_reading_size(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError for a measured reading with a value beyond LARGEST_READING.

    A value of that size isn't a measurement. It comes from damaged data, for example a changed
    byte in a number's exponent, and it would decide the axis's mean and variance alone.
    """
    for field in FIELDS:
        sample = first_sample_with_an_unexpected_reading(samples, field, is_of_an_imus_size)
        if sample is not None:
            raise MessageError(f'{format_reading(sample, field)}. The tool takes a value '
                               f'beyond {LARGEST_READING:g} {field.unit} as damaged data')


def check_reading_count(samples: Sequence[ImuSample]) -> None:
    """Raise a MessageError when no field has the two readings that a variance needs."""
    reading_counts = [len(field_readings(samples, field)) for field in FIELDS]
    if max(reading_counts) < 2:
        counts_by_field = ' and '.join(f'{field.name} has {count_of(reading_count, "reading")}'
                                       for field, reading_count in zip(FIELDS, reading_counts))
        raise MessageError(f'{counts_by_field}. At least one of them needs two')


SampleCheck = Callable[[Sequence[ImuSample]], None]
# In this order: the later checks and the statistics rely on the earlier ones
SAMPLE_CHECKS: Tuple[SampleCheck, ...] = (
    check_message_count,
    check_named_frame,
    check_one_frame,
    check_finite_readings,
    check_reading_size,
    check_reading_count,
)


def check_samples(samples: Sequence[ImuSample]) -> None:
    """Raise the MessageError of the first check in SAMPLE_CHECKS that the samples fail."""
    for check in SAMPLE_CHECKS:
        check(samples)


def read_samples(bag_path: str, topic: str) -> List[ImuSample]:
    """Return the samples of the topic's messages, in the order of recording, after all checks.

    An InputError about the messages names the bag and the topic.
    """
    try:
        with open_bag(bag_path) as bag:
            check_topic(bag, topic)
            samples = recorded_samples(bag, topic)
        check_samples(samples)
    except MessageError as error:
        raise InputError(f'{bag_path}, {topic}: {error}') from error
    return samples


def sample_variance(values: Sequence[float]) -> float:
    """Return the sample variance of the values.

    The squared deviations from the mean are added up and divided by the number of values minus
    one, not by their number: the deviations are measured from the values' own mean, which lies
    closer to them than the true mean. It needs at least two values.

    statistics.variance() gets the values as fractions, which hold each float exactly, so the
    mean, the deviations and their squares aren't rounded. With floats, the mean of two
    neighbouring floats rounds to one of them, which doubles their variance, and Python 3.8
    stops on deviations so small that their squares are zero.
    """
    return float(statistics.variance([Fraction(value) for value in values]))


def axis_statistics(values: Sequence[float]) -> AxisStatistics:
    """Return the mean, the sample variance and the number of distinct values of one axis."""
    return AxisStatistics(mean=statistics.mean(values),
                          variance=sample_variance(values),
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

    The samples are in the order of recording and have passed check_samples(): at least two of
    them, all in one frame.
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
        description="Print an IMU's bias and noise from a rosbag recorded at rest.",
        epilog=f'The report counts the intervals between the header stamps that are longer than '
               f'{LONG_INTERVAL // NANOSECONDS_PER_MILLISECOND} ms: one and a half periods of '
               "the 50 Hz at which the robot's firmware publishes. For a topic with another "
               "rate, compare the shortest and the longest interval with that rate's period.")
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
