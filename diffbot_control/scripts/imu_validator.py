#!/usr/bin/env python3
"""Forward an IMU sample to the EKF only if its frame, values, variances and stamp are valid.

The IMU sample validator checks the timing and the input contract that the EKF of
config/ekf.yaml needs, and forwards each valid sample unchanged, with its stamp. It doesn't
validate every field of sensor_msgs/Imu.

On the robot, the Teensy's firmware publishes imu/data_raw; in the simulation,
diffbot_gazebo/scripts/imu_raw_adapter.py does. The EKF's per-input timestamp-order check does
not enforce freshness or reject equal timestamps: robot_localization 2.7.7 on Noetic, with
smooth_lagged_data off as here, takes a sample whose stamp isn't older than the last one on its
topic, and fuses a sample that arrives late, or one with the same stamp as the one before, into
its current estimate. A sample that sat in a buffer would then count as the robot's current yaw
rate. This node forwards a sample only if
- the ROS time is nonzero (in the simulation, a /clock message has arrived),
- its frame is the expected one (the URDF's imu_link, which the EKF needs to turn the rate into
  the robot's frame),
- the three components of its angular velocity and of its linear acceleration are finite,
- the nine values of its angular velocity covariance are finite, and the diagonal ones (indices
  0, 4 and 8), the variances that the EKF weighs the angular velocity with, are positive,
- its stamp is at most max_age before and at most max_future after the ROS time,
- and its stamp is later than the stamp of the last sample it forwarded.
It doesn't check the orientation, the linear acceleration's covariance, or whether a covariance
matrix is positive definite. It doesn't compare a sample with the EKF's estimate either: that is
a statistical test, robot_localization's rejection thresholds such as
imu0_twist_rejection_threshold, which config/ekf.yaml doesn't set. robot_localization 2.7.7 reads
that name for the IMU's angular velocity, although its parameter documentation lists
imu0_angular_velocity_rejection_threshold. It never restamps a sample: an old sample is dropped,
not made current.

When the ROS clock goes back (Gazebo's world was reset, or the computer's clock was corrected),
the order check starts over from the new time: otherwise every sample would count as out of order
until the clock reaches the old time again.

Its counts of forwarded and dropped samples, by reason, are on /diagnostics (rqt_robot_monitor
shows them), apart from the EKF's own diagnostics.

Parameters (private):
- frame_id: the frame the samples must have, default imu_link
- max_age: s, default 0.1, five samples at 50 Hz. An initial policy value, not a standard IMU
  limit, until the latency of the robot's USB serial link and of rosserial's node is measured
- max_future: s, default 0.02, one sample. Also an initial policy value awaiting latency
  measurements: the Teensy's clock follows the computer's through rosserial's time sync, which
  can be off by a few milliseconds; in the simulation, a node's clock lags Gazebo's sensors by up
  to one /clock message
"""

import collections
import enum
import math
import threading
from typing import Callable
from typing import Counter
from typing import Dict
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import rospy
import rospy.timer
from sensor_msgs.msg import Imu

INPUT_TOPIC = 'imu/data_raw'
OUTPUT_TOPIC = 'imu/data_validated'
QUEUE_SIZE = 10  # messages, 0.2 s at the IMU's 50 Hz

# The parameters' defaults; the module's docstring explains them
DEFAULT_FRAME_ID = 'imu_link'
DEFAULT_MAX_AGE = 0.1  # s
DEFAULT_MAX_FUTURE = 0.02  # s

# The diagonal of a 3x3 covariance, stored as 9 values row by row as in sensor_msgs/Imu: the
# variances
VARIANCE_INDICES = (0, 4, 8)

DIAGNOSTICS_NAME = 'IMU sample validator (imu/data_raw to imu/data_validated)'
HARDWARE_ID = 'imu'
FORWARDED_KEY = 'Samples forwarded'
DROPPED_KEY_PREFIX = 'Dropped, '
DIAGNOSTICS_PERIOD = 1.0  # s of ROS time
# s of ROS time without a forwarded sample, after which the diagnostics report an error
SILENCE_LIMIT = 1.0
WARNING_PERIOD = 5.0  # s, at most one warning about a dropped sample in this time


@enum.unique
class DropReason(enum.Enum):
    """Why the validator drops a sample. The values are the texts in the log and diagnostics."""

    NO_CLOCK = 'no ROS clock'
    WRONG_FRAME = 'wrong frame'
    NOT_FINITE = 'not finite'
    INVALID_ANGULAR_VELOCITY_COVARIANCE = 'invalid angular velocity covariance'
    FROM_THE_FUTURE = 'from the future'
    TOO_OLD = 'too old'
    NOT_NEWER = 'not newer than the last one'


# A check of one sample at a ROS time: None if the sample is valid for this check, else the
# reason for dropping it
SampleCheck = Callable[[Imu, rospy.Time], Optional[DropReason]]


class Health(NamedTuple):
    """The level and the message of the validator's diagnostics status."""

    level: int  # a level of diagnostic_msgs/DiagnosticStatus
    message: str


class DiagnosticSnapshot(NamedTuple):
    """The validator's numbers for one diagnostics report, all from one moment."""

    forwarded_count: int
    drops_by_reason: Dict[DropReason, int]
    new_drop_count: int  # samples dropped since the report before
    # As ImuValidator.seconds_since_last_valid_sample() returns it
    seconds_since_last_valid_sample: Optional[float]


def health(seconds_since_last_valid_sample: Optional[float], new_drop_count: int) -> Health:
    """Return the diagnostics level and message for the validator's state.

    seconds_since_last_valid_sample is what ImuValidator.seconds_since_last_valid_sample()
    returns, None without a valid sample. new_drop_count is the number of samples dropped since
    the report before. Silence is an error and comes before the warning about new drops.
    """
    if seconds_since_last_valid_sample is None:
        return Health(DiagnosticStatus.ERROR,
                      'no IMU sample forwarded yet, or since the clock went back')
    if seconds_since_last_valid_sample > SILENCE_LIMIT:
        return Health(DiagnosticStatus.ERROR,
                      'no IMU sample forwarded for {:.1f} s'.format(
                          seconds_since_last_valid_sample))
    if new_drop_count > 0:
        return Health(DiagnosticStatus.WARN,
                      'dropped {} samples since the last report'.format(new_drop_count))
    return Health(DiagnosticStatus.OK, 'forwarding samples')


class SampleCounts:
    """The numbers of forwarded and dropped samples, without ROS communication."""

    def __init__(self) -> None:
        self.forwarded_count = 0
        self.drops_by_reason: Counter[DropReason] = collections.Counter()
        # The number of dropped samples that the last snapshot counted
        self.drop_count_at_last_snapshot = 0

    def count_forwarded(self) -> None:
        """Count one forwarded sample."""
        self.forwarded_count += 1

    def count_dropped(self, reason: DropReason) -> None:
        """Count one sample dropped for reason."""
        self.drops_by_reason[reason] += 1

    def take_snapshot(self,
                      seconds_since_last_valid_sample: Optional[float]) -> DiagnosticSnapshot:
        """Return the numbers for one report and start counting new drops for the next one.

        A dropped sample is new in exactly one snapshot, whatever that report's level is.
        """
        drop_count = sum(self.drops_by_reason.values())
        new_drop_count = drop_count - self.drop_count_at_last_snapshot
        self.drop_count_at_last_snapshot = drop_count
        return DiagnosticSnapshot(self.forwarded_count, dict(self.drops_by_reason),
                                  new_drop_count, seconds_since_last_valid_sample)


def has_finite_covariance_and_positive_variances(covariance: Sequence[float]) -> bool:
    """Return whether a 3x3 covariance has finite values and positive variances.

    The covariance has 9 values, row by row, as in sensor_msgs/Imu. The variances are its
    diagonal. The function doesn't check whether the matrix is positive definite.
    """
    return (all(math.isfinite(value) for value in covariance)
            and all(covariance[index] > 0.0 for index in VARIANCE_INDICES))


class ImuValidator:
    """The checks of a sample and the history they need, without ROS communication.

    The node calls validate_and_record() for each sample and seconds_since_last_valid_sample()
    for the diagnostics. Both get the ROS time as an argument, so the unit tests can call them
    directly. The other methods and the attributes are for this class itself.
    """

    def __init__(self, frame_id: str = DEFAULT_FRAME_ID, max_age: float = DEFAULT_MAX_AGE,
                 max_future: float = DEFAULT_MAX_FUTURE) -> None:
        self.frame_id = frame_id
        self.max_age = rospy.Duration.from_sec(max_age)
        self.max_future = rospy.Duration.from_sec(max_future)
        # The stamp of the last valid sample, and the ROS time at which it was recorded
        self.last_valid_stamp: Optional[rospy.Time] = None
        self.last_valid_at: Optional[rospy.Time] = None
        # The latest ROS time that observe_time() recorded
        self.last_observed_time: Optional[rospy.Time] = None
        # In this order; the first check that fails gives the reason
        self.sample_checks: Tuple[SampleCheck, ...] = (
            self.check_frame,
            self.check_finite_values,
            self.check_angular_velocity_covariance,
            self.check_not_from_the_future,
            self.check_not_too_old,
            self.check_newer_than_the_last,
        )

    def observe_time(self, now: rospy.Time) -> None:
        """Record the ROS time now, and forget the last valid sample if the clock went back.

        now is the ROS time of a sample's arrival or of a diagnostics report. When now is earlier
        than the time recorded before, the clock went back: the order check and the time since
        the last valid sample start over, so that only a sample that is valid in the new time
        counts again.
        """
        if self.last_observed_time is not None and now < self.last_observed_time:
            self.last_valid_stamp = None
            self.last_valid_at = None
        self.last_observed_time = now

    def validate_and_record(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return the reason for dropping message at ROS time now, or record it and return None.

        None means that the sample is valid. The method then records it as the last valid one:
        the next sample's stamp has to be later, and seconds_since_last_valid_sample() counts
        from now. So the same sample is valid only once. For an invalid sample, the method
        records only the ROS time (observe_time()).
        """
        # Before observe_time(): a ROS time of 0 means no clock yet, not a clock that went back
        if now.is_zero():
            return DropReason.NO_CLOCK
        self.observe_time(now)
        for sample_check in self.sample_checks:
            reason = sample_check(message, now)
            if reason is not None:
                return reason
        self.last_valid_stamp = message.header.stamp
        self.last_valid_at = now
        return None

    def check_frame(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return WRONG_FRAME unless the sample is in the expected frame."""
        return None if message.header.frame_id == self.frame_id else DropReason.WRONG_FRAME

    def check_finite_values(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return NOT_FINITE unless the angular velocity and linear acceleration are finite."""
        components = (message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z, message.linear_acceleration.x,
                      message.linear_acceleration.y, message.linear_acceleration.z)
        return None if all(math.isfinite(value) for value in components) else DropReason.NOT_FINITE

    def check_angular_velocity_covariance(self, message: Imu,
                                          now: rospy.Time) -> Optional[DropReason]:
        """Return INVALID_ANGULAR_VELOCITY_COVARIANCE unless the covariance is valid.

        Valid means finite values and positive variances.
        """
        if has_finite_covariance_and_positive_variances(message.angular_velocity_covariance):
            return None
        return DropReason.INVALID_ANGULAR_VELOCITY_COVARIANCE

    def check_not_from_the_future(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return FROM_THE_FUTURE if the stamp is more than max_future after now."""
        return DropReason.FROM_THE_FUTURE if message.header.stamp > now + self.max_future else None

    def check_not_too_old(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return TOO_OLD if the stamp is more than max_age before now."""
        return DropReason.TOO_OLD if now - message.header.stamp > self.max_age else None

    def check_newer_than_the_last(self, message: Imu, now: rospy.Time) -> Optional[DropReason]:
        """Return NOT_NEWER unless the stamp is later than the last valid sample's."""
        if (self.last_valid_stamp is not None
                and message.header.stamp <= self.last_valid_stamp):
            return DropReason.NOT_NEWER
        return None

    def seconds_since_last_valid_sample(self, now: rospy.Time) -> Optional[float]:
        """Return the ROS time in s since the last valid sample was recorded, at ROS time now.

        The time counts from the ROS time of that sample's validate_and_record() call, not from
        its stamp. None if no sample was valid since the clock last went back, or none at all. A
        sample from before the clock went back doesn't count: it says nothing about the IMU in
        the new time, even when the clock went back by less than the time since that sample.
        The method also records now (observe_time()), so it notices a clock that went back
        while no sample arrived.
        """
        self.observe_time(now)
        if self.last_valid_at is None:
            return None
        return (now - self.last_valid_at).to_sec()


class ImuValidatorNode:
    """The ROS node: forwards the valid samples, drops the others and reports its health."""

    def __init__(self) -> None:
        self.validator = ImuValidator(rospy.get_param('~frame_id', DEFAULT_FRAME_ID),
                                      rospy.get_param('~max_age', DEFAULT_MAX_AGE),
                                      rospy.get_param('~max_future', DEFAULT_MAX_FUTURE))
        self.counts = SampleCounts()
        # The subscriber and the diagnostics timer run in different threads, and a second
        # publisher of the input would add another. Inside this lock they read the clock, check
        # or query the validator, count, and publish. So the validator sees the times in the
        # order they were read (two nearly simultaneous calls could otherwise look like a clock
        # that went back), the samples leave in the order of their checks, and a diagnostics
        # report has numbers from one moment
        self.lock = threading.Lock()
        self.publisher = rospy.Publisher(OUTPUT_TOPIC, Imu, queue_size=QUEUE_SIZE)
        rospy.Subscriber(INPUT_TOPIC, Imu, self.on_sample, queue_size=QUEUE_SIZE)
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID(HARDWARE_ID)
        self.updater.add(DIAGNOSTICS_NAME, self.diagnose)
        # reset: the timer goes on when the clock goes back (Updater.update() would wait until the
        # clock has caught up)
        rospy.Timer(rospy.Duration(DIAGNOSTICS_PERIOD), self.publish_diagnostics, reset=True)

    def on_sample(self, message: Imu) -> None:
        """Forward message if it is valid, else drop it, count it by its reason and warn."""
        with self.lock:
            now = rospy.Time.now()
            reason = self.validator.validate_and_record(message, now)
            if reason is None:
                self.counts.count_forwarded()
                # The publisher has a queue (queue_size), so this call doesn't wait for the
                # subscribers
                self.publisher.publish(message)
            else:
                self.counts.count_dropped(reason)
        if reason is not None:
            warning = 'IMU sample dropped: {} (stamp {:.3f} s, ROS time {:.3f} s)'.format(
                reason.value, message.header.stamp.to_sec(), now.to_sec())
            rospy.logwarn_throttle(WARNING_PERIOD, warning)

    def publish_diagnostics(self, event: rospy.timer.TimerEvent) -> None:
        """Publish the diagnostics now, also when the clock went back."""
        self.updater.force_update()

    def diagnose(self, status: diagnostic_updater.DiagnosticStatusWrapper
                 ) -> diagnostic_updater.DiagnosticStatusWrapper:
        """Fill status with the sample counts and a summary of the validator's health."""
        snapshot = self.take_snapshot()
        status.add(FORWARDED_KEY, snapshot.forwarded_count)
        for reason in sorted(snapshot.drops_by_reason, key=lambda reason: reason.value):
            status.add(DROPPED_KEY_PREFIX + reason.value, snapshot.drops_by_reason[reason])
        level, message = health(snapshot.seconds_since_last_valid_sample, snapshot.new_drop_count)
        status.summary(level, message)
        return status

    def take_snapshot(self) -> DiagnosticSnapshot:
        """Return the numbers for one diagnostics report, read inside the lock."""
        with self.lock:
            now = rospy.Time.now()
            return self.counts.take_snapshot(self.validator.seconds_since_last_valid_sample(now))


def main() -> None:
    """Run the node until ROS shuts down."""
    rospy.init_node('imu_validator')
    ImuValidatorNode()
    rospy.spin()


if __name__ == '__main__':
    main()
