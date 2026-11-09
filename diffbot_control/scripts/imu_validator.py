#!/usr/bin/env python3
"""Forward only the IMU samples that the EKF can use, from imu/data_raw to imu/data_validated.

The IMU sample validator checks the timing and the input contract that the EKF of
config/ekf.yaml needs, and forwards each accepted sample unchanged, with its stamp. It doesn't
validate every field of sensor_msgs/Imu.

On the robot, the Teensy's firmware publishes imu/data_raw; in the simulation,
diffbot_gazebo/scripts/imu_raw_adapter.py does. The EKF's per-input timestamp-order check does
not enforce freshness or reject equal timestamps: robot_localization 2.7.7 on Noetic, with
smooth_lagged_data off as here, accepts a sample whose stamp isn't older than the last one on its
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
import math
import threading
from typing import Callable
from typing import Counter
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

# Reasons for dropping a sample, as the diagnostics name them
NO_CLOCK = 'no ROS clock'
WRONG_FRAME = 'wrong frame'
NOT_FINITE = 'not finite'
NO_VARIANCE = 'angular velocity variance not usable'
FROM_THE_FUTURE = 'from the future'
TOO_OLD = 'too old'
NOT_NEWER = 'not newer than the last one'

DIAGNOSTICS_NAME = 'IMU sample validator (imu/data_raw to imu/data_validated)'
HARDWARE_ID = 'imu'
FORWARDED_KEY = 'Samples passed on'
DROPPED_KEY_PREFIX = 'Dropped, '
DIAGNOSTICS_PERIOD = 1.0  # s of ROS time
# s of ROS time without a forwarded sample, after which the diagnostics report an error
SILENCE_LIMIT = 1.0
WARNING_PERIOD = 5.0  # s, at most one warning about a dropped sample in this time

# A check of one sample at a ROS time: None if the sample passes, else the reason for dropping it
SampleCheck = Callable[[Imu, rospy.Time], Optional[str]]


def usable_variances(covariance: Sequence[float]) -> bool:
    """Return whether a 3x3 covariance has finite values and positive variances.

    The covariance has 9 values, row by row, as in sensor_msgs/Imu.
    """
    return (all(math.isfinite(value) for value in covariance)
            and all(covariance[index] > 0.0 for index in VARIANCE_INDICES))


class ImuValidator:
    """The checks, without ROS communication, so that the unit tests can call them directly."""

    def __init__(self, frame_id: str = DEFAULT_FRAME_ID, max_age: float = DEFAULT_MAX_AGE,
                 max_future: float = DEFAULT_MAX_FUTURE) -> None:
        self.frame_id = frame_id
        self.max_age = rospy.Duration.from_sec(max_age)
        self.max_future = rospy.Duration.from_sec(max_future)
        self.last_forwarded_stamp: Optional[rospy.Time] = None
        self.last_noted_time: Optional[rospy.Time] = None  # the latest ROS time observe() noted
        # The ROS time at which the last sample was forwarded
        self.last_forward_time: Optional[rospy.Time] = None
        # In this order; the first check that fails gives the reason
        self.sample_checks: Tuple[SampleCheck, ...] = (
            self.check_frame,
            self.check_finite_values,
            self.check_angular_velocity_variances,
            self.check_not_from_the_future,
            self.check_not_too_old,
            self.check_newer_than_the_last,
        )

    def observe(self, now: rospy.Time) -> None:
        """Note the ROS time of a sample's arrival or a health query.

        When it's earlier than the last one noted, the clock went back: the order check and the
        health start over, so that only a sample forwarded in the new time counts again.
        """
        if self.last_noted_time is not None and now < self.last_noted_time:
            self.last_forwarded_stamp = None
            self.last_forward_time = None
        self.last_noted_time = now

    def check(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return None if the EKF can use message at ROS time now, else the reason why not.

        A sample that passes becomes the last forwarded one, which the next one has to be later
        than.
        """
        # Before observe(): a ROS time of 0 means no clock yet, not a clock that went back
        if now.is_zero():
            return NO_CLOCK
        self.observe(now)
        for sample_check in self.sample_checks:
            reason = sample_check(message, now)
            if reason is not None:
                return reason
        self.last_forwarded_stamp = message.header.stamp
        self.last_forward_time = now
        return None

    def check_frame(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return WRONG_FRAME unless the sample is in the expected frame."""
        return None if message.header.frame_id == self.frame_id else WRONG_FRAME

    def check_finite_values(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return NOT_FINITE unless the angular velocity and linear acceleration are finite."""
        components = (message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z, message.linear_acceleration.x,
                      message.linear_acceleration.y, message.linear_acceleration.z)
        return None if all(math.isfinite(value) for value in components) else NOT_FINITE

    def check_angular_velocity_variances(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return NO_VARIANCE unless the angular velocity's covariance is usable."""
        return None if usable_variances(message.angular_velocity_covariance) else NO_VARIANCE

    def check_not_from_the_future(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return FROM_THE_FUTURE if the stamp is more than max_future after now."""
        return FROM_THE_FUTURE if message.header.stamp > now + self.max_future else None

    def check_not_too_old(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return TOO_OLD if the stamp is more than max_age before now."""
        return TOO_OLD if now - message.header.stamp > self.max_age else None

    def check_newer_than_the_last(self, message: Imu, now: rospy.Time) -> Optional[str]:
        """Return NOT_NEWER unless the stamp is later than the last forwarded sample's."""
        if (self.last_forwarded_stamp is not None
                and message.header.stamp <= self.last_forwarded_stamp):
            return NOT_NEWER
        return None

    def seconds_since_pass(self, now: rospy.Time) -> Optional[float]:
        """Return how long ago, in s of ROS time, the last sample was forwarded.

        None if none was forwarded since the clock last went back, or none at all. A sample
        forwarded before the clock went back doesn't count: it says nothing about the IMU in the
        new time, even when the clock went back by less than the time since that sample.
        """
        self.observe(now)
        if self.last_forward_time is None:
            return None
        return (now - self.last_forward_time).to_sec()


class ImuValidatorNode:
    """The ROS node: forwards the samples that pass the checks and reports its health."""

    def __init__(self) -> None:
        self.validator = ImuValidator(rospy.get_param('~frame_id', DEFAULT_FRAME_ID),
                                      rospy.get_param('~max_age', DEFAULT_MAX_AGE),
                                      rospy.get_param('~max_future', DEFAULT_MAX_FUTURE))
        self.forwarded_count = 0
        self.drops_by_reason: Counter[str] = collections.Counter()
        self.drops_at_last_report = 0
        # The subscriber and the diagnostics timer run in different threads. Both read the clock
        # inside this lock, so the validator sees the times in the order they were read;
        # otherwise two nearly simultaneous calls could look like a clock that went back
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
        """Forward message if it passes the checks, else count it by reason and warn."""
        with self.lock:
            now = rospy.Time.now()
            reason = self.validator.check(message, now)
        if reason is None:
            self.forwarded_count += 1
            self.publisher.publish(message)
        else:
            self.drops_by_reason[reason] += 1
            warning = 'IMU sample dropped: {} (stamp {:.3f} s, ROS time {:.3f} s)'.format(
                reason, message.header.stamp.to_sec(), now.to_sec())
            rospy.logwarn_throttle(WARNING_PERIOD, warning)

    def publish_diagnostics(self, event: rospy.timer.TimerEvent) -> None:
        """Publish the diagnostics now, also when the clock went back."""
        self.updater.force_update()

    def diagnose(self, status: diagnostic_updater.DiagnosticStatusWrapper
                 ) -> diagnostic_updater.DiagnosticStatusWrapper:
        """Fill status with the sample counts and a summary of the validator's health."""
        self.add_counts(status)
        self.summarize(status)
        return status

    def add_counts(self, status: diagnostic_updater.DiagnosticStatusWrapper) -> None:
        """Add the number of forwarded samples, and of dropped ones by reason, to status."""
        status.add(FORWARDED_KEY, self.forwarded_count)
        for reason in sorted(self.drops_by_reason):
            status.add(DROPPED_KEY_PREFIX + reason, self.drops_by_reason[reason])

    def summarize(self, status: diagnostic_updater.DiagnosticStatusWrapper) -> None:
        """Set the level and message of status from the forwarded and the new dropped samples."""
        drop_count = sum(self.drops_by_reason.values())
        with self.lock:
            seconds_since_forward = self.validator.seconds_since_pass(rospy.Time.now())
        if seconds_since_forward is None:
            status.summary(DiagnosticStatus.ERROR,
                           'no IMU sample passed on yet, or since the clock went back')
        elif seconds_since_forward > SILENCE_LIMIT:
            status.summary(DiagnosticStatus.ERROR,
                           'no IMU sample passed on for {:.1f} s'.format(seconds_since_forward))
        elif drop_count > self.drops_at_last_report:
            status.summary(DiagnosticStatus.WARN,
                           'dropped {} samples since the last report'.format(
                               drop_count - self.drops_at_last_report))
        else:
            status.summary(DiagnosticStatus.OK, 'passing samples on')
        self.drops_at_last_report = drop_count


def main() -> None:
    """Run the node until ROS shuts down."""
    rospy.init_node('imu_validator')
    ImuValidatorNode()
    rospy.spin()


if __name__ == '__main__':
    main()
