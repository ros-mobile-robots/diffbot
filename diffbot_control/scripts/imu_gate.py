#!/usr/bin/env python3
"""Passes on only the IMU samples that the EKF can use, from imu/data_raw to imu/data_validated.

On the robot, the Teensy's firmware publishes imu/data_raw; in the simulation,
diffbot_gazebo/scripts/imu_raw_adapter.py does. robot_localization's EKF checks only that the
stamps of one topic don't go back: it fuses a sample that arrives late, or one with the same stamp
as the one before, by correcting its current estimate with it. A sample that sat in a buffer would
then count as the robot's current yaw rate. This node republishes, unchanged and with its stamp, a
sample that
- arrives while the ROS clock runs (in the simulation, /clock),
- is in the expected frame (the URDF's imu_link, which the EKF needs to turn the rate into the
  robot's frame),
- has finite values, and finite, positive variances for the angular velocity, which the EKF fuses,
- is at most max_age old and at most max_future ahead of the ROS clock,
- and is later than the last sample it passed on.
It never restamps a sample: an old sample is dropped, not made current.

When the ROS clock goes back (Gazebo's world was reset, or the computer's clock was corrected), the
order check starts over from the new time: otherwise every sample would count as out of order until
the clock reaches the old time again.

Its counts of passed and dropped samples, by reason, are on /diagnostics (rqt_robot_monitor shows
them), apart from the EKF's own diagnostics.

Parameters (private):
- frame_id: the frame the samples must have, default imu_link
- max_age: s, default 0.1, five samples at 50 Hz. A starting value, not measured yet: the delay of
  the robot's USB serial link and of rosserial's node isn't known
- max_future: s, default 0.02, one sample. The Teensy's clock follows the computer's through
  rosserial's time sync, which can be off by a few milliseconds; in the simulation, a node's clock
  lags Gazebo's sensors by up to one /clock message
"""

import collections
import math
import threading

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
import rospy
from sensor_msgs.msg import Imu

# Reasons for dropping a sample, as the diagnostics name them
NO_CLOCK = "no ROS clock"
WRONG_FRAME = "wrong frame"
NOT_FINITE = "not finite"
NO_VARIANCE = "angular velocity variance not usable"
FROM_THE_FUTURE = "from the future"
TOO_OLD = "too old"
NOT_NEWER = "not newer than the last one"


def usable_variances(covariance):
    """True for a 3x3 covariance (9 values, row by row) with finite values and positive variances."""
    return all(math.isfinite(value) for value in covariance) and all(covariance[i] > 0.0 for i in (0, 4, 8))


class ImuGate:
    """The checks, without ROS communication, so that the unit tests can call them directly."""

    def __init__(self, frame_id="imu_link", max_age=0.1, max_future=0.02):
        self.frame_id = frame_id
        self.max_age = rospy.Duration.from_sec(max_age)
        self.max_future = rospy.Duration.from_sec(max_future)
        self.last_stamp = None
        self.last_now = None
        self.last_passed = None  # ROS time at which the last sample passed

    def observe(self, now):
        """Notes the ROS time of a sample's arrival or a health query. When it's earlier than the
        last one noted, the clock went back: the order check and the health start over, so that
        only a sample that passes in the new time counts again."""
        if self.last_now is not None and now < self.last_now:
            self.last_stamp = None
            self.last_passed = None
        self.last_now = now

    def check(self, message, now):
        """Returns None if the EKF can use message at ROS time now, else the reason why not.

        A sample that passes becomes the last one, which the next one has to be later than.
        """
        if now.is_zero():
            return NO_CLOCK
        self.observe(now)
        if message.header.frame_id != self.frame_id:
            return WRONG_FRAME
        values = (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                  message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z)
        if not all(math.isfinite(value) for value in values):
            return NOT_FINITE
        if not usable_variances(message.angular_velocity_covariance):
            return NO_VARIANCE
        stamp = message.header.stamp
        if stamp > now + self.max_future:
            return FROM_THE_FUTURE
        if now - stamp > self.max_age:
            return TOO_OLD
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return NOT_NEWER
        self.last_stamp = stamp
        self.last_passed = now
        return None

    def seconds_since_pass(self, now):
        """How long ago (s of ROS time) the last sample passed; None if none passed since the
        clock last went back, or none at all. A sample that passed before the clock went back
        doesn't count: it says nothing about the IMU in the new time, even when the clock went
        back by less than the time since that sample."""
        self.observe(now)
        if self.last_passed is None:
            return None
        return (now - self.last_passed).to_sec()


class ImuGateNode:

    def __init__(self):
        self.gate = ImuGate(rospy.get_param("~frame_id", "imu_link"),
                            rospy.get_param("~max_age", 0.1),
                            rospy.get_param("~max_future", 0.02))
        self.passed = 0
        self.dropped = collections.Counter()
        self.reported_drops = 0
        # The subscriber and the diagnostics timer run in different threads. Both read the clock
        # inside this lock, so the gate sees the times in the order they were read; otherwise two
        # nearly simultaneous calls could look like a clock that went back
        self.lock = threading.Lock()
        self.publisher = rospy.Publisher("imu/data_validated", Imu, queue_size=10)
        rospy.Subscriber("imu/data_raw", Imu, self.callback, queue_size=10)
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("imu")
        self.updater.add("IMU gate (imu/data_raw to imu/data_validated)", self.diagnose)
        # Once per second of ROS time. reset: the timer goes on when the clock goes back
        # (Updater.update() would wait until the clock has caught up)
        rospy.Timer(rospy.Duration(1.0), lambda event: self.updater.force_update(), reset=True)

    def callback(self, message):
        with self.lock:
            now = rospy.Time.now()
            reason = self.gate.check(message, now)
        if reason is None:
            self.passed += 1
            self.publisher.publish(message)
        else:
            self.dropped[reason] += 1
            rospy.logwarn_throttle(5.0, "IMU sample dropped: {} (stamp {:.3f} s, ROS time {:.3f} s)".format(
                reason, message.header.stamp.to_sec(), now.to_sec()))

    def diagnose(self, status):
        status.add("Samples passed on", self.passed)
        for reason in sorted(self.dropped):
            status.add("Dropped, " + reason, self.dropped[reason])
        dropped = sum(self.dropped.values())
        with self.lock:
            since_pass = self.gate.seconds_since_pass(rospy.Time.now())
        if since_pass is None:
            status.summary(DiagnosticStatus.ERROR, "no IMU sample passed on yet, or since the clock went back")
        elif since_pass > 1.0:
            status.summary(DiagnosticStatus.ERROR, "no IMU sample passed on for {:.1f} s".format(since_pass))
        elif dropped > self.reported_drops:
            status.summary(DiagnosticStatus.WARN, "dropped {} samples since the last report".format(
                dropped - self.reported_drops))
        else:
            status.summary(DiagnosticStatus.OK, "passing samples on")
        self.reported_drops = dropped
        return status


if __name__ == "__main__":
    rospy.init_node("imu_gate")
    ImuGateNode()
    rospy.spin()
