#!/usr/bin/env python3
"""Feeds the EKF of ekf.launch (imu_gate.py and robot_localization) synthetic wheel odometry and IMU
samples on a stepped clock, and checks its estimate of the yaw rate (ekf_synthetic.test).

The test publishes /clock itself (use_sim_time) and advances it in 20 ms steps, the period of the
controller's odometry and of the IMU. Each step publishes that step's inputs, stamped with the
current time, waits on the wall clock until the IMU gate's output reaches the test (the EKF gets
it at the same time) and 20 ms more, and then advances the clock, which lets the EKF's 50 Hz
update process them; the step ends with an EKF output for that step, waited for with a time
limit. These windows don't prove that the EKF's callbacks ran before the clock moved, but they
make it very likely, and the checks don't depend on it: a late input is fused one step later.
Both inputs show in the estimate: the forward speed comes only from the wheels, the yaw rate
mostly from the IMU.

The scenarios run one after the other on one filter, from its start:
1. Fusion: the wheels report 0.4 rad/s, as with a wheel separation that is too large, the IMU
   0.5 rad/s with a much smaller variance. The estimate follows the IMU.
2. IMU samples 0.5 s old, reporting 2.0 rad/s, while the wheels go on: the gate drops them, and
   the estimate goes to the wheels' 0.4 rad/s, as when the IMU stops.
3. to 5. Valid samples again, each followed by one reporting 2.0 rad/s that has the same stamp
   (3), a stamp 0.5 s in the future (4), or a stamp older than the valid one (5). The gate drops
   those, and the estimate stays at the valid samples' 0.5 rad/s; one of them passing would pull
   it far up.
6. The clock goes back, as when Gazebo's world is reset: the gate and the EKF start over from the
   new time (reset_on_time_jump), and the estimate follows the IMU again.
At the end, the gate's diagnostics have to count each passed and dropped sample.
"""

import collections
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rospy
import rostest
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
import tf2_ros

STEP = rospy.Duration(0, 20000000)  # 20 ms
START = rospy.Time(100, 0)
RESTART = rospy.Time(50, 0)  # where the clock goes back to in scenario 6
WAIT = 5.0  # s of wall time for one step's messages
DELIVERY = 0.02  # s of wall time for the inputs to arrive before the clock moves on

SPEED = 0.1  # m/s, forward, from the wheels
WHEEL_RATE = 0.4  # rad/s
IMU_RATE = 0.5  # rad/s
BAD_RATE = 2.0  # rad/s, in the samples the gate has to drop
# The inputs' variances, as the robot's: diffbot_control.yaml's twist_covariance_diagonal and
# diffbot_base_config.h's IMU variances
WHEEL_VARIANCES = [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
IMU_ANGULAR_VELOCITY_VARIANCE = 4.0e-4
IMU_LINEAR_ACCELERATION_VARIANCE = 1.0e-2

# The gate's reasons, as its diagnostics name them (scripts/imu_gate.py)
TOO_OLD = "too old"
NOT_NEWER = "not newer than the last one"
FROM_THE_FUTURE = "from the future"


def key(stamp):
    return (stamp.secs, stamp.nsecs)


def wheel_odometry(stamp, yaw_rate):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "odom"
    message.child_frame_id = "base_footprint"
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = SPEED
    message.twist.twist.angular.z = yaw_rate
    for i, variance in enumerate(WHEEL_VARIANCES):
        message.pose.covariance[7 * i] = variance
        message.twist.covariance[7 * i] = variance
    return message


def imu_sample(stamp, yaw_rate):
    """A sample as the firmware publishes it, in DiffBot's imu_link (aligned with base_footprint)."""
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = "imu_link"
    message.orientation_covariance[0] = -1.0
    message.angular_velocity.z = yaw_rate
    message.linear_acceleration.z = 9.81
    for i in (0, 4, 8):
        message.angular_velocity_covariance[i] = IMU_ANGULAR_VELOCITY_VARIANCE
        message.linear_acceleration_covariance[i] = IMU_LINEAR_ACCELERATION_VARIANCE
    return message


def wait_for(condition, message):
    deadline = time.monotonic() + WAIT
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError(message)
        time.sleep(0.001)


class TestEkfSynthetic(unittest.TestCase):
    """The scenarios share one filter and run in the order of their names."""

    @classmethod
    def setUpClass(cls):
        cls.now = START
        cls.passed = set()  # stamps of the samples the gate passed on
        cls.outputs = []
        cls.diagnostics = []
        cls.expected_passes = 0
        cls.expected_drops = collections.Counter()
        cls.clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
        cls.wheels = rospy.Publisher("/diffbot/mobile_base_controller/odom", Odometry, queue_size=100)
        cls.imu = rospy.Publisher("/diffbot/imu/data_raw", Imu, queue_size=100)
        cls.validated = rospy.Subscriber("/diffbot/imu/data_validated", Imu,
                                         lambda m: cls.passed.add(key(m.header.stamp)), queue_size=100)
        cls.filtered = rospy.Subscriber("/diffbot/odometry/filtered", Odometry, cls.outputs.append, queue_size=100)
        rospy.Subscriber("/diagnostics", DiagnosticArray, cls.diagnostics.append, queue_size=100)
        cls.clock.publish(Clock(cls.now))
        buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(buffer)
        # Before the first input: the EKF and the gate listen, and the production URDF's imu_link
        # transform is there. A paused clock doesn't advance ROS-time waits, so these wait on the
        # wall clock
        wait_for(lambda: (cls.wheels.get_num_connections() > 0 and cls.imu.get_num_connections() > 0
                          and cls.validated.get_num_connections() > 0 and cls.filtered.get_num_connections() > 0),
                 "the EKF and the gate didn't connect")
        wait_for(lambda: buffer.can_transform("base_footprint", "imu_link", rospy.Time(0)),
                 "no transform from imu_link to base_footprint (robot_state_publisher)")
        time.sleep(1.0)  # the EKF's own TF listener gets the same latched transform

    @classmethod
    def step(cls, wheel_rate, imu=(), output=True):
        """Publishes one step's inputs, stamped with the current time, then advances the clock.

        imu: (stamp offset in s, yaw rate, reason) per sample; reason None for a sample the gate
        has to pass on, else the reason it has to drop it for. output: True if the EKF has to
        publish for this step, False if it may not. Returns the EKF's output, or None.
        """
        stamp = cls.now
        received = len(cls.outputs)
        cls.wheels.publish(wheel_odometry(stamp, wheel_rate))
        passing = []
        for offset, yaw_rate, reason in imu:
            sample_stamp = stamp + rospy.Duration.from_sec(offset)
            cls.imu.publish(imu_sample(sample_stamp, yaw_rate))
            if reason is None:
                passing.append(key(sample_stamp))
                cls.expected_passes += 1
            else:
                cls.expected_drops[reason] += 1
        wait_for(lambda: all(k in cls.passed for k in passing),
                 "the gate didn't pass on the IMU samples of the step at {:.2f} s".format(stamp.to_sec()))
        time.sleep(DELIVERY)
        cls.now = stamp + STEP
        cls.clock.publish(Clock(cls.now))

        def new_output():
            return any(m.header.stamp >= stamp for m in cls.outputs[received:])
        if output:
            wait_for(new_output, "no EKF output for the step at {:.2f} s".format(stamp.to_sec()))
        else:
            time.sleep(10 * DELIVERY)
            if not new_output():
                return None
        return [m for m in cls.outputs[received:] if m.header.stamp >= stamp][-1]

    @classmethod
    def run_steps(cls, count, wheel_rate, imu=()):
        return [cls.step(wheel_rate, imu) for _ in range(count)]

    def assertYawRate(self, output, expected, delta, what):
        rate = output.twist.twist.angular.z
        self.assertAlmostEqual(rate, expected, delta=delta,
                               msg="{}: estimated {:.4f} rad/s, expected {} ± {}".format(what, rate, expected, delta))

    def assertStaysAtMost(self, outputs, limit, what):
        highest = max(m.twist.twist.angular.z for m in outputs)
        self.assertLessEqual(highest, limit, "{}: the estimate went up to {:.4f} rad/s".format(what, highest))

    def test_1_fusion(self):
        outputs = self.run_steps(150, WHEEL_RATE, [(0.0, IMU_RATE, None)])
        last = outputs[-1]
        print("fusion: wheels {} rad/s, IMU {} rad/s, estimate {:.4f} rad/s, forward {:.4f} m/s".format(
            WHEEL_RATE, IMU_RATE, last.twist.twist.angular.z, last.twist.twist.linear.x))
        self.assertEqual(last.header.frame_id, "odom")
        self.assertEqual(last.child_frame_id, "base_footprint")
        self.assertAlmostEqual(last.twist.twist.linear.x, SPEED, delta=0.002, msg="the forward speed comes from the wheels")
        self.assertYawRate(last, IMU_RATE, 0.01, "the IMU's variance is far smaller than the wheels'")

    def test_2_stale_samples_are_dropped(self):
        outputs = self.run_steps(50, WHEEL_RATE, [(-0.5, BAD_RATE, TOO_OLD)])
        print("stale IMU: estimate from {:.4f} to {:.4f} rad/s".format(
            outputs[0].twist.twist.angular.z, outputs[-1].twist.twist.angular.z))
        self.assertStaysAtMost(outputs, IMU_RATE + 0.01, "stale samples reporting 2.0 rad/s")
        self.assertYawRate(outputs[-1], WHEEL_RATE, 0.02, "without IMU samples, the estimate follows the wheels")

    def test_3_duplicate_stamps_are_dropped(self):
        outputs = self.run_steps(50, WHEEL_RATE, [(0.0, IMU_RATE, None), (0.0, BAD_RATE, NOT_NEWER)])
        self.assertStaysAtMost(outputs, IMU_RATE + 0.01, "samples with a repeated stamp")
        self.assertYawRate(outputs[-1], IMU_RATE, 0.01, "valid samples again")

    def test_4_future_stamps_are_dropped(self):
        outputs = self.run_steps(50, WHEEL_RATE, [(0.0, IMU_RATE, None), (0.5, BAD_RATE, FROM_THE_FUTURE)])
        self.assertStaysAtMost(outputs, IMU_RATE + 0.01, "samples from the future")
        self.assertYawRate(outputs[-1], IMU_RATE, 0.01, "valid samples")

    def test_5_older_stamps_are_dropped(self):
        outputs = self.run_steps(50, WHEEL_RATE, [(0.0, IMU_RATE, None), (-0.01, BAD_RATE, NOT_NEWER)])
        self.assertStaysAtMost(outputs, IMU_RATE + 0.01, "samples older than the last valid one")
        self.assertYawRate(outputs[-1], IMU_RATE, 0.01, "valid samples")

    def test_6_clock_goes_back(self):
        cls = type(self)
        cls.now = RESTART
        cls.clock.publish(Clock(cls.now))
        time.sleep(0.5)
        # The first measurement after the reset initializes the filter, and robot_localization
        # doesn't publish that estimate, because its stamp is older than the last one it
        # published. The measurement that showed the jump is dropped, and so can be the other
        # one of its step: the reset clears the node's callback queue. So the outputs start
        # again within a few steps
        for _ in range(5):
            if cls.step(WHEEL_RATE, [(0.0, IMU_RATE, None)], output=False) is not None:
                break
        else:
            self.fail("no EKF output within 5 steps after the clock went back")
        outputs = cls.run_steps(100, WHEEL_RATE, [(0.0, IMU_RATE, None)])
        print("after the clock went back to {:.0f} s: estimate {:.4f} rad/s at {:.2f} s".format(
            RESTART.to_sec(), outputs[-1].twist.twist.angular.z, outputs[-1].header.stamp.to_sec()))
        self.assertLess(outputs[0].header.stamp, START, "the outputs should carry the new time")
        self.assertYawRate(outputs[-1], IMU_RATE, 0.01, "after the reset")

    def test_7_diagnostics_count_the_samples(self):
        cls = type(self)
        received = len(cls.diagnostics)
        # No more IMU samples: the gate publishes its diagnostics once per second of ROS time
        start = cls.now
        while cls.now < start + rospy.Duration(1.5):
            cls.step(WHEEL_RATE)
        wait_for(lambda: len(cls.diagnostics) > received, "no diagnostics from the gate")
        values = None
        for array in reversed(cls.diagnostics[received:]):
            for status in array.status:
                if "IMU gate" in status.name:
                    values = {pair.key: pair.value for pair in status.values}
                    break
            if values is not None:
                break
        self.assertIsNotNone(values, "no status of the IMU gate on /diagnostics")
        print("gate diagnostics: {}".format(values))
        self.assertEqual(str(cls.expected_passes), values.get("Samples passed on"))
        for reason, count in cls.expected_drops.items():
            self.assertEqual(str(count), values.get("Dropped, " + reason), reason)
        self.assertEqual(len(cls.expected_drops) + 1, len(values), "other drop reasons: {}".format(values))


if __name__ == "__main__":
    rospy.init_node("test_ekf_synthetic")
    rostest.rosrun("diffbot_control", "test_ekf_synthetic", TestEkfSynthetic)
