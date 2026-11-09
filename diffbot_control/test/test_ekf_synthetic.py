#!/usr/bin/env python3
"""Check the EKF of ekf.launch on synthetic inputs and a stepped clock (ekf_synthetic.test).

The test feeds the EKF of ekf.launch, imu_validator.py and robot_localization, synthetic wheel
odometry and IMU samples, and checks its estimate of the yaw rate. It publishes /clock itself
(use_sim_time) and advances it in 20 ms steps, the period of the controller's odometry and of the
IMU. Each step publishes that step's inputs, stamped with the current time, waits on the wall
clock until the IMU sample validator's output reaches the test (the EKF gets it at the same time)
and 20 ms more, and then advances the clock, which lets the EKF's 50 Hz update process them; the
step ends with an EKF output for that step, waited for with a time limit. These windows don't
prove that the EKF's callbacks ran before the clock moved, but they make it very likely, and the
checks don't depend on it: a late input is fused one step later. Both inputs show in the
estimate: the forward speed comes only from the wheels, the yaw rate mostly from the IMU.

The scenarios run one after the other on one filter, from its start:
1. Fusion: the wheels report 0.4 rad/s, as with a wheel separation that is too large, the IMU
   0.5 rad/s with a much smaller variance. The estimate follows the IMU.
2. IMU samples 0.5 s old, reporting 2.0 rad/s, while the wheels go on: the validator drops them,
   and the estimate goes to the wheels' 0.4 rad/s, as when the IMU stops.
3. to 5. Valid samples again, each followed by one reporting 2.0 rad/s that has the same stamp
   (3), a stamp 0.5 s in the future (4), or a stamp older than the valid one (5). The validator
   drops those, and the estimate stays at the valid samples' 0.5 rad/s; one of them passing would
   pull it far up.
6. The clock goes back, as when Gazebo's world is reset: the validator and the EKF start over
   from the new time (reset_on_time_jump), and the estimate follows the IMU again.
At the end, the validator's diagnostics have to count each forwarded and dropped sample.
"""

import collections
import time
from typing import Callable
from typing import Counter
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Set
from typing import Tuple
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import rospy
import rostest
from sensor_msgs.msg import Imu
import tf2_ros

CLOCK_TOPIC = '/clock'
WHEEL_ODOMETRY_TOPIC = '/diffbot/mobile_base_controller/odom'
RAW_IMU_TOPIC = '/diffbot/imu/data_raw'
VALIDATED_IMU_TOPIC = '/diffbot/imu/data_validated'
FILTERED_TOPIC = '/diffbot/odometry/filtered'
DIAGNOSTICS_TOPIC = '/diagnostics'
QUEUE_SIZE = 100  # messages buffered per topic between callbacks
CLOCK_QUEUE_SIZE = 10  # messages

STEP = rospy.Duration(0, 20000000)  # 20 ms
START = rospy.Time(100, 0)
RESTART = rospy.Time(50, 0)  # where the clock goes back to in scenario 6
WAIT = 5.0  # s of wall time for one step's messages
DELIVERY = 0.02  # s of wall time for the inputs to arrive before the clock moves on
POLL_INTERVAL = 0.001  # s of wall time between two checks of a condition that wait_for() waits for
NO_OUTPUT_WAIT = 10 * DELIVERY  # s of wall time for an EKF output that may not come
TF_DELIVERY = 1.0  # s of wall time for the EKF's own TF listener to get the latched transform
RESET_DELIVERY = 0.5  # s of wall time for the clock's jump back to reach the nodes
RESET_STEPS = 5  # steps within which the EKF publishes again after the clock went back

# Steps per scenario, 20 ms each
FUSION_STEPS = 150
SCENARIO_STEPS = 50
AFTER_RESET_STEPS = 100
# s of ROS time without IMU samples before test 7 reads the diagnostics: longer than the 1 s
# period of the validator's diagnostics
DIAGNOSTICS_WAIT = rospy.Duration(1.5)

SPEED = 0.1  # m/s, forward, from the wheels
WHEEL_RATE = 0.4  # rad/s
IMU_RATE = 0.5  # rad/s
BAD_RATE = 2.0  # rad/s, in the samples the validator has to drop
# s, the stamps of the samples to drop relative to the step's time
STALE_OFFSET = -0.5
FUTURE_OFFSET = 0.5
OLDER_OFFSET = -0.01

# The inputs' variances, as the robot's: diffbot_control.yaml's twist_covariance_diagonal and
# diffbot_base_config.h's IMU variances
WHEEL_VARIANCES = [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
IMU_ANGULAR_VELOCITY_VARIANCE = 4.0e-4  # (rad/s)²
IMU_LINEAR_ACCELERATION_VARIANCE = 1.0e-2  # (m/s²)²
# In a 6x6 covariance stored row by row, the diagonal's elements are 7 apart
DIAGONAL_STRIDE = 7
VARIANCE_INDICES = (0, 4, 8)  # the diagonal of a 3x3 covariance stored row by row
ORIENTATION_UNAVAILABLE = -1.0  # REP 145: orientation_covariance[0] = -1
GRAVITY = 9.81  # m/s², on the upward axis at rest

# Tolerances of the estimate
SPEED_TOLERANCE = 0.002  # m/s
YAW_RATE_TOLERANCE = 0.01  # rad/s
# rad/s, for the estimate that approaches the wheels' rate after the IMU samples stop passing
WHEEL_RATE_TOLERANCE = 0.02

# The validator's status on /diagnostics contains this name; its values have these keys
# (scripts/imu_validator.py)
VALIDATOR_STATUS_NAME = 'IMU sample validator'
FORWARDED_KEY = 'Samples passed on'
DROPPED_KEY_PREFIX = 'Dropped, '
# The validator's reasons, as its diagnostics name them
TOO_OLD = 'too old'
NOT_NEWER = 'not newer than the last one'
FROM_THE_FUTURE = 'from the future'

StampKey = Tuple[int, int]  # a stamp's seconds and nanoseconds, which a set can hold


class ImuInput(NamedTuple):
    """An IMU sample that a step publishes, and what the validator has to do with it."""

    offset: float  # s, the sample's stamp relative to the step's time
    yaw_rate: float  # rad/s
    drop_reason: Optional[str]  # None for a sample the validator has to forward


# A valid sample stamped at the step's time, which the validator has to forward
VALID_SAMPLE = ImuInput(0.0, IMU_RATE, None)


def stamp_key(stamp: rospy.Time) -> StampKey:
    """Return the stamp's seconds and nanoseconds."""
    return (stamp.secs, stamp.nsecs)


def wheel_odometry(stamp: rospy.Time, yaw_rate: float) -> Odometry:
    """Return the drive controller's odometry at stamp, for a robot turning at yaw_rate."""
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = SPEED
    message.twist.twist.angular.z = yaw_rate
    for axis, variance in enumerate(WHEEL_VARIANCES):
        message.pose.covariance[DIAGONAL_STRIDE * axis] = variance
        message.twist.covariance[DIAGONAL_STRIDE * axis] = variance
    return message


def imu_sample(stamp: rospy.Time, yaw_rate: float) -> Imu:
    """Return a sample as the firmware publishes it, in DiffBot's imu_link.

    DiffBot's imu_link is aligned with base_footprint.
    """
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = 'imu_link'
    message.orientation_covariance[0] = ORIENTATION_UNAVAILABLE
    message.angular_velocity.z = yaw_rate
    message.linear_acceleration.z = GRAVITY
    for index in VARIANCE_INDICES:
        message.angular_velocity_covariance[index] = IMU_ANGULAR_VELOCITY_VARIANCE
        message.linear_acceleration_covariance[index] = IMU_LINEAR_ACCELERATION_VARIANCE
    return message


def wait_for(condition: Callable[[], bool], failure: str) -> None:
    """Wait on the wall clock until condition() is true, at most WAIT s, else fail with failure."""
    deadline = time.monotonic() + WAIT
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError(failure)
        time.sleep(POLL_INTERVAL)


class TestEkfSynthetic(unittest.TestCase):
    """The scenarios share one filter and run in the order of their names."""

    now: rospy.Time
    forwarded: Set[StampKey]  # stamps of the samples the validator forwarded
    outputs: List[Odometry]
    diagnostics: List[DiagnosticArray]
    expected_forward_count: int
    expected_drops: Counter[str]
    clock: rospy.Publisher
    wheels: rospy.Publisher
    imu: rospy.Publisher
    validated: rospy.Subscriber
    filtered: rospy.Subscriber
    listener: tf2_ros.TransformListener

    @classmethod
    def setUpClass(cls) -> None:
        cls.now = START
        cls.forwarded = set()
        cls.outputs = []
        cls.diagnostics = []
        cls.expected_forward_count = 0
        cls.expected_drops = collections.Counter()
        cls.clock = rospy.Publisher(CLOCK_TOPIC, Clock, queue_size=CLOCK_QUEUE_SIZE, latch=True)
        cls.wheels = rospy.Publisher(WHEEL_ODOMETRY_TOPIC, Odometry, queue_size=QUEUE_SIZE)
        cls.imu = rospy.Publisher(RAW_IMU_TOPIC, Imu, queue_size=QUEUE_SIZE)
        cls.validated = rospy.Subscriber(VALIDATED_IMU_TOPIC, Imu, cls.on_validated,
                                         queue_size=QUEUE_SIZE)
        cls.filtered = rospy.Subscriber(FILTERED_TOPIC, Odometry, cls.outputs.append,
                                        queue_size=QUEUE_SIZE)
        rospy.Subscriber(DIAGNOSTICS_TOPIC, DiagnosticArray, cls.diagnostics.append,
                         queue_size=QUEUE_SIZE)
        cls.clock.publish(Clock(cls.now))
        buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(buffer)
        # Before the first input: the EKF and the validator listen, and the production URDF's
        # imu_link transform is there. A paused clock doesn't advance ROS-time waits, so these
        # wait on the wall clock
        wait_for(cls.all_connected, "the EKF and the validator didn't connect")
        wait_for(lambda: buffer.can_transform('base_footprint', 'imu_link', rospy.Time(0)),
                 'no transform from imu_link to base_footprint (robot_state_publisher)')
        time.sleep(TF_DELIVERY)

    @classmethod
    def on_validated(cls, message: Imu) -> None:
        """Note the stamp of a sample that the validator forwarded."""
        cls.forwarded.add(stamp_key(message.header.stamp))

    @classmethod
    def all_connected(cls) -> bool:
        """Return whether the EKF and the validator subscribe and publish to the test."""
        return (cls.wheels.get_num_connections() > 0 and cls.imu.get_num_connections() > 0
                and cls.validated.get_num_connections() > 0
                and cls.filtered.get_num_connections() > 0)

    @classmethod
    def advance(cls, wheel_rate: float, imu: Sequence[ImuInput]) -> Tuple[rospy.Time, int]:
        """Publish one step's inputs, stamped with the current time, then advance the clock.

        Return the step's time and the number of EKF outputs before the step.
        """
        stamp = cls.now
        received = len(cls.outputs)
        cls.wheels.publish(wheel_odometry(stamp, wheel_rate))
        forwarded_keys = []
        for sample in imu:
            sample_stamp = stamp + rospy.Duration.from_sec(sample.offset)
            cls.imu.publish(imu_sample(sample_stamp, sample.yaw_rate))
            if sample.drop_reason is None:
                forwarded_keys.append(stamp_key(sample_stamp))
                cls.expected_forward_count += 1
            else:
                cls.expected_drops[sample.drop_reason] += 1
        wait_for(lambda: all(key in cls.forwarded for key in forwarded_keys),
                 "the validator didn't forward the IMU samples of the step at {:.2f} s".format(
                     stamp.to_sec()))
        time.sleep(DELIVERY)
        cls.now = stamp + STEP
        cls.clock.publish(Clock(cls.now))
        return stamp, received

    @classmethod
    def outputs_since(cls, stamp: rospy.Time, received: int) -> List[Odometry]:
        """Return the EKF's outputs after the first received ones, stamped at stamp or later."""
        return [message for message in cls.outputs[received:] if message.header.stamp >= stamp]

    @classmethod
    def step(cls, wheel_rate: float, imu: Sequence[ImuInput] = ()) -> Odometry:
        """Run one step and return the EKF's last output for it, which has to come."""
        stamp, received = cls.advance(wheel_rate, imu)
        wait_for(lambda: bool(cls.outputs_since(stamp, received)),
                 'no EKF output for the step at {:.2f} s'.format(stamp.to_sec()))
        return cls.outputs_since(stamp, received)[-1]

    @classmethod
    def step_allowing_no_output(cls, wheel_rate: float,
                                imu: Sequence[ImuInput] = ()) -> Optional[Odometry]:
        """Run one step and return the EKF's last output for it, or None if none came."""
        stamp, received = cls.advance(wheel_rate, imu)
        time.sleep(NO_OUTPUT_WAIT)
        outputs = cls.outputs_since(stamp, received)
        return outputs[-1] if outputs else None

    @classmethod
    def run_steps(cls, count: int, wheel_rate: float,
                  imu: Sequence[ImuInput] = ()) -> List[Odometry]:
        """Run count steps with the same inputs and return the EKF's output for each."""
        return [cls.step(wheel_rate, imu) for _ in range(count)]

    def assert_yaw_rate(self, output: Odometry, expected: float, delta: float, what: str) -> None:
        """Assert that the output's yaw rate is expected ± delta; what describes the case."""
        rate = output.twist.twist.angular.z
        self.assertAlmostEqual(rate, expected, delta=delta,
                               msg='{}: estimated {:.4f} rad/s, expected {} ± {}'.format(
                                   what, rate, expected, delta))

    def assert_stays_at_most(self, outputs: List[Odometry], limit: float, what: str) -> None:
        """Assert that no output's yaw rate exceeds limit; what describes the case."""
        highest = max(message.twist.twist.angular.z for message in outputs)
        self.assertLessEqual(highest, limit,
                             '{}: the estimate went up to {:.4f} rad/s'.format(what, highest))

    def test_1_estimate_follows_the_imu_with_the_smaller_variance(self) -> None:
        outputs = self.run_steps(FUSION_STEPS, WHEEL_RATE, [VALID_SAMPLE])
        last = outputs[-1]
        print('fusion: wheels {} rad/s, IMU {} rad/s, estimate {:.4f} rad/s, forward {:.4f} m/s'
              .format(WHEEL_RATE, IMU_RATE, last.twist.twist.angular.z, last.twist.twist.linear.x))
        self.assertEqual(last.header.frame_id, 'odom')
        self.assertEqual(last.child_frame_id, 'base_footprint')
        self.assertAlmostEqual(last.twist.twist.linear.x, SPEED, delta=SPEED_TOLERANCE,
                               msg='the forward speed comes from the wheels')
        self.assert_yaw_rate(last, IMU_RATE, YAW_RATE_TOLERANCE,
                             "the IMU's variance is far smaller than the wheels'")

    def test_2_validator_drops_stale_samples_and_the_estimate_follows_the_wheels(self) -> None:
        outputs = self.run_steps(SCENARIO_STEPS, WHEEL_RATE,
                                 [ImuInput(STALE_OFFSET, BAD_RATE, TOO_OLD)])
        print('stale IMU: estimate from {:.4f} to {:.4f} rad/s'.format(
            outputs[0].twist.twist.angular.z, outputs[-1].twist.twist.angular.z))
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'stale samples reporting 2.0 rad/s')
        self.assert_yaw_rate(outputs[-1], WHEEL_RATE, WHEEL_RATE_TOLERANCE,
                             'without IMU samples, the estimate follows the wheels')

    def test_3_validator_drops_samples_with_a_repeated_stamp(self) -> None:
        outputs = self.run_steps(SCENARIO_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE, ImuInput(0.0, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'samples with a repeated stamp')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'valid samples again')

    def test_4_validator_drops_samples_from_the_future(self) -> None:
        outputs = self.run_steps(SCENARIO_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE,
                                  ImuInput(FUTURE_OFFSET, BAD_RATE, FROM_THE_FUTURE)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'samples from the future')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'valid samples')

    def test_5_validator_drops_samples_older_than_the_last_forwarded_one(self) -> None:
        outputs = self.run_steps(SCENARIO_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE, ImuInput(OLDER_OFFSET, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'samples older than the last valid one')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'valid samples')

    def test_6_estimate_follows_the_imu_again_after_the_clock_goes_back(self) -> None:
        cls = type(self)
        cls.now = RESTART
        cls.clock.publish(Clock(cls.now))
        time.sleep(RESET_DELIVERY)
        # The first measurement after the reset initializes the filter, and robot_localization
        # doesn't publish that estimate, because its stamp is older than the last one it
        # published. The measurement that showed the jump is dropped, and so can be the other
        # one of its step: the reset clears the node's callback queue. So the outputs start
        # again within a few steps
        for _ in range(RESET_STEPS):
            if cls.step_allowing_no_output(WHEEL_RATE, [VALID_SAMPLE]) is not None:
                break
        else:
            self.fail('no EKF output within {} steps after the clock went back'.format(
                RESET_STEPS))
        outputs = cls.run_steps(AFTER_RESET_STEPS, WHEEL_RATE, [VALID_SAMPLE])
        print('after the clock went back to {:.0f} s: estimate {:.4f} rad/s at {:.2f} s'.format(
            RESTART.to_sec(), outputs[-1].twist.twist.angular.z,
            outputs[-1].header.stamp.to_sec()))
        self.assertLess(outputs[0].header.stamp, START, 'the outputs should carry the new time')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'after the reset')

    def test_7_validator_diagnostics_count_each_forwarded_and_dropped_sample(self) -> None:
        cls = type(self)
        received = len(cls.diagnostics)
        # No more IMU samples: the validator publishes diagnostics once per second of ROS time
        start = cls.now
        while cls.now < start + DIAGNOSTICS_WAIT:
            cls.step(WHEEL_RATE)
        wait_for(lambda: len(cls.diagnostics) > received, 'no diagnostics from the validator')
        values = None
        for array in reversed(cls.diagnostics[received:]):
            for status in array.status:
                if VALIDATOR_STATUS_NAME in status.name:
                    values = {pair.key: pair.value for pair in status.values}
                    break
            if values is not None:
                break
        if values is None:
            self.fail('no status of the IMU sample validator on /diagnostics')
        print('validator diagnostics: {}'.format(values))
        self.assertEqual(str(cls.expected_forward_count), values.get(FORWARDED_KEY))
        for reason, count in cls.expected_drops.items():
            self.assertEqual(str(count), values.get(DROPPED_KEY_PREFIX + reason), reason)
        self.assertEqual(len(cls.expected_drops) + 1, len(values),
                         'other drop reasons: {}'.format(values))


if __name__ == '__main__':
    rospy.init_node('test_ekf_synthetic')
    rostest.rosrun('diffbot_control', 'test_ekf_synthetic', TestEkfSynthetic)
