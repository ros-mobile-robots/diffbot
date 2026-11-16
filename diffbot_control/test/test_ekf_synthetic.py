#!/usr/bin/env python3
"""Check the EKF of ekf.launch on synthetic inputs and a stepped clock (ekf_synthetic.launch).

The test feeds the EKF of ekf.launch, imu_validator.py and robot_localization, synthetic wheel
odometry and IMU samples, and checks its estimate of the yaw rate. It publishes the topic /clock
itself (use_sim_time) and advances it in 20 ms steps, the period of the IMU. The docstring of
step() lists what one step does. A step's waits don't prove that the EKF's callbacks ran before
the clock moved, but they make it very likely, and the checks don't depend on it: a late input
is fused one step later. Both inputs show in the estimate: the forward speed comes only from the
wheels, the yaw rate mostly from the IMU.

The file has two scenarios. Each is one test on one filter, from the filter's start, and its
class's docstring lists its phases. A .test file runs one scenario, which the test node's
argument --scenario names (SCENARIOS), so that each scenario starts with a fresh EKF:
1. invalid_imu_samples (ekf_synthetic.test): both inputs in every step, at 50 Hz as in the
   simulation, IMU samples that the validator has to drop, and a clock that goes back.
2. robot_rates (ekf_synthetic_robot_rates.test): the IMU in every step and the wheel odometry
   in every fifth step, at 10 Hz as on the robot, where diffbot_base's control loop runs the
   drive controller at 10 Hz. Then the IMU stops.
"""

import argparse
import collections
import threading
import time
from typing import Callable
from typing import Counter
from typing import Dict
from typing import Generic
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple
from typing import Type
from typing import TypeVar
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
RESTART = rospy.Time(50, 0)  # where the clock goes back to in phase 6
WAIT = 5.0  # s of wall time for one step's messages
DELIVERY = 0.02  # s of wall time for the inputs to arrive before the clock moves on
POLL_INTERVAL = 0.001  # s of wall time between two checks of a condition that wait_for() waits for
NO_OUTPUT_WAIT = 10 * DELIVERY  # s of wall time for an EKF output that may not come
TF_DELIVERY = 1.0  # s of wall time for the EKF's own TF listener to get the latched transform
RESET_DELIVERY = 0.5  # s of wall time for the clock's jump back to reach the nodes
RESET_STEPS = 5  # steps within which the EKF publishes again after the clock went back

# The robot's wheel odometry period: diffbot_base's control loop runs the drive controller at
# 10 Hz, which publishes at most once per run. The IMU's period on the robot is STEP
ROBOT_WHEEL_ODOMETRY_PERIOD = rospy.Duration(0, 100000000)  # 100 ms
STEPS_PER_WHEEL_MESSAGE = round(ROBOT_WHEEL_ODOMETRY_PERIOD.to_sec() / STEP.to_sec())  # 5

# Steps per phase, 20 ms each
FUSION_STEPS = 150
PHASE_STEPS = 50
AFTER_RESET_STEPS = 100
# Wheel messages, one every STEPS_PER_WHEEL_MESSAGE steps, after the IMU stopped
IMU_STOPPED_WHEEL_MESSAGES = 20
# s of ROS time without IMU samples before phase 7 reads the diagnostics: longer than the 1 s
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

# The inputs' variances, of the size of the robot's: diffbot_control.yaml's
# twist_covariance_diagonal and diffbot_base_config.h's IMU variances. The test doesn't read
# those files
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

# The validator's status on the topic /diagnostics contains this name, and its values have these
# keys (scripts/imu_validator.py). The texts are written out here and not imported from the
# script: the test pins what a user reads on that topic
VALIDATOR_STATUS_NAME = 'IMU sample validator'
FORWARDED_KEY = 'Samples forwarded'
DROPPED_KEY_PREFIX = 'Dropped, '
# The validator's reasons, as its diagnostics name them
TOO_OLD = 'too old'
NOT_NEWER = 'not newer than the last one'
FROM_THE_FUTURE = 'from the future'

StampKey = Tuple[int, int]  # a stamp's seconds and nanoseconds, which compare by value


class ImuInput(NamedTuple):
    """An IMU sample that a step publishes, and what the validator has to do with it."""

    offset: float  # s, the sample's stamp relative to the step's time
    yaw_rate: float  # rad/s
    drop_reason: Optional[str]  # None for a sample the validator has to forward


# A valid sample stamped at the step's time, which the validator has to forward
VALID_SAMPLE = ImuInput(0.0, IMU_RATE, None)


Item = TypeVar('Item')


class ArrivalLog(Generic[Item]):
    """What arrived on a topic, in the order of arrival.

    rospy calls a subscriber's callback in a thread of its own, which appends to the log, while
    the test's thread reads it. A lock guards every access, so that the test's thread never
    reads the log while the callback changes it.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: List[Item] = []

    def append(self, item: Item) -> None:
        """Add an item at the end."""
        with self._lock:
            self._items.append(item)

    def count(self) -> int:
        """Return the number of items so far."""
        with self._lock:
            return len(self._items)

    def since(self, count: int) -> List[Item]:
        """Return a copy of the items after the first count."""
        with self._lock:
            return self._items[count:]

    def contains_all(self, items: Sequence[Item]) -> bool:
        """Return whether each of the items has arrived."""
        with self._lock:
            return all(item in self._items for item in items)


class StepStart(NamedTuple):
    """Where a step started: what tells the step's EKF outputs from the earlier ones."""

    stamp: rospy.Time  # the step's ROS time
    output_count_before: int  # the number of EKF outputs that the test had before the step


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


class SteppedClockTestCase(unittest.TestCase):
    """What both scenarios share: the topics, the steps of the clock and the comparisons.

    A scenario is a subclass with one test method. Each of its check_...() methods is one
    phase: it publishes the phase's inputs and asserts on the EKF's estimate or on the
    validator's diagnostics. The test method calls them in order and stops at the first phase
    that fails, because the later ones need its result.
    """

    def setUp(self) -> None:
        self.now = START
        # The stamps of the samples that the validator forwarded
        self.forwarded_stamp_keys: ArrivalLog[StampKey] = ArrivalLog()
        self.ekf_outputs: ArrivalLog[Odometry] = ArrivalLog()
        self.diagnostics: ArrivalLog[DiagnosticArray] = ArrivalLog()
        self.expected_forward_count = 0
        self.expected_drops: Counter[str] = collections.Counter()
        self.clock_publisher = rospy.Publisher(
            CLOCK_TOPIC, Clock, queue_size=CLOCK_QUEUE_SIZE, latch=True)
        self.wheel_odometry_publisher = rospy.Publisher(
            WHEEL_ODOMETRY_TOPIC, Odometry, queue_size=QUEUE_SIZE)
        self.raw_imu_publisher = rospy.Publisher(RAW_IMU_TOPIC, Imu, queue_size=QUEUE_SIZE)
        self.validated_imu_subscriber = rospy.Subscriber(
            VALIDATED_IMU_TOPIC, Imu, self.on_validated, queue_size=QUEUE_SIZE)
        self.filtered_odometry_subscriber = rospy.Subscriber(
            FILTERED_TOPIC, Odometry, self.ekf_outputs.append, queue_size=QUEUE_SIZE)
        self.diagnostics_subscriber = rospy.Subscriber(
            DIAGNOSTICS_TOPIC, DiagnosticArray, self.diagnostics.append, queue_size=QUEUE_SIZE)
        self.clock_publisher.publish(Clock(self.now))
        buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(buffer)
        # Before the first input: the EKF and the validator listen, and the production URDF's
        # imu_link transform is there. A paused clock doesn't advance ROS-time waits, so these
        # wait on the wall clock
        wait_for(self.all_connected, "the EKF and the validator didn't connect")
        wait_for(lambda: buffer.can_transform('base_footprint', 'imu_link', rospy.Time(0)),
                 'no transform from imu_link to base_footprint (robot_state_publisher)')
        time.sleep(TF_DELIVERY)

    def on_validated(self, message: Imu) -> None:
        """Note the stamp of a sample that the validator forwarded."""
        self.forwarded_stamp_keys.append(stamp_key(message.header.stamp))

    def all_connected(self) -> bool:
        """Return whether the EKF and the validator subscribe and publish to the test."""
        return (self.wheel_odometry_publisher.get_num_connections() > 0
                and self.raw_imu_publisher.get_num_connections() > 0
                and self.validated_imu_subscriber.get_num_connections() > 0
                and self.filtered_odometry_subscriber.get_num_connections() > 0)

    def publish_inputs(self, stamp: rospy.Time, wheel_rate: Optional[float],
                       imu_inputs: Sequence[ImuInput]) -> List[StampKey]:
        """Publish one step's wheel odometry and IMU samples, stamped relative to stamp.

        A wheel_rate of None publishes no wheel odometry in the step. Return the stamp keys of
        the samples that the validator has to forward.
        """
        if wheel_rate is not None:
            self.wheel_odometry_publisher.publish(wheel_odometry(stamp, wheel_rate))
        valid_stamp_keys = []
        for imu_input in imu_inputs:
            sample_stamp = stamp + rospy.Duration.from_sec(imu_input.offset)
            self.raw_imu_publisher.publish(imu_sample(sample_stamp, imu_input.yaw_rate))
            self.expect(imu_input)
            if imu_input.drop_reason is None:
                valid_stamp_keys.append(stamp_key(sample_stamp))
        return valid_stamp_keys

    def expect(self, imu_input: ImuInput) -> None:
        """Count the sample as one that the validator has to forward or to drop."""
        if imu_input.drop_reason is None:
            self.expected_forward_count += 1
        else:
            self.expected_drops[imu_input.drop_reason] += 1

    def wait_for_forwarded(self, stamp_keys: Sequence[StampKey], stamp: rospy.Time) -> None:
        """Wait on the wall clock until the validator forwarded the samples with these stamps."""
        wait_for(lambda: self.forwarded_stamp_keys.contains_all(stamp_keys),
                 "the validator didn't forward the IMU samples of the step at {:.2f} s".format(
                     stamp.to_sec()))

    def advance_clock(self) -> None:
        """Move the ROS time on by STEP and publish it."""
        self.now += STEP
        self.clock_publisher.publish(Clock(self.now))

    def advance(self, wheel_rate: Optional[float], imu_inputs: Sequence[ImuInput]) -> StepStart:
        """Do 1 to 4 of a step, as step() lists them, and return where the step started."""
        start = StepStart(self.now, self.ekf_outputs.count())
        valid_stamp_keys = self.publish_inputs(start.stamp, wheel_rate, imu_inputs)
        self.wait_for_forwarded(valid_stamp_keys, start.stamp)
        time.sleep(DELIVERY)
        self.advance_clock()
        return start

    def outputs_since(self, start: StepStart) -> List[Odometry]:
        """Return the EKF's outputs after the step's start, stamped at its time or later."""
        return [message for message in self.ekf_outputs.since(start.output_count_before)
                if message.header.stamp >= start.stamp]

    def step(self, wheel_rate: Optional[float], imu_inputs: Sequence[ImuInput] = ()) -> Odometry:
        """Run one step and return the EKF's last output for it, which has to come.

        One step:
        1. Publish the step's wheel odometry, if wheel_rate isn't None, stamped with the step's
           ROS time, and its IMU samples, each stamped with that time plus the sample's offset
           (publish_inputs()).
        2. Wait on the wall clock until the IMU sample validator's output for the step's valid
           samples reaches the test (wait_for_forwarded()). The EKF subscribes to the same
           topic.
        3. Wait DELIVERY more, for the inputs to arrive at the EKF.
        4. Advance the clock by STEP (advance_clock()). The EKF's 50 Hz update then processes the
           inputs.
        5. Wait, with the time limit WAIT, for an EKF output stamped at the step's time or later.
        advance() does 1 to 4.
        """
        start = self.advance(wheel_rate, imu_inputs)
        wait_for(lambda: bool(self.outputs_since(start)),
                 'no EKF output for the step at {:.2f} s'.format(start.stamp.to_sec()))
        return self.outputs_since(start)[-1]

    def step_allowing_no_output(self, wheel_rate: Optional[float],
                                imu_inputs: Sequence[ImuInput] = ()) -> Optional[Odometry]:
        """Run one step and return the EKF's last output for it, or None if none came."""
        start = self.advance(wheel_rate, imu_inputs)
        time.sleep(NO_OUTPUT_WAIT)
        outputs = self.outputs_since(start)
        return outputs[-1] if outputs else None

    def run_steps(self, count: int, wheel_rate: float,
                  imu_inputs: Sequence[ImuInput] = ()) -> List[Odometry]:
        """Run count steps with the same inputs and return the EKF's output for each."""
        return [self.step(wheel_rate, imu_inputs) for _ in range(count)]

    def assert_yaw_rate(self, output: Odometry, expected: float, delta: float, what: str) -> None:
        """Assert that the output's yaw rate is expected ± delta. what names the case."""
        rate = output.twist.twist.angular.z
        self.assertAlmostEqual(rate, expected, delta=delta,
                               msg='{}: estimated {:.4f} rad/s, expected {} ± {}'.format(
                                   what, rate, expected, delta))

    def assert_stays_at_most(self, outputs: List[Odometry], limit: float, what: str) -> None:
        """Assert that no output's yaw rate exceeds limit. what names the case."""
        highest = max(message.twist.twist.angular.z for message in outputs)
        self.assertLessEqual(highest, limit,
                             '{}: the estimate went up to {:.4f} rad/s'.format(what, highest))

    def assert_forward_speed(self, output: Odometry, what: str) -> None:
        """Assert that the output's forward speed is the wheels' SPEED. what names the case."""
        self.assertAlmostEqual(output.twist.twist.linear.x, SPEED, delta=SPEED_TOLERANCE,
                               msg=what + ': the forward speed comes from the wheels')

    def assert_fused_estimate(self, output: Odometry, what: str) -> None:
        """Assert that output has the EKF's frames, the wheels' speed and the IMU's yaw rate.

        what names the case.
        """
        self.assertEqual(output.header.frame_id, 'odom', what)
        self.assertEqual(output.child_frame_id, 'base_footprint', what)
        self.assert_forward_speed(output, what)
        self.assert_yaw_rate(output, IMU_RATE, YAW_RATE_TOLERANCE,
                             what + ": the IMU's variance is far smaller than the wheels'")

    def print_fusion(self, what: str, output: Odometry) -> None:
        """Print the inputs' yaw rates and the output's estimate. what names the case."""
        print('{}: wheels {} rad/s, IMU {} rad/s, estimate {:.4f} rad/s, forward {:.4f} m/s'
              .format(what, WHEEL_RATE, IMU_RATE, output.twist.twist.angular.z,
                      output.twist.twist.linear.x))


class TestEkfWithInvalidImuSamples(SteppedClockTestCase):
    """The scenario invalid_imu_samples: both inputs in every step, at 50 Hz.

    Its phases run in this order, and each needs the state that the phases before it left:
    1. Fusion: the wheels report 0.4 rad/s, as with a wheel separation that is too large, the
       IMU 0.5 rad/s with a much smaller variance. The estimate follows the IMU.
    2. IMU samples 0.5 s old, reporting 2.0 rad/s, while the wheels go on: the validator drops
       them, and the estimate goes to the wheels' 0.4 rad/s, as when the IMU stops.
    3. to 5. Valid samples again, each followed by one reporting 2.0 rad/s that has the same
       stamp (3), a stamp 0.5 s in the future (4), or a stamp older than the valid one (5). The
       validator drops those, and the estimate stays at the valid samples' 0.5 rad/s. A
       forwarded one would pull the estimate far up.
    6. The clock goes back, as when Gazebo's world is reset: the validator and the EKF start
       over from the new time (reset_on_time_jump), and the estimate follows the IMU again.
    7. The validator's status on the topic /diagnostics counts each sample that the phases
       before sent, forwarded or dropped.
    """

    def test_ekf_handles_invalid_imu_samples_and_recovers_after_the_clock_goes_back(self) -> None:
        self.check_estimate_follows_the_imu_with_the_smaller_variance()
        self.check_estimate_follows_the_wheels_when_imu_samples_are_stale()
        self.check_validator_drops_samples_with_a_repeated_stamp()
        self.check_validator_drops_samples_from_the_future()
        self.check_validator_drops_samples_older_than_the_last_valid_one()
        self.check_estimate_follows_the_imu_again_after_the_clock_goes_back()
        self.check_validator_diagnostics_count_each_forwarded_and_dropped_sample()

    def check_estimate_follows_the_imu_with_the_smaller_variance(self) -> None:
        """Phase 1: valid IMU samples and wheel odometry that disagree on the yaw rate."""
        outputs = self.run_steps(FUSION_STEPS, WHEEL_RATE, [VALID_SAMPLE])
        self.print_fusion('fusion', outputs[-1])
        self.assert_fused_estimate(outputs[-1], 'phase 1')

    def check_estimate_follows_the_wheels_when_imu_samples_are_stale(self) -> None:
        """Phase 2: only IMU samples that are 0.5 s old, which the validator drops."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_RATE,
                                 [ImuInput(STALE_OFFSET, BAD_RATE, TOO_OLD)])
        print('stale IMU: estimate from {:.4f} to {:.4f} rad/s'.format(
            outputs[0].twist.twist.angular.z, outputs[-1].twist.twist.angular.z))
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 2: stale samples reporting 2.0 rad/s')
        self.assert_yaw_rate(outputs[-1], WHEEL_RATE, WHEEL_RATE_TOLERANCE,
                             'phase 2: without IMU samples, the estimate follows the wheels')

    def check_validator_drops_samples_with_a_repeated_stamp(self) -> None:
        """Phase 3: each valid sample is followed by one with the same stamp."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE, ImuInput(0.0, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 3: samples with a repeated stamp')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE,
                             'phase 3: valid samples again')

    def check_validator_drops_samples_from_the_future(self) -> None:
        """Phase 4: each valid sample is followed by one stamped 0.5 s in the future."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE,
                                  ImuInput(FUTURE_OFFSET, BAD_RATE, FROM_THE_FUTURE)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 4: samples from the future')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'phase 4: valid samples')

    def check_validator_drops_samples_older_than_the_last_valid_one(self) -> None:
        """Phase 5: each valid sample is followed by one with an older stamp."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_RATE,
                                 [VALID_SAMPLE, ImuInput(OLDER_OFFSET, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 5: samples older than the last valid one')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'phase 5: valid samples')

    def check_estimate_follows_the_imu_again_after_the_clock_goes_back(self) -> None:
        """Phase 6: the clock goes back to RESTART, then valid samples as in phase 1."""
        self.now = RESTART
        self.clock_publisher.publish(Clock(self.now))
        time.sleep(RESET_DELIVERY)
        # The first measurement after the reset initializes the filter, and robot_localization
        # doesn't publish that estimate, because its stamp is older than the last one it
        # published. The measurement that showed the jump is dropped, and so can be the other
        # one of its step: the reset clears the node's callback queue. So the outputs start
        # again within a few steps
        for _ in range(RESET_STEPS):
            if self.step_allowing_no_output(WHEEL_RATE, [VALID_SAMPLE]) is not None:
                break
        else:
            self.fail('phase 6: no EKF output within {} steps after the clock went back'.format(
                RESET_STEPS))
        outputs = self.run_steps(AFTER_RESET_STEPS, WHEEL_RATE, [VALID_SAMPLE])
        print('after the clock went back to {:.0f} s: estimate {:.4f} rad/s at {:.2f} s'.format(
            RESTART.to_sec(), outputs[-1].twist.twist.angular.z,
            outputs[-1].header.stamp.to_sec()))
        self.assertLess(outputs[0].header.stamp, START,
                        'phase 6: the outputs should carry the new time')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE,
                             'phase 6: after the clock went back')

    def check_validator_diagnostics_count_each_forwarded_and_dropped_sample(self) -> None:
        """Phase 7: the validator's diagnostics against the samples that phases 1 to 6 sent."""
        values = self.wait_for_validator_diagnostics()
        print('validator diagnostics: {}'.format(values))
        self.assert_diagnostic_counts(values)

    def validator_values(self, array_count_before: int) -> Optional[Dict[str, str]]:
        """Return the validator's latest status values after the first array_count_before arrays.

        None if no array after them has a status of the validator.
        """
        for array in reversed(self.diagnostics.since(array_count_before)):
            for status in array.status:
                if VALIDATOR_STATUS_NAME in status.name:
                    return {pair.key: pair.value for pair in status.values}
        return None

    def wait_for_validator_diagnostics(self) -> Dict[str, str]:
        """Advance the clock without IMU samples and return the validator's latest values.

        The validator publishes its diagnostics once per second of ROS time.
        """
        array_count_before = self.diagnostics.count()
        start = self.now
        while self.now < start + DIAGNOSTICS_WAIT:
            self.step(WHEEL_RATE)
        wait_for(lambda: self.validator_values(array_count_before) is not None,
                 'phase 7: no status of the IMU sample validator on the topic /diagnostics')
        values = self.validator_values(array_count_before)
        assert values is not None  # wait_for() returned
        return values

    def assert_diagnostic_counts(self, values: Dict[str, str]) -> None:
        """Assert that values has the expected count for forwarded and for each drop reason."""
        self.assertEqual(str(self.expected_forward_count), values.get(FORWARDED_KEY),
                         'phase 7: forwarded samples')
        for reason, count in self.expected_drops.items():
            self.assertEqual(str(count), values.get(DROPPED_KEY_PREFIX + reason),
                             'phase 7: dropped, ' + reason)
        self.assertEqual(len(self.expected_drops) + 1, len(values),
                         'phase 7: other drop reasons: {}'.format(values))


class TestEkfAtTheRobotRates(SteppedClockTestCase):
    """The scenario robot_rates: the IMU at 50 Hz and the wheel odometry at 10 Hz, as on the robot.

    Its phases run in this order, and the second needs the state that the first left:
    1. Fusion at the robot's rates: the IMU reports 0.5 rad/s in every step, the wheels 0.4 rad/s
       in every fifth step. The filter publishes in every step, and the estimate follows the
       IMU, with the wheels' forward speed also between two wheel messages.
    2. The IMU stops, the wheel odometry goes on in every fifth step. The filter publishes one
       output per wheel message, stamped with the message's time, and nothing between two wheel
       messages. The estimate goes to the wheels' 0.4 rad/s.
    """

    def test_ekf_follows_the_imu_at_the_robot_rates_and_the_wheels_after_the_imu_stops(
            self) -> None:
        self.check_estimate_follows_the_imu_with_wheel_odometry_every_fifth_step()
        self.check_ekf_publishes_at_the_wheel_rate_and_follows_the_wheels_when_the_imu_stops()

    def check_estimate_follows_the_imu_with_wheel_odometry_every_fifth_step(self) -> None:
        """Phase 1: as phase 1 of invalid_imu_samples, with the wheel odometry every fifth step."""
        # The last of the FUSION_STEPS steps is the fourth after a wheel message, so the checks
        # of the forward speed and the yaw rate look at an output between two wheel messages
        outputs = self.run_steps_at_the_robot_rates(FUSION_STEPS, [VALID_SAMPLE])
        self.print_fusion("fusion at the robot's rates", outputs[-1])
        self.assert_fused_estimate(outputs[-1], 'phase 1')

    def check_ekf_publishes_at_the_wheel_rate_and_follows_the_wheels_when_the_imu_stops(
            self) -> None:
        """Phase 2: no IMU samples, the wheel odometry every fifth step.

        Between two wheel messages, the filter has no measurement to fuse. It predicts without
        one only when the ROS time is sensor_timeout (ekf.yaml, 0.1 s) past its last
        measurement, and that is the wheel odometry's period. So the filter publishes nothing
        between two wheel messages. Its output at a wheel message's time is that prediction,
        published before the filter fused the message, or the fused message. The filter
        publishes no second output with the same stamp.
        """
        output_count_before = self.ekf_outputs.count()
        wheel_stamps = self.run_wheel_periods_without_imu(IMU_STOPPED_WHEEL_MESSAGES)
        outputs = self.ekf_outputs.since(output_count_before)
        print('IMU stopped: {} outputs for {} wheel messages, estimate from {:.4f} to {:.4f} '
              'rad/s, forward {:.4f} m/s'.format(
                  len(outputs), len(wheel_stamps), outputs[0].twist.twist.angular.z,
                  outputs[-1].twist.twist.angular.z, outputs[-1].twist.twist.linear.x))
        self.assertEqual([stamp_key(message.header.stamp) for message in outputs],
                         [stamp_key(stamp) for stamp in wheel_stamps],
                         'phase 2: one EKF output per wheel message, stamped with its time')
        self.assert_forward_speed(outputs[-1], 'phase 2')
        self.assert_yaw_rate(outputs[-1], WHEEL_RATE, WHEEL_RATE_TOLERANCE,
                             'phase 2: without IMU samples, the estimate follows the wheels')

    def run_steps_at_the_robot_rates(self, count: int,
                                     imu_inputs: Sequence[ImuInput]) -> List[Odometry]:
        """Run count steps with the wheel odometry in every fifth and return each step's output.

        Every step publishes imu_inputs. The first step publishes the wheel odometry, then
        every STEPS_PER_WHEEL_MESSAGE-th step.
        """
        return [self.step(WHEEL_RATE if index % STEPS_PER_WHEEL_MESSAGE == 0 else None,
                          imu_inputs)
                for index in range(count)]

    def run_wheel_periods_without_imu(self, count: int) -> List[rospy.Time]:
        """Publish count wheel messages, one every fifth step, and return their stamps.

        No step publishes IMU samples. A step with a wheel message waits until the EKF published
        an output stamped at the message's time. Between two wheel messages, the
        STEPS_PER_WHEEL_MESSAGE - 1 steps publish nothing and allow no output. The last step
        publishes the last wheel message.
        """
        output_count_before = self.ekf_outputs.count()
        wheel_stamps: List[rospy.Time] = []
        for index in range(count):
            if index > 0:
                for _ in range(STEPS_PER_WHEEL_MESSAGE - 1):
                    self.step_allowing_no_output(None)
            wheel_stamps.append(self.advance(WHEEL_RATE, ()).stamp)
            self.wait_for_output_stamped(wheel_stamps[-1], output_count_before)
        return wheel_stamps

    def wait_for_output_stamped(self, stamp: rospy.Time, output_count_before: int) -> None:
        """Wait on the wall clock for an EKF output stamped at stamp.

        Look at the outputs after the first output_count_before.
        """
        wait_for(lambda: any(message.header.stamp == stamp
                             for message in self.ekf_outputs.since(output_count_before)),
                 'no EKF output stamped at the wheel message at {:.2f} s'.format(stamp.to_sec()))


# The scenarios, by the name that a .test file gives the test node with --scenario
SCENARIOS: Dict[str, Type[SteppedClockTestCase]] = {
    'invalid_imu_samples': TestEkfWithInvalidImuSamples,
    'robot_rates': TestEkfAtTheRobotRates,
}


def scenario_from_arguments(arguments: Sequence[str]) -> Type[SteppedClockTestCase]:
    """Return the scenario that the argument --scenario names.

    arguments may hold others, such as the results file that rostest gives each test node.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=sorted(SCENARIOS), required=True)
    known_arguments, _ = parser.parse_known_args(arguments)
    return SCENARIOS[known_arguments.scenario]


if __name__ == '__main__':
    scenario = scenario_from_arguments(rospy.myargv()[1:])
    rospy.init_node('test_ekf_synthetic')
    rostest.rosrun('diffbot_control', 'test_ekf_synthetic', scenario)
