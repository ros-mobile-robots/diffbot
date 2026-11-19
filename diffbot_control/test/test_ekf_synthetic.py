#!/usr/bin/env python3
"""Check the EKF of ekf.launch on synthetic inputs and a stepped clock (ekf_synthetic.launch).

The test feeds the EKF of ekf.launch, imu_validator.py and robot_localization, synthetic wheel
odometry and IMU samples, and checks its estimate of the yaw rate. It publishes the topic /clock
itself (use_sim_time) and advances it in 20 ms steps, the period of the IMU. The docstring of
step() lists what one step does. A step's waits don't prove that the EKF's callbacks ran before
the clock moved, but they make it very likely, and the checks allow for a late input. In a phase
that waits for an output in every step, a late input is fused one step later. Phase 2 of
robot_rates, whose clock moves on without outputs, allows a late wheel message within its wheel
period. Both inputs show in the estimate: the forward speed comes only from the wheels, the yaw
rate mostly from the IMU.

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
# Wheel periods of STEPS_PER_WHEEL_MESSAGE steps after the IMU stopped. The first step of each
# period publishes a wheel message
IMU_STOPPED_WHEEL_PERIODS = 20
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

# robot_rates' phase 1 changes the wheels' forward speed from SPEED to CHANGED_SPEED with the wheel
# message of this step, the 21st, 2 s into the phase
SPEED_CHANGE_STEP = 100
# m/s, SPEED plus ten times SPEED_TOLERANCE. The filter estimates an acceleration from the change,
# which moves the estimate between two wheel messages: by less than SPEED_TOLERANCE for this change
CHANGED_SPEED = 0.12
# Steps from SPEED_CHANGE_STEP on within which the estimate has to reach CHANGED_SPEED: the second
# wheel message after the change brings it within SPEED_TOLERANCE, one more step allows for a late
# message
SPEED_CHANGE_WINDOW = STEPS_PER_WHEEL_MESSAGE + 1  # 6

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


class WheelInput(NamedTuple):
    """The wheel odometry that a step publishes: what the wheels report."""

    speed: float  # m/s, forward
    yaw_rate: float  # rad/s


# What the wheels report in every wheel message, apart from robot_rates' phase 1 after its change
WHEEL_INPUT = WheelInput(SPEED, WHEEL_RATE)
CHANGED_SPEED_WHEEL_INPUT = WheelInput(CHANGED_SPEED, WHEEL_RATE)


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


def wheel_odometry(stamp: rospy.Time, wheel_input: WheelInput) -> Odometry:
    """Return the drive controller's odometry at stamp, for wheels that report wheel_input."""
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_footprint'
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = wheel_input.speed
    message.twist.twist.angular.z = wheel_input.yaw_rate
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


def wait_until(condition: Callable[[], bool]) -> bool:
    """Wait at most WAIT s of wall time for condition() to be true, and return whether it is."""
    deadline = time.monotonic() + WAIT
    while not condition():
        if time.monotonic() > deadline:
            return False
        time.sleep(POLL_INTERVAL)
    return True


def wait_for(condition: Callable[[], bool], failure: str) -> None:
    """Wait on the wall clock until condition() is true, at most WAIT s, else fail with failure."""
    if not wait_until(condition):
        raise AssertionError(failure)


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

    def publish_inputs(self, stamp: rospy.Time, wheel_input: Optional[WheelInput],
                       imu_inputs: Sequence[ImuInput]) -> List[StampKey]:
        """Publish one step's wheel odometry and IMU samples, stamped relative to stamp.

        A wheel_input of None publishes no wheel odometry in the step. Return the stamp keys of
        the samples that the validator has to forward.
        """
        if wheel_input is not None:
            self.wheel_odometry_publisher.publish(wheel_odometry(stamp, wheel_input))
        valid_stamp_keys = []
        for imu_input in imu_inputs:
            sample_stamp = stamp + rospy.Duration.from_sec(imu_input.offset)
            self.raw_imu_publisher.publish(imu_sample(sample_stamp, imu_input.yaw_rate))
            self.count_expected_outcome(imu_input)
            if imu_input.drop_reason is None:
                valid_stamp_keys.append(stamp_key(sample_stamp))
        return valid_stamp_keys

    def count_expected_outcome(self, imu_input: ImuInput) -> None:
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

    def advance(self, wheel_input: Optional[WheelInput],
                imu_inputs: Sequence[ImuInput]) -> StepStart:
        """Do 1 to 4 of a step, as step() lists them, and return where the step started."""
        start = StepStart(self.now, self.ekf_outputs.count())
        valid_stamp_keys = self.publish_inputs(start.stamp, wheel_input, imu_inputs)
        self.wait_for_forwarded(valid_stamp_keys, start.stamp)
        time.sleep(DELIVERY)
        self.advance_clock()
        return start

    def outputs_since(self, start: StepStart) -> List[Odometry]:
        """Return the EKF's outputs that arrived after the step's start and belong to the step.

        An output belongs to the step if its stamp lies between the step's time and the ROS time
        that the test has published. An output that arrives late from before the clock went back
        carries a later stamp, and one from an earlier step an earlier stamp.
        """
        return [message for message in self.ekf_outputs.since(start.output_count_before)
                if start.stamp <= message.header.stamp <= self.now]

    def step(self, wheel_input: Optional[WheelInput],
             imu_inputs: Sequence[ImuInput] = ()) -> Odometry:
        """Run one step and return the EKF's last output for it, which has to come.

        One step:
        1. Publish the step's wheel odometry, if wheel_input isn't None, stamped with the step's
           ROS time, and its IMU samples, each stamped with that time plus the sample's offset
           (publish_inputs()).
        2. Wait on the wall clock until the IMU sample validator's output for the step's valid
           samples reaches the test (wait_for_forwarded()). The EKF subscribes to the same
           topic.
        3. Wait DELIVERY more, for the inputs to arrive at the EKF.
        4. Advance the clock by STEP (advance_clock()). The EKF's 50 Hz update then processes the
           inputs.
        5. Wait, with the time limit WAIT, for an EKF output that belongs to the step
           (outputs_since()).
        advance() does 1 to 4.
        """
        start = self.advance(wheel_input, imu_inputs)
        wait_for(lambda: bool(self.outputs_since(start)),
                 'no EKF output for the step at {:.2f} s'.format(start.stamp.to_sec()))
        return self.outputs_since(start)[-1]

    def step_allowing_no_output(self, wheel_input: Optional[WheelInput],
                                imu_inputs: Sequence[ImuInput] = ()) -> Optional[Odometry]:
        """Run one step and return the EKF's last output for it, or None if none came."""
        start = self.advance(wheel_input, imu_inputs)
        time.sleep(NO_OUTPUT_WAIT)
        outputs = self.outputs_since(start)
        return outputs[-1] if outputs else None

    def run_steps(self, count: int, wheel_input: WheelInput,
                  imu_inputs: Sequence[ImuInput] = ()) -> List[Odometry]:
        """Run count steps with the same inputs and return the EKF's output for each."""
        return [self.step(wheel_input, imu_inputs) for _ in range(count)]

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

    def assert_forward_speed(self, output: Odometry, speed: float, what: str) -> None:
        """Assert that the output's forward speed is the wheels' speed. what names the case."""
        self.assertAlmostEqual(output.twist.twist.linear.x, speed, delta=SPEED_TOLERANCE,
                               msg=what + ': the forward speed comes from the wheels')

    def assert_fused_estimate(self, output: Odometry, speed: float, what: str) -> None:
        """Assert that output has the EKF's frames, the wheels' speed and the IMU's yaw rate.

        what names the case.
        """
        self.assertEqual(output.header.frame_id, 'odom', what)
        self.assertEqual(output.child_frame_id, 'base_footprint', what)
        self.assert_forward_speed(output, speed, what)
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
        """Check that the estimate follows the IMU's yaw rate when the wheels report another."""
        outputs = self.run_steps(FUSION_STEPS, WHEEL_INPUT, [VALID_SAMPLE])
        self.print_fusion('fusion', outputs[-1])
        self.assert_fused_estimate(outputs[-1], SPEED, 'phase 1')

    def check_estimate_follows_the_wheels_when_imu_samples_are_stale(self) -> None:
        """Check that the estimate goes to the wheels' yaw rate when every IMU sample is stale."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_INPUT,
                                 [ImuInput(STALE_OFFSET, BAD_RATE, TOO_OLD)])
        print('stale IMU: estimate from {:.4f} to {:.4f} rad/s'.format(
            outputs[0].twist.twist.angular.z, outputs[-1].twist.twist.angular.z))
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 2: stale samples reporting 2.0 rad/s')
        self.assert_yaw_rate(outputs[-1], WHEEL_RATE, WHEEL_RATE_TOLERANCE,
                             'phase 2: without IMU samples, the estimate follows the wheels')

    def check_validator_drops_samples_with_a_repeated_stamp(self) -> None:
        """Check that the estimate stays at the valid samples' rate despite repeated stamps."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_INPUT,
                                 [VALID_SAMPLE,
                                  ImuInput(VALID_SAMPLE.offset, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 3: samples with a repeated stamp')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE,
                             'phase 3: valid samples again')

    def check_validator_drops_samples_from_the_future(self) -> None:
        """Check that the estimate stays at the valid samples' rate despite future stamps."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_INPUT,
                                 [VALID_SAMPLE,
                                  ImuInput(FUTURE_OFFSET, BAD_RATE, FROM_THE_FUTURE)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 4: samples from the future')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'phase 4: valid samples')

    def check_validator_drops_samples_older_than_the_last_valid_one(self) -> None:
        """Check that the estimate stays at the valid samples' rate despite older stamps."""
        outputs = self.run_steps(PHASE_STEPS, WHEEL_INPUT,
                                 [VALID_SAMPLE, ImuInput(OLDER_OFFSET, BAD_RATE, NOT_NEWER)])
        self.assert_stays_at_most(outputs, IMU_RATE + YAW_RATE_TOLERANCE,
                                  'phase 5: samples older than the last valid one')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE, 'phase 5: valid samples')

    def check_estimate_follows_the_imu_again_after_the_clock_goes_back(self) -> None:
        """Check that the estimate follows the IMU again after the clock goes back to RESTART."""
        self.now = RESTART
        self.clock_publisher.publish(Clock(self.now))
        time.sleep(RESET_DELIVERY)
        # The first measurement after the reset initializes the filter, and robot_localization
        # doesn't publish that estimate, because its stamp is older than the last one it
        # published. The measurement that showed the jump is dropped, and so can be the other
        # one of its step: the reset clears the node's callback queue. So the outputs start
        # again within a few steps
        for _ in range(RESET_STEPS):
            if self.step_allowing_no_output(WHEEL_INPUT, [VALID_SAMPLE]) is not None:
                break
        else:
            self.fail('phase 6: no EKF output within {} steps after the clock went back'.format(
                RESET_STEPS))
        outputs = self.run_steps(AFTER_RESET_STEPS, WHEEL_INPUT, [VALID_SAMPLE])
        print('after the clock went back to {:.0f} s: estimate {:.4f} rad/s at {:.2f} s'.format(
            RESTART.to_sec(), outputs[-1].twist.twist.angular.z,
            outputs[-1].header.stamp.to_sec()))
        self.assertLess(outputs[0].header.stamp, START,
                        'phase 6: the outputs should carry the new time')
        self.assert_yaw_rate(outputs[-1], IMU_RATE, YAW_RATE_TOLERANCE,
                             'phase 6: after the clock went back')

    def check_validator_diagnostics_count_each_forwarded_and_dropped_sample(self) -> None:
        """Check that the validator's diagnostics count each sample that phases 1 to 6 sent."""
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
            self.step(WHEEL_INPUT)
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
       IMU, with the wheels' forward speed also between two wheel messages. From the 21st wheel
       message on, the wheels report 0.12 m/s instead of 0.1 m/s, and the estimate's forward
       speed follows within 6 steps.
    2. The IMU stops, the wheel odometry goes on in every fifth step, at 0.1 m/s again, and the
       clock moves on in every step. The filter publishes one output per wheel period, and the
       estimate goes to the wheels' 0.4 rad/s and 0.1 m/s.
    """

    def setUp(self) -> None:
        super().setUp()
        # The stamps of the wheel messages on their topic, which phase 2 counts
        self.wheel_message_stamps: ArrivalLog[rospy.Time] = ArrivalLog()
        self.wheel_odometry_subscriber = rospy.Subscriber(
            WHEEL_ODOMETRY_TOPIC, Odometry, self.on_wheel_odometry, queue_size=QUEUE_SIZE)
        wait_for(lambda: self.wheel_odometry_subscriber.get_num_connections() > 0,
                 "the test's subscriber to the wheel odometry didn't connect")

    def on_wheel_odometry(self, message: Odometry) -> None:
        """Note the stamp of a wheel message on the topic."""
        self.wheel_message_stamps.append(message.header.stamp)

    def test_ekf_follows_the_imu_at_the_robot_rates_and_the_wheels_after_the_imu_stops(
            self) -> None:
        self.check_estimate_follows_the_imu_and_a_wheel_speed_change_at_the_robot_rates()
        self.check_ekf_publishes_at_the_wheel_rate_and_follows_the_wheels_when_the_imu_stops()

    def check_estimate_follows_the_imu_and_a_wheel_speed_change_at_the_robot_rates(
            self) -> None:
        """Check that the estimate follows the IMU's yaw rate and the change of the wheels' speed.

        The wheel messages report SPEED before SPEED_CHANGE_STEP and CHANGED_SPEED from it on.
        The forward speed comes only from the wheels, so the estimate reaches CHANGED_SPEED only
        if the filter fuses wheel messages after the change.
        """
        # The step before SPEED_CHANGE_STEP and the last of the FUSION_STEPS steps are each the
        # fourth after a wheel message, so the checks of the fused estimate look at outputs between
        # two wheel messages
        outputs = self.run_steps_at_the_robot_rates(FUSION_STEPS, [VALID_SAMPLE])
        self.print_fusion("fusion at the robot's rates", outputs[-1])
        self.print_speed_change(outputs)
        self.assert_fused_estimate(outputs[SPEED_CHANGE_STEP - 1], SPEED,
                                   'phase 1, before the speed change')
        self.assert_forward_speeds(outputs[SPEED_CHANGE_STEP + SPEED_CHANGE_WINDOW:],
                                   CHANGED_SPEED, 'phase 1, after the speed change')
        self.assert_fused_estimate(outputs[-1], CHANGED_SPEED, 'phase 1')

    def check_ekf_publishes_at_the_wheel_rate_and_follows_the_wheels_when_the_imu_stops(
            self) -> None:
        """Check that the EKF follows the wheels, one output per wheel period, once the IMU stops.

        Without the IMU, the filter has a measurement to fuse only when a wheel message arrives.
        Without one, the filter predicts only when the ROS time is sensor_timeout (ekf.yaml, 0.1 s)
        past its last measurement, and that is the wheel odometry's period. The filter publishes no
        second output with the same stamp. So it publishes one output per wheel period, as long as
        each wheel message reaches the filter, and the filter's updates run, before the clock
        reaches the next wheel message's stamp. The steps' sleeps give a message about 0.9 s of
        wall time for that, 5 DELIVERY and 4 NO_OUTPUT_WAIT, but they don't prove that the filter's
        updates ran. When the message reaches the filter before the update STEP after its stamp,
        the output is stamped with the message's time: the prediction to that time, published
        before the filter fused the message, or the fused message. A message that arrives later can
        leave an update of its period to predict, with that update's time, or is fused in a later
        update. So the check allows any stamp within the period. The test also takes the wheel
        messages from their topic and checks one at the start of each period. It picks outputs
        and wheel messages by their stamps, so that one whose callback runs late counts in its
        own phase. It reads them after one more wheel period, whose wheel message and an output
        from its start on show that every callback of the measured periods has run
        (wait_for_messages_from()). The wheel messages report SPEED again, so the estimate's
        forward speed goes back from phase 1's CHANGED_SPEED only if the filter fuses them.
        """
        period_starts = self.run_wheel_periods_without_imu(IMU_STOPPED_WHEEL_PERIODS)
        [next_period_start] = self.run_wheel_periods_without_imu(1)
        missing_after_the_periods = self.wait_for_messages_from(next_period_start)
        outputs = self.outputs_within_the_periods(period_starts)
        wheel_message_stamps = self.wheel_message_stamps_within_the_periods(period_starts)
        if not outputs:
            self.fail('phase 2: no EKF output in the {} wheel periods without the IMU'.format(
                len(period_starts)))
        self.print_imu_stopped(period_starts, wheel_message_stamps, outputs)
        self.assertFalse(missing_after_the_periods,
                         'phase 2: after the wheel periods, the test got no {}, so it may lack '
                         'messages of the periods too'.format(
                             ' and no '.join(missing_after_the_periods)))
        self.assert_one_wheel_message_per_period(wheel_message_stamps, period_starts)
        self.assert_one_output_per_period(outputs, period_starts)
        self.assert_forward_speed(outputs[-1], SPEED, 'phase 2')
        self.assert_yaw_rate(outputs[-1], WHEEL_RATE, WHEEL_RATE_TOLERANCE,
                             'phase 2: without IMU samples, the estimate follows the wheels')

    def run_steps_at_the_robot_rates(self, count: int,
                                     imu_inputs: Sequence[ImuInput]) -> List[Odometry]:
        """Run count steps with the wheel odometry in every fifth and return each step's output.

        Every step publishes imu_inputs, and the steps that wheel_input_at_the_robot_rates()
        gives one publish the wheel odometry.
        """
        return [self.step(wheel_input_at_the_robot_rates(index), imu_inputs)
                for index in range(count)]

    def run_wheel_periods_without_imu(self, count: int) -> List[rospy.Time]:
        """Run count wheel periods without IMU samples and return the times when they start.

        A wheel period is STEPS_PER_WHEEL_MESSAGE steps. Its first step publishes a wheel
        message, stamped with the period's start, the others publish nothing. Every step allows
        no output: the clock moves on whether the filter published or not, after NO_OUTPUT_WAIT
        for the filter's update.
        """
        period_starts: List[rospy.Time] = []
        for _ in range(count):
            period_starts.append(self.now)
            self.step_allowing_no_output(WHEEL_INPUT)
            for _ in range(STEPS_PER_WHEEL_MESSAGE - 1):
                self.step_allowing_no_output(None)
        return period_starts

    def wait_for_messages_from(self, stamp: rospy.Time) -> List[str]:
        """Wait for a wheel message stamped stamp and an output from stamp on, return the missing.

        The wait lasts at most WAIT s and doesn't fail. Each subscription gets the messages of its
        one publisher in order and runs their callbacks in that order. So when both messages have
        arrived, the callbacks of the earlier wheel messages and outputs have run. The caller moves
        the clock on past stamp first, through a whole wheel period, so that the filter publishes
        an output even when the wheel message reaches it late. Return the names of the messages
        that didn't arrive.
        """
        arrivals: Dict[str, Callable[[], bool]] = {
            'wheel message at {:.2f} s'.format(stamp.to_sec()):
                lambda: self.wheel_message_stamps.contains_all([stamp]),
            'EKF output stamped at {:.2f} s or later'.format(stamp.to_sec()):
                lambda: any(output.header.stamp >= stamp for output in self.ekf_outputs.since(0)),
        }
        wait_until(lambda: all(arrived() for arrived in arrivals.values()))
        return [name for name, arrived in arrivals.items() if not arrived()]

    def outputs_within_the_periods(self, period_starts: Sequence[rospy.Time]) -> List[Odometry]:
        """Return the EKF's outputs stamped within the wheel periods, in the order of arrival."""
        return [message for message in self.ekf_outputs.since(0)
                if within_wheel_periods(message.header.stamp, period_starts)]

    def wheel_message_stamps_within_the_periods(
            self, period_starts: Sequence[rospy.Time]) -> List[rospy.Time]:
        """Return the stamps of the wheel messages within the periods, in the order of arrival."""
        return [stamp for stamp in self.wheel_message_stamps.since(0)
                if within_wheel_periods(stamp, period_starts)]

    def assert_forward_speeds(self, outputs: Sequence[Odometry], speed: float,
                              what: str) -> None:
        """Assert that each output's forward speed is the wheels' speed. what names the case."""
        for output in outputs:
            self.assert_forward_speed(
                output, speed, '{} at {:.2f} s'.format(what, output.header.stamp.to_sec()))

    def assert_one_wheel_message_per_period(self, wheel_message_stamps: Sequence[rospy.Time],
                                            period_starts: Sequence[rospy.Time]) -> None:
        """Assert that one wheel message arrived, stamped with each period's start, in order."""
        received = [stamp.to_sec() for stamp in wheel_message_stamps]
        expected = [start.to_sec() for start in period_starts]
        missing = [start for start in expected if start not in received]
        self.assertEqual(received, expected,
                         'phase 2: one wheel message at the start of each wheel period'
                         + (', none at {} s'.format(missing) if missing else ''))

    def assert_one_output_per_period(self, outputs: Sequence[Odometry],
                                     period_starts: Sequence[rospy.Time]) -> None:
        """Assert that each wheel period holds one output's stamp, in the order of the periods."""
        self.assertEqual([wheel_period_index(message.header.stamp, period_starts)
                          for message in outputs],
                         list(range(len(period_starts))),
                         'phase 2: one EKF output per wheel period, in the order of the periods')

    def print_speed_change(self, outputs: Sequence[Odometry]) -> None:
        """Print the estimated forward speed before, at and after phase 1's change of the speed."""
        # The stamp of the first wheel message that reports CHANGED_SPEED
        change_stamp = START + STEP * SPEED_CHANGE_STEP
        speeds_after_the_window = [output.twist.twist.linear.x
                                   for output in outputs[SPEED_CHANGE_STEP + SPEED_CHANGE_WINDOW:]]
        print('speed change at {:.2f} s, wheels {} to {} m/s: estimate {:.4f} m/s before, {:.4f} '
              'm/s in the step of the change, {:.4f} to {:.4f} m/s from {} steps after it'.format(
                  change_stamp.to_sec(), SPEED, CHANGED_SPEED,
                  outputs[SPEED_CHANGE_STEP - 1].twist.twist.linear.x,
                  outputs[SPEED_CHANGE_STEP].twist.twist.linear.x, min(speeds_after_the_window),
                  max(speeds_after_the_window), SPEED_CHANGE_WINDOW))

    def print_imu_stopped(self, period_starts: Sequence[rospy.Time],
                          wheel_message_stamps: Sequence[rospy.Time],
                          outputs: Sequence[Odometry]) -> None:
        """Print how many wheel periods, wheel messages and outputs phase 2 had, and estimates."""
        stamped_at_a_wheel_message = [message for message in outputs
                                      if message.header.stamp in wheel_message_stamps]
        print('IMU stopped: {} outputs for {} wheel periods with {} wheel messages, {} of the '
              'outputs stamped at a wheel message, estimate from {:.4f} to {:.4f} rad/s, '
              'forward from {:.4f} to {:.4f} m/s'.format(
                  len(outputs), len(period_starts), len(wheel_message_stamps),
                  len(stamped_at_a_wheel_message), outputs[0].twist.twist.angular.z,
                  outputs[-1].twist.twist.angular.z, outputs[0].twist.twist.linear.x,
                  outputs[-1].twist.twist.linear.x))


def wheel_input_at_the_robot_rates(step_index: int) -> Optional[WheelInput]:
    """Return the wheel odometry of phase 1's step at step_index in robot_rates, None for none.

    The first step publishes the wheel odometry, then every STEPS_PER_WHEEL_MESSAGE-th step:
    WHEEL_INPUT before SPEED_CHANGE_STEP, CHANGED_SPEED_WHEEL_INPUT from it on.
    """
    if step_index % STEPS_PER_WHEEL_MESSAGE != 0:
        return None
    return WHEEL_INPUT if step_index < SPEED_CHANGE_STEP else CHANGED_SPEED_WHEEL_INPUT


def within_wheel_periods(stamp: rospy.Time, period_starts: Sequence[rospy.Time]) -> bool:
    """Return whether stamp lies from the first period's start to before the last period's end."""
    return period_starts[0] <= stamp < period_starts[-1] + ROBOT_WHEEL_ODOMETRY_PERIOD


def wheel_period_index(stamp: rospy.Time, period_starts: Sequence[rospy.Time]) -> int:
    """Return the index of the wheel period that stamp lies in, -1 before the first.

    A wheel period starts at its wheel message's stamp and ends where the next one starts.
    """
    return sum(1 for period_start in period_starts if period_start <= stamp) - 1


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
