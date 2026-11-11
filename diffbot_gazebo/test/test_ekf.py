#!/usr/bin/env python3
"""Check that the EKF's heading follows the simulated robot's true turn (sim_ekf_<model>.test).

The test turns the simulated robot with the EKF on and checks that the EKF's estimate follows the
true turn where the wheel odometry doesn't. The rostest file gives the drive controller a wheel
separation 25 % larger than the robot's. The controller then spins the wheels 25 % faster against
each other than the command asks for, and its odometry, computed with the same wrong separation,
reports the commanded turn: the robot really turns 25 % more than its odometry says, a stand-in
for the slip and calibration errors of a real robot's wheels. The IMU measures the true yaw rate,
so the EKF's heading has to stay with the true one (from Gazebo, /gazebo/model_states), while the
odometry's falls behind.

The same simulation also checks that only the EKF publishes odom → base_footprint
(odom_tf_observer.py) and that the IMU sample validator forwards the simulated IMU at its rate.
The rostest file names the robot in the private parameter ~model.
"""

import math
import os
import time
from typing import Dict
from typing import NamedTuple
import unittest

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from odom_tf_observer import OBSERVE  # this folder's odom_tf_observer.py
from odom_tf_observer import OdomTfObserver
import rospkg
import rospy
import rostest
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import yaml

COMMAND = '/diffbot/mobile_base_controller/cmd_vel'
ODOMETRY = '/diffbot/mobile_base_controller/odom'
FILTERED = '/diffbot/odometry/filtered'
VALIDATED = '/diffbot/imu/data_validated'
MODEL_STATES = '/gazebo/model_states'
MODEL_NAME = 'diffbot'  # the spawned model's name, for DiffBot and Remo (diffbot_view.launch)
EKF_NODE = '/diffbot/ekf_localization'
ODOM_TF_PARAMETER = '/diffbot/mobile_base_controller/enable_odom_tf'
WHEEL_SEPARATION_PARAMETER = '/diffbot/mobile_base_controller/wheel_separation'
COMMAND_QUEUE_SIZE = 1  # only the latest command matters
MODEL_STATES_QUEUE_SIZE = 1  # only the latest true pose matters
IMU_QUEUE_SIZE = 100  # messages, 2 s at IMU_RATE: the test counts them all

# The sources of a heading, as yaws holds them
TRUE = 'true'
ODOMETRY_SOURCE = 'odometry'
EKF_SOURCE = 'EKF'
SOURCES = (TRUE, ODOMETRY_SOURCE, EKF_SOURCE)

TURN_VELOCITY = 0.5  # rad/s, commanded, to the left
DURATION = 3.0  # s
COMMAND_RATE = 20  # Hz
IMU_RATE = 50.0  # Hz
# The controller's wheel separation over the robot's, as the rostest file sets it
SEPARATION_SCALE = 1.25
STARTUP_TIMEOUT = 120.0  # s of wall time
POLL_INTERVAL = 0.1  # s of wall time between checks for the controller and the robot
SETTLE = 1.0  # s of simulation time for the robot to stop before the test reads the headings
# Limits
SEPARATION_TOLERANCE = 1e-6  # m, between the controller's separation and the scaled one
MIN_IMU_RATE_SHARE = 0.9  # the share of IMU_RATE that the validated IMU must reach at least
# rad, the least by which the odometry's turn must fall behind the true one. With the scaled
# separation, the odometry reports about 1 / SEPARATION_SCALE of the true turn
MIN_ODOMETRY_ERROR = 0.15
EKF_TOLERANCE = 0.05  # rad, how far the EKF's turn may differ from the true one


class TurnMeasurement(NamedTuple):
    """What one commanded turn did, by each source of the heading."""

    true_turn: float  # rad, from Gazebo
    odometry_turn: float  # rad, from the drive controller's odometry
    ekf_turn: float  # rad, from the EKF's estimate
    validated_imu_rate: float  # Hz, of the samples that the validator forwarded during the turn

    def report(self) -> str:
        """Return the three turns as text, for the test's output and its failure messages."""
        return 'true {:.3f} rad, odometry {:.3f} rad, EKF {:.3f} rad'.format(
            self.true_turn, self.odometry_turn, self.ekf_turn)


def yaw(pose: Pose) -> float:
    """Return the heading of pose, its rotation about z (rad)."""
    orientation = pose.orientation
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]


def turned(start: float, end: float) -> float:
    """Return the change of heading from start to end (rad), in -pi..pi."""
    return math.atan2(math.sin(end - start), math.cos(end - start))


def nominal_wheel_separation(model: str) -> float:
    """Return the wheel separation (m) in the model's geometry.yaml of diffbot_control."""
    path = os.path.join(rospkg.RosPack().get_path('diffbot_control'), 'config', model,
                        'geometry.yaml')
    with open(path) as geometry_file:
        geometry = yaml.safe_load(geometry_file)
    return float(geometry['diffbot']['mobile_base_controller']['wheel_separation'])


class TestEkf(unittest.TestCase):
    """The tests run in one simulation and don't depend on each other or on their order."""

    yaws: Dict[str, float]  # the latest heading of each source (rad)
    imu_sample_count: int  # validated IMU samples received so far
    observer: OdomTfObserver
    command: rospy.Publisher

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_ekf')
        cls.yaws = {}
        cls.imu_sample_count = 0
        cls.observer = OdomTfObserver()
        cls.command = rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)
        rospy.Subscriber(ODOMETRY, Odometry, cls.on_odometry)
        rospy.Subscriber(FILTERED, Odometry, cls.on_filtered)
        rospy.Subscriber(MODEL_STATES, ModelStates, cls.on_model_states,
                         queue_size=MODEL_STATES_QUEUE_SIZE)
        rospy.Subscriber(VALIDATED, Imu, cls.on_imu, queue_size=IMU_QUEUE_SIZE)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while not cls.ready():
            if time.monotonic() > deadline:
                raise AssertionError(
                    'not ready after {} s: headings from {} of {}, {} subscribers to the '
                    'commands'.format(STARTUP_TIMEOUT, sorted(cls.yaws), SOURCES,
                                      cls.command.get_num_connections()))
            time.sleep(POLL_INTERVAL)

    @classmethod
    def ready(cls) -> bool:
        """Return whether a heading of each source arrived and the controller takes commands."""
        return (all(source in cls.yaws for source in SOURCES)
                and cls.command.get_num_connections() > 0)

    @classmethod
    def on_odometry(cls, message: Odometry) -> None:
        """Note the heading of the controller's odometry."""
        cls.yaws[ODOMETRY_SOURCE] = yaw(message.pose.pose)

    @classmethod
    def on_filtered(cls, message: Odometry) -> None:
        """Note the heading of the EKF's estimate."""
        cls.yaws[EKF_SOURCE] = yaw(message.pose.pose)

    @classmethod
    def on_model_states(cls, message: ModelStates) -> None:
        """Note the robot's true heading in Gazebo."""
        if MODEL_NAME in message.name:
            cls.yaws[TRUE] = yaw(message.pose[message.name.index(MODEL_NAME)])

    @classmethod
    def on_imu(cls, message: Imu) -> None:
        """Count a sample that the validator forwarded."""
        cls.imu_sample_count += 1

    def measure_turn(self) -> TurnMeasurement:
        """Turn the robot on the spot for DURATION, stop it, and return what each source saw."""
        start = dict(self.yaws)
        sample_count = self.imu_sample_count
        started = rospy.Time.now()
        turn = Twist()
        turn.angular.z = TURN_VELOCITY
        rate = rospy.Rate(COMMAND_RATE)
        end = started + rospy.Duration(DURATION)
        try:
            while rospy.Time.now() < end and not rospy.is_shutdown():
                self.command.publish(turn)
                rate.sleep()
            imu_rate = ((self.imu_sample_count - sample_count)
                        / (rospy.Time.now() - started).to_sec())
        finally:
            self.command.publish(Twist())
        rospy.sleep(SETTLE)
        stopped = dict(self.yaws)
        return TurnMeasurement(turned(start[TRUE], stopped[TRUE]),
                               turned(start[ODOMETRY_SOURCE], stopped[ODOMETRY_SOURCE]),
                               turned(start[EKF_SOURCE], stopped[EKF_SOURCE]), imu_rate)

    def assert_wheel_separation_is_scaled(self, separation: float) -> None:
        """Assert that the controller's separation is SEPARATION_SCALE times the robot's."""
        model = rospy.get_param('~model')
        nominal = nominal_wheel_separation(model)
        self.assertAlmostEqual(
            separation, SEPARATION_SCALE * nominal, delta=SEPARATION_TOLERANCE,
            msg="the controller's wheel separation should be {} times {}'s {} m".format(
                SEPARATION_SCALE, model, nominal))

    def assert_validated_imu_rate(self, measurement: TurnMeasurement) -> None:
        """Assert that the validator forwarded the IMU's samples at nearly IMU_RATE."""
        self.assertGreater(
            measurement.validated_imu_rate, MIN_IMU_RATE_SHARE * IMU_RATE,
            'the validator forwarded the IMU at {:.1f} Hz'.format(measurement.validated_imu_rate))

    def assert_odometry_falls_behind(self, measurement: TurnMeasurement) -> None:
        """Assert that the odometry reports a clearly smaller left turn than the true one."""
        self.assertGreater(measurement.true_turn - measurement.odometry_turn, MIN_ODOMETRY_ERROR,
                           'the odometry should fall behind the true turn: '
                           + measurement.report())

    def assert_ekf_follows_the_true_turn(self, measurement: TurnMeasurement) -> None:
        """Assert that the EKF's turn is the true one within EKF_TOLERANCE."""
        self.assertAlmostEqual(measurement.ekf_turn, measurement.true_turn, delta=EKF_TOLERANCE,
                               msg='the EKF should follow the true turn: ' + measurement.report())

    def test_ekf_is_the_only_odom_tf_publisher(self) -> None:
        self.assertTrue(self.observer.wait_for_first_transform(STARTUP_TIMEOUT),
                        'nobody published odom → base_footprint')
        rospy.sleep(OBSERVE)
        counts = self.observer.counts()
        print('odom → base_footprint published by: {}'.format(counts))
        self.assertEqual([EKF_NODE], list(counts),
                         'publishers of odom → base_footprint: {}'.format(counts))
        self.assertFalse(rospy.get_param(ODOM_TF_PARAMETER))

    def test_ekf_follows_the_true_turn_with_a_wrong_wheel_separation(self) -> None:
        separation = rospy.get_param(WHEEL_SEPARATION_PARAMETER)
        self.assert_wheel_separation_is_scaled(separation)
        measurement = self.measure_turn()
        print("controller's wheel separation {:.4f} m; turned: {}; validated IMU {:.1f} Hz".format(
            separation, measurement.report(), measurement.validated_imu_rate))
        self.assert_validated_imu_rate(measurement)
        # The scaled separation shows in the odometry, which reports about 1 / 1.25 of the true
        # turn, and not in the EKF's estimate, which follows the IMU
        self.assert_odometry_falls_behind(measurement)
        self.assert_ekf_follows_the_true_turn(measurement)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_ekf', TestEkf)
