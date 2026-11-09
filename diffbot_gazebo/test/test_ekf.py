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

The same simulation also checks that only the EKF publishes odom → base_footprint (test_odom_tf.py)
and that the IMU sample validator forwards the simulated IMU at its rate.
"""

import math
import time
from typing import Dict
import unittest

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest
from sensor_msgs.msg import Imu
from test_odom_tf import OdomTfPublishers  # this folder's test_odom_tf.py
from tf.transformations import euler_from_quaternion

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

TURN_VELOCITY = 0.5  # rad/s, commanded
DURATION = 3.0  # s
COMMAND_RATE = 20  # Hz
IMU_RATE = 50.0  # Hz
SEPARATION_ERROR = 1.25  # the controller's wheel separation over the robot's
STARTUP_TIMEOUT = 120.0  # s of wall time
POLL_INTERVAL = 0.1  # s of wall time between checks for the controller and the robot
SETTLE = 1.0  # s of simulation time for the robot to stop before the test reads the headings
# Tolerances
IMU_RATE_TOLERANCE = 0.9  # the share of IMU_RATE that the validated IMU must reach at least
# rad, the minimum absolute difference between the odometry's turn and the true one. The injected
# error makes the odometry report about 1 / SEPARATION_ERROR of the true turn
MIN_ODOMETRY_ERROR = 0.15
EKF_TOLERANCE = 0.05  # rad, how far the EKF's turn may differ from the true one


def yaw(pose: Pose) -> float:
    """Return the heading of pose, its rotation about z (rad)."""
    orientation = pose.orientation
    return euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]


def turned(start: float, end: float) -> float:
    """Return the change of heading from start to end (rad), in -pi..pi."""
    return math.atan2(math.sin(end - start), math.cos(end - start))


class TestEkf(unittest.TestCase):
    """unittest runs the tests in the order of their names, in one simulation."""

    yaws: Dict[str, float]  # the latest heading of each source (rad)
    imu_sample_count: int  # validated IMU samples received so far
    publishers: OdomTfPublishers
    command: rospy.Publisher

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_ekf')
        cls.yaws = {}
        cls.imu_sample_count = 0
        cls.publishers = OdomTfPublishers()
        cls.command = rospy.Publisher(COMMAND, Twist, queue_size=COMMAND_QUEUE_SIZE)
        rospy.Subscriber(ODOMETRY, Odometry, cls.on_odometry)
        rospy.Subscriber(FILTERED, Odometry, cls.on_filtered)
        rospy.Subscriber(MODEL_STATES, ModelStates, cls.on_model_states,
                         queue_size=MODEL_STATES_QUEUE_SIZE)
        rospy.Subscriber(VALIDATED, Imu, cls.on_imu, queue_size=IMU_QUEUE_SIZE)
        for topic in (ODOMETRY, FILTERED):
            rospy.wait_for_message(topic, Odometry, timeout=STARTUP_TIMEOUT)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while cls.command.get_num_connections() == 0 or TRUE not in cls.yaws:
            if time.monotonic() > deadline:
                raise AssertionError(
                    "the controller doesn't subscribe to the commands, or Gazebo has no robot")
            time.sleep(POLL_INTERVAL)

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

    def test_1_only_the_ekf_publishes_odom_tf(self) -> None:
        counts = self.publishers.observe(self)
        print('odom → base_footprint published by: {}'.format(counts))
        self.assertEqual([EKF_NODE], list(counts),
                         'publishers of odom → base_footprint: {}'.format(counts))
        self.assertFalse(rospy.get_param(ODOM_TF_PARAMETER))

    def test_2_ekf_follows_the_true_turn_with_a_wrong_wheel_separation(self) -> None:
        separation = rospy.get_param(WHEEL_SEPARATION_PARAMETER)
        start = dict(self.yaws)
        sample_count = self.imu_sample_count
        started = rospy.Time.now()
        turn = Twist()
        turn.angular.z = TURN_VELOCITY
        rate = rospy.Rate(COMMAND_RATE)
        end = started + rospy.Duration(DURATION)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.command.publish(turn)
            rate.sleep()
        imu_rate = (self.imu_sample_count - sample_count) / (rospy.Time.now() - started).to_sec()
        self.command.publish(Twist())
        rospy.sleep(SETTLE)
        turns = {source: turned(start[source], self.yaws[source])
                 for source in (TRUE, ODOMETRY_SOURCE, EKF_SOURCE)}
        report = ', '.join('{} {:.3f} rad'.format(source, turn) for source, turn in turns.items())
        print("controller's wheel separation {:.4f} m; turned: {}; validated IMU {:.1f} Hz".format(
            separation, report, imu_rate))
        self.assertGreater(imu_rate, IMU_RATE_TOLERANCE * IMU_RATE,
                           'the validator forwarded the IMU at {:.1f} Hz'.format(imu_rate))
        # The injected error shows in the odometry: it reports about 1 / 1.25 of the true turn
        odometry_error = abs(turns[ODOMETRY_SOURCE] - turns[TRUE])
        self.assertGreater(odometry_error, MIN_ODOMETRY_ERROR,
                           'the odometry should fall behind the true turn: ' + report)
        # and not in the EKF's estimate, which follows the IMU
        self.assertAlmostEqual(turns[EKF_SOURCE], turns[TRUE], delta=EKF_TOLERANCE,
                               msg='the EKF should follow the true turn: ' + report)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_ekf', TestEkf)
