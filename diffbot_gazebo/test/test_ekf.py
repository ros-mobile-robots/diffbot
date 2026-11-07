#!/usr/bin/env python3
"""Turns the simulated robot with the EKF on (sim_ekf_<model>.test) and checks that the EKF's
estimate follows the true turn where the wheel odometry doesn't.

The rostest file gives the drive controller a wheel separation 25 % larger than the robot's. The
controller then spins the wheels 25 % faster against each other than the command asks for, and its
odometry, computed with the same wrong separation, reports the commanded turn: the robot really
turns 25 % more than its odometry says, a stand-in for the slip and calibration errors of a real
robot's wheels. The IMU measures the true yaw rate, so the EKF's heading has to stay with the true
one (from Gazebo, /gazebo/model_states), while the odometry's falls behind.

The same simulation also checks that only the EKF publishes odom → base_footprint (test_odom_tf.py)
and that the IMU gate passes the simulated IMU on at its rate.
"""

import math
import time
import unittest

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from test_odom_tf import OdomTfPublishers

COMMAND = "/diffbot/mobile_base_controller/cmd_vel"
ODOMETRY = "/diffbot/mobile_base_controller/odom"
FILTERED = "/diffbot/odometry/filtered"
VALIDATED = "/diffbot/imu/data_validated"
MODEL_STATES = "/gazebo/model_states"
MODEL_NAME = "diffbot"  # the spawned model's name, for DiffBot and Remo (diffbot_view.launch)
EKF_NODE = "/diffbot/ekf_localization"

TURN_VELOCITY = 0.5  # rad/s, commanded
DURATION = 3.0  # s
COMMAND_RATE = 20  # Hz
IMU_RATE = 50.0  # Hz
SEPARATION_ERROR = 1.25  # the controller's wheel separation over the robot's
STARTUP_TIMEOUT = 120.0  # s of wall time


def yaw(pose):
    q = pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def turned(start, end):
    """The change of heading from start to end (rad), in -pi..pi."""
    return math.atan2(math.sin(end - start), math.cos(end - start))


class TestEkf(unittest.TestCase):
    """unittest runs the tests in the order of their names, in one simulation."""

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_ekf")
        cls.yaws = {}
        cls.imu_samples = 0
        cls.publishers = OdomTfPublishers()
        cls.command = rospy.Publisher(COMMAND, Twist, queue_size=1)
        rospy.Subscriber(ODOMETRY, Odometry, lambda m: cls.yaws.__setitem__("odometry", yaw(m.pose.pose)))
        rospy.Subscriber(FILTERED, Odometry, lambda m: cls.yaws.__setitem__("EKF", yaw(m.pose.pose)))
        rospy.Subscriber(MODEL_STATES, ModelStates, cls.on_model_states, queue_size=1)
        rospy.Subscriber(VALIDATED, Imu, cls.on_imu, queue_size=100)
        for topic in (ODOMETRY, FILTERED):
            rospy.wait_for_message(topic, Odometry, timeout=STARTUP_TIMEOUT)
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while cls.command.get_num_connections() == 0 or "true" not in cls.yaws:
            if time.monotonic() > deadline:
                raise AssertionError("the controller doesn't subscribe to the commands, or Gazebo has no robot")
            time.sleep(0.1)

    @classmethod
    def on_model_states(cls, message):
        if MODEL_NAME in message.name:
            cls.yaws["true"] = yaw(message.pose[message.name.index(MODEL_NAME)])

    @classmethod
    def on_imu(cls, message):
        cls.imu_samples += 1

    def test_1_only_the_ekf_publishes_odom_tf(self):
        counts = self.publishers.observe(self)
        print("odom → base_footprint published by: {}".format(counts))
        self.assertEqual([EKF_NODE], list(counts), "publishers of odom → base_footprint: {}".format(counts))
        self.assertFalse(rospy.get_param("/diffbot/mobile_base_controller/enable_odom_tf"))

    def test_2_ekf_follows_the_true_turn(self):
        separation = rospy.get_param("/diffbot/mobile_base_controller/wheel_separation")
        start = dict(self.yaws)
        samples = self.imu_samples
        started = rospy.Time.now()
        turn = Twist()
        turn.angular.z = TURN_VELOCITY
        rate = rospy.Rate(COMMAND_RATE)
        end = started + rospy.Duration(DURATION)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.command.publish(turn)
            rate.sleep()
        imu_rate = (self.imu_samples - samples) / (rospy.Time.now() - started).to_sec()
        self.command.publish(Twist())
        rospy.sleep(1.0)
        turns = {source: turned(start[source], self.yaws[source]) for source in ("true", "odometry", "EKF")}
        report = ", ".join("{} {:.3f} rad".format(source, value) for source, value in turns.items())
        print("controller's wheel separation {:.4f} m; turned: {}; IMU after the gate {:.1f} Hz".format(
            separation, report, imu_rate))
        self.assertGreater(imu_rate, 0.9 * IMU_RATE, "the gate passed the IMU on at {:.1f} Hz".format(imu_rate))
        # The injected error shows in the odometry: it reports about 1 / 1.25 of the true turn
        odometry_error = abs(turns["odometry"] - turns["true"])
        self.assertGreater(odometry_error, 0.15, "the odometry should fall behind the true turn: " + report)
        # and not in the EKF's estimate, which follows the IMU
        self.assertAlmostEqual(turns["EKF"], turns["true"], delta=0.05,
                               msg="the EKF should follow the true turn: " + report)


if __name__ == "__main__":
    rostest.rosrun("diffbot_gazebo", "test_ekf", TestEkf)
