#!/usr/bin/env python3
"""Drives the simulated robot with velocity commands and checks where it went.

The rostest files next to this script start the robot in Gazebo without a GUI, with the diff
drive controller from diffbot_control. Each move is checked twice: with the controller's
odometry, which it computes from the wheel joints and its configuration, and with the robot's
true pose in Gazebo (/gazebo/model_states). Only the true pose shows mistakes that the odometry
repeats, for example swapped wheel joints, which make the robot turn the wrong way while the
odometry still reports the commanded turn.

Durations are simulation time, so the expected distances don't depend on how fast the computer
runs the simulation.
"""
import math
import unittest
from collections import namedtuple

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

CMD_VEL_TOPIC = '/diffbot/mobile_base_controller/cmd_vel'
ODOM_TOPIC = '/diffbot/mobile_base_controller/odom'
MODEL_STATES_TOPIC = '/gazebo/model_states'
# Name of the spawned model in Gazebo, the same for DiffBot and Remo (diffbot_view.launch)
MODEL_NAME = 'diffbot'


# A relative motion: rotate by rot1 (rad), drive straight for trans (m), rotate by rot2 (rad)
Motion = namedtuple('Motion', 'rot1 trans rot2')


def planar(pose):
    """x, y and heading theta (rad) of a geometry_msgs/Pose. The robot drives on flat ground, so
    roll and pitch stay about 0."""
    q = pose.orientation
    _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return pose.position.x, pose.position.y, theta


def wrap(angle):
    """The angle in -pi..pi: theta jumps from pi to -pi when the robot turns past 180 degrees."""
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(start, end):
    """The motion between two poses (x, y, theta), as in the odometry motion model of
    Probabilistic Robotics (Thrun, Burgard, Fox, section 5.4): a rotation rot1 towards the end
    position, a straight translation trans to it, and a rotation rot2 to the end heading.

    The odometry and Gazebo give poses in different frames: the odometry starts at 0, 0, 0 where
    the controller started, while Gazebo's world frame has the robot spawned at x = -2, y = -0.5.
    The relative motion is the same in both frames, so the two can be compared.
    """
    x0, y0, theta0 = start
    x1, y1, theta1 = end
    trans = math.hypot(x1 - x0, y1 - y0)
    # When the robot turns on the spot, its position hardly changes, and the direction of such a
    # short translation is noise: then rot1 is 0 and rot2 is the whole turn, as in ROS amcl's
    # odometry model (amcl_odom.cpp)
    rot1 = wrap(math.atan2(y1 - y0, x1 - x0) - theta0) if trans >= 0.01 else 0.0
    rot2 = wrap(theta1 - theta0 - rot1)
    return Motion(rot1, trans, rot2)


def turned(motion):
    """The angle the robot turned in total (rad, positive to the left)."""
    return wrap(motion.rot1 + motion.rot2)


class TestDrive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_drive')
        cls.odom = None
        cls.truth = None
        cls.cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        rospy.Subscriber(ODOM_TOPIC, Odometry, cls.on_odom)
        rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, cls.on_model_states, queue_size=1)
        # Gazebo and the controllers take a while to start, especially in CI
        rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=120)
        rospy.wait_for_message(MODEL_STATES_TOPIC, ModelStates, timeout=30)

    @classmethod
    def on_odom(cls, msg):
        cls.odom = planar(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg):
        if MODEL_NAME in msg.name:
            cls.truth = planar(msg.pose[msg.name.index(MODEL_NAME)])

    def drive(self, linear, angular, seconds):
        """Sends the velocity for the given simulation time and stops. Returns the relative motion
        measured by the odometry and by the true pose."""
        odom0, truth0 = self.odom, self.truth
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        # Send the command repeatedly: the controller stops the robot when no command arrives for
        # 0.5 s (diff_drive_controller's default cmd_vel_timeout)
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(seconds)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_vel.publish(cmd)
            rate.sleep()
        self.cmd_vel.publish(Twist())
        rospy.sleep(1.0)  # let the robot stop
        odom, truth = relative_motion(odom0, self.odom), relative_motion(truth0, self.truth)
        for name, m in (('odometry', odom), ('true pose', truth)):
            rospy.loginfo('%s: rot1 %.3f rad, trans %.3f m, rot2 %.3f rad', name, m.rot1, m.trans, m.rot2)
        return odom, truth

    # unittest runs the test methods in alphabetical order, here forward and then turn. Each one
    # checks only its own move, from wherever the robot is when it starts, so neither depends on
    # the other

    def test_forward(self):
        # 0.2 m/s for 3 s is 0.6 m, a bit less with the controller's acceleration limit
        odom, truth = self.drive(0.2, 0.0, 3.0)
        for motion in (odom, truth):
            self.assertGreater(motion.trans, 0.4)
            self.assertLess(motion.trans, 0.75)
            # Straight ahead: it moved in the direction it faced (0.002 rad measured), and its
            # heading didn't change
            self.assertLess(abs(motion.rot1), 0.15)
            self.assertLess(abs(turned(motion)), 0.2)
        # The odometry agrees with the true distance (about 0.01 m apart) unless the controller's
        # wheel radius doesn't match the model: 20 % too big gave 0.09 m
        self.assertAlmostEqual(odom.trans, truth.trans, delta=0.05)

    def test_turn(self):
        # 0.5 rad/s for 3 s is 1.5 rad, turning on the spot. The wheels don't always reach the
        # commanded speed when turning on the spot (Remo turned about 1.1 rad), so this only
        # checks that the robot turns left, by a clear amount
        odom, truth = self.drive(0.0, 0.5, 3.0)
        for motion in (odom, truth):
            self.assertGreater(turned(motion), 0.5)
            self.assertLess(turned(motion), 2.0)
            self.assertLess(motion.trans, 0.15)
        # The wheels slip a little when turning on the spot (0.02 to 0.11 rad apart); a wheel
        # radius 20 % too big gave up to 0.31 rad
        self.assertAlmostEqual(turned(odom), turned(truth), delta=0.25)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)
