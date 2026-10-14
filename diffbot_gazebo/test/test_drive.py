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

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_VEL = '/diffbot/mobile_base_controller/cmd_vel'
ODOM = '/diffbot/mobile_base_controller/odom'
MODEL_STATES = '/gazebo/model_states'
MODEL = 'diffbot'  # name of the spawned model in Gazebo, for DiffBot and Remo (diffbot_view.launch)


def yaw_of(orientation):
    """Yaw angle (rad) of a geometry_msgs/Quaternion."""
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def planar(pose):
    """x, y and yaw of a geometry_msgs/Pose."""
    return pose.position.x, pose.position.y, yaw_of(pose.orientation)


def change(before, after):
    """Pose change in the starting frame: forward and left (m), and the turned angle (rad)."""
    x0, y0, yaw0 = before
    x1, y1, yaw1 = after
    dx, dy = x1 - x0, y1 - y0
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    turned = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    return forward, left, turned


class TestDrive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_drive')
        cls.odom = None
        cls.truth = None
        cls.cmd_vel = rospy.Publisher(CMD_VEL, Twist, queue_size=1)
        rospy.Subscriber(ODOM, Odometry, cls.on_odom)
        rospy.Subscriber(MODEL_STATES, ModelStates, cls.on_model_states, queue_size=1)
        # Gazebo and the controllers take a while to start, especially in CI
        rospy.wait_for_message(ODOM, Odometry, timeout=120)
        rospy.wait_for_message(MODEL_STATES, ModelStates, timeout=30)

    @classmethod
    def on_odom(cls, msg):
        cls.odom = planar(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg):
        if MODEL in msg.name:
            cls.truth = planar(msg.pose[msg.name.index(MODEL)])

    def drive(self, linear, angular, seconds):
        """Sends the velocity for the given simulation time and stops. Returns the change of the
        odometry and of the true pose, each as (forward, left, turned)."""
        odom0, truth0 = self.odom, self.truth
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(seconds)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_vel.publish(cmd)
            rate.sleep()
        self.cmd_vel.publish(Twist())
        rospy.sleep(1.0)  # let the robot stop
        odom, truth = change(odom0, self.odom), change(truth0, self.truth)
        for name, (forward, left, turned) in (('odometry', odom), ('true pose', truth)):
            rospy.loginfo('%s: forward %.3f m, left %.3f m, turned %.3f rad', name, forward, left, turned)
        return odom, truth

    def test_1_forward(self):
        # 0.2 m/s for 3 s is 0.6 m, a bit less with the controller's acceleration limit
        odom, truth = self.drive(0.2, 0.0, 3.0)
        for forward, left, turned in (odom, truth):
            self.assertGreater(forward, 0.4)
            self.assertLess(forward, 0.75)
            self.assertLess(abs(left), 0.1)
            self.assertLess(abs(turned), 0.2)
        # The odometry agrees with the true distance (about 0.01 m apart) unless the controller's
        # wheel radius doesn't match the model: 20 % too big gave 0.09 m
        self.assertAlmostEqual(odom[0], truth[0], delta=0.05)

    def test_2_turn(self):
        # 0.5 rad/s for 3 s is 1.5 rad, turning on the spot. The wheels don't always reach the
        # commanded speed when turning on the spot (Remo turned about 1.1 rad), so this only
        # checks that the robot turns left, by a clear amount
        odom, truth = self.drive(0.0, 0.5, 3.0)
        for forward, left, turned in (odom, truth):
            self.assertGreater(turned, 0.5)
            self.assertLess(turned, 2.0)
            self.assertLess(math.hypot(forward, left), 0.15)
        # The wheels slip a little when turning on the spot (0.02 to 0.11 rad apart); a wheel
        # radius 20 % too big gave up to 0.31 rad
        self.assertAlmostEqual(odom[2], truth[2], delta=0.25)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)
