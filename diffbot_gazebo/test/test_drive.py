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
from tf.transformations import euler_from_quaternion

CMD_VEL_TOPIC = '/diffbot/mobile_base_controller/cmd_vel'
ODOM_TOPIC = '/diffbot/mobile_base_controller/odom'
MODEL_STATES_TOPIC = '/gazebo/model_states'
# Name of the spawned model in Gazebo, the same for DiffBot and Remo (diffbot_view.launch)
MODEL_NAME = 'diffbot'


def planar(pose):
    """x, y and yaw (rad) of a geometry_msgs/Pose. The robot drives on flat ground, so roll and
    pitch stay about 0."""
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return pose.position.x, pose.position.y, yaw


def moved(start, end):
    """How the robot moved between two planar poses (x, y, yaw), seen from the start pose:
    forward and to the left (m), and the angle it turned (rad, positive to the left).

    The odometry and Gazebo give poses in different frames: the odometry starts at 0, 0, 0 where
    the controller started, while Gazebo's world frame has the robot spawned at x = -2, y = -0.5.
    So the poses can't be compared directly, but the moves can, relative to the start pose.
    """
    x0, y0, yaw0 = start
    x1, y1, yaw1 = end
    # The move in the frame of the poses (odometry or Gazebo world)
    dx, dy = x1 - x0, y1 - y0
    # The same move in the robot's frame at the start, by rotating it by -yaw0: forward is its
    # part along the robot's heading (cos yaw0, sin yaw0), left its part along the robot's left
    # side (-sin yaw0, cos yaw0)
    forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    left = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    # The difference of the yaw angles, wrapped to -pi..pi, because yaw jumps from pi to -pi
    # when the robot turns past 180 degrees
    turned = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    return forward, left, turned


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
        """Sends the velocity for the given simulation time and stops. Returns the change of the
        odometry and of the true pose, each as (forward, left, turned)."""
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
        odom, truth = moved(odom0, self.odom), moved(truth0, self.truth)
        for name, (forward, left, turned) in (('odometry', odom), ('true pose', truth)):
            rospy.loginfo('%s: forward %.3f m, left %.3f m, turned %.3f rad', name, forward, left, turned)
        return odom, truth

    # unittest runs the test methods in alphabetical order, here forward and then turn. Each one
    # checks only its own move, from wherever the robot is when it starts, so neither depends on
    # the other

    def test_forward(self):
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

    def test_turn(self):
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
