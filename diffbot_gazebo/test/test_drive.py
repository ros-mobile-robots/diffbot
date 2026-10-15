#!/usr/bin/env python3
"""Drives the simulated robot with velocity commands and checks where it went.

The rostest files next to this script start the robot in Gazebo without a GUI, with the diff
drive controller from diffbot_control. Each motion is checked twice: with the controller's
odometry, which it computes from the wheel joints and its configuration, and with the robot's
true pose in Gazebo (/gazebo/model_states). Only the true pose shows mistakes that the odometry
repeats, for example swapped wheel joints, which make the robot turn the wrong way while the
odometry still reports the commanded turn.

Durations are simulation time, so the expected distances don't depend on how fast the computer
runs the simulation.
"""
import math
from typing import NamedTuple
from typing import Tuple
import unittest

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest
from tf.transformations import euler_from_quaternion

CMD_VEL_TOPIC = '/diffbot/mobile_base_controller/cmd_vel'
ODOM_TOPIC = '/diffbot/mobile_base_controller/odom'
MODEL_STATES_TOPIC = '/gazebo/model_states'
# Name of the spawned model in Gazebo, the same for DiffBot and Remo (diffbot_view.launch)
MODEL_NAME = 'diffbot'


class Pose2D(NamedTuple):
    """A pose on the ground: position x and y (m), heading theta (rad, 0 along the x axis)."""

    x: float
    y: float
    theta: float


class Motion(NamedTuple):
    """A relative motion: rotate by rot1 (rad), drive straight for trans (m), rotate by rot2."""

    rot1: float
    trans: float
    rot2: float


def planar(pose: Pose) -> Pose2D:
    """Return the 2D pose of a geometry_msgs/Pose.

    ROS messages give the orientation as a quaternion. euler_from_quaternion converts it to roll,
    pitch and yaw, the rotations about the x, y and z axes. The robot drives on flat ground, so
    roll and pitch stay about 0, and the yaw is its heading theta.
    """
    q = pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return Pose2D(pose.position.x, pose.position.y, yaw)


def wrap(angle: float) -> float:
    """Return the same angle in -pi..pi.

    theta jumps from pi to -pi when the robot turns past 180 degrees, so a difference of two
    headings can be off by 2 pi. sin and cos don't change by 2 pi, and atan2 returns the angle
    of the point (cos, sin) in -pi..pi. ROS amcl normalizes its angles the same way.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(start_pose: Pose2D, end_pose: Pose2D) -> Motion:
    """Return the motion between two poses, as in the odometry motion model.

    The model (Thrun, Burgard, Fox, section 5.4) describes the motion as a rotation rot1
    towards the end position, a straight translation trans to it, and a rotation rot2 to the
    end heading.

    The odometry and Gazebo give poses in different frames: the odometry starts at 0, 0, 0 where
    the controller started, while Gazebo's world frame has the robot spawned at x = -2, y = -0.5.
    The relative motion is the same in both frames, so the two can be compared.
    """
    x0, y0, theta0 = start_pose
    x1, y1, theta1 = end_pose
    # The straight-line distance, sqrt((x1 - x0)**2 + (y1 - y0)**2): hypot is the length of the
    # hypotenuse of a right triangle with these two sides
    trans = math.hypot(x1 - x0, y1 - y0)
    # When the robot turns on the spot, its position hardly changes, and the direction of such a
    # short translation is noise: then rot1 is 0 and rot2 is the whole turn, as in ROS amcl's
    # odometry model (amcl_odom.cpp)
    rot1 = wrap(math.atan2(y1 - y0, x1 - x0) - theta0) if trans >= 0.01 else 0.0
    rot2 = wrap(theta1 - theta0 - rot1)
    return Motion(rot1, trans, rot2)


def turned(motion: Motion) -> float:
    """Return the angle the robot turned in total (rad, positive to the left)."""
    return wrap(motion.rot1 + motion.rot2)


class TestDrive(unittest.TestCase):
    """Drive tests for the robot that the rostest file starts in Gazebo."""

    # Set in setUpClass() and in the subscriber callbacks. They belong to the class, not to a
    # TestDrive object: unittest creates a new object for each test method, while setUpClass()
    # starts the node, its publisher and its subscribers only once
    cmd_vel: rospy.Publisher
    # The latest pose from the odometry and from Gazebo, kept up to date by on_odom() and
    # on_model_states()
    odom_pose: Pose2D
    true_pose: Pose2D

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_drive')
        cls.cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        rospy.Subscriber(ODOM_TOPIC, Odometry, cls.on_odom)
        rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, cls.on_model_states, queue_size=1)
        # Gazebo and the controllers take a while to start, especially in CI
        rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=120)
        rospy.wait_for_message(MODEL_STATES_TOPIC, ModelStates, timeout=30)

    @classmethod
    def on_odom(cls, msg: Odometry) -> None:
        cls.odom_pose = planar(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg: ModelStates) -> None:
        if MODEL_NAME in msg.name:
            cls.true_pose = planar(msg.pose[msg.name.index(MODEL_NAME)])

    def drive(self, linear_velocity: float, angular_velocity: float, duration: float) -> None:
        """Send the velocity command (m/s, rad/s) for duration seconds of simulation time."""
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        # Send the command repeatedly: the controller stops the robot when no command arrives for
        # 0.5 s (diff_drive_controller's default cmd_vel_timeout)
        rate = rospy.Rate(20)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_vel.publish(cmd)
            rate.sleep()

    def stop(self) -> None:
        """Send a zero velocity and wait a second for the robot to come to rest."""
        self.cmd_vel.publish(Twist())
        rospy.sleep(1.0)

    def measure_motion(self, linear_velocity: float, angular_velocity: float,
                       duration: float) -> Tuple[Motion, Motion]:
        """Measure the relative motion of a drive with the odometry and with the true pose."""
        odom_start, true_start = self.odom_pose, self.true_pose
        self.drive(linear_velocity, angular_velocity, duration)
        self.stop()
        odom = relative_motion(odom_start, self.odom_pose)
        truth = relative_motion(true_start, self.true_pose)
        for source, motion in (('odometry', odom), ('true pose', truth)):
            rospy.loginfo('%s: rot1 %.3f rad, trans %.3f m, rot2 %.3f rad', source, *motion)
        return odom, truth

    # unittest runs the test methods in alphabetical order, here forward and then turn. Each one
    # checks only its own motion, from wherever the robot is when it starts, so neither depends
    # on the other

    def test_forward(self) -> None:
        # 0.2 m/s for 3 s is 0.6 m, a bit less with the controller's acceleration limit
        odom, truth = self.measure_motion(linear_velocity=0.2, angular_velocity=0.0, duration=3.0)
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

    def test_turn(self) -> None:
        # 0.5 rad/s for 3 s is 1.5 rad, turning on the spot. In Gazebo the robots turn less
        # (DiffBot about 1.3 rad, Remo 1.0 to 1.2 rad): Gazebo's wheel velocity controller
        # (diffbot_control/config/pid.yaml) has only a P gain, which leaves Remo's wheels at about
        # 80 % of the commanded speed, and the wheels slip. Until the gains are tuned (#116),
        # this only checks that the robot turns left, by a clear amount
        odom, truth = self.measure_motion(linear_velocity=0.0, angular_velocity=0.5, duration=3.0)
        for motion in (odom, truth):
            self.assertGreater(turned(motion), 0.5)
            self.assertLess(turned(motion), 2.0)
            self.assertLess(motion.trans, 0.15)
        # The wheels slip a little when turning on the spot (0.02 to 0.11 rad apart); a wheel
        # radius 20 % too big gave up to 0.31 rad
        self.assertAlmostEqual(turned(odom), turned(truth), delta=0.25)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)
