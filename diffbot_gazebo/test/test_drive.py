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

# The two tests' commands, each sent for DURATION seconds of simulation time
DURATION = 3.0  # s
FORWARD_VELOCITY = 0.2  # m/s
TURN_VELOCITY = 0.5  # rad/s
# How often drive() sends the command. The controller stops the robot when no command arrives
# for 0.5 s (diff_drive_controller's default cmd_vel_timeout), and drive() checks the end time
# once per cycle, so the robot drives nominally up to one cycle too long, here 1 cm or 0.025 rad.
# A late wake-up or message can add more; the tolerances leave room for it
COMMAND_RATE = 20  # Hz


class Pose2D(NamedTuple):
    """A pose on the ground: position x and y (m), heading theta (rad, 0 along the x axis)."""

    x: float
    y: float
    theta: float


class RelativePose(NamedTuple):
    """A pose relative to an earlier one, in the robot's frame at the earlier pose.

    forward and left (m) are along the robot's x and y axes at the earlier pose, and turned (rad)
    is the change of its heading, positive to the left.
    """

    forward: float
    left: float
    turned: float


def to_pose2d(pose: Pose) -> Pose2D:
    """Convert a geometry_msgs/Pose to a Pose2D.

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
    of the point (cos, sin) in -pi..pi.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(start_pose: Pose2D, end_pose: Pose2D) -> RelativePose:
    """Return the end pose relative to the start pose, in the robot's frame at the start pose.

    The odometry and Gazebo give poses in different frames: the odometry starts at 0, 0, 0 where
    the controller started, while Gazebo's world frame has the robot spawned at x = -2, y = -0.5.
    The relative pose is the same in both frames, so the two can be compared. With homogeneous
    transforms, it is inv(T_start) @ T_end (Corke, Robotics, Vision and Control, chapter 2).
    """
    # The displacement in the frame of the poses (odometry or Gazebo world)
    dx = end_pose.x - start_pose.x
    dy = end_pose.y - start_pose.y
    # The same displacement in the robot's frame at the start, rotated by -theta: forward is its
    # part along the robot's heading (cos theta, sin theta), left its part along the robot's left
    # side (-sin theta, cos theta)
    cos_theta = math.cos(start_pose.theta)
    sin_theta = math.sin(start_pose.theta)
    forward = dx * cos_theta + dy * sin_theta
    left = -dx * sin_theta + dy * cos_theta
    turned = wrap(end_pose.theta - start_pose.theta)
    return RelativePose(forward, left, turned)


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
        cls.odom_pose = to_pose2d(msg.pose.pose)

    @classmethod
    def on_model_states(cls, msg: ModelStates) -> None:
        # The message lists every model in the world, here the ground plane and the robot, with
        # the poses in the same order as the names. Gazebo publishes it before the robot is
        # spawned, so the first messages don't contain the robot
        if MODEL_NAME in msg.name:
            cls.true_pose = to_pose2d(msg.pose[msg.name.index(MODEL_NAME)])

    def drive(self, linear_velocity: float, angular_velocity: float, duration: float) -> None:
        """Send the velocity command (m/s, rad/s) for duration seconds of simulation time."""
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        # Send the command repeatedly, so the controller doesn't stop the robot (see COMMAND_RATE)
        rate = rospy.Rate(COMMAND_RATE)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_vel.publish(cmd)
            rate.sleep()

    def stop(self) -> None:
        """Send a zero velocity and wait a second for the robot to come to rest."""
        self.cmd_vel.publish(Twist())
        rospy.sleep(1.0)

    def measure_motion(self, linear_velocity: float, angular_velocity: float,
                       duration: float) -> Tuple[RelativePose, RelativePose]:
        """Drive, and return the motion measured with the odometry and with the true pose.

        Each motion is the end pose relative to the start pose.
        """
        odom_start, true_start = self.odom_pose, self.true_pose
        self.drive(linear_velocity, angular_velocity, duration)
        self.stop()
        odom_motion = relative_pose(odom_start, self.odom_pose)
        true_motion = relative_pose(true_start, self.true_pose)
        for source, motion in (('odometry', odom_motion), ('true pose', true_motion)):
            rospy.loginfo('%s: forward %.3f m, left %.3f m, turned %.3f rad', source, *motion)
        return odom_motion, true_motion

    # unittest runs the test methods in alphabetical order, here forward and then turn. Each one
    # checks only its own motion, from wherever the robot is when it starts, so neither depends
    # on the other. Each check compares a measured value with the expected one, within a
    # tolerance that lies between what the robot measures in Gazebo and what a mistake would give

    def test_forward(self) -> None:
        odom_motion, true_motion = self.measure_motion(
            linear_velocity=FORWARD_VELOCITY, angular_velocity=0.0, duration=DURATION)
        # v * t = 0.6 m straight ahead. The controller's acceleration limit costs 3 cm when the
        # robot starts and gives them back when it brakes; 0.57 to 0.62 m measured. A robot that
        # doesn't move, or drives much too slow or too fast, fails
        expected_forward = FORWARD_VELOCITY * DURATION
        for motion in (odom_motion, true_motion):
            self.assertAlmostEqual(motion.forward, expected_forward, delta=0.1)
            # Straight ahead (about 1 mm to the side measured), and without turning
            self.assertAlmostEqual(motion.left, 0.0, delta=0.1)
            self.assertAlmostEqual(motion.turned, 0.0, delta=0.2)
        # The odometry agrees with the true distance (about 0.01 m apart) unless the controller's
        # wheel radius doesn't match the model: 20 % too big gave 0.09 m
        self.assertAlmostEqual(odom_motion.forward, true_motion.forward, delta=0.05)

    def test_turn(self) -> None:
        odom_motion, true_motion = self.measure_motion(
            linear_velocity=0.0, angular_velocity=TURN_VELOCITY, duration=DURATION)
        # w * t = 1.5 rad to the left, on the spot. In Gazebo the robots turn less (DiffBot about
        # 1.3 rad, Remo 0.92 to 1.34 rad): Gazebo's wheel velocity controller
        # (diffbot_control/config/pid.yaml) has only a P gain, which leaves Remo's wheels at about
        # 80 % of the commanded speed, and the wheels slip. Until the gains are tuned (#116), the
        # tolerance is wide; a robot that turns right or not at all still fails
        expected_turn = TURN_VELOCITY * DURATION
        for motion in (odom_motion, true_motion):
            self.assertAlmostEqual(motion.turned, expected_turn, delta=1.0)
            # On the spot: its center drifts 2 to 4 cm in Gazebo
            self.assertAlmostEqual(math.hypot(motion.forward, motion.left), 0.0, delta=0.15)
        # The wheels slip a little when turning on the spot (0.02 to 0.11 rad apart); a wheel
        # radius 20 % too big gave up to 0.31 rad
        self.assertAlmostEqual(odom_motion.turned, true_motion.turned, delta=0.25)


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_drive', TestDrive)
