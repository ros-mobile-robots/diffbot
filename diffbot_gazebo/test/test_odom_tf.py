#!/usr/bin/env python3
"""Checks that exactly one node publishes the transform odom → base_footprint: the node that the
rostest file names in the private parameter ~publisher. Without the EKF that is the drive
controller, which runs inside Gazebo (/gazebo); with the EKF (ekf:=true), the EKF
(/diffbot/ekf_localization), and the controller doesn't publish it. Two publishers of the same
transform would make the robot's pose jump between their two estimates.

It listens from its start, before the controllers run, so the start-up counts too, and then for
OBSERVE seconds of simulation time after the first such transform. test_ekf.py uses the same
check in its simulation.
"""

import collections
import unittest

import rospy
import rostest
from tf2_msgs.msg import TFMessage

OBSERVE = 3.0  # s
STARTUP_TIMEOUT = 120.0  # s of wall time, for Gazebo and the controller spawner


class OdomTfPublishers:
    """Counts, per publishing node, the transforms odom → base_footprint on /tf."""

    def __init__(self):
        self.counts = collections.Counter()
        rospy.Subscriber("/tf", TFMessage, self.callback, queue_size=100)

    def callback(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint":
                self.counts[message._connection_header["callerid"]] += 1

    def observe(self, test):
        """Waits for the first transform, then watches for OBSERVE seconds; returns the counts."""
        rospy.wait_for_message("/tf", TFMessage, timeout=STARTUP_TIMEOUT)
        deadline = rospy.Time.now() + rospy.Duration(STARTUP_TIMEOUT)
        while not self.counts and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
        test.assertTrue(self.counts, "nobody published odom → base_footprint")
        rospy.sleep(OBSERVE)
        return dict(self.counts)


class TestOdomTf(unittest.TestCase):

    def test_one_publisher(self):
        expected = rospy.get_param("~publisher")
        counts = publishers.observe(self)
        print("odom → base_footprint published by: {}".format(counts))
        self.assertEqual([expected], list(counts), "publishers of odom → base_footprint: {}".format(counts))


if __name__ == "__main__":
    rospy.init_node("test_odom_tf")
    publishers = OdomTfPublishers()
    rostest.rosrun("diffbot_gazebo", "test_odom_tf", TestOdomTf)
