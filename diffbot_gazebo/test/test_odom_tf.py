#!/usr/bin/env python3
"""Check that exactly one node publishes the transform odom → base_footprint.

The expected node is the one that the rostest file names in the private parameter ~publisher.
Without the EKF that is the drive controller, which runs inside Gazebo (/gazebo); with the EKF
(ekf:=true), the EKF (/diffbot/ekf_localization), and the controller doesn't publish it. Two
publishers of the same transform would make the robot's pose jump between their two estimates.

The test counts from its own start until OBSERVE seconds of simulation time after the first such
transform. rostest starts it after the simulation's nodes, and /tf doesn't repeat old messages: a
transform published before it subscribed isn't counted. The controller and the EKF usually start
seconds later, after Gazebo has spawned the robot, but that isn't guaranteed. test_ekf.py uses
the same check in its simulation.
"""

import collections
from typing import Counter
from typing import Dict
import unittest

import rospy
import rostest
from tf2_msgs.msg import TFMessage

TF_TOPIC = '/tf'
TF_QUEUE_SIZE = 100  # messages
PARENT_FRAME = 'odom'
CHILD_FRAME = 'base_footprint'
OBSERVE = 3.0  # s
STARTUP_TIMEOUT = 120.0  # s of wall time, for Gazebo and the controller spawner
POLL_INTERVAL = 0.1  # s of simulation time between checks for the first transform


class OdomTfPublishers:
    """Count, per publishing node, the transforms odom → base_footprint on /tf."""

    def __init__(self) -> None:
        self.counts: Counter[str] = collections.Counter()
        rospy.Subscriber(TF_TOPIC, TFMessage, self.on_tf, queue_size=TF_QUEUE_SIZE)

    def on_tf(self, message: TFMessage) -> None:
        """Count the message's transforms odom → base_footprint for the node that sent it."""
        for transform in message.transforms:
            if (transform.header.frame_id == PARENT_FRAME
                    and transform.child_frame_id == CHILD_FRAME):
                self.counts[message._connection_header['callerid']] += 1

    def observe(self, test: unittest.TestCase) -> Dict[str, int]:
        """Wait for the first transform, watch for OBSERVE seconds, then return the counts."""
        rospy.wait_for_message(TF_TOPIC, TFMessage, timeout=STARTUP_TIMEOUT)
        deadline = rospy.Time.now() + rospy.Duration(STARTUP_TIMEOUT)
        while not self.counts and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(POLL_INTERVAL)
        test.assertTrue(self.counts, 'nobody published odom → base_footprint')
        rospy.sleep(OBSERVE)
        return dict(self.counts)


class TestOdomTf(unittest.TestCase):

    def test_only_the_expected_node_publishes_odom_tf(self) -> None:
        expected = rospy.get_param('~publisher')
        counts = publishers.observe(self)
        print('odom → base_footprint published by: {}'.format(counts))
        self.assertEqual([expected], list(counts),
                         'publishers of odom → base_footprint: {}'.format(counts))


if __name__ == '__main__':
    rospy.init_node('test_odom_tf')
    publishers = OdomTfPublishers()
    rostest.rosrun('diffbot_gazebo', 'test_odom_tf', TestOdomTf)
