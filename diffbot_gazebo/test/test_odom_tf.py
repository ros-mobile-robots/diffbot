#!/usr/bin/env python3
"""Check that exactly one node publishes the transform odom → base_footprint on the topic /tf.

The expected node is the one that the rostest file names in the private parameter ~publisher.
Without the EKF that is the drive controller, which runs inside Gazebo (/gazebo). With the EKF
(ekf:=true), the EKF (/diffbot/ekf_localization), and the controller doesn't publish it. Two
publishers of the same transform would make the robot's pose jump between their two estimates.

The test counts from its own start until OBSERVE seconds of simulation time after the first such
transform (odom_tf_observer.py). rostest starts it after the simulation's nodes, and /tf doesn't
repeat old messages: a transform published before it subscribed isn't counted. The controller and
the EKF usually start seconds later, after Gazebo has spawned the robot, but that isn't
guaranteed. test_ekf.py uses the same check in its simulation.
"""

import unittest

from odom_tf_observer import OBSERVE  # this folder's odom_tf_observer.py
from odom_tf_observer import OdomTfObserver
import rospy
import rostest

STARTUP_TIMEOUT = 120.0  # s of wall time, for Gazebo and the controller spawner


class TestOdomTf(unittest.TestCase):

    observer: OdomTfObserver

    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node('test_odom_tf')
        cls.observer = OdomTfObserver()

    def test_expected_node_is_the_only_odom_tf_publisher(self) -> None:
        expected = rospy.get_param('~publisher')
        self.assertTrue(self.observer.wait_for_first_transform(STARTUP_TIMEOUT),
                        'nobody published odom → base_footprint')
        rospy.sleep(OBSERVE)
        counts = self.observer.counts()
        print('odom → base_footprint published by: {}'.format(counts))
        self.assertEqual([expected], list(counts),
                         'publishers of odom → base_footprint: {}'.format(counts))


if __name__ == '__main__':
    rostest.rosrun('diffbot_gazebo', 'test_odom_tf', TestOdomTf)
