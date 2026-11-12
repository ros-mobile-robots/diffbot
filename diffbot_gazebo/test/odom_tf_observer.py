"""Observe which nodes publish the transform odom → base_footprint on the topic /tf.

A helper for test_odom_tf.py and test_ekf.py, which check that exactly one node publishes this
transform. Two publishers of the same transform would make the robot's pose jump between their
two estimates.
"""

import collections
import threading
import time
from typing import Counter
from typing import Dict

import rospy
from tf2_msgs.msg import TFMessage

TF_TOPIC = '/tf'
TF_QUEUE_SIZE = 100  # messages
PARENT_FRAME = 'odom'
CHILD_FRAME = 'base_footprint'
# s of simulation time for which the tests watch /tf after the first such transform
OBSERVE = 3.0
POLL_INTERVAL = 0.1  # s of wall time between checks for the first transform


class OdomTfObserver:
    """Count, per publishing node, the transforms odom → base_footprint on /tf.

    The observer counts from its creation. /tf doesn't repeat old messages, so a transform
    published before that isn't counted.
    """

    def __init__(self) -> None:
        self.counts_by_publisher: Counter[str] = collections.Counter()
        # rospy calls on_tf() in one thread per publisher of /tf, and the test reads the counts
        # in another
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(TF_TOPIC, TFMessage, self.on_tf,
                                           queue_size=TF_QUEUE_SIZE)

    def on_tf(self, message: TFMessage) -> None:
        """Count the message's transforms odom → base_footprint for the node that sent it."""
        # rospy attaches the connection's header to each message that it receives. Its callerid
        # is the name of the node that published the message
        publisher = message._connection_header['callerid']
        for transform in message.transforms:
            if (transform.header.frame_id == PARENT_FRAME
                    and transform.child_frame_id == CHILD_FRAME):
                with self.lock:
                    self.counts_by_publisher[publisher] += 1

    def counts(self) -> Dict[str, int]:
        """Return the number of transforms odom → base_footprint so far, by publishing node."""
        with self.lock:
            return dict(self.counts_by_publisher)

    def wait_for_first_transform(self, timeout: float) -> bool:
        """Wait for a transform odom → base_footprint and return whether one came.

        The wait takes at most timeout seconds of wall time, so it ends even when the
        simulation's clock doesn't run.
        """
        deadline = time.monotonic() + timeout
        while not self.counts() and time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(POLL_INTERVAL)
        return bool(self.counts())
