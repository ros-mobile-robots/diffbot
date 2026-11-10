#!/usr/bin/env python3
"""Check that the simulated IMU publishes again right after a reset of the simulation.

/gazebo/reset_simulation sets Gazebo's clock back to zero. The IMU's Gazebo plugin limits its
rate by comparing the sensor's time with the time of its last update, so it has to notice that
the clock went back. A plugin that keeps the time from before the reset publishes nothing until
the clock has passed that time, and the EKF gets no yaw rate for as long as the simulation ran
before the reset.

The test lets the IMU publish for RUN_BEFORE_RESET of simulation time, resets the simulation and
waits for samples whose stamps lie in the first RECOVERY of the new simulation time. RECOVERY is
shorter than RUN_BEFORE_RESET, so such a sample can only come from after the reset, from before
the clock is back at its old value. The test reads /diffbot/imu/data_raw, so the samples pass
through the plugin and the adapter (imu_raw_adapter.py).

RUN_BEFORE_RESET and RECOVERY are simulation time, read from the samples' stamps. Every wait is
wall time: rospy.sleep and rospy.Rate raise an exception when the clock they wait on goes back.
The timeouts only limit how long a failing test takes. rostest starts the simulation of
sim_drive_<model>.test again for each of its tests, so the reset doesn't reach the other tests.
"""

import time
from typing import Callable
from typing import List
import unittest

import rospy
import rostest
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty

TOPIC = '/diffbot/imu/data_raw'
RESET_SERVICE = '/gazebo/reset_simulation'
QUEUE_SIZE = 100  # messages, 2 s at the IMU's 50 Hz: the test notes them all

RUN_BEFORE_RESET = 3.0  # s of simulation time that the IMU publishes for before the reset
RECOVERY = 1.0  # s of simulation time after the reset in which the IMU has to publish again
# Samples from the first RECOVERY after the reset that the test waits for: 0.2 s at the IMU's
# 50 Hz, which shows that the IMU keeps publishing. The IMU's rate is test_imu.py's check
FRESH_SAMPLE_COUNT = 10
STARTUP_TIMEOUT = 60.0  # s of wall time for Gazebo to start and offer its reset service
# s of wall time for the IMU's stamps to reach RUN_BEFORE_RESET. The world runs the simulation
# at real time at most (diffbot_empty.world), and slower on a busy computer
RUN_TIMEOUT = 60.0
RECOVERY_TIMEOUT = 30.0  # s of wall time for the fresh samples to arrive after the reset
POLL_INTERVAL = 0.02  # s of wall time between two looks at the received samples


class TestImuReset(unittest.TestCase):

    def setUp(self) -> None:
        self.stamps: List[float] = []  # s of simulation time, in the order the samples arrived
        self.sample_count_at_reset = 0
        rospy.Subscriber(TOPIC, Imu, self.note_stamp, queue_size=QUEUE_SIZE)
        rospy.wait_for_service(RESET_SERVICE, timeout=STARTUP_TIMEOUT)
        self.reset_simulation = rospy.ServiceProxy(RESET_SERVICE, Empty)

    def note_stamp(self, message: Imu) -> None:
        """Note the sample's stamp."""
        self.stamps.append(message.header.stamp.to_sec())

    def imu_ran_before_reset(self) -> bool:
        """Return whether the IMU's stamps have reached RUN_BEFORE_RESET."""
        return bool(self.stamps) and self.stamps[-1] >= RUN_BEFORE_RESET

    def fresh_stamps(self) -> List[float]:
        """Return the stamps from the first RECOVERY of simulation time after the reset.

        A sample that the IMU published before the reset can still arrive after it. Its stamp
        is RUN_BEFORE_RESET or later, so it doesn't count.
        """
        stamps_after_reset = self.stamps[self.sample_count_at_reset:]
        return [stamp for stamp in stamps_after_reset if stamp < RECOVERY]

    def imu_recovered(self) -> bool:
        """Return whether enough fresh samples have arrived."""
        return len(self.fresh_stamps()) >= FRESH_SAMPLE_COUNT

    @staticmethod
    def wait_for(condition: Callable[[], bool], timeout: float) -> bool:
        """Return whether the condition became true within timeout seconds of wall time."""
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                return False
            time.sleep(POLL_INTERVAL)
        return True

    def test_imu_publishes_again_right_after_a_simulation_reset(self) -> None:
        self.assertTrue(self.wait_for(self.imu_ran_before_reset, RUN_TIMEOUT),
                        "{}'s stamps didn't reach {} s within {} s of wall time, got {} "
                        'samples'.format(TOPIC, RUN_BEFORE_RESET, RUN_TIMEOUT, len(self.stamps)))
        self.sample_count_at_reset = len(self.stamps)
        stamp_before_reset = self.stamps[-1]
        self.reset_simulation()
        recovered = self.wait_for(self.imu_recovered, RECOVERY_TIMEOUT)
        fresh_stamps = self.fresh_stamps()
        stamps_after_reset = self.stamps[self.sample_count_at_reset:]
        print('last stamp before the reset {:.3f} s; after the reset {} samples, first stamps {}, '
              '{} of them earlier than {} s'.format(
                  stamp_before_reset, len(stamps_after_reset),
                  ['{:.3f}'.format(stamp) for stamp in stamps_after_reset[:3]],
                  len(fresh_stamps), RECOVERY))
        self.assertTrue(
            recovered,
            'after the reset at stamp {:.3f} s, {} published {} samples stamped earlier than '
            '{} s within {} s of wall time, expected {}. It published {} samples in all, the '
            'first stamped {}'.format(
                stamp_before_reset, TOPIC, len(fresh_stamps), RECOVERY, RECOVERY_TIMEOUT,
                FRESH_SAMPLE_COUNT, len(stamps_after_reset),
                ['{:.3f} s'.format(stamp) for stamp in stamps_after_reset[:3]]))


if __name__ == '__main__':
    rospy.init_node('test_imu_reset')
    rostest.rosrun('diffbot_gazebo', 'test_imu_reset', TestImuReset)
