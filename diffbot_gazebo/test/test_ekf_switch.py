#!/usr/bin/env python3
"""Check the ekf switch in the launch files, both ways, without starting them.

roslaunch's loader reads each launch file with its includes, as roslaunch would, and the test
looks at the nodes, parameters and remappings it would start.

- On the robot (diffbot_bringup) and in the simulation (diffbot_gazebo), ekf:=true starts the IMU
  sample validator and the EKF in /diffbot, with ekf.yaml's parameters, and sets the drive
  controller's enable_odom_tf to false, so that only the EKF publishes odom → base_footprint.
  ekf:=false, the default, starts neither and leaves the controller's transform on.
- The launch files to start with ekf:=true also start robot_state_publisher. The EKF needs its
  transform imu_link → base_footprint and drops every IMU sample without it.
- The planners (diffbot_navigation's and diffbot_mbf's move_base) read the EKF's odometry with
  ekf:=true and the controller's without it.
catkin runs it with nose (catkin_add_nosetests in CMakeLists.txt).
"""

import os
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
import unittest

from roslaunch.config import load_config_default
from roslaunch.core import Node
import rospkg

EKF = '/diffbot/ekf_localization'
VALIDATOR = '/diffbot/imu_validator'
STATE_PUBLISHER = '/diffbot/robot_state_publisher'
ODOM_TF = '/diffbot/mobile_base_controller/enable_odom_tf'
CONTROLLER_ODOMETRY = '/diffbot/mobile_base_controller/odom'
FILTERED = '/diffbot/odometry/filtered'
MOVE_BASE = '/move_base'
MOVE_BASE_FLEX = '/move_base_flex'
# The two ways to run without the EKF: ekf:=false, and no ekf argument, the default
SWITCHED_OFF: List[Dict[str, str]] = [{'ekf': 'false'}, {}]

PACKAGES = rospkg.RosPack()


class LaunchConfiguration(NamedTuple):
    """What roslaunch would start: the nodes by their full name, and the parameters."""

    nodes: Dict[str, Node]
    parameters: Dict[str, Any]


def load(package: str, launch_file: str, **args: str) -> LaunchConfiguration:
    """Return what roslaunch would start for the package's launch file with the arguments."""
    path = os.path.join(PACKAGES.get_path(package), 'launch', launch_file)
    argv = ['{}:={}'.format(name, value) for name, value in args.items()]
    config = load_config_default([(path, argv)], None, verbose=False)
    nodes = {}
    for node in config.nodes:
        name = '/' + '/'.join(part for part in (node.namespace.strip('/'), node.name) if part)
        nodes[name] = node
    parameters = {name: parameter.value for name, parameter in config.params.items()}
    return LaunchConfiguration(nodes, parameters)


def remapping(node: Node, name: str) -> Optional[str]:
    """Return what the node's topic name is remapped to, or None."""
    return dict(node.remap_args).get(name)


class TestEkfSwitch(unittest.TestCase):

    def check_switch_off(self, package: str, launch_file: str, **args: str) -> None:
        """Check that neither node starts and the controller's transform stays on without the EKF.

        Without the EKF means ekf:=false, and no ekf argument at all, the default.
        """
        for switch in SWITCHED_OFF:
            nodes, parameters = load(package, launch_file, **switch, **args)
            for name in (EKF, VALIDATOR):
                self.assertNotIn(name, nodes, '{} starts {} with the arguments {}'.format(
                    launch_file, name, switch))
            # Set, not left out: roslaunch writes each parameter at each start, so a false value
            # from an earlier run with the EKF on the same ROS master doesn't remain
            self.assertIs(True, parameters.get(ODOM_TF),
                          'the controller should publish its odom transform with the arguments '
                          '{}'.format(switch))

    def check_switch_on(self, package: str, launch_file: str, **args: str) -> None:
        """Check that ekf:=true configures both nodes and disables the controller's odom TF."""
        nodes, parameters = load(package, launch_file, ekf='true', **args)
        self.assertEqual(('robot_localization', 'ekf_localization_node'),
                         (nodes[EKF].package, nodes[EKF].type))
        self.assertEqual(('diffbot_control', 'imu_validator.py'),
                         (nodes[VALIDATOR].package, nodes[VALIDATOR].type))
        self.assertIs(False, parameters[ODOM_TF])
        # ekf.yaml in the EKF's private namespace: the filter works in odom, for base_footprint,
        # on the controller's odometry and the validator's output
        self.assertEqual('base_footprint', parameters[EKF + '/base_link_frame'])
        self.assertEqual('odom', parameters[EKF + '/world_frame'])
        self.assertEqual('mobile_base_controller/odom', parameters[EKF + '/odom0'])
        self.assertEqual('imu/data_validated', parameters[EKF + '/imu0'])
        self.assertEqual('imu_link', parameters[VALIDATOR + '/frame_id'])
        # The EKF looks up imu_link → base_footprint, which robot_state_publisher publishes
        self.assertIn(STATE_PUBLISHER, sorted(nodes),
                      '{} starts the EKF without robot_state_publisher'.format(launch_file))

    def check_estimator(self, package: str, launch_file: str, **args: str) -> None:
        """Check both positions of the switch in the package's launch file."""
        self.check_switch_off(package, launch_file, **args)
        self.check_switch_on(package, launch_file, **args)

    def check_planner(self, package: str, launch_file: str, node_name: str, **args: str) -> None:
        """Check that the planner reads the EKF's odometry with ekf:=true only."""
        nodes, _ = load(package, launch_file, **args)
        self.assertEqual(CONTROLLER_ODOMETRY, remapping(nodes[node_name], 'odom'))
        nodes, _ = load(package, launch_file, ekf='true', **args)
        self.assertEqual(FILTERED, remapping(nodes[node_name], 'odom'))

    def test_bringup_starts_the_ekf_and_the_validator_only_with_ekf_true(self) -> None:
        self.check_estimator('diffbot_bringup', 'bringup.launch')
        self.check_estimator('diffbot_bringup', 'bringup_with_laser.launch')

    def test_simulation_starts_the_ekf_and_the_validator_only_with_ekf_true(self) -> None:
        self.check_estimator('diffbot_gazebo', 'diffbot.launch')

    def test_planners_read_the_filtered_odometry_only_with_ekf_true(self) -> None:
        self.check_planner('diffbot_navigation', 'move_base.launch', MOVE_BASE)
        self.check_planner('diffbot_navigation', 'diffbot_navigation.launch', MOVE_BASE)
        self.check_planner('diffbot_navigation', 'diffbot_hw.launch', MOVE_BASE)
        self.check_planner('diffbot_mbf', 'amcl_demo.launch', MOVE_BASE_FLEX)
        # An explicit odom_topic still wins
        nodes, _ = load('diffbot_navigation', 'move_base.launch', ekf='true',
                        odom_topic='/other/odom')
        self.assertEqual('/other/odom', remapping(nodes[MOVE_BASE], 'odom'))

    def test_navigation_simulation_passes_ekf_true_to_the_ekf_and_the_planner(self) -> None:
        nodes, parameters = load('diffbot_navigation', 'diffbot.launch', ekf='true',
                                 open_rviz='false')
        self.assertIn(EKF, nodes)
        self.assertIs(False, parameters[ODOM_TF])
        self.assertEqual(FILTERED, remapping(nodes[MOVE_BASE], 'odom'))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('diffbot_gazebo', 'test_ekf_switch', TestEkfSwitch)
