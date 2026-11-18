#!/usr/bin/env python3
"""Check the ekf switch in the launch files, both ways, without starting them.

roslaunch's loader reads each launch file with its includes, as roslaunch would, and the test
looks at the nodes, parameters and remappings it would start.

- On the robot (diffbot_bringup) and in the simulation (diffbot_gazebo, and diffbot_control's
  diffbot.launch with RViz), ekf:=true starts the IMU sample validator and the EKF in /diffbot,
  with ekf.yaml's parameters, and sets the drive controller's enable_odom_tf to false, so that
  only the EKF publishes the transform odom → base_footprint on the topic /tf.
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

EKF_NODE = '/diffbot/ekf_localization'
VALIDATOR_NODE = '/diffbot/imu_validator'
STATE_PUBLISHER_NODE = '/diffbot/robot_state_publisher'
ODOM_TF_PARAMETER = '/diffbot/mobile_base_controller/enable_odom_tf'
WHEEL_ODOMETRY_TOPIC = '/diffbot/mobile_base_controller/odom'
FILTERED_TOPIC = '/diffbot/odometry/filtered'
MOVE_BASE_NODE = '/move_base'
MOVE_BASE_FLEX_NODE = '/move_base_flex'
# The two ways to run without the EKF: ekf:=false, and no ekf argument, the default
SWITCHED_OFF: List[Dict[str, str]] = [{'ekf': 'false'}, {}]
# ekf.yaml's frames and inputs in the EKF's private namespace, and the validator's frame: the
# filter works in odom, for base_footprint, on the controller's odometry and the validator's
# output
EKF_PARAMETERS = {
    EKF_NODE + '/base_link_frame': 'base_footprint',
    EKF_NODE + '/world_frame': 'odom',
    EKF_NODE + '/odom0': 'mobile_base_controller/odom',
    EKF_NODE + '/imu0': 'imu/data_validated',
    VALIDATOR_NODE + '/frame_id': 'imu_link',
}

PACKAGES = rospkg.RosPack()


class LaunchConfiguration(NamedTuple):
    """What roslaunch would start: the nodes by their full name, and the parameters."""

    nodes: Dict[str, Node]
    parameters: Dict[str, Any]
    description: str  # the launch file and its arguments, for failure messages


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
    description = ' '.join([package, launch_file] + argv)
    return LaunchConfiguration(nodes, parameters, description)


def remapping(node: Node, name: str) -> Optional[str]:
    """Return what the node's topic name is remapped to, or None."""
    return dict(node.remap_args).get(name)


class TestEkfSwitch(unittest.TestCase):
    """The assert_...() methods each check one thing that the ekf switch has to do."""

    def assert_node(self, configuration: LaunchConfiguration, name: str, package: str,
                    node_type: str) -> None:
        """Assert that the launch file starts the node name from package's node_type."""
        self.assertIn(name, sorted(configuration.nodes),
                      "{} doesn't start {}".format(configuration.description, name))
        node = configuration.nodes[name]
        self.assertEqual((package, node_type), (node.package, node.type),
                         '{}: the executable of {}'.format(configuration.description, name))

    def assert_ekf_nodes_start(self, configuration: LaunchConfiguration) -> None:
        """Assert that the launch file starts the EKF and the IMU sample validator."""
        self.assert_node(configuration, EKF_NODE, 'robot_localization', 'ekf_localization_node')
        self.assert_node(configuration, VALIDATOR_NODE, 'diffbot_control', 'imu_validator.py')

    def assert_no_ekf_nodes_start(self, configuration: LaunchConfiguration) -> None:
        """Assert that the launch file starts neither the EKF nor the IMU sample validator."""
        for name in (EKF_NODE, VALIDATOR_NODE):
            self.assertNotIn(name, sorted(configuration.nodes),
                             '{} starts {}'.format(configuration.description, name))

    def assert_controller_odom_tf(self, configuration: LaunchConfiguration,
                                  enabled: bool) -> None:
        """Assert that the launch file sets the controller's enable_odom_tf to enabled.

        Set, not left out: roslaunch writes each parameter at each start, so a false value from
        an earlier run with the EKF on the same ROS master doesn't remain.
        """
        self.assertIs(enabled, configuration.parameters.get(ODOM_TF_PARAMETER),
                      "{}: the controller's enable_odom_tf".format(configuration.description))

    def assert_ekf_reads_the_controller_and_the_validator(
            self, configuration: LaunchConfiguration) -> None:
        """Assert the EKF's frames and inputs and the validator's frame (EKF_PARAMETERS)."""
        for name, value in EKF_PARAMETERS.items():
            self.assertEqual(value, configuration.parameters.get(name),
                             '{}: {}'.format(configuration.description, name))

    def assert_state_publisher_starts(self, configuration: LaunchConfiguration) -> None:
        """Assert that the launch file starts robot_state_publisher.

        The EKF looks up imu_link → base_footprint, which TF has from robot_state_publisher.
        """
        self.assertIn(STATE_PUBLISHER_NODE, sorted(configuration.nodes),
                      '{} starts the EKF without robot_state_publisher'.format(
                          configuration.description))

    def assert_ekf_on(self, package: str, launch_file: str) -> None:
        """Assert everything that ekf:=true has to do in the package's launch file."""
        configuration = load(package, launch_file, ekf='true')
        self.assert_ekf_nodes_start(configuration)
        self.assert_controller_odom_tf(configuration, enabled=False)
        self.assert_ekf_reads_the_controller_and_the_validator(configuration)
        self.assert_state_publisher_starts(configuration)

    def assert_ekf_off(self, package: str, launch_file: str) -> None:
        """Assert that the launch file runs without the EKF, with ekf:=false and by default."""
        for switch in SWITCHED_OFF:
            configuration = load(package, launch_file, **switch)
            self.assert_no_ekf_nodes_start(configuration)
            self.assert_controller_odom_tf(configuration, enabled=True)

    def assert_ekf_switch(self, package: str, launch_file: str) -> None:
        """Assert both positions of the switch in the package's launch file."""
        self.assert_ekf_off(package, launch_file)
        self.assert_ekf_on(package, launch_file)

    def assert_planner_odometry(self, configuration: LaunchConfiguration, node_name: str,
                                topic: str) -> None:
        """Assert that the planner node_name reads its odometry from topic."""
        self.assertIn(node_name, sorted(configuration.nodes),
                      "{} doesn't start {}".format(configuration.description, node_name))
        self.assertEqual(topic, remapping(configuration.nodes[node_name], 'odom'),
                         "{}: {}'s odom".format(configuration.description, node_name))

    def assert_planner_follows_the_switch(self, package: str, launch_file: str,
                                          node_name: str) -> None:
        """Assert that the planner reads the EKF's odometry with ekf:=true only."""
        self.assert_planner_odometry(load(package, launch_file), node_name, WHEEL_ODOMETRY_TOPIC)
        self.assert_planner_odometry(load(package, launch_file, ekf='true'), node_name,
                                     FILTERED_TOPIC)

    def test_bringup_starts_the_ekf_and_the_validator_only_with_ekf_true(self) -> None:
        self.assert_ekf_switch('diffbot_bringup', 'bringup.launch')
        self.assert_ekf_switch('diffbot_bringup', 'bringup_with_laser.launch')

    def test_simulation_starts_the_ekf_and_the_validator_only_with_ekf_true(self) -> None:
        self.assert_ekf_switch('diffbot_gazebo', 'diffbot.launch')
        self.assert_ekf_switch('diffbot_control', 'diffbot.launch')

    def test_planners_read_the_filtered_odometry_only_with_ekf_true(self) -> None:
        self.assert_planner_follows_the_switch('diffbot_navigation', 'move_base.launch',
                                               MOVE_BASE_NODE)
        self.assert_planner_follows_the_switch('diffbot_navigation', 'diffbot_navigation.launch',
                                               MOVE_BASE_NODE)
        self.assert_planner_follows_the_switch('diffbot_navigation', 'diffbot_hw.launch',
                                               MOVE_BASE_NODE)
        self.assert_planner_follows_the_switch('diffbot_mbf', 'amcl_demo.launch',
                                               MOVE_BASE_FLEX_NODE)

    def test_planner_reads_an_explicit_odom_topic_also_with_ekf_true(self) -> None:
        configuration = load('diffbot_navigation', 'move_base.launch', ekf='true',
                             odom_topic='/other/odom')
        self.assert_planner_odometry(configuration, MOVE_BASE_NODE, '/other/odom')

    def test_navigation_simulation_starts_the_ekf_for_its_planner_with_ekf_true(self) -> None:
        configuration = load('diffbot_navigation', 'diffbot.launch', ekf='true',
                             open_rviz='false')
        self.assert_ekf_nodes_start(configuration)
        self.assert_controller_odom_tf(configuration, enabled=False)
        self.assert_planner_odometry(configuration, MOVE_BASE_NODE, FILTERED_TOPIC)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('diffbot_gazebo', 'test_ekf_switch', TestEkfSwitch)
