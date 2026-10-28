#!/usr/bin/env python3
"""Checks the ekf switch in the launch files, both ways, without starting them: roslaunch's loader
reads each launch file with its includes, as roslaunch would, and the test looks at the nodes,
parameters and remappings it would start.

- On the robot (diffbot_bringup) and in the simulation (diffbot_gazebo), ekf:=true starts the IMU
  gate and the EKF in /diffbot, with ekf.yaml's parameters, and sets the drive controller's
  enable_odom_tf to false, so that only the EKF publishes odom → base_footprint. ekf:=false, the
  default, starts neither and leaves the controller's transform on.
- The planners (diffbot_navigation's and diffbot_mbf's move_base) read the EKF's odometry with
  ekf:=true and the controller's without it.
catkin runs it with nose (catkin_add_nosetests in CMakeLists.txt).
"""

import os
import unittest

from roslaunch.config import load_config_default
import rospkg

EKF = "/diffbot/ekf_localization"
GATE = "/diffbot/imu_gate"
ODOM_TF = "/diffbot/mobile_base_controller/enable_odom_tf"
CONTROLLER_ODOMETRY = "/diffbot/mobile_base_controller/odom"
FILTERED = "/diffbot/odometry/filtered"

PACKAGES = rospkg.RosPack()


def load(package, launch_file, **args):
    """The configuration roslaunch would start: nodes by their full name, and parameters."""
    path = os.path.join(PACKAGES.get_path(package), "launch", launch_file)
    argv = ["{}:={}".format(name, value) for name, value in args.items()]
    config = load_config_default([(path, argv)], None, verbose=False)
    nodes = {}
    for node in config.nodes:
        name = "/" + "/".join(part for part in (node.namespace.strip("/"), node.name) if part)
        nodes[name] = node
    params = {name: param.value for name, param in config.params.items()}
    return nodes, params


def remapping(node, name):
    return dict(node.remap_args).get(name)


class TestEkfSwitch(unittest.TestCase):

    def check_estimator(self, package, launch_file, **args):
        nodes, params = load(package, launch_file, ekf="false", **args)
        for name in (EKF, GATE):
            self.assertNotIn(name, nodes, "{} starts {} without ekf:=true".format(launch_file, name))
        self.assertNotIn(ODOM_TF, params, "the controller should keep its default, its own odom transform")

        nodes, params = load(package, launch_file, ekf="true", **args)
        self.assertEqual(("robot_localization", "ekf_localization_node"), (nodes[EKF].package, nodes[EKF].type))
        self.assertEqual(("diffbot_control", "imu_gate.py"), (nodes[GATE].package, nodes[GATE].type))
        self.assertIs(False, params[ODOM_TF])
        # ekf.yaml in the EKF's private namespace: the filter works in odom, for base_footprint,
        # on the controller's odometry and the gate's output
        self.assertEqual("base_footprint", params[EKF + "/base_link_frame"])
        self.assertEqual("odom", params[EKF + "/world_frame"])
        self.assertEqual("mobile_base_controller/odom", params[EKF + "/odom0"])
        self.assertEqual("imu/data_validated", params[EKF + "/imu0"])
        self.assertEqual("imu_link", params[GATE + "/frame_id"])

    def test_robot(self):
        self.check_estimator("diffbot_bringup", "bringup.launch")
        self.check_estimator("diffbot_bringup", "bringup_with_laser.launch")

    def test_simulation(self):
        self.check_estimator("diffbot_gazebo", "diffbot.launch")

    def check_planner(self, package, launch_file, node_name, **args):
        nodes, _ = load(package, launch_file, **args)
        self.assertEqual(CONTROLLER_ODOMETRY, remapping(nodes[node_name], "odom"))
        nodes, _ = load(package, launch_file, ekf="true", **args)
        self.assertEqual(FILTERED, remapping(nodes[node_name], "odom"))

    def test_planners(self):
        self.check_planner("diffbot_navigation", "move_base.launch", "/move_base")
        self.check_planner("diffbot_navigation", "diffbot_navigation.launch", "/move_base")
        self.check_planner("diffbot_navigation", "diffbot_hw.launch", "/move_base")
        self.check_planner("diffbot_mbf", "amcl_demo.launch", "/move_base_flex")
        # An explicit odom_topic still wins
        nodes, _ = load("diffbot_navigation", "move_base.launch", ekf="true", odom_topic="/other/odom")
        self.assertEqual("/other/odom", remapping(nodes["/move_base"], "odom"))

    def test_navigation_simulation(self):
        # The simulation with navigation passes the switch to both the EKF and the planner
        nodes, params = load("diffbot_navigation", "diffbot.launch", ekf="true", open_rviz="false")
        self.assertIn(EKF, nodes)
        self.assertIs(False, params[ODOM_TF])
        self.assertEqual(FILTERED, remapping(nodes["/move_base"], "odom"))


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("diffbot_gazebo", "test_ekf_switch", TestEkfSwitch)
