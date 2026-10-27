#!/usr/bin/env python3
"""Checks that each robot's URDF and its controller configuration agree on the wheel radius and
the wheel separation.

Both are written down twice: in the URDF, as the drive wheels' collision and joints, and in
diffbot_control/config/<model>/geometry.yaml, which diff_drive_controller uses for the odometry and
the real robot's hardware interface for the wheel speeds. If one changes without the other, the
odometry and the simulated robot disagree, and the simulation test only notices large differences.

The test asks roslaunch which parameters the launch files really set (--dump-params, without
starting anything), so it also notices a launch file that loads the wrong file:
- the simulation: diffbot_gazebo/launch/diffbot_view.launch, which loads the URDF, and
  diffbot_control/launch/diffbot_control.launch, which loads geometry.yaml;
- the real robot: diffbot_base/launch/diffbot.launch, which loads both.

It compares the nominal values only. A calibration of the real robot belongs in
diff_drive_controller's multipliers (wheel_separation_multiplier and the wheel radius
multipliers), which this test leaves alone. It can't show that the URDF matches the real wheels.
"""

import math
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml

MODELS = ("diffbot", "remo")
TOLERANCE = 1e-4  # m
NAMESPACE = "/diffbot"
LEFT, RIGHT = "front_left_wheel", "front_right_wheel"


def dump_params(package, launch_file, **args):
    """The parameters that a launch file sets, as a dict from parameter name to value."""
    command = ["roslaunch", "--dump-params", package, launch_file]
    command += ["{}:={}".format(name, value) for name, value in args.items()]
    return yaml.safe_load(subprocess.check_output(command, universal_newlines=True))


def _matrix(rpy):
    """Rotation matrix of URDF roll, pitch and yaw (fixed axes x, y, z)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _origin(element):
    origin = element.find("origin")
    xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
    return xyz, _matrix(rpy)


def joint_position(robot, joint_name):
    """Position of a joint's origin in the URDF's root link, following the chain of parent joints."""
    joints = {j.get("name"): j for j in robot.findall("joint")}
    parent_joint = {j.find("child").get("link"): j for j in robot.findall("joint")}
    joint = joints[joint_name]
    position, _ = _origin(joint)
    link = joint.find("parent").get("link")
    while link in parent_joint:
        xyz, rotation = _origin(parent_joint[link])
        position = [xyz[i] + sum(rotation[i][k] * position[k] for k in range(3)) for i in range(3)]
        link = parent_joint[link].find("parent").get("link")
    return position


def collision_radius(robot, link_name):
    """Radius of a wheel's collision, a sphere or a cylinder."""
    link = next(l for l in robot.findall("link") if l.get("name") == link_name)
    geometry = link.find("collision/geometry")
    shape = geometry.find("sphere")
    if shape is None:
        shape = geometry.find("cylinder")
    return float(shape.get("radius"))


def mismatches(urdf, params):
    """Where the URDF and the controller parameters disagree, as messages; empty if they agree."""
    robot = ET.fromstring(urdf)
    controller = NAMESPACE + "/mobile_base_controller/"
    radius = params[controller + "wheel_radius"]
    separation = params[controller + "wheel_separation"]
    found = []
    for wheel in (LEFT, RIGHT):
        urdf_radius = collision_radius(robot, wheel)
        if abs(urdf_radius - radius) > TOLERANCE:
            found.append("{} collision radius {:.4f} m, but wheel_radius is {:.4f} m".format(
                wheel, urdf_radius, radius))
    left = joint_position(robot, LEFT + "_joint")
    right = joint_position(robot, RIGHT + "_joint")
    urdf_separation = math.dist(left, right)
    if abs(urdf_separation - separation) > TOLERANCE:
        found.append("the wheel joints are {:.4f} m apart, but wheel_separation is {:.4f} m".format(
            urdf_separation, separation))
    return found


class TestComparison(unittest.TestCase):
    """The comparison itself notices a wrong radius and a wrong separation."""

    URDF = """<robot name="test">
      <link name="base_footprint"/><link name="chassis_link"/>
      <link name="front_left_wheel"><collision><geometry><sphere radius="0.033"/></geometry></collision></link>
      <link name="front_right_wheel"><collision><geometry><sphere radius="0.033"/></geometry></collision></link>
      <joint name="chassis_joint" type="fixed"><parent link="base_footprint"/><child link="chassis_link"/>
        <origin xyz="-0.036 0 0.0666" rpy="0 0 0"/></joint>
      <joint name="front_left_wheel_joint" type="continuous"><parent link="chassis_link"/><child link="front_left_wheel"/>
        <origin xyz="0.036 0.072 -0.0336" rpy="0 0 0"/></joint>
      <joint name="front_right_wheel_joint" type="continuous"><parent link="chassis_link"/><child link="front_right_wheel"/>
        <origin xyz="0.036 -0.072 -0.0336" rpy="0 0 0"/></joint>
    </robot>"""

    def params(self, radius, separation):
        return {NAMESPACE + "/mobile_base_controller/wheel_radius": radius,
                NAMESPACE + "/mobile_base_controller/wheel_separation": separation}

    def test_agreement(self):
        self.assertEqual(mismatches(self.URDF, self.params(0.033, 0.144)), [])

    def test_wrong_radius(self):
        self.assertEqual(len(mismatches(self.URDF, self.params(0.0325, 0.144))), 2)

    def test_wrong_separation(self):
        found = mismatches(self.URDF, self.params(0.033, 0.145))
        self.assertEqual(len(found), 1)
        self.assertIn("wheel_separation", found[0])


class TestModels(unittest.TestCase):
    """Each robot's URDF agrees with its geometry.yaml, as the launch files load them."""

    def check(self, model, params):
        urdf = params[NAMESPACE + "/robot_description"]
        self.assertEqual(mismatches(urdf, params), [], "{}: URDF and geometry.yaml disagree".format(model))

    def test_simulation(self):
        for model in MODELS:
            with self.subTest(model=model):
                params = dump_params("diffbot_gazebo", "diffbot_view.launch", model=model)
                params.update(dump_params("diffbot_control", "diffbot_control.launch", model=model,
                                          rqt_robot_steering="false"))
                self.check(model, params)

    def test_robot(self):
        for model in MODELS:
            with self.subTest(model=model):
                self.check(model, dump_params("diffbot_base", "diffbot.launch", model=model))


if __name__ == "__main__":
    unittest.main()
