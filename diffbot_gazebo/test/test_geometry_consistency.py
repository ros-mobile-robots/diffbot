#!/usr/bin/env python3
"""Check that each robot's URDF and its controller configuration agree on the drive wheels.

The wheel radius and the wheel separation are written down twice: in the URDF, as the drive
wheels' collision and joints, and in diffbot_control/config/<model>/geometry.yaml, which
diff_drive_controller uses for the odometry and the wheel commands. If one changes without the
other, the odometry and the simulated robot disagree, and the simulation test only notices large
differences.

The test also checks the drive wheels' joints, which are found by name. The controller's
left_wheel and right_wheel name continuous joints in the URDF. In the simulation, Gazebo gives the
controller a velocity interface for each of them from the first interface of its transmission. On
the real robot, diffbot_base's hardware interface registers the joints in
hardware_interface/joints, the left motor first, so that list must be the controller's wheels in
this order. The simulation loads the same list without using it, so there it's a check of the
shared configuration. Taking the wheel joints from the controller's parameters and checking their
interfaces follows K9's model contract test, tests/test_model_contract.py in
https://github.com/hopkira/k9_gazebo_pkg.

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
from typing import Any
from typing import Dict
from typing import List
from typing import Set
from typing import Tuple
import unittest
import xml.etree.ElementTree as ET

import yaml

MODELS = ('diffbot', 'remo')
TOLERANCE = 1e-4  # m
NAMESPACE = '/diffbot'
# The prefix of diff_drive_controller's parameters
CONTROLLER = NAMESPACE + '/mobile_base_controller/'
# The joints that diffbot_base's hardware interface drives, in the order of its motors
HARDWARE_JOINTS = NAMESPACE + '/hardware_interface/joints'
# diff_drive_controller commands the wheels' velocity. Gazebo's DefaultRobotHWSim takes the first
# interface of a joint's transmission and accepts these two spellings
VELOCITY_INTERFACES = ('hardware_interface/VelocityJointInterface', 'VelocityJointInterface')

# Parameters by name, as roslaunch --dump-params prints them
Params = Dict[str, Any]
Vector = Tuple[float, float, float]
Matrix = Tuple[Vector, Vector, Vector]


def dump_params(package: str, launch_file: str, **launch_args: str) -> Params:
    """Return the parameters that a launch file sets, by parameter name.

    roslaunch --dump-params reads the launch file and the files it includes, and prints the
    parameters as YAML, without starting a node.
    """
    command = ['roslaunch', '--dump-params', package, launch_file]
    command += ['{}:={}'.format(name, value) for name, value in launch_args.items()]
    launch = ' '.join(command[2:])
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, check=True)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(
            'roslaunch failed for {}: {}'.format(launch, error.stderr.strip())) from error
    params = yaml.safe_load(result.stdout)
    # roslaunch ends with 0 and prints nothing when it can't load a file, for example when a
    # package such as remo_description is missing
    if not isinstance(params, dict):
        raise RuntimeError(
            'roslaunch printed no parameters for {}: {}'.format(launch, result.stderr.strip()))
    return params


def required(element: ET.Element, path: str) -> ET.Element:
    """Return the subelement at path, which the URDF must have."""
    subelement = element.find(path)
    if subelement is None:
        raise ValueError('{} {} has no {}'.format(element.tag, element.get('name'), path))
    return subelement


def rotation_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """Return the rotation matrix of a URDF origin's roll, pitch and yaw.

    URDF rotates about the fixed axes x, y and z, in this order, so the matrix is
    Rz(yaw) Ry(pitch) Rx(roll).
    """
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (
        (cos_yaw * cos_pitch,
         cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
         cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll),
        (sin_yaw * cos_pitch,
         sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
         sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll),
        (-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll),
    )


def joint_origin(joint: ET.Element) -> Tuple[Vector, Matrix]:
    """Return a joint's position and rotation in its parent link's frame."""
    origin = joint.find('origin')
    xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
    rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
    x, y, z = (float(value) for value in xyz.split())
    roll, pitch, yaw = (float(value) for value in rpy.split())
    return (x, y, z), rotation_matrix(roll, pitch, yaw)


def transform(offset: Vector, rotation: Matrix, point: Vector) -> Vector:
    """Return a point in the parent frame: rotated into it, then moved by the offset."""
    x, y, z = (offset[row] + sum(rotation[row][column] * point[column] for column in range(3))
               for row in range(3))
    return (x, y, z)


def joint_position(robot: ET.Element, joint_name: str) -> Vector:
    """Return the position of a joint in the frame of the URDF's root link.

    The joint's origin is given in its parent link's frame. Each joint further up the tree moves
    it into the frame of that joint's parent link, up to the root link.
    """
    joints_by_name = {joint.get('name'): joint for joint in robot.findall('joint')}
    joints_by_child_link = {
        required(joint, 'child').get('link'): joint for joint in robot.findall('joint')}
    joint = joints_by_name[joint_name]
    position, _ = joint_origin(joint)
    link_name = required(joint, 'parent').get('link')
    while link_name in joints_by_child_link:
        parent_joint = joints_by_child_link[link_name]
        offset, rotation = joint_origin(parent_joint)
        position = transform(offset, rotation, position)
        link_name = required(parent_joint, 'parent').get('link')
    return position


def wheel_joint_names(params: Params, side: str) -> List[str]:
    """Return the controller's wheel joints on one side, 'left' or 'right'.

    diff_drive_controller takes a joint name or a list of joint names for each side.
    """
    names = params[CONTROLLER + side + '_wheel']
    return [names] if isinstance(names, str) else list(names)


def collision_radius(robot: ET.Element, link_name: str) -> float:
    """Return the radius of a wheel link's collision, a sphere or a cylinder."""
    link = next(link for link in robot.findall('link') if link.get('name') == link_name)
    geometry = required(link, 'collision/geometry')
    shape = geometry.find('sphere')
    if shape is None:
        shape = required(geometry, 'cylinder')
    return float(shape.attrib['radius'])


def transmission_interfaces(robot: ET.Element) -> Tuple[Dict[str, str], Set[str]]:
    """Return each joint's first transmission interface, and the joints in shared transmissions.

    Gazebo's DefaultRobotHWSim gives a joint the first interface of its transmission, and skips a
    transmission with more than one joint, so only single-joint transmissions give an interface.
    """
    interface_by_joint: Dict[str, str] = {}
    joints_in_shared_transmissions: Set[str] = set()
    for transmission in robot.findall('transmission'):
        transmission_joints = transmission.findall('joint')
        if len(transmission_joints) == 1:
            joint = transmission_joints[0]
            interface = joint.findtext('hardwareInterface', '').strip()
            interface_by_joint[joint.get('name', '')] = interface
        else:
            joints_in_shared_transmissions.update(
                joint.get('name', '') for joint in transmission_joints)
    return interface_by_joint, joints_in_shared_transmissions


def within_tolerance(urdf_value: float, controller_value: float) -> bool:
    """Return whether the two values differ by at most TOLERANCE.

    A NaN isn't within any tolerance: ordered comparisons with it are false, so a check that a
    difference is too large would let it pass.
    """
    return abs(urdf_value - controller_value) <= TOLERANCE


def mismatches(urdf: str, params: Params) -> List[str]:
    """Return where the URDF and the controller's parameters disagree, as messages.

    The list is empty when they agree.
    """
    robot = ET.fromstring(urdf)
    wheel_radius = params[CONTROLLER + 'wheel_radius']
    wheel_separation = params[CONTROLLER + 'wheel_separation']
    left_joint_names = wheel_joint_names(params, 'left')
    right_joint_names = wheel_joint_names(params, 'right')
    joints_by_name = {joint.get('name'): joint for joint in robot.findall('joint')}
    interface_by_joint, joints_in_shared_transmissions = transmission_interfaces(robot)
    problems = []
    for side, joint_names in (('left', left_joint_names), ('right', right_joint_names)):
        if not joint_names:
            problems.append("the controller's {}_wheel names no joint".format(side))
    for joint_name in left_joint_names + right_joint_names:
        joint = joints_by_name.get(joint_name)
        if joint is None:
            problems.append("the controller's wheel joint {} isn't in the URDF".format(joint_name))
            continue
        if joint.get('type') != 'continuous':
            problems.append('{} is {}, not continuous'.format(joint_name, joint.get('type')))
        if joint_name in joints_in_shared_transmissions and joint_name not in interface_by_joint:
            problems.append(
                '{} is in a transmission with more than one joint, which Gazebo skips'.format(
                    joint_name))
        elif interface_by_joint.get(joint_name) not in VELOCITY_INTERFACES:
            problems.append("{}'s transmission doesn't start with {}".format(
                joint_name, VELOCITY_INTERFACES[0]))
        link_name = required(joint, 'child').get('link', '')
        urdf_radius = collision_radius(robot, link_name)
        if not within_tolerance(urdf_radius, wheel_radius):
            problems.append('{} collision radius {:.4f} m, but wheel_radius is {:.4f} m'.format(
                link_name, urdf_radius, wheel_radius))
    # diffbot_base's hardware interface drives its joints in this order: slot 0 is the left motor
    hardware_joint_names = params.get(HARDWARE_JOINTS)
    controller_joint_names = left_joint_names + right_joint_names
    if hardware_joint_names != controller_joint_names:
        problems.append("the hardware interface's joints {} aren't the controller's wheels {}, "
                        'in order'.format(hardware_joint_names, controller_joint_names))
    # Between the first wheel of each side, as diff_drive_controller takes them
    first_joint_names = left_joint_names[:1] + right_joint_names[:1]
    if len(first_joint_names) == 2 and all(name in joints_by_name for name in first_joint_names):
        left_position, right_position = (joint_position(robot, name) for name in first_joint_names)
        urdf_separation = math.dist(left_position, right_position)
        if not within_tolerance(urdf_separation, wheel_separation):
            problems.append(
                'the wheel joints are {:.4f} m apart, but wheel_separation is {:.4f} m'.format(
                    urdf_separation, wheel_separation))
    return problems


class TestComparison(unittest.TestCase):
    """The comparison itself notices a wrong radius, separation, joint or interface."""

    URDF = """<robot name="test">
      <link name="base_footprint"/>
      <link name="chassis_link"/>
      <link name="front_left_wheel">
        <collision><geometry><sphere radius="0.033"/></geometry></collision>
      </link>
      <link name="front_right_wheel">
        <collision><geometry><sphere radius="0.033"/></geometry></collision>
      </link>
      <joint name="chassis_joint" type="fixed">
        <parent link="base_footprint"/><child link="chassis_link"/>
        <origin xyz="-0.036 0 0.0666" rpy="0 0 0"/>
      </joint>
      <joint name="front_left_wheel_joint" type="continuous">
        <parent link="chassis_link"/><child link="front_left_wheel"/>
        <origin xyz="0.036 0.072 -0.0336" rpy="0 0 0"/>
      </joint>
      <joint name="front_right_wheel_joint" type="continuous">
        <parent link="chassis_link"/><child link="front_right_wheel"/>
        <origin xyz="0.036 -0.072 -0.0336" rpy="0 0 0"/>
      </joint>
      <transmission name="left"><joint name="front_left_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint></transmission>
      <transmission name="right"><joint name="front_right_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint></transmission>
    </robot>"""

    def controller_params(self, wheel_radius: float = 0.033, wheel_separation: float = 0.144,
                          left_wheel: Any = 'front_left_wheel_joint',
                          right_wheel: Any = 'front_right_wheel_joint') -> Params:
        """Return parameters that agree with URDF, unless the arguments change them."""
        return {CONTROLLER + 'wheel_radius': wheel_radius,
                CONTROLLER + 'wheel_separation': wheel_separation,
                CONTROLLER + 'left_wheel': left_wheel,
                CONTROLLER + 'right_wheel': right_wheel,
                HARDWARE_JOINTS: ['front_left_wheel_joint', 'front_right_wheel_joint']}

    def test_agreement(self) -> None:
        self.assertEqual(mismatches(self.URDF, self.controller_params()), [])

    def test_wheel_lists(self) -> None:
        params = self.controller_params(left_wheel=['front_left_wheel_joint'],
                                        right_wheel=['front_right_wheel_joint'])
        self.assertEqual(mismatches(self.URDF, params), [])

    def test_wrong_radius(self) -> None:
        problems = mismatches(self.URDF, self.controller_params(wheel_radius=0.0325))
        self.assertEqual(len(problems), 2)

    def test_wrong_separation(self) -> None:
        problems = mismatches(self.URDF, self.controller_params(wheel_separation=0.145))
        self.assertEqual(len(problems), 1)
        self.assertIn('wheel_separation', problems[0])

    def test_unknown_wheel_joint(self) -> None:
        problems = mismatches(self.URDF, self.controller_params(left_wheel='left_wheel_joint'))
        self.assertEqual(len(problems), 2)
        self.assertIn("isn't in the URDF", problems[0])
        self.assertIn('hardware interface', problems[1])

    def test_renamed_joints_and_links(self) -> None:
        urdf = self.URDF
        for old_name, new_name in (('front_left_wheel_joint', 'axle_l'),
                                   ('front_right_wheel_joint', 'axle_r'),
                                   ('"front_left_wheel"', '"tire_a"'),
                                   ('"front_right_wheel"', '"tire_b"')):
            urdf = urdf.replace(old_name, new_name)
        params = self.controller_params(left_wheel='axle_l', right_wheel='axle_r')
        params[HARDWARE_JOINTS] = ['axle_l', 'axle_r']
        self.assertEqual(mismatches(urdf, params), [])

    def test_effort_interface(self) -> None:
        # The left wheel's transmission
        urdf = self.URDF.replace('hardware_interface/VelocityJointInterface',
                                 'hardware_interface/EffortJointInterface', 1)
        problems = mismatches(urdf, self.controller_params())
        self.assertEqual(len(problems), 1)
        self.assertIn("front_left_wheel_joint's transmission", problems[0])

    def test_velocity_interface_second(self) -> None:
        # Gazebo uses only the first interface, so the controller can't get a velocity handle
        effort = '<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>'
        urdf = self.URDF.replace('<hardwareInterface>', effort + '<hardwareInterface>', 1)
        problems = mismatches(urdf, self.controller_params())
        self.assertEqual(len(problems), 1)
        self.assertIn("front_left_wheel_joint's transmission", problems[0])

    def test_shared_transmission(self) -> None:
        # Both wheels in the left wheel's transmission, the right one's removed
        start = self.URDF.index('<transmission name="right">')
        end = self.URDF.index('</transmission>', start) + len('</transmission>')
        right_wheel_joint = ('<joint name="front_right_wheel_joint">'
                             '<hardwareInterface>hardware_interface/VelocityJointInterface'
                             '</hardwareInterface></joint>')
        urdf = self.URDF[:start] + self.URDF[end:]
        urdf = urdf.replace('</joint></transmission>',
                            '</joint>' + right_wheel_joint + '</transmission>', 1)
        problems = mismatches(urdf, self.controller_params())
        self.assertEqual(len(problems), 2)
        self.assertTrue(all('more than one joint' in problem for problem in problems))

    def test_nan_radius(self) -> None:
        urdf = self.URDF.replace('radius="0.033"', 'radius="nan"', 1)
        problems = mismatches(urdf, self.controller_params())
        self.assertEqual(len(problems), 1)
        self.assertIn('front_left_wheel collision radius nan m', problems[0])

    def test_nan_separation(self) -> None:
        problems = mismatches(self.URDF, self.controller_params(wheel_separation=math.nan))
        self.assertEqual(len(problems), 1)
        self.assertIn('wheel_separation is nan m', problems[0])

    def test_no_right_wheel(self) -> None:
        params = self.controller_params(right_wheel=[])
        params[HARDWARE_JOINTS] = ['front_left_wheel_joint']
        problems = mismatches(self.URDF, params)
        self.assertEqual(problems, ["the controller's right_wheel names no joint"])

    def test_hardware_joints_reversed(self) -> None:
        params = self.controller_params()
        params[HARDWARE_JOINTS] = ['front_right_wheel_joint', 'front_left_wheel_joint']
        problems = mismatches(self.URDF, params)
        self.assertEqual(len(problems), 1)
        self.assertIn('in order', problems[0])

    def test_hardware_joints_missing(self) -> None:
        params = self.controller_params()
        del params[HARDWARE_JOINTS]
        self.assertEqual(len(mismatches(self.URDF, params)), 1)


class TestModels(unittest.TestCase):
    """Each robot's URDF and controller configuration agree, as the launch files load them."""

    def check(self, model: str, params: Params) -> None:
        """Assert that the URDF in params agrees with the controller's parameters."""
        urdf = params[NAMESPACE + '/robot_description']
        self.assertEqual(mismatches(urdf, params), [],
                         '{}: URDF and controller disagree'.format(model))

    def test_simulation(self) -> None:
        for model in MODELS:
            with self.subTest(model=model):
                params = dump_params('diffbot_gazebo', 'diffbot_view.launch', model=model)
                params.update(dump_params('diffbot_control', 'diffbot_control.launch',
                                          model=model, rqt_robot_steering='false'))
                self.check(model, params)

    def test_robot(self) -> None:
        for model in MODELS:
            with self.subTest(model=model):
                self.check(model, dump_params('diffbot_base', 'diffbot.launch', model=model))


if __name__ == '__main__':
    unittest.main()
