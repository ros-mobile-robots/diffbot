#!/usr/bin/env python3
"""Write a copy of a Gazebo world with a video camera and a floor grid, and without shadows.

record_drive_video.sh records the simulation test's drive on a copy of the test's world, so the
physics are the test's. The camera is a Gazebo camera sensor on a static model. With
<save enabled="true">, the Gazebo server writes every frame it renders into a folder as a JPEG
file, without ROS. The grid's lines, 0.5 m apart, show the robot's motion on the plain ground.
Shadows are off: with Mesa's software renderer, they made the rendering two to three times slower
when the recording was tried out, and they didn't show in the image.

Usage: make_video_world.py BASE_WORLD OUTPUT_WORLD --frames DIR [--width W --height H --rate HZ]
"""
import argparse
import math
import re
from typing import NamedTuple
import xml.etree.ElementTree as ET

# Gazebo creates the frames folder with a shell command that contains the path (mkdir in its
# Camera.cc), so the path may only contain characters that a shell takes literally
SAFE_PATH = re.compile(r'[A-Za-z0-9_./-]+')

# The floor grid: lines every GRID_SPACING m, from -GRID_HALF_SIZE to GRID_HALF_SIZE m in x and y
GRID_SPACING = 0.5  # m
GRID_HALF_SIZE = 3.0  # m
GRID_LINE_WIDTH = 0.012  # m


class CameraPose(NamedTuple):
    """Where the camera is and where it looks: position (m), pitch and yaw (rad), field of view.

    A positive pitch looks down. horizontal_fov is the angle (rad) that the image's width covers.
    """

    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    horizontal_fov: float


# The robot starts at x = -2, y = -0.5, facing along x (diffbot_view.launch), drives 0.6 m and
# turns on the spot. The camera stands beside that path and looks along y, at the path's middle,
# so the robot drives from left to right through the image
CAMERA_POSE = CameraPose(x=-1.7, y=-1.55, z=0.75, pitch=0.62, yaw=math.pi / 2,
                         horizontal_fov=0.9)


def camera_model(frames_dir: str, width: int, height: int, rate: float) -> ET.Element:
    """Return a static model with a camera that saves every frame to frames_dir."""
    pose = CAMERA_POSE
    return ET.fromstring(f"""
    <model name="video_camera">
      <static>true</static>
      <pose>{pose.x} {pose.y} {pose.z} 0 {pose.pitch:.4f} {pose.yaw:.4f}</pose>
      <link name="link">
        <sensor name="video" type="camera">
          <always_on>true</always_on>
          <update_rate>{rate}</update_rate>
          <visualize>false</visualize>
          <camera name="camera">
            <horizontal_fov>{pose.horizontal_fov}</horizontal_fov>
            <image>
              <width>{width}</width>
              <height>{height}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.05</near>
              <far>30</far>
            </clip>
            <save enabled="true">
              <path>{frames_dir}</path>
            </save>
          </camera>
        </sensor>
      </link>
    </model>""")


def grid_line(name: str, size_x: float, size_y: float, x: float, y: float) -> ET.Element:
    """Return a dark box visual, 1 mm high, lying on the ground at x, y: one line of the grid."""
    return ET.fromstring(f"""
        <visual name="{name}">
          <pose>{x} {y} 0.0005 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <box>
              <size>{size_x} {size_y} 0.001</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/DarkGrey</name>
            </script>
          </material>
        </visual>""")


def grid_model() -> ET.Element:
    """Return a static model with the floor grid's lines, visuals only, so nothing collides."""
    model = ET.fromstring('<model name="floor_grid"><static>true</static></model>')
    link = ET.SubElement(model, 'link', name='link')
    line_count = round(2 * GRID_HALF_SIZE / GRID_SPACING) + 1
    length = 2 * GRID_HALF_SIZE
    for index in range(line_count):
        offset = -GRID_HALF_SIZE + index * GRID_SPACING
        link.append(grid_line(f'line_x_{index}', length, GRID_LINE_WIDTH, 0.0, offset))
        link.append(grid_line(f'line_y_{index}', GRID_LINE_WIDTH, length, offset, 0.0))
    return model


def disable_shadows(world: ET.Element) -> None:
    """Switch the world's shadows off, adding the scene and shadows elements if needed."""
    scene = world.find('scene')
    if scene is None:
        scene = ET.SubElement(world, 'scene')
    shadows = scene.find('shadows')
    if shadows is None:
        shadows = ET.SubElement(scene, 'shadows')
    shadows.text = 'false'


def parse_arguments() -> argparse.Namespace:
    """Return the command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('base_world', help='the world to copy, for example diffbot_empty.world')
    parser.add_argument('output_world', help='where to write the copy')
    parser.add_argument('--frames', required=True, help='folder for the camera frames')
    parser.add_argument('--width', type=int, default=960, help='image width (pixels)')
    parser.add_argument('--height', type=int, default=540, help='image height (pixels)')
    parser.add_argument('--rate', type=float, default=30.0,
                        help='frames per second of simulation time')
    return parser.parse_args()


def main() -> None:
    """Read the base world, add the camera and the grid, switch shadows off, write the copy."""
    args = parse_arguments()
    if not SAFE_PATH.fullmatch(args.frames):
        raise SystemExit(f'make_video_world.py: the frames folder {args.frames!r} may contain '
                         'only letters, digits and _ . / -')
    try:
        tree = ET.parse(args.base_world)
    except (OSError, ET.ParseError) as error:
        raise SystemExit(f'make_video_world.py: {args.base_world}: {error}')
    world = tree.getroot().find('world')
    if world is None:
        raise SystemExit(f'make_video_world.py: {args.base_world} has no <world>')
    disable_shadows(world)
    world.append(grid_model())
    world.append(camera_model(args.frames, args.width, args.height, args.rate))
    tree.write(args.output_world, encoding='unicode', xml_declaration=False)


if __name__ == '__main__':
    main()
