#!/usr/bin/env python3
"""Cut the camera's frames to the drive, encode them as an MP4 video, check it, describe it.

record_drive_video.sh runs this after the simulation has stopped. drive_for_video.py noted how
many frames the camera had saved when the drive started and when it ended. The video starts
pad seconds of simulation time before the drive and ends pad seconds after it. With lockstep,
the camera saves a frame every 1/rate s of simulation time, so a video with rate frames per
second shows the drive at the speed of simulation time, however slowly the computer ran it.

The video is H.264 in an MP4 file, which browsers play, without metadata. ffprobe and a full
decode check it. Next to it goes a JSON file with the model, the commit, the commands, the
image's software and the settings. Any failed check stops this script with an error before the
video is in place.

Usage: encode_drive_video.py --frames DIR --drive DRIVE_JSON --world WORLD --model MODEL
                             [--lockstep] --video VIDEO_MP4 --info VIDEO_JSON
"""
import argparse
from fractions import Fraction
import json
import os
import re
import subprocess
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
import xml.etree.ElementTree as ET

# The camera saves frames as <sensor's scoped name>-<number>.jpg, numbered from 0 with at least
# four digits (Gazebo's Camera.cc)
FRAME_NAME = re.compile(r'(?P<prefix>.+)-(?P<number>\d{4,})\.jpg')
# How far the frames per second of simulation time may be from the camera's rate. With
# lockstep they match, apart from a frame that was being written while drive_for_video.py
# counted. Without lockstep, Gazebo drops frames and the video would play too fast
FRAME_RATE_TOLERANCE = 0.03
# x264's constant quality, lower is better: 26 instead of its default 23 made a 20 s clip of
# DiffBot about a third smaller (205 instead of 316 kB) when the recording was tried out
CRF = 26
PRESET = 'slow'
PIXEL_FORMAT = 'yuv420p'


class RenderSettings(NamedTuple):
    """The camera's image size (pixels) and rate (frames per second of simulation time).

    shadows tells whether the world renders shadows.
    """

    width: int
    height: int
    rate: float
    shadows: bool


class SavedFrames(NamedTuple):
    """The camera's frames: their file names as a pattern for ffmpeg (%04d), and how many."""

    pattern: str
    total: int


class Clip(NamedTuple):
    """The frames of the video: the first frame's number and how many frames follow."""

    first: int
    length: int


class VideoProbe(NamedTuple):
    """What ffprobe reports of the encoded video's stream."""

    codec: str
    width: int
    height: int
    pixel_format: str
    frame_rate: Fraction
    frames: int
    duration: float


def element_text(world: ET.Element, path: str) -> str:
    """Return the text of the world's element at path, or raise ValueError if it has none."""
    text = world.findtext(path)
    if text is None:
        raise ValueError(f'the world has no {path}')
    return text


def read_render_settings(world_path: str) -> RenderSettings:
    """Return the video camera's settings and the shadows from the world that Gazebo used."""
    world = ET.parse(world_path).getroot()
    camera = 'world/model[@name="video_camera"]/link/sensor'
    return RenderSettings(width=int(element_text(world, f'{camera}/camera/image/width')),
                          height=int(element_text(world, f'{camera}/camera/image/height')),
                          rate=float(element_text(world, f'{camera}/update_rate')),
                          shadows=element_text(world, 'world/scene/shadows') == 'true')


def find_frames(frames_dir: str) -> SavedFrames:
    """Return the saved frames, after checking that they come from one camera, none missing."""
    numbers = []
    prefixes = set()
    for name in os.listdir(frames_dir):
        match = FRAME_NAME.fullmatch(name)
        if match:
            prefixes.add(match['prefix'])
            numbers.append(int(match['number']))
    if len(prefixes) != 1:
        raise ValueError(f'expected the frames of one camera in {frames_dir}, found {prefixes}')
    if sorted(numbers) != list(range(len(numbers))):
        raise ValueError(f'frames are missing in {frames_dir}')
    return SavedFrames(pattern=os.path.join(frames_dir, prefixes.pop() + '-%04d.jpg'),
                       total=len(numbers))


def check_frame_rate(drive: Dict[str, Any], rate: float) -> float:
    """Return the frames saved per second of simulation time during the drive.

    Raise ValueError if they differ from the camera's rate.
    """
    frames = drive['end']['frames'] - drive['start']['frames']
    sim_time = drive['end']['sim_time'] - drive['start']['sim_time']
    measured_rate = frames / sim_time
    if abs(measured_rate - rate) > FRAME_RATE_TOLERANCE * rate:
        raise ValueError(f'the camera saved {measured_rate:.1f} frames per second of simulation '
                         f'time instead of {rate:g}: does Gazebo run with lockstep?')
    return measured_rate


def select_clip(drive: Dict[str, Any], rate: float, saved_frames: int) -> Clip:
    """Return the frames from pad seconds before the drive's start to pad seconds after its end.

    The clip never contains the last saved frame: the camera may still have been writing it
    when the simulation stopped.
    """
    pad_frames = round(drive['pad'] * rate)
    first = drive['start']['frames'] - pad_frames
    end = drive['end']['frames'] + pad_frames
    if first < 0:
        raise ValueError(f'the drive started after {drive["start"]["frames"]} frames, too '
                         f'early for {pad_frames} frames of padding')
    if end >= saved_frames:
        raise ValueError(f'the camera saved {saved_frames} frames, the video needs more than '
                         f'{end}')
    return Clip(first=first, length=end - first)


def encode(frames: SavedFrames, clip: Clip, rate: float, video_path: str) -> None:
    """Encode the clip's frames as an H.264 MP4 at rate frames per second."""
    subprocess.run(
        ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-y',
         '-framerate', f'{rate:g}', '-start_number', str(clip.first), '-i', frames.pattern,
         '-frames:v', str(clip.length), '-an',
         '-c:v', 'libx264', '-preset', PRESET, '-crf', str(CRF), '-pix_fmt', PIXEL_FORMAT,
         # No metadata, no encoder name and no x264 settings (SEI) in the file, so it doesn't
         # change with the tools' versions. faststart puts the index first, for the web
         '-map_metadata', '-1', '-fflags', '+bitexact', '-flags:v', '+bitexact',
         '-bsf:v', 'filter_units=remove_types=6', '-movflags', '+faststart',
         '-f', 'mp4', video_path],
        check=True)


def probe(video_path: str) -> VideoProbe:
    """Return ffprobe's report of the video's first stream, counting its frames by decoding."""
    result = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_frames',
         '-show_entries',
         'stream=codec_name,width,height,pix_fmt,avg_frame_rate,nb_read_frames,duration',
         '-of', 'json', video_path],
        check=True, stdout=subprocess.PIPE, universal_newlines=True)
    stream = json.loads(result.stdout)['streams'][0]
    return VideoProbe(codec=stream['codec_name'], width=stream['width'],
                      height=stream['height'], pixel_format=stream['pix_fmt'],
                      frame_rate=Fraction(stream['avg_frame_rate']),
                      frames=int(stream['nb_read_frames']),
                      duration=float(stream['duration']))


def check_video(video: VideoProbe, camera: RenderSettings, clip: Clip) -> None:
    """Check that the video has the camera's size and rate and all of the clip's frames."""
    expected = {
        'codec': (video.codec, 'h264'),
        'width': (video.width, camera.width),
        'height': (video.height, camera.height),
        'pixel format': (video.pixel_format, PIXEL_FORMAT),
        'frame rate': (video.frame_rate, Fraction(camera.rate)),
        'frames': (video.frames, clip.length),
    }
    problems = [f'{name} {found}, expected {wanted}'
                for name, (found, wanted) in expected.items() if found != wanted]
    expected_duration = clip.length / camera.rate
    if abs(video.duration - expected_duration) > 1.0 / camera.rate:
        problems.append(f'duration {video.duration} s, expected {expected_duration} s')
    if problems:
        raise ValueError('the video is not as encoded: ' + ', '.join(problems))


def check_decoding(video_path: str) -> None:
    """Decode the whole video and fail at the first error (-xerror)."""
    subprocess.run(['ffmpeg', '-hide_banner', '-v', 'error', '-nostdin', '-xerror',
                    '-i', video_path, '-f', 'null', '-'], check=True)


def run_text(command: List[str], cwd: Optional[str] = None) -> Optional[str]:
    """Return the command's output, without surrounding whitespace, or None if it fails."""
    try:
        result = subprocess.run(command, cwd=cwd, check=True, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, universal_newlines=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def source_revision() -> Dict[str, Any]:
    """Return the commit of this script's repository, and whether tracked files differ from it.

    Both are None outside a Git checkout, for example in an install space.
    """
    script_dir = os.path.dirname(os.path.realpath(__file__))
    commit = run_text(['git', 'rev-parse', 'HEAD'], cwd=script_dir)
    changes = run_text(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=script_dir)
    return {'commit': commit, 'modified': None if changes is None else changes != ''}


def os_name() -> Optional[str]:
    """Return the operating system's name and version from /etc/os-release."""
    try:
        with open('/etc/os-release') as os_release:
            match = re.search(r'PRETTY_NAME="([^"]*)"', os_release.read())
    except OSError:
        return None
    return match.group(1) if match else None


def package_version(package: str) -> Optional[str]:
    """Return the version of the installed Ubuntu package, or None."""
    return run_text(['dpkg-query', '--show', '--showformat', '${Version}', package])


def image_software() -> Dict[str, Optional[str]]:
    """Return the versions of the software in the image that made the video."""
    return {
        'os': os_name(),
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'gazebo': package_version('gazebo11'),
        'mesa': package_version('libgl1-mesa-dri'),
        'ffmpeg': package_version('ffmpeg'),
    }


def video_info(args: argparse.Namespace, drive: Dict[str, Any], camera: RenderSettings,
               measured_rate: float, video: VideoProbe) -> Dict[str, Any]:
    """Return the description of the video for its JSON file."""
    drive_sim_time = drive['end']['sim_time'] - drive['start']['sim_time']
    drive_wall_time = drive['end']['wall_time'] - drive['start']['wall_time']
    return {
        'description': "The simulation test's commands, driven once more for this video in a "
                       'run of their own, not a recording of the test',
        'model': args.model,
        'source': source_revision(),
        'commands': {key: drive[key] for key in ('topic', 'command_rate', 'motions',
                                                 'stop_time')},
        'image': image_software(),
        'settings': {
            **camera._asdict(),
            'lockstep': args.lockstep,
            'software_rendering': os.environ.get('LIBGL_ALWAYS_SOFTWARE') == '1',
            'pad': drive['pad'],
            'encoder': {'codec': 'libx264', 'preset': PRESET, 'crf': CRF},
        },
        'drive': {
            'sim_time': round(drive_sim_time, 3),
            'wall_time': round(drive_wall_time, 3),
            'real_time_factor': round(drive_sim_time / drive_wall_time, 3),
            'frames_per_sim_second': round(measured_rate, 2),
        },
        'video': {
            'file': os.path.basename(args.video),
            'bytes': os.path.getsize(args.video),
            'codec': video.codec,
            'width': video.width,
            'height': video.height,
            'frame_rate': str(video.frame_rate),
            'frames': video.frames,
            'duration': video.duration,
        },
    }


def parse_arguments() -> argparse.Namespace:
    """Return the command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--frames', required=True, help="the camera's frames folder")
    parser.add_argument('--drive', required=True, help="drive_for_video.py's JSON file")
    parser.add_argument('--world', required=True, help='the world with the video camera')
    parser.add_argument('--model', required=True, help='the robot model, for the JSON file')
    parser.add_argument('--lockstep', action='store_true',
                        help='Gazebo ran with lockstep, for the JSON file')
    parser.add_argument('--video', required=True, help='where to write the MP4 video')
    parser.add_argument('--info', required=True, help="where to write the video's JSON file")
    return parser.parse_args()


def main() -> None:
    """Cut, encode and check the video, then move it in place and write its JSON file."""
    args = parse_arguments()
    with open(args.drive) as drive_file:
        drive = json.load(drive_file)
    camera = read_render_settings(args.world)
    frames = find_frames(args.frames)
    measured_rate = check_frame_rate(drive, camera.rate)
    clip = select_clip(drive, camera.rate, frames.total)
    # Encode and check a temporary file, so the video's name only ever holds a checked video
    partial_video = args.video + '.partial'
    try:
        encode(frames, clip, camera.rate, partial_video)
        video = probe(partial_video)
        check_video(video, camera, clip)
        check_decoding(partial_video)
        os.replace(partial_video, args.video)
    finally:
        if os.path.exists(partial_video):
            os.remove(partial_video)
    info = video_info(args, drive, camera, measured_rate, video)
    with open(args.info, 'w') as info_file:
        json.dump(info, info_file, indent=2)
        info_file.write('\n')
    size = info['video']['bytes']
    print(f'encode_drive_video.py: {args.video}: {video.frames} frames, {video.duration:g} s, '
          f'{size} bytes')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        raise SystemExit(f'encode_drive_video.py: {error}')
