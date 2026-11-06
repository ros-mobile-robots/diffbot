#!/usr/bin/env python3
"""Drive the simulated robot with the simulation test's commands, for record_drive_video.sh.

The commands are those of test/test_drive.py, in the same order: 0.2 m/s forward for 3 s and a
stop, then 0.5 rad/s on the spot for 3 s and a stop. This script doesn't check where the robot
went. It's a new drive for the video, not the test.

Before it drives, it waits until the simulation time advances, the controller listens and the
camera saves frames, each for a limited wall time, and stops with an error if one of them doesn't
happen. During the drive, it stops with an error when the simulation time stands still. It
writes a JSON file with the commands, the drive's simulation and wall times, and how many frames
the camera had saved when the drive started and when it ended. encode_drive_video.py cuts the
video with these frame counts.

Usage: drive_for_video.py --frames DIR --pad SECONDS --output DRIVE_JSON
"""
import argparse
import json
import os
import threading
import time
from typing import Any
from typing import Callable
from typing import Dict
from typing import NamedTuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rosgraph
from rosgraph_msgs.msg import Clock
import rospy

CMD_VEL_TOPIC = '/diffbot/mobile_base_controller/cmd_vel'
ODOM_TOPIC = '/diffbot/mobile_base_controller/odom'
CLOCK_TOPIC = '/clock'

# The simulation test's commands: DURATION, FORWARD_VELOCITY, TURN_VELOCITY and COMMAND_RATE in
# test/test_drive.py, and the second that its stop() waits. The test doesn't import them from
# here, so it doesn't depend on the recording: change both files together
DURATION = 3.0  # s of simulation time
FORWARD_VELOCITY = 0.2  # m/s
TURN_VELOCITY = 0.5  # rad/s
COMMAND_RATE = 20  # Hz
STOP_TIME = 1.0  # s of simulation time

# How long to wait, in wall time, for each step of the start. With lockstep, the camera slows
# the simulation down, on a CI runner to about a third of real time
SIM_TIME_PARAMETER_TIMEOUT = 60.0  # s, for roslaunch to set /use_sim_time
CLOCK_TIMEOUT = 120.0  # s, for Gazebo to load the world and advance the simulation time
CONTROLLER_TIMEOUT = 120.0  # s, for the controller's odometry, as in test_drive.py
FRAMES_TIMEOUT = 60.0  # s, for the camera's first frames
# Wall time without a new /clock message after which the drive stops: Gazebo has stopped
STALL_TIMEOUT = 30.0  # s
# How many new frames show that the camera saves frames
FIRST_FRAMES = 3
# Simulation time that the drive waits after the trailing padding, so the video's last frame is
# never the one the camera may still be writing when the simulation stops
END_MARGIN = 0.5  # s


class Motion(NamedTuple):
    """A velocity command, sent for duration seconds of simulation time and then stopped."""

    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s
    duration: float  # s


# test_forward and then test_turn, the order in which unittest runs them
MOTIONS = (
    Motion(linear_velocity=FORWARD_VELOCITY, angular_velocity=0.0, duration=DURATION),
    Motion(linear_velocity=0.0, angular_velocity=TURN_VELOCITY, duration=DURATION),
)


class FrameMark(NamedTuple):
    """A moment of the recording: its simulation and wall time (s) and the frames saved so far."""

    sim_time: float
    wall_time: float
    frames: int


class ClockWatch:
    """Watch /clock: whether the simulation time advances, and when it stands still."""

    def __init__(self) -> None:
        self.first_time = -1.0
        self.latest_time = -1.0
        self.latest_wall_time = time.monotonic()
        rospy.Subscriber(CLOCK_TOPIC, Clock, self.on_clock)

    def on_clock(self, msg: Clock) -> None:
        if self.first_time < 0.0:
            self.first_time = msg.clock.to_sec()
        self.latest_time = msg.clock.to_sec()
        self.latest_wall_time = time.monotonic()

    def advances(self) -> bool:
        """Return whether the simulation time has advanced since the first /clock message."""
        return self.latest_time > self.first_time >= 0.0

    def stop_node_when_stalled(self, stall_timeout: float) -> None:
        """Stop the node when no /clock message arrives for stall_timeout seconds of wall time.

        A stopped Gazebo server stops the simulation time, and rospy's sleep() would then wait
        forever. Stopping the node makes it raise an exception instead.
        """
        def watch() -> None:
            while not rospy.is_shutdown():
                if time.monotonic() - self.latest_wall_time > stall_timeout:
                    rospy.logerr('No /clock message for %.0f s: has Gazebo stopped?',
                                 stall_timeout)
                    rospy.signal_shutdown('simulation time stands still')
                time.sleep(1.0)
        threading.Thread(target=watch, daemon=True).start()


def count_frames(frames_dir: str) -> int:
    """Return how many frames the camera has saved in frames_dir so far."""
    if not os.path.isdir(frames_dir):
        return 0
    return sum(1 for name in os.listdir(frames_dir) if name.endswith('.jpg'))


def wait_until(condition: Callable[[], bool], timeout: float, what: str) -> None:
    """Check condition() every 0.1 s of wall time until it's true.

    Raise TimeoutError, naming what was awaited, when timeout seconds of wall time pass first.
    """
    deadline = time.monotonic() + timeout
    while not condition():
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException(f'stopped while waiting for {what}')
        if time.monotonic() > deadline:
            raise TimeoutError(f'no {what} within {timeout:.0f} s')
        time.sleep(0.1)


def sim_time_is_set() -> bool:
    """Return whether the ROS master runs and /use_sim_time is true."""
    try:
        return rosgraph.Master('/drive_for_video').getParam('/use_sim_time') is True
    except (OSError, rosgraph.MasterException):
        # The master doesn't run yet, or roslaunch hasn't set the parameter yet
        return False


def mark(frames_dir: str) -> FrameMark:
    """Return the current simulation and wall time and the number of saved frames."""
    return FrameMark(rospy.get_time(), time.time(), count_frames(frames_dir))


def drive(cmd_vel: rospy.Publisher, motion: Motion) -> None:
    """Send the motion's command for its duration, as the simulation test drives."""
    cmd = Twist()
    cmd.linear.x = motion.linear_velocity
    cmd.angular.z = motion.angular_velocity
    # The controller stops the robot when no command arrives for 0.5 s, so send it repeatedly
    rate = rospy.Rate(COMMAND_RATE)
    end_time = rospy.Time.now() + rospy.Duration(motion.duration)
    while rospy.Time.now() < end_time:
        cmd_vel.publish(cmd)
        rate.sleep()


def stop(cmd_vel: rospy.Publisher) -> None:
    """Send a zero velocity and wait for the robot to come to rest, as the simulation test."""
    cmd_vel.publish(Twist())
    rospy.sleep(STOP_TIME)


def drive_info(start: FrameMark, end: FrameMark, pad: float) -> Dict[str, Any]:
    """Return what encode_drive_video.py and the video's JSON file need to know of the drive."""
    return {
        'topic': CMD_VEL_TOPIC,
        'command_rate': COMMAND_RATE,
        'motions': [motion._asdict() for motion in MOTIONS],
        'stop_time': STOP_TIME,
        'pad': pad,
        'start': start._asdict(),
        'end': end._asdict(),
    }


def parse_arguments() -> argparse.Namespace:
    """Return the command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--frames', required=True, help="the camera's frames folder")
    parser.add_argument('--pad', type=float, required=True,
                        help='simulation time (s) to record before and after the drive')
    parser.add_argument('--output', required=True, help='where to write the JSON file')
    return parser.parse_args()


def main() -> None:
    """Wait until the simulation is ready, drive, and write the JSON file."""
    args = parse_arguments()
    # rospy reads /use_sim_time only when the node starts. Started before roslaunch has set it,
    # the node would time the commands in wall time
    wait_until(sim_time_is_set, SIM_TIME_PARAMETER_TIMEOUT, '/use_sim_time from roslaunch')
    rospy.init_node('drive_for_video')
    clock = ClockWatch()
    wait_until(clock.advances, CLOCK_TIMEOUT, 'advancing simulation time on /clock')
    cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=CONTROLLER_TIMEOUT)
    wait_until(lambda: cmd_vel.get_num_connections() > 0, CONTROLLER_TIMEOUT,
               f'controller listening on {CMD_VEL_TOPIC}')
    first_frames = count_frames(args.frames) + FIRST_FRAMES
    wait_until(lambda: count_frames(args.frames) >= first_frames, FRAMES_TIMEOUT,
               f'camera frames in {args.frames}')
    clock.stop_node_when_stalled(STALL_TIMEOUT)

    # The robot at rest before the drive, for the video's leading padding
    rospy.sleep(args.pad)
    start = mark(args.frames)
    for motion in MOTIONS:
        drive(cmd_vel, motion)
        stop(cmd_vel)
    end = mark(args.frames)
    rospy.sleep(args.pad + END_MARGIN)
    rospy.loginfo('Drove %.1f s of simulation time in %.1f s of wall time, the camera saved '
                  '%d frames', end.sim_time - start.sim_time, end.wall_time - start.wall_time,
                  end.frames - start.frames)
    with open(args.output, 'w') as output:
        json.dump(drive_info(start, end, args.pad), output, indent=2)
        output.write('\n')


if __name__ == '__main__':
    try:
        main()
    except (TimeoutError, rospy.ROSException) as error:
        raise SystemExit(f'drive_for_video.py: {error}')
