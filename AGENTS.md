# Agent instructions for DiffBot

Read [CONTRIBUTING.md](CONTRIBUTING.md) first; this file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## What this is

ROS 1 Noetic packages for two differential drive robots, **DiffBot** and **Remo**, plus their Teensy firmware and a dev container. Most launch files take `model:=diffbot` or `model:=remo`. Remo's URDF lives in the separate repository [remo_description](https://github.com/ros-mobile-robots/remo_description), which the dev container clones next to this one. The documentation is the separate site [ros-mobile-robots.com](https://ros-mobile-robots.com) ([ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io)).

| Package | What's in it |
|---|---|
| `diffbot_base` | Hardware interface for the real robot |
| `diffbot_firmware` | Teensy firmware (PlatformIO project, not a ROS package); old Arduino test sketches in `sketches/` |
| `diffbot_control` | `diff_drive_controller` configuration; per robot: `config/<model>/geometry.yaml` (wheel radius and separation) and `config/<model>/pid.yaml` (Gazebo's wheel gains, simulation only) |
| `diffbot_description` | DiffBot's URDF (xacro) and its part sizes and masses in `config/diffbot/*.yaml` |
| `diffbot_gazebo` | Worlds, launch files and the simulation test `test/test_drive.py` |
| `diffbot_bringup`, `diffbot_robot`, `diffbot_navigation`, `diffbot_slam`, `diffbot_mbf`, `diffbot_msgs` | Bringup, navigation, SLAM, move_base_flex, messages |

## Commands

Run them in the dev container (`.devcontainer/noetic`), in `~/catkin_ws`:

```bash
catkin build && source devel/setup.bash
catkin build --catkin-make-args run_tests && catkin_test_results build   # all tests
rostest diffbot_gazebo sim_drive_diffbot.test                            # or sim_drive_remo.test
```

The firmware needs PlatformIO, which isn't in the dev container. On the host, install it with PlatformIO's installer script or in a virtual environment (CONTRIBUTING.md shows one), not with `pip` into the system Python; then run `pio run -e teensy31 && pio run -e teensy40` in `diffbot_firmware` of the host's clone.

Without a display, Gazebo runs headless in the tests. Expect about a minute per robot for the simulation test.

## Before you call a change done

- **Model, controller or simulation changes:** run both robots' simulation tests and report the measured motions from the result files, not only pass/fail. [Simulation Test](https://ros-mobile-robots.com/development/simulation-test/) explains what they measure.
- **Firmware changes:** build both PlatformIO environments.
- **Python code:** PEP 8 with the settings of ROS 2's `ament_flake8` (99 columns, google import order, one import per line, single quotes), function docstrings after PEP 257 whose first line is a command ("Return …", "Check …"), type hints that mypy checks for Python 3.8, Noetic's version, and names that say what a value holds, for example `left_joint_names` for a list of joint names. `diffbot_gazebo/test/test_drive.py` and `test_geometry_consistency.py` follow it. Until CI checks it (#117), install flake8 with ament's plugins, mypy 1.10.1 (newer versions can't check for Python 3.8) and the PyYAML type stubs in a virtual environment, and run from the repository root:

  ```bash
  pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions \
    flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mypy==1.10.1 types-PyYAML
  flake8 --max-line-length 99 --import-order-style google \
    --extend-ignore B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202 \
    diffbot_gazebo/test/*.py
  mypy --python-version 3.8 --ignore-missing-imports diffbot_gazebo/test/*.py
  ```

- **Design, in Python and C++:** a function does one thing at one level of abstraction, and its name says what. A function that checks several things becomes one function per check and one that runs them, as `test_geometry_consistency.py` does with its `check_…()` functions and `CHECKS`. Values that belong together travel as one type, such as its `WheelSetup`, instead of long argument lists. The refactor of diffbot_base and the firmware (#148) follows the same rules.
- **Everything:** `catkin build` without new warnings, and CI green on the pull request. Don't loosen a tolerance, skip a test or silence a warning to get there; explain the failure instead.

## Things that are easy to get wrong

- **The wheel geometry is in two places:** the URDF (`diffbot_description/config/diffbot/front_wheel.yaml`, Remo's in remo_description) and the controller (`diffbot_control/config/<model>/geometry.yaml`). Change both, or the odometry and the simulated robot disagree; `diffbot_gazebo/test/test_geometry_consistency.py` checks that they agree, and that the controller's wheel joints and their velocity interfaces are in the URDF.
- **A wheel's mass limits Gazebo's wheel gain:** if you change a wheel's mass or inertia in the URDF, check the P gain in `diffbot_control/config/<model>/pid.yaml` against the comment there.
- **remo_description is pinned to one version** in `diffbot_dev.repos` and `remo_robot.repos`. A new version of Remo's model reaches the diffbot packages only when the pin moves: move it in the pull request that adapts the diffbot packages to the new model, and test both robots. An existing workspace keeps its old checkout when the pin moves (the setup imports with `--skip-existing`): check out the new version in `src/remo_description` and rebuild.
- **The documentation quotes this code** at a pinned commit, by line ranges, for example `test_drive.py` on the Simulation Test page. If you add or remove lines there, say so in the PR, so the docs can be repinned.
- **Units and frames** follow [REP 103](https://www.ros.org/reps/rep-0103.html); `base_footprint` is the robot's reference frame, on the ground below the drive axle's midpoint, with `base_link` above it; the chassis and its parts hang off `chassis_link`.
- **The real robot reads the same configuration** as the simulation (the hardware interface and the firmware read `wheel_radius`). A configuration change affects both.

## Commits and pull requests

- One feature or fix per pull request and per commit; branch `feat/…`, `fix/…`, `docs/…` or `chore/…`.
- Squash-merged: the title is the commit subject, `Area: what changed`, 65 characters or fewer before GitHub's PR-number suffix. The whole description becomes the commit message: what and why in two or three sentences, then how it was tested, then `Closes #N`. Use the pull request template.
- Code comments explain why, not what changed or how it was before.
- A feature also needs a new or updated page on ros-mobile-robots.com: a pull request in ros-mobile-robots.github.io.
- No attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
- Write descriptions and review replies so that a maintainer can check them quickly: specific, short, no claims you didn't verify.
