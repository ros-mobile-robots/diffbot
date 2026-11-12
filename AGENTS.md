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
| `diffbot_gazebo_plugins` | The simulated IMU's Gazebo plugin: a copy of gazebo_plugins' `GazeboRosImuSensor` without `ros::spinOnce()` (#185), under the Apache License 2.0 |
| `diffbot_bringup`, `diffbot_robot`, `diffbot_navigation`, `diffbot_slam`, `diffbot_mbf`, `diffbot_msgs` | Bringup, navigation, SLAM, move_base_flex, messages |

## Commands

Run them in the dev container (`.devcontainer/noetic`), in `~/catkin_ws`:

```bash
catkin build && source devel/setup.bash
catkin build --catkin-make-args run_tests && catkin_test_results build   # all tests
rostest diffbot_gazebo sim_drive_diffbot.test                            # or sim_drive_remo.test
```

The firmware needs PlatformIO, which isn't in the dev container. Build it in its own container (`.docker/firmware`), on the host, not in the dev container: `.docker/firmware/run.sh pio run -e teensy31` and `.docker/firmware/run.sh pio run -e teensy40`, and the unit tests with `.docker/firmware/run.sh pio test -e native`. The script builds the image (cached when unchanged) and runs the command in `diffbot_firmware` as the host user (`LOCAL_UID` and `LOCAL_GID` override the IDs). Or install PlatformIO on the host with its installer script or in a virtual environment (CONTRIBUTING.md shows both), not with `pip` into the system Python, and run `pio run -e teensy31 && pio run -e teensy40` and `pio test -e native` in `diffbot_firmware`.

Without a display, Gazebo runs headless in the tests. Expect about a minute per robot for the simulation test.

## Before you call a change done

- **Model, controller or simulation changes:** run both robots' simulation tests and report the measured motions from the result files, not only pass/fail. [Simulation Test](https://ros-mobile-robots.com/development/simulation-test/) explains what they measure.
- **Firmware changes:** build both Teensy environments and run the unit tests on the PC, in the firmware container: `.docker/firmware/run.sh pio test -e native`.
- **Code and tests:** follow [Code and tests](#code-and-tests), and run its checks on every file you changed: flake8 and mypy for Python, clang-tidy's include cleaner for C++, shellcheck for shell scripts.
- **Tests for new logic:** new logic comes with a test that CI runs. That is a unit test for a function that computes or decides, and a rostest file for a node. A test that was run by hand and isn't in the repository doesn't count.
- **Everything:** `catkin build` without new warnings, and CI green on the pull request. Don't loosen a tolerance, skip a test or silence a warning to get there. Explain the failure instead.

## Code and tests

These rules hold for every language in this repository. `diffbot_gazebo/test/test_geometry_consistency.py` shows the design rules, and the firmware's tests in `diffbot_firmware/test/` show the test names.

### Design

- **One thing per function:** a function does one thing at one level of abstraction, and its name says what. A `main()` reads as the list of its steps.
- **Checks and a runner:** a function that checks several things becomes one function per check and one that runs them, as `test_geometry_consistency.py` does with its `check_…()` functions and `CHECKS`.
- **Ask or change:** a function returns an answer or changes state. One that has to do both says so in its name, such as the firmware's `readAndPublishSample()`.
- **One type for what belongs together:** values that travel together are a `NamedTuple` or a struct, such as `WheelSetup`. Not a long argument list, and not a tuple or a dictionary whose positions or keys the reader has to remember.
- **A type for a closed set:** states, kinds and reasons that the code itself defines are an `Enum` (`enum class` in C++), not strings or numbers.
- **One lock for shared state:** what a callback, a timer or another thread writes and someone else reads is guarded by one lock, and the comment at the lock names the threads. Values that belong to one report are read together, inside the lock.
- **Use what exists:** before you write something, look for an established library or tool that does it, and for a helper in the package. Own code is for what is specific to the robots. Prefer one shared source. Where a second copy of a value or a helper is needed, a comment names the other copy, and a test checks that the two agree.

The first two lines below leave the reader to remember which of the three numbers is the heading. The type under them, from `diffbot_gazebo/test/test_drive.py`, names it:

```python
start_pose = (0.0, 0.0, 1.57)
heading = start_pose[2]
```

```python
from typing import NamedTuple


class Pose2D(NamedTuple):
    """A pose on the ground: position x and y (m), heading theta (rad, 0 along the x axis)."""

    x: float
    y: float
    theta: float


start_pose = Pose2D(x=0.0, y=0.0, theta=1.57)
heading = start_pose.theta
```

### Names

- **What a name says:** what the thing is or does, such as `left_joint_names` for a list of joint names and `joints_by_name` for a dictionary. Not `left`, `found` or `l`, which a newcomer can't expand.
- **One word for one thing:** in the code, the tests and the comments of all packages. For example, `wrap` is the word for a counter that starts at zero again.
- **Constants:** a constant says its kind and its unit, such as `CMD_VEL_TOPIC`, `MODEL_NAME` and `DURATION = 3.0  # s`.
- **Functions:** a function is a verb, and one that returns a truth value reads as a statement, such as `within_tolerance()`.

### Comments and docstrings

- **Why:** a comment says why, or what the reader can't see in the code. It never says how the code was before ("now", "no longer", "used to"). That belongs in the pull request.
- **Docstrings:** Python docstrings follow [PEP 257](https://peps.python.org/pep-0257/). The first line of a function's or a method's docstring is one line and a command ("Return …", "Check …"). A module's and a class's first line is one line that says what it is. Then an empty line, then the description.
- **Flows:** a flow of several steps is a numbered list in the order of execution. Say who calls the function and when.
- **Named things:** the SBC (the robot's single-board computer), "the topic `/diagnostics`", "`WheelControl`'s command timeout". No "it" or "this" that the reader has to guess.
- **Launch files:** the first comment says what the file starts, who includes it and what it includes.
- **Lists by position:** a configuration list whose meaning lies in the position of its values, such as the six values of a controller's `twist_covariance_diagonal`, gets a label for each position, and each value that is set gets a comment that says what it is and where it comes from.
- **Issue numbers:** a number explains nothing by itself. Write the reason, and add the number only when the issue holds more that the reader needs.

The first comment below tells history, and nobody can say what its "it" is. The second one, from `test_drive.py`, says why the code sends the command in a loop:

```python
# Now sends the command in a loop, it stopped too early before
```

```python
# Send the command repeatedly, so the controller doesn't stop the robot (see COMMAND_RATE)
```

### Tests

- **Names:** a test's name says what the test checks, as a sentence: the subject, the observable result, then the condition, for example `test_bno055_startup_fails_at_the_reset_deadline_without_another_poll` or `test_imu_publisher_publishes_nothing_before_the_time_sync`. Omit the condition when none is needed to identify the behavior (`test_wire_bus_read_registers_uses_a_repeated_start`). The form follows the article [Naming standards for unit tests](https://osherove.com/blog/2005/4/3/naming-standards-for-unit-tests.html) (`MethodName_StateUnderTest_ExpectedBehavior`), with the result before the condition so that the name reads as a sentence. No numbers that put tests in an order (`test_1_…`), no name without a result (`test_counts`), and no name that claims more than the test checks. The older Python tests with short names, such as `test_forward`, keep them until their test changes. Then that test gets a name after this rule, without renaming the others.
- **One behavior per test:** several cases of one behavior can share a test. Loop over them and put the case into the assertion's message. Don't rely on `subTest` in tests that nose or rostest runs: nose stops at the first failing case without naming it, and rostest's result file doesn't record the failure. Cases that all have to run, whatever the others do, get a test each.
- **No order between tests:** a scenario whose steps need each other is one test that calls named phases (`check_…()` methods) in order, and the class's docstring says why.
- **Helpers:** a helper that asserts is called `assert_…` and says the case in its message. A helper that returns a value doesn't assert.
- **Values:** name the scenario's values with their unit and derive the rest from them. An expected result that the test exists to pin stays a value of the test's own. A tolerance's comment says where it comes from: a measurement, an analysis or a required precision.
- **State from callbacks:** what a subscriber's callback writes and the test reads is guarded by a lock. A small class that holds the lock keeps that out of the test class.
- **Waiting:** wait for readiness with a time limit instead of a fixed sleep. A test that has to observe for a fixed time names that time as a constant.

In the first block below, the deadline silently goes wrong when the timeout changes. The second line, from `diffbot_firmware/test/test_command_timeout/test_command_timeout.cpp`, derives it:

```cpp
constexpr uint32_t TEST_TIMEOUT_MS = 1000;
constexpr uint32_t ARRIVAL_MS = 100;
constexpr uint32_t DEADLINE_MS = 1100;
```

```cpp
constexpr uint32_t DEADLINE_MS = ARRIVAL_MS + TEST_TIMEOUT_MS;
```

### Python

PEP 8 with the settings of ROS 2's `ament_flake8` (99 columns, google import order, one import per line, single quotes), and type hints on every function and method (`-> None` on tests) that mypy checks for Python 3.8, Noetic's version. `diffbot_gazebo/test/test_drive.py` and `test_geometry_consistency.py` follow it. A script keeps its logic in functions that a test can import. Until CI checks it (#117), install flake8 with ament's plugins, mypy 1.10.1 (the version the project checks with) and the PyYAML type stubs in a virtual environment. The commands below check every Python file that differs from `noetic-devel`, committed or not, and every new one. Run them in bash from the repository root:

```bash
pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions \
  flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mypy==1.10.1 types-PyYAML
mapfile -d '' -t files < <({ git diff -z --name-only --diff-filter=d "$(git merge-base origin/noetic-devel HEAD)" -- '*.py'
                             git ls-files -z --others --exclude-standard -- '*.py'; } | sort -zu)
status=0
if [ "${#files[@]}" -gt 0 ]; then
  flake8 --max-line-length 99 --import-order-style google \
    --extend-ignore B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202 \
    "${files[@]}" || status=1
  for file in "${files[@]}"; do
    mypy --python-version 3.8 --ignore-missing-imports --disallow-untyped-defs "$file" || status=1
  done
fi
echo "checked ${#files[@]} files"
(exit "$status")  # the block's exit status: 1 if flake8 or mypy failed on any file
```

### C++ (diffbot_base and the firmware)

- **Include what you use:** every file includes the header of each name it uses itself, and doesn't rely on another header to bring it in. A header that can't be included alone stays behind its entry header with `// IWYU pragma: keep` and a comment that says why, as `ros/node_handle.h` in `diffbot_firmware/lib/imu/imu_publisher.h`. clang-tidy 18's check `misc-include-cleaner` finds missing and unused includes. It needs a compile database. Until CI runs it (#117), say in the pull request which files you checked with it.
- **The firmware's core:** it includes no Arduino, rosserial or vendor header, and its comments name no adapter. `diffbot_firmware/README.md` says which class is core, port and adapter.
- **Words:** a command is valid or invalid, and the firmware receives or drops it. Ports are primary and secondary.
- **Unit tests:** Unity tests in `diffbot_firmware/test/<suite>/`, named as above: `void test_<subject>_<result>_<condition>()`. A package that gets GoogleTest tests names them `TEST(Subject, ResultCondition)`, with the same three parts.
- **Final newline:** every file ends with one.

The first version below compiles only because `bno055.h` happens to include `i2c_bus.h`. The second one, as in `diffbot_firmware/test/fakes/fake_bno055.h`, names the header of each name it uses:

```cpp
#include "bno055.h"

class FakeBno055Bus : public diffbot::I2cBus { /* … */ };
```

```cpp
#include "bno055.h"
#include "i2c_bus.h"

class FakeBno055Bus : public diffbot::I2cBus { /* … */ };
```

### Launch files, shell scripts and workflows

- **Launch files:** the first comment as above. A Boolean argument's `doc` says what `true` does, and another argument's `doc` names the values it takes. A `$(eval …)` expression gets a comment that says what it computes.
- **Shell scripts:** a script that is run, not sourced, starts with `set -euo pipefail`. One with several steps has a function per step, `main "$@"` at the end, its constants as `readonly` with their unit, and a header that lists the steps as a numbered list. shellcheck passes.
- **Workflows:** the first comment says when the workflow runs and why it is a workflow of its own. Read-only permissions at the top, and more only in the job that needs them. Values from the event come in through `env:`, not through `${{ }}` inside `run:`, as in `.github/workflows/pr_description.yml`.

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
- **Comments:** see [Comments and docstrings](#comments-and-docstrings).
- **Documentation:** every feature comes with documentation on ros-mobile-robots.com, as a pull request in ros-mobile-robots.github.io that is planned together with the code, not after the review. Its pages say what the feature does and how to use it, and how the code works, with excerpts of the code and a figure for each flow. A tests page walks through each test in the order a reader would run it: what the test sets up, what goes in, what the robot or the node does, and what is checked with which tolerance, each with its code excerpt, and one picture per test. The writing rules are in that repository's `AGENTS.md`. Link the pull request under `## Documentation` in the description as `ros-mobile-robots/ros-mobile-robots.github.io#N`, also when it's opened after the code pull request. A page on ros-mobile-robots.com that already explains the change works too, and `None` when nothing user-visible changes. The PR description check (`.github/workflows/pr_description.yml`) fails otherwise. After an edit, the new run counts, a re-run of an old run still sees the old text.
- No attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
- Write descriptions and review replies so that a maintainer can check them quickly: specific, short, no claims you didn't verify.
