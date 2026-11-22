# Agent instructions for DiffBot

Read [CONTRIBUTING.md](CONTRIBUTING.md) first; this file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## Before you push

Find your change in this list. Each line says what to run or check, and links the section with the details.

- **Robot model, controller or simulation:** run both robots' simulation tests and report the measured motions from the result files, not only pass/fail ([Commands](#commands)). [Simulation Test](https://ros-mobile-robots.com/development/simulation-test/) explains what they measure.
- **Firmware:** build both Teensy environments and run the unit tests on the PC ([Commands](#commands)).
- **Python:** flake8 and mypy pass on every `.py` file you changed ([Python](#python)).
- **C++:** clang-tidy's include check on every C++ file you changed, headers and board-only files included ([Check the includes](#check-the-includes)).
- **Shell script:** shellcheck reports no finding for a new script, and no finding that the script didn't have before your change. A change to the robot image's entry point also passes the image's smoke test ([Launch files, shell scripts and workflows](#launch-files-shell-scripts-and-workflows)).
- **New logic:** new logic comes with a test that CI runs. That is a unit test for a function that computes or decides, and a rostest file for a node. A test that was run by hand and isn't in the repository doesn't count ([Tests](#tests)).
- **A feature of the robots' code, of CI or of the development setup:** the documentation pull request is planned with the code and linked under `## Documentation` ([Commits and pull requests](#commits-and-pull-requests)).
- **Every change:** `catkin build` without new warnings, and CI green on the pull request. Don't loosen a tolerance, skip a test or silence a warning to get there. Explain the failure instead. The commits are signed and hold one topic ([Commits and pull requests](#commits-and-pull-requests)).

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

**In the dev container** (`.devcontainer/noetic`), in `~/catkin_ws`:

```bash
catkin build && source devel/setup.bash                                   # Summary: All … packages succeeded!
catkin build --catkin-make-args run_tests && catkin_test_results build   # all tests. Summary: … tests, 0 errors, 0 failures, 0 skipped
rostest diffbot_gazebo sim_drive_diffbot.test                            # or sim_drive_remo.test. RESULT: SUCCESS
```

Without a display, Gazebo runs headless in the tests. Expect about a minute per robot for the simulation test. [Read the results](https://ros-mobile-robots.com/development/simulation-test/#read-the-results) on the Simulation Test page says where the result file with the measured motions is.

**On the PC, not in the dev container**, in the repository's root. The firmware needs PlatformIO, which isn't in the dev container. `.docker/firmware/run.sh` builds the image of the firmware's own container (cached when unchanged) and runs a command in that container, in `diffbot_firmware`, as the host user (`LOCAL_UID` and `LOCAL_GID` override the IDs):

```bash
.docker/firmware/run.sh pio run -e teensy31   # DiffBot's Teensy 3.2. Ends with [SUCCESS]
.docker/firmware/run.sh pio run -e teensy40   # Remo's Teensy 4.0. Ends with [SUCCESS]
.docker/firmware/run.sh pio test -e native    # the unit tests on the PC. Ends with "… test cases: … succeeded"
```

Or install PlatformIO on the PC with its installer script or in a virtual environment (CONTRIBUTING.md shows both), not with `pip` into the system Python, and run `pio run -e teensy31 && pio run -e teensy40` and `pio test -e native` in `diffbot_firmware`.

**The checks of the code's style**, each with the place where it runs:

| Changed files | Check | Where it runs | It passes when |
|---|---|---|---|
| `*.py` | flake8 and mypy, with `lint_changed_python` ([Python](#python)) | PC, repository root, in the lint environment | The last line is `checked … files`, and the exit status is 0 |
| C++ in `diffbot_base` and `diffbot_gazebo_plugins` | clang-tidy's include check with catkin's compile commands ([Check the includes](#check-the-includes)) | Dev container, `~/catkin_ws` | No `misc-include-cleaner` warning is in a line you added or changed |
| C++ in `diffbot_firmware` | The same check, with the flags of the PC build or of the board ([Check the includes](#check-the-includes)) | Firmware container, `diffbot_firmware` | The same |
| Shell scripts | shellcheck ([Launch files, shell scripts and workflows](#launch-files-shell-scripts-and-workflows)) | PC, repository root | A new script has no finding, and a changed script has no finding that `noetic-devel`'s version doesn't have |

## Code and tests

These rules hold for every language in this repository. `diffbot_gazebo/test/test_geometry_consistency.py` shows the design rules, and the firmware's tests in `diffbot_firmware/test/` show the test names.

### Design

- **One thing per function:** a function does one thing at one level of abstraction, and its name says what. A `main()` reads as the list of its steps.
- **Checks and a runner:** a function that checks several things becomes one function per check and one that runs them, as `test_geometry_consistency.py` does with its `check_…()` functions and `CHECKS`.
- **Ask or change:** a function returns an answer or changes state. One that has to do both says so in its name, so that a caller doesn't read the call as a plain question. The firmware's `takeSamplingSlot(now_ms)` returns whether a sampling slot is due, and when one is due it also notes the time of that slot. The verb "take" tells the caller that a second call gives another answer.
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

### READMEs

- **What a README holds:** what its folder contains, how to build, run and test it, and the facts a contributor looks up while changing the code, such as which class is a port. A design or a feature is explained in one place, its page on ros-mobile-robots.com, and the README links that page's section instead of explaining it again.
- **Links:** between two headings, link each destination once, at its first name. Later names for the same destination stay inline code.

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

PEP 8 with the settings of ROS 2's `ament_flake8` (99 columns, google import order, one import per line, single quotes), and type hints on every function and method (`-> None` on tests) that mypy checks for Python 3.8, Noetic's version. `diffbot_gazebo/test/test_drive.py` and `test_geometry_consistency.py` follow it. A script keeps its logic in functions that a test can import.

- **Which files:** every `.py` file. The files of dynamic_reconfigure in a package's `cfg` folder, such as `diffbot_base/cfg/Parameters.cfg`, are Python too, but they keep the layout of dynamic_reconfigure's generator: a star import and a table with aligned columns, which flake8 and mypy reject. The commands below don't select them. `catkin build` runs such a file and fails on an error in it.
- **The environment:** CI doesn't check the style yet (#117). Install flake8 with ament's plugins, mypy 1.10.1 (the version the project checks with) and the PyYAML type stubs in a virtual environment of Python 3.8 to 3.12, the versions that mypy 1.10.1 supports. Keep the environment outside the checkout. Inside the checkout, Git would list the environment's Python files as new files, and the commands would check them.
- **What the commands check:** every `.py` file that differs from the branch that the pull request goes to, committed or not, and every new one. That branch is `noetic-devel` of this repository, ros-mobile-robots/diffbot, not of a fork.
- **Which branch to pass:** in a clone of this repository, call the function without an argument. It then compares with `origin/noetic-devel`. In a clone of your fork, `origin` is the fork, whose `noetic-devel` can hold your own commits, so a comparison with that branch could find nothing to check. The function therefore takes `origin` only when the address that Git fetches from is this repository's on github.com, over HTTPS or SSH. With any other address it stops with an `error:` line. Fetch this repository's branch and pass that one. The last lines of the block show both calls.

On the PC, in the repository's root, in bash 4.4 or newer:

```bash
python3 -m venv ~/.venvs/diffbot-lint   # once
~/.venvs/diffbot-lint/bin/pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline \
  flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes \
  mypy==1.10.1 types-PyYAML             # once
source ~/.venvs/diffbot-lint/bin/activate

lint_changed_python() {
  local base_ref=${1:-} base list status=0 file
  local -a files
  # 1. The branch that the pull request goes to, and the commit where this branch left it.
  #    Without both, Git can't tell what changed
  if [ -z "$base_ref" ]; then
    # origin has to be this repository itself: the whole HTTPS or SSH address that Git fetches from
    if ! git remote get-url origin 2> /dev/null | grep -qxE \
        '(https://github\.com/|git@github\.com:|ssh://git@github\.com/)ros-mobile-robots/diffbot(\.git)?/?'; then
      echo "error: origin isn't github.com's ros-mobile-robots/diffbot. Name the branch that the" >&2
      echo "pull request goes to, for example: lint_changed_python upstream/noetic-devel" >&2
      return 2
    fi
    base_ref=origin/noetic-devel
  fi
  if ! git rev-parse --verify --quiet "$base_ref^{commit}" > /dev/null; then
    echo "error: $base_ref isn't in this clone. Fetch the branch first:" >&2
    echo "  git fetch ${base_ref%%/*} ${base_ref#*/}:refs/remotes/$base_ref" >&2
    return 2
  fi
  if ! base=$(git merge-base "$base_ref" HEAD); then
    echo "error: $base_ref and HEAD have no common commit. In a shallow clone, run" >&2
    echo "  git fetch --unshallow" >&2
    return 2
  fi
  # 2. The changed and the new Python files, each name followed by a NUL character
  list=$(mktemp) || return 2
  if ! git diff -z --name-only --diff-filter=d "$base" -- '*.py' > "$list" ||
     ! git ls-files -z --others --exclude-standard -- '*.py' >> "$list" ||
     ! sort -zu -o "$list" "$list"; then
    echo "error: Git couldn't list the changed files" >&2
    rm -f "$list"
    return 2
  fi
  mapfile -d '' -t files < "$list"
  rm -f "$list"
  # 3. flake8 on all files, mypy on each file
  if [ "${#files[@]}" -gt 0 ]; then
    flake8 --max-line-length 99 --import-order-style google \
      --extend-ignore B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202 \
      "${files[@]}" || status=1
    for file in "${files[@]}"; do
      mypy --python-version 3.8 --ignore-missing-imports --disallow-untyped-defs "$file" || status=1
    done
  fi
  echo "checked ${#files[@]} files"
  return "$status"
}

# In a clone of this repository:
lint_changed_python
# In a clone of your fork, compare with this repository's branch instead:
#   git remote add upstream https://github.com/ros-mobile-robots/diffbot.git   # once
#   git fetch upstream noetic-devel
#   lint_changed_python upstream/noetic-devel
```

The last line of the output is `checked … files`. The exit status (`echo $?`) is 0 when flake8 and mypy found nothing, also when no Python file changed. It is 1 when flake8 or mypy reported a finding, and 2 with an `error:` line when Git couldn't work out the changed files. A status of 2 means that nothing was checked.

### C++

The C++ of this repository is in three places: `diffbot_base`, `diffbot_gazebo_plugins` and the firmware in `diffbot_firmware`.

- **Include what you use:** every file includes the header of each name it uses itself, and doesn't rely on another header to bring it in. A header that can't be included alone stays behind its entry header with `// IWYU pragma: keep` and a comment that says why, as `ros/node_handle.h` in `diffbot_firmware/lib/imu/imu_publisher.h`, and `Arduino.h` and `Wire.h` in `diffbot_firmware/src/main.cpp`. [Check the includes](#check-the-includes) has the commands.
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

#### Check the includes

clang-tidy 18's check `misc-include-cleaner` reports a name whose header isn't included directly, and an include that isn't used. CI doesn't run the check yet (#117). Run it on every C++ file you changed, headers and board-only files included, and say in the pull request which files you checked.

- **Pass criterion:** no `misc-include-cleaner` warning is in a line you added or changed. A file you add has no warning at all: with `--warnings-as-errors='*'`, clang-tidy's exit status is then 0.
- **Older lines:** the files of `diffbot_base` and `diffbot_gazebo_plugins` and the firmware's board files have warnings in older lines. They belong to #117, not to your pull request.
- **Entry headers:** the firmware's commands below ignore the headers behind the entry headers `unity.h`, `Arduino.h` and `Wire.h`, so that a name from them counts as included.

**`diffbot_base` and `diffbot_gazebo_plugins`**, in the dev container, in `~/catkin_ws`. catkin writes each package's compile commands to `build/<package>/compile_commands.json`. For a header, clang-tidy takes the flags of the source file that is closest to the header:

```bash
sudo apt-get update && sudo apt-get install -y python3-venv   # once
python3 -m venv ~/.venvs/clang-tidy && ~/.venvs/clang-tidy/bin/pip install clang-tidy==18.1.8   # once
catkin build diffbot_base diffbot_gazebo_plugins --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
~/.venvs/clang-tidy/bin/clang-tidy -p build/diffbot_base --checks='-*,misc-include-cleaner' \
  src/diffbot/diffbot_base/src/pid.cpp src/diffbot/diffbot_base/include/diffbot_base/pid.h
~/.venvs/clang-tidy/bin/clang-tidy -p build/diffbot_gazebo_plugins --checks='-*,misc-include-cleaner' \
  src/diffbot/diffbot_gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
```

**The firmware**, in the firmware container. On the PC, in the repository's root, `.docker/firmware/run.sh bash` opens a shell there, in `diffbot_firmware`. Build both boards and run the unit tests first ([Commands](#commands)), so that PlatformIO has downloaded the libraries. In that shell:

```bash
python -m venv .pio/clang-tidy && .pio/clang-tidy/bin/pip install clang-tidy==18.1.8   # once
tidy=(.pio/clang-tidy/bin/clang-tidy --checks='-*,misc-include-cleaner'
      --config='{CheckOptions: {misc-include-cleaner.IgnoreHeaders: "unity.*;core_pins.h;WireKinetis.h;WireIMXRT.h"}}')
```

Files that build on the PC are the libraries without Arduino, `test/fakes` and the `test_*` suites. They get the flags of the `native` environment, and a header gets `-x c++-header` in front of them:

```bash
pc_flags=(-std=c++17 -Wall -DUNIT_TEST -Itest -I.pio/libdeps/native/Unity/src
          '-I.pio/libdeps/native/Rosserial Arduino Library/src')
for dir in lib/*/; do pc_flags+=("-I${dir%/}"); done
"${tidy[@]}" test/test_bno055/test_bno055.cpp -- "${pc_flags[@]}"
"${tidy[@]}" lib/imu/bno055.h -- -x c++-header "${pc_flags[@]}"
```

Board-only files are `src/main.cpp`, the libraries that use Arduino (`base_controller`, `config`, `encoder`, `motor_controllers`) and the tests that run on a Teensy. They get the flags that PlatformIO compiles `src/main.cpp` with, because that command names every include folder of the firmware. The function `set_board_flags` puts these flags into the array `board_flags`, in three steps. It stops with an `error:` line, an empty array and status 1 when a step fails or when the command lacks the board's processor, name or include folders. The checks are chained to it with `&&`, so they don't run with wrong flags, and the block's status (`echo $?`) is 0 only when the flags were found and clang-tidy could read both files:

```bash
set_board_flags() {
  local board=$1 list sysroot cxx multilib
  local gcc=/pio-core/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-g++
  board_flags=()
  # 1. PlatformIO's compile commands for the board, written anew
  rm -f compile_commands.json
  if ! pio run -e "$board" -t compiledb > /dev/null; then
    echo "error: PlatformIO wrote no compile commands for $board" >&2
    return 1
  fi
  # 2. The flags of the command for src/main.cpp: without the compiler, the file names and the
  #    switches that only GCC knows
  list=$(mktemp) || return 1
  if ! python - > "$list" << 'PY'
import json
import shlex

GCC_ONLY = {'-c', '-felide-constructors', '-fsingle-precision-constant', '-fpermissive',
            '-Wno-psabi', '-Wno-error=narrowing', '-Wno-maybe-uninitialized'}
with open('compile_commands.json') as database:
    command = next(entry['command'] for entry in json.load(database)
                   if entry['file'] == 'src/main.cpp')
flags = shlex.split(command)[1:-1]                  # without the compiler and the source file
del flags[flags.index('-o'):flags.index('-o') + 2]  # without -o and the object file
flags = [flag for flag in flags if flag not in GCC_ONLY]
for start in ('-mcpu=', '-DARDUINO_TEENSY', '-I'):  # the board's processor, its name, an include folder
    if not any(flag.startswith(start) for flag in flags):
        raise SystemExit(f'the command for src/main.cpp has no {start} flag')
print('\n'.join(flags))
PY
  then
    echo "error: compile_commands.json has no usable command for src/main.cpp" >&2
    rm -f "$list" compile_commands.json
    return 1
  fi
  mapfile -t board_flags < "$list"
  rm -f "$list" compile_commands.json   # the compile commands aren't committed
  # 3. What the board's GCC knows by itself and clang doesn't: the target, two integer types and
  #    the folders of the toolchain's C and C++ headers. Then Unity, for the tests on a Teensy
  sysroot=$(dirname "$(dirname "$gcc")")/arm-none-eabi
  if ! cxx=$sysroot/include/c++/$("$gcc" -dumpfullversion) ||
     ! multilib=$("$gcc" "${board_flags[@]}" -print-multi-directory); then
    echo "error: the toolchain's GCC didn't answer: $gcc" >&2
    board_flags=()
    return 1
  fi
  board_flags+=(--target=arm-none-eabi -D__INT32_TYPE__='long int' -D__UINT32_TYPE__='long unsigned int'
                -isystem "$cxx" -isystem "$cxx/arm-none-eabi/$multilib" -isystem "$sysroot/include"
                "-I.pio/libdeps/$board/Unity/src" -UUNITY_INCLUDE_CONFIG_H)
}

set_board_flags teensy31 &&   # or teensy40
  "${tidy[@]}" src/main.cpp -- "${board_flags[@]}" &&
  "${tidy[@]}" lib/base_controller/base_controller.h -- -x c++-header "${board_flags[@]}"
```

- **One board is enough today:** the firmware's own files have no code for one board only, and both boards give the same warnings. A file that gets such code is checked with both boards.
- **The two integer types:** `int32_t` is `long` for the board's GCC and `int` for clang. Without the two definitions, clang stops at two declarations of `random()` that then differ in their return type.
- **Unity:** the last two flags are for the tests that run on a Teensy. They name Unity's folder, and they leave out the configuration header that PlatformIO writes only when it builds such a test.

### Launch files, shell scripts and workflows

- **Launch files:** the first comment as above. A Boolean argument's `doc` says what `true` does, and another argument's `doc` names the values it takes. A `$(eval …)` expression gets a comment that says what it computes.
- **Shell scripts:** a Bash script that is run, not sourced, starts with `set -euo pipefail`. One with several steps has a function per step, `main "$@"` at the end, its constants as `readonly` with their unit, and a header that lists the steps as a numbered list.
- **Exceptions to `set -euo pipefail`:** a script may leave out one of the three options where it meets an environment that it doesn't control, with the reason in a comment at that line. The known case is ROS: Noetic's `setup.bash` reads `ROS_MASTER_URI` when the variable isn't set, so a script that sources that file can't use `-u`. Under `-u`, read a variable that may be unset as `${ROS_IP:-}`.
- **shellcheck:** a new script passes `shellcheck <script>` without a finding. Four of the six older scripts have findings. A change to one of them adds none, wherever in the script the new finding would show up: run shellcheck on the script as `noetic-devel` has it and on your version, and compare the two lists. On the PC, in the repository's root, for example for `.docker/robot/smoke_test.sh` (in a clone of a fork, with `upstream/noetic-devel`, see [Python](#python)):

    ```bash
    git show origin/noetic-devel:.docker/robot/smoke_test.sh | shellcheck -   # the findings before your change
    shellcheck .docker/robot/smoke_test.sh                                     # the findings with your change
    ```

    Without shellcheck on the PC, `docker run --rm -i -v "$PWD":/mnt:ro koalaman/shellcheck:v0.10.0` takes the place of `shellcheck` in both lines. shellcheck can't read ROS's `setup.bash` and reports SC1091 at a line that sources the file. `# shellcheck source=/dev/null` above the line says so. No workflow runs shellcheck yet.
- **The robot image's entry point:** a change to `.docker/robot/entrypoint.sh` is checked by the image's smoke test, `.docker/robot/smoke_test.sh`. The workflow Robot container runs the smoke test for the pull request, in a container that starts through the entry point.
- **Workflows:** the first comment says when the workflow runs and why it is a workflow of its own. Read-only permissions at the top, and more only in the job that needs them. Values from the event come in through `env:`, not through `${{ }}` inside `run:`, as in `.github/workflows/pr_description.yml`.

## Things that are easy to get wrong

- **The wheel geometry is in two places:** the URDF (`diffbot_description/config/diffbot/front_wheel.yaml`, Remo's in remo_description) and the controller (`diffbot_control/config/<model>/geometry.yaml`). Change both, or the odometry and the simulated robot disagree; `diffbot_gazebo/test/test_geometry_consistency.py` checks that they agree, and that the controller's wheel joints and their velocity interfaces are in the URDF.
- **A wheel's mass limits Gazebo's wheel gain:** if you change a wheel's mass or inertia in the URDF, check the P gain in `diffbot_control/config/<model>/pid.yaml` against the comment there.
- **remo_description is pinned to one version** in `diffbot_dev.repos` and `remo_robot.repos`. A new version of Remo's model reaches the diffbot packages only when the pin moves: move it in the pull request that adapts the diffbot packages to the new model, and test both robots. An existing workspace keeps its old checkout when the pin moves (the setup imports with `--skip-existing`): check out the new version in `src/remo_description` and rebuild.
- **The documentation quotes this code** at a pinned commit, by line ranges, for example `test_drive.py` on the Simulation Test page. If you add or remove lines there, say so in the PR, so the docs can be repinned.
- **Units and frames** follow [REP 103](https://www.ros.org/reps/rep-0103.html); `base_footprint` is the robot's reference frame, on the ground below the drive axle's midpoint, with `base_link` above it; the chassis and its parts hang off `chassis_link`.
- **The real robot reads the same configuration** as the simulation (the hardware interface and the firmware read `wheel_radius`). A configuration change affects both.

## Commits and pull requests

- **One topic:** one feature or fix per pull request and per commit.
- **Branch:** `feat/…`, `fix/…`, `docs/…`, `chore/…`, `refactor/…` or `test/…`, followed by a short topic in kebab-case, such as `fix/encoder-ticks`.
- **Title and description:** pull requests are squash-merged. The title is the commit subject, `Area: what changed`, 65 characters or fewer before GitHub's PR-number suffix. The whole description becomes the commit message: what and why in two or three sentences, then how it was tested. Use the pull request template.
- **Issues:** end the description with `Closes #N` when the pull request resolves issue N. For an issue that stays open, name the issue without a closing keyword, as in `Part of #N`.
- **Signed commits:** sign every commit with GPG, `git commit -S`. `git log --show-signature -1` shows the signature of the last commit. A coding agent that can't sign stops and reports that, and doesn't commit unsigned.
- **Comments:** see [Comments and docstrings](#comments-and-docstrings).
- **Documentation:** every feature comes with documentation on ros-mobile-robots.com, as a pull request in ros-mobile-robots.github.io that is planned together with the code, not after the review. Its pages say what the feature does and how to use it, and how the code works, with excerpts of the code and a figure for each flow. A tests page walks through each test in the order a reader would run it: what the test sets up, what goes in, what the robot or the node does, and what is checked with which tolerance, each with its code excerpt, and one picture per test. The writing rules are in that repository's `AGENTS.md`. Link the pull request under `## Documentation` in the description as `ros-mobile-robots/ros-mobile-robots.github.io#N`, also when it's opened after the code pull request. A page on ros-mobile-robots.com that already explains the change works too, and `None` when nothing user-visible changes. The PR description check (`.github/workflows/pr_description.yml`) fails otherwise. After an edit, the new run counts, a re-run of an old run still sees the old text.
- **AI tools:** no attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
- **Review:** write descriptions and review replies so that a maintainer can check them quickly: specific, short, no claims you didn't verify.
