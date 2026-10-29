# Contributing to DiffBot

DiffBot and Remo are two differential drive robots for ROS 1 Noetic. This repository has their ROS packages, the Teensy firmware and the dev container. Remo's robot model is in [remo_description](https://github.com/ros-mobile-robots/remo_description), and the documentation is at [ros-mobile-robots.com](https://ros-mobile-robots.com), from [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io).

Ask questions and get help with your build in [Discussions](https://github.com/ros-mobile-robots/diffbot/discussions/categories/q-a), where the answers stay easy to find. Report bugs and request features as an issue in the repository they affect: [diffbot](https://github.com/ros-mobile-robots/diffbot/issues), [remo_description](https://github.com/ros-mobile-robots/remo_description/issues) for Remo's model, or [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/issues) for the documentation. For a larger change, open an issue first, so the approach can be agreed before you write the code. This branch is ROS 1 Noetic; a move to ROS 2 is [future work](README.md#ros-2), so please discuss it in an issue first.

The documentation's [Contributing](https://ros-mobile-robots.com/development/) section explains the repositories, the roadmap, the dev container and the automated checks in more detail.

## Set up

The quickest working setup is the dev container in [`.devcontainer/noetic`](.devcontainer/noetic): open your clone in VS Code and choose **Reopen in Container**. It has ROS Noetic, Gazebo 11 and everything the packages need, clones `remo_description`, at the version pinned in [`diffbot_dev.repos`](diffbot_dev.repos), and `rplidar_ros`, and builds the workspace in `~/catkin_ws`. CI builds the same image. Without the container, follow the [installation steps](README.md#installation) in the README.

When the pin moves to a new version, an existing workspace keeps its old checkout of `remo_description`, because the setup doesn't replace repositories that are already there. Check out the new version yourself, then build again. If `git status` shows changes of your own, commit them on a branch first: the checkout would either stop on them or carry them over to the new version.

```bash
cd ~/catkin_ws/src/remo_description
git status                                          # no changes of your own?
git fetch --tags
git checkout 2.0.0                                  # the version in diffbot_dev.repos
```

## Build and test

In `~/catkin_ws`:

```bash
catkin build                                        # build the workspace
source devel/setup.bash
catkin build --catkin-make-args run_tests           # build and run all tests
catkin_test_results build                           # summary: 10 tests, 0 errors, 0 failures
rostest diffbot_gazebo sim_drive_diffbot.test       # one robot's simulation test
```

The [simulation test](https://ros-mobile-robots.com/development/simulation-test/) drives DiffBot and Remo in Gazebo and checks where they went. Run it for any change to the robot models, the controllers or the simulation.

The Teensy firmware builds with [PlatformIO](https://platformio.org/), which isn't in the dev container. The firmware has a container of its own, [`.docker/firmware`](.docker/firmware), with PlatformIO 6.2.0 and a compiler, so Docker is all you need. The script [`.docker/firmware/pio`](.docker/firmware/pio) runs PlatformIO's `pio` command in that container. Build both boards that CI builds, from your diffbot clone on the PC:

```bash
.docker/firmware/pio run -e teensy31
.docker/firmware/pio run -e teensy40
```

The script builds the image first, when it's missing or its Dockerfile changed. It runs the container as your user (`id -u` and `id -g`), so the files it writes belong to you. To use other IDs, set `LOCAL_UID` and `LOCAL_GID`, for example `LOCAL_UID=1001 LOCAL_GID=1001 .docker/firmware/pio run -e teensy40`. The first firmware build downloads the Teensy toolchain, which needs the internet and took about a minute on our test PC. The firmware files land in `diffbot_firmware/.pio/build/<board>/`, for example `firmware.hex`, which PJRC's [Teensy Loader](https://www.pjrc.com/teensy/loader.html) can flash. Don't build in the container and on the PC at the same time: both write to that `.pio` folder. CI builds the same way.

Or install PlatformIO on your PC, outside the dev container, with its [installer script](https://docs.platformio.org/en/stable/core/installation/methods/installer-script.html) or in a virtual environment. Many Linux distributions don't allow `pip install` into the system Python. Then build from your diffbot clone on the PC:

```bash
python3 -m venv ~/.venvs/platformio && ~/.venvs/platformio/bin/pip install platformio   # once
cd diffbot_firmware
~/.venvs/platformio/bin/pio run -e teensy31 && ~/.venvs/platformio/bin/pio run -e teensy40
```

CI runs on every pull request: [industrial_ci](https://github.com/ros-industrial/industrial_ci) builds and tests the workspace for Noetic, the firmware is built for both boards, and the dev container image is built.

## Pull requests

- **One feature or fix per pull request**, on a branch named `feat/…`, `fix/…`, `docs/…` or `chore/…`. Keep unrelated changes for another PR.
- **Pull requests are squash-merged.** The PR title becomes the commit's subject, so write it as `Area: what changed`, for example `Simulation test: narrower turn checks`. Aim for 65 characters or fewer; GitHub adds the PR number. The whole description becomes the commit message, so start it with what changes and why.
- **Describe how you tested it:** what you ran, on which robot or in simulation, and what you saw. Link the issue with `Closes #N`.
- **Comments explain why** the code is the way it is, not what changed or how it used to be. The history belongs in the commit and the PR.
- **Every feature also gets documentation** on ros-mobile-robots.com, a new or updated page: open a pull request in [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) and link it.
- Follow [REP 103](https://www.ros.org/reps/rep-0103.html) for units and coordinate frames.

## AI-assisted contributions

You're welcome to use AI tools. Whatever produced the code, you're its author and responsible for it:

- Understand every change and be able to explain it in review, in your own words.
- Build and test it as described above, and report what you actually ran. Don't claim tests that didn't run.
- Check that the APIs, parameters, files and links you use exist. Pull requests with invented ones are closed.
- Keep the PR description and your review replies short and specific: what changed, why, and how it was tested.

You don't need to say whether you used AI. Please don't add attribution lines for AI tools to commits or pull requests, such as `Co-Authored-By:` for a tool or "Generated with …".

## License

By contributing, you agree that your contributions are licensed under this repository's [BSD 3-Clause License](LICENSE).
