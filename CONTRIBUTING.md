# Contributing to DiffBot

DiffBot and Remo are two differential drive robots for ROS 1 Noetic. This repository has their ROS packages, the Teensy firmware and the dev container. Remo's robot model is in [remo_description](https://github.com/ros-mobile-robots/remo_description), and the documentation is at [ros-mobile-robots.com](https://ros-mobile-robots.com), from [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io).

Questions, bug reports and ideas are welcome as issues. For a larger change, open an issue first, so the approach can be agreed before you write the code.

## Set up

The quickest working setup is the dev container in [`.devcontainer/noetic`](.devcontainer/noetic): open your clone in VS Code and choose **Reopen in Container**. It has ROS Noetic, Gazebo 11 and everything the packages need, clones `remo_description` and `rplidar_ros`, and builds the workspace in `~/catkin_ws`. CI builds the same image. Without the container, follow the [installation steps](README.md#installation) in the README.

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

The Teensy firmware builds with [PlatformIO](https://platformio.org/), which isn't in the dev container; install it on your PC with `pip install platformio`, as CI does. Build both boards that CI builds:

```bash
cd diffbot_base/scripts/base_controller
pio run -e teensy31 && pio run -e teensy40
```

CI runs on every pull request: [industrial_ci](https://github.com/ros-industrial/industrial_ci) builds and tests the workspace for Noetic, the firmware is built for both boards, and the dev container image is built.

## Pull requests

- **One feature or fix per pull request**, on a branch named `feat/…`, `fix/…`, `docs/…` or `chore/…`. Keep unrelated changes for another PR.
- **Pull requests are squash-merged.** The PR title becomes the commit's subject, so write it as `Area: what changed`, at most about 65 characters, for example `Simulation test: narrower turn checks`. The first paragraph of the description should say what changes and why; it becomes the commit message.
- **Describe how you tested it:** what you ran, on which robot or in simulation, and what you saw. Link the issue with `Closes #N`.
- **Comments explain why** the code is the way it is, not what changed or how it used to be. The history belongs in the commit and the PR.
- **Every feature also gets documentation** on ros-mobile-robots.com: open a pull request in [ros-mobile-robots.github.io](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io) and link it.
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
